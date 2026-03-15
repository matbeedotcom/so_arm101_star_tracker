"""
SpecklePipeline — orchestrates capture → save → process → reconstruct.

Processing runs in a background process so it never blocks the tracking loop.
"""

import os
import time
import multiprocessing as mp
from datetime import datetime

from .config import CaptureConfig, StabilityConfig, ProcessingConfig
from .capture import SpeckleCapture, BurstResult
from .storage import save_burst, load_burst
from .processing import SpeckleProcessor


def _process_worker(burst_dir, processing_config_dict):
    """Run in a child process: load burst from disk, process, save results."""
    import numpy as np

    config = ProcessingConfig(**processing_config_dict)
    processor = SpeckleProcessor(config)

    frames_by_cam, metadata = load_burst(burst_dir)

    results_dir = os.path.join(burst_dir, "results")
    os.makedirs(results_dir, exist_ok=True)

    for cam_idx, frames in frames_by_cam.items():
        if not frames:
            continue

        t0 = time.time()
        ps = processor.compute_power_spectrum(frames)
        t_ps = time.time() - t0

        t0 = time.time()
        ac = processor.compute_autocorrelation_from_ps(ps)
        t_ac = time.time() - t0

        np.save(os.path.join(results_dir, f"cam{cam_idx}_power_spectrum.npy"), ps)
        np.save(os.path.join(results_dir, f"cam{cam_idx}_autocorrelation.npy"), ac)

        t_recon = 0
        if len(frames) >= 10:
            t0 = time.time()
            recon = processor.reconstruct_from_speckle(frames, ps)
            t_recon = time.time() - t0
            np.save(os.path.join(results_dir, f"cam{cam_idx}_reconstruction.npy"), recon)

        print(f"      cam{cam_idx}: ps={t_ps:.1f}s ac={t_ac:.1f}s recon={t_recon:.1f}s "
              f"({len(frames)} frames, {frames[0].shape})")

    print(f"    [process] Results saved to {results_dir}")


class SpecklePipeline:
    """End-to-end speckle interferometry pipeline."""

    def __init__(self, cam, capture_config=None, stability_config=None,
                 processing_config=None, output_dir='speckle_captures'):
        """
        Args:
            cam: Initialized ArducamQuadCapture (or None for offline-only).
            capture_config: CaptureConfig (defaults used if None).
            stability_config: StabilityConfig (defaults used if None).
            processing_config: ProcessingConfig (defaults used if None).
            output_dir: Base directory for burst storage.
        """
        self.capture_config = capture_config or CaptureConfig()
        self.stability_config = stability_config or StabilityConfig()
        self.processing_config = processing_config or ProcessingConfig()
        self.output_dir = output_dir

        self.capturer = None
        if cam is not None:
            self.capturer = SpeckleCapture(
                cam, self.capture_config, self.stability_config)

        self.processor = SpeckleProcessor(self.processing_config)
        self._session = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
        self._burst_counter = 0
        self._bg_processes = []  # track background workers

    def run(self, imu_bus=None, servo_hw=None, context=None):
        """Capture a burst, save it, kick off background processing.

        Args:
            imu_bus: smbus2.SMBus instance (optional).
            servo_hw: (port_handler, packet_handler) tuple (optional).
            context: dict of metadata (target name, alt/az, etc.).

        Returns:
            dict with keys:
                'burst': BurstResult
                'burst_dir': path to saved burst
        """
        if self.capturer is None:
            raise RuntimeError("No camera — use run_offline() for saved data")

        context = context or {}

        # Reap finished background workers
        self._reap_workers()

        # Capture
        burst = self.capturer.capture_burst(imu_bus, servo_hw, context)
        if not burst.stable or burst.burst_count == 0:
            return {
                'burst': burst,
                'burst_dir': None,
            }

        # Build metadata for storage
        meta = {
            'burst_count': burst.burst_count,
            'duration': burst.duration,
            'timestamps': burst.timestamps,
            'imu_before': burst.imu_before,
            'imu_after': burst.imu_after,
            'servo_positions': burst.servo_positions,
            'context': burst.context,
            'exposure_us': self.capture_config.exposure_us,
        }

        target_name = context.get('target_name', 'unknown')
        print(f"    [pipeline] Saving burst #{self._burst_counter}...")
        burst_dir = save_burst(
            burst.frames_by_cam, meta,
            self.output_dir, target_name, self._session, self._burst_counter)
        self._burst_counter += 1
        print(f"    [pipeline] Saved to {burst_dir}")

        # Launch processing in background process
        self._launch_processing(burst_dir)

        return {
            'burst': burst,
            'burst_dir': burst_dir,
        }

    def _launch_processing(self, burst_dir):
        """Spawn a background process to run speckle processing on saved burst."""
        # Serialize config as dict so it's picklable
        config_dict = {
            'image_size': self.processing_config.image_size,
            'pixel_size_um': self.processing_config.pixel_size_um,
            'wavelength_nm': self.processing_config.wavelength_nm,
            'camera_baselines': self.processing_config.camera_baselines,
            'max_bispectrum_triangles': self.processing_config.max_bispectrum_triangles,
        }
        p = mp.Process(
            target=_process_worker,
            args=(burst_dir, config_dict),
            daemon=True,
        )
        p.start()
        self._bg_processes.append(p)
        print(f"    [pipeline] Processing started in background (PID {p.pid})")

    def _reap_workers(self):
        """Clean up finished background processes and report."""
        still_running = []
        for p in self._bg_processes:
            if p.is_alive():
                still_running.append(p)
            else:
                p.join(timeout=0)
                if p.exitcode == 0:
                    print(f"    [pipeline] Background process PID {p.pid} finished OK")
                else:
                    print(f"    [pipeline] Background process PID {p.pid} "
                          f"exited with code {p.exitcode}")
        if len(still_running) > 0:
            print(f"    [pipeline] {len(still_running)} background process(es) still running")
        self._bg_processes = still_running

    def wait_for_processing(self, timeout=None):
        """Block until all background processing is done."""
        for p in self._bg_processes:
            p.join(timeout=timeout)
        self._reap_workers()

    def run_offline(self, burst_dir):
        """Load a saved burst, process synchronously, return results.

        Args:
            burst_dir: Path to a burst directory.

        Returns:
            dict with processing results.
        """
        frames_by_cam, metadata = load_burst(burst_dir)

        results = self._process_burst_sync(frames_by_cam)
        results['burst'] = None
        results['burst_dir'] = burst_dir
        results['metadata'] = metadata
        return results

    def _process_burst_sync(self, frames_by_cam):
        """Process frames per-camera synchronously (for offline use)."""
        power_spectra = {}
        autocorrelations = {}
        reconstructions = {}

        for cam_idx, frames in frames_by_cam.items():
            if not frames:
                continue

            t0 = time.time()
            ps = self.processor.compute_power_spectrum(frames)
            power_spectra[cam_idx] = ps
            t_ps = time.time() - t0

            t0 = time.time()
            autocorrelations[cam_idx] = self.processor.compute_autocorrelation_from_ps(ps)
            t_ac = time.time() - t0

            t_recon = 0
            if len(frames) >= 10:
                t0 = time.time()
                reconstructions[cam_idx] = self.processor.reconstruct_from_speckle(frames, ps)
                t_recon = time.time() - t0

            print(f"      cam{cam_idx}: ps={t_ps:.1f}s ac={t_ac:.1f}s recon={t_recon:.1f}s "
                  f"({len(frames)} frames, {frames[0].shape})")

        return {
            'power_spectra': power_spectra,
            'autocorrelations': autocorrelations,
            'reconstructions': reconstructions,
        }

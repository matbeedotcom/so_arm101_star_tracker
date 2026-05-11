"""Slew and tracking loops, speckle capture integration."""

import sys
import time
import math
import signal

from .config import (
    MOVE_SPEED, TRACK_SPEED, TOLERANCE_DEG,
    SLEW_SETTLE, TRACK_INTERVAL, TRACK_SETTLE,
    MODE_IMU, angle_diff,
)
from .imu import read_imu, calib_str, gravity_pitch
from .servos import read_servo, wait_all_stopped
from .celestial import compute_altaz
from .strategy import MotionStrategy

# Global flag for graceful shutdown
running = True


def _signal_handler(sig, frame):
    global running
    print("\n\nStopping tracker...")
    running = False


signal.signal(signal.SIGINT, _signal_handler)


# ── Camera / speckle pipeline ──

def init_camera(exposure):
    """Try to initialize quad camera. Returns ArducamQuadCapture or None."""
    try:
        sys.path.insert(0, '/home/acidhax/dev/telescope/src')
        from capture.arducam_quad_capture import ArducamQuadCapture

        cam = ArducamQuadCapture(resolution=(5120, 800), bit_depth=8)
        cam.initialize()
        if cam.picam2 and exposure:
            cam.picam2.set_controls({"ExposureTime": exposure})
        print(f"Quad camera initialized (exposure={exposure}us)")
        return cam
    except Exception as e:
        print(f"Camera init failed ({e}) — tracking without capture")
        return None


def init_speckle_pipeline(cam, exposure, burst_count, capture_dir):
    """Create a SpecklePipeline wrapping the camera. Returns pipeline or None."""
    if cam is None:
        return None
    from speckle import (SpecklePipeline, CaptureConfig, StabilityConfig,
                         ProcessingConfig)
    return SpecklePipeline(
        cam,
        capture_config=CaptureConfig(exposure_us=exposure, burst_count=burst_count),
        stability_config=StabilityConfig(),
        processing_config=ProcessingConfig(),
        output_dir=capture_dir,
    )


def run_capture(pipeline, ph, pkt, imu_bus, target_info, target_name,
                active_mode=MODE_IMU):
    """Run a speckle burst capture. Returns elapsed time or None."""
    if pipeline is None:
        return None
    t0 = time.time()
    try:
        target_alt, target_az = compute_altaz(target_info)
    except Exception:
        target_alt, target_az = None, None

    context = {
        'target_name': target_name,
        'target_alt': target_alt,
        'target_az': target_az,
        'imu_mode': 'IMU' if active_mode == MODE_IMU else 'NDOF',
    }
    print(f"  [pipeline] Starting capture for {target_name}...")
    results = pipeline.run(imu_bus, (ph, pkt), context)
    burst = results.get('burst')
    if burst and not burst.stable:
        print(f"  [pipeline] Skipped — not stable")
        return None
    if burst and burst.burst_count == 0:
        print(f"  [pipeline] Skipped — 0 frames captured")
        return None
    elapsed = time.time() - t0
    n = burst.burst_count if burst else 0
    bdir = results.get('burst_dir', '')
    print(f"  [pipeline] Done: {n} frames captured+saved in {elapsed:.1f}s"
          + (f"\n  [pipeline] Saved: {bdir}" if bdir else ""))
    return elapsed


# ── Slew ──

def slew_to_target(target_info, imu_bus, ph, pkt, strategy,
                    target_name='target', pipeline=None, active_mode=MODE_IMU,
                    should_stop=None, status_cb=None):
    """Initial slew to get close to the target.

    Optional hooks for remote control:
        should_stop(): -> bool, called per iteration. Defaults to the module
                       `running` flag so CLI Ctrl+C still works.
        status_cb(dict): live telemetry on each iteration.
    """
    print("\n--- Slewing to target ---")

    if should_stop is None:
        should_stop = lambda: not running  # noqa: E731

    best_error = float('inf')

    for iteration in range(50):
        if should_stop():
            return False

        target_alt, target_az = compute_altaz(target_info)
        target_alt = max(0.0, target_alt)
        imu = read_imu(imu_bus, samples=3, interval=0.02)
        gpitch = gravity_pitch(imu_bus)
        az_error = angle_diff(target_az, imu['heading'])
        alt_error = target_alt - gpitch
        total_error = math.sqrt(az_error**2 + alt_error**2)

        status = strategy.get_status(ph, pkt)
        status_str = ' '.join(f'{k}={v}' for k, v in status.items() if v is not None)
        print(f"  [{iteration+1}] Az={imu['heading']:.1f}->{target_az:.1f} ({az_error:+.1f}°)  "
              f"Alt={gpitch:.1f}->{target_alt:.1f} ({alt_error:+.1f}°)  "
              f"Err={total_error:.1f}°  [{status_str}]")

        if status_cb is not None:
            status_cb({
                'phase': 'slew', 'iteration': iteration + 1,
                'target_alt': target_alt, 'target_az': target_az,
                'imu_heading': imu['heading'], 'imu_pitch': gpitch,
                'az_err': az_error, 'alt_err': alt_error,
                'total_err': total_error, 'calib': calib_str(imu, active_mode),
            })

        best_error = min(best_error, total_error)

        if total_error < TOLERANCE_DEG:
            print(f"  Slew complete — error {total_error:.2f}°")
            if pipeline:
                elapsed = run_capture(pipeline, ph, pkt, imu_bus,
                                       target_info, target_name, active_mode)
                if elapsed is not None:
                    print(f"  First light captured in {elapsed:.1f}s")
            return True

        # Ramp gain
        gain = min(0.6, total_error / 50.0 + 0.15)
        strategy.apply_correction(az_error, alt_error, gain, MOVE_SPEED,
                                   ph, pkt, imu_bus)
        time.sleep(SLEW_SETTLE)

    print(f"  Slew did not fully converge (best {best_error:.2f}°), switching to tracking.")
    return True


# ── Track ──

def track_target(target_info, target_name, imu_bus, ph, pkt, strategy,
                  pipeline=None, active_mode=MODE_IMU,
                  should_stop=None, status_cb=None):
    """Continuously track the target, correcting for sky motion.

    Optional hooks for remote control: see slew_to_target.
    """
    print(f"\n--- Tracking {target_name} (Ctrl+C to stop) ---")
    print(f"{'Time':>10}  {'Target Az':>10} {'Target Alt':>10}  "
          f"{'IMU Az':>8} {'IMU Alt':>8}  {'Err':>5}  {'Calib':>6}")
    print("-" * 75)

    if should_stop is None:
        should_stop = lambda: not running  # noqa: E731

    correction_count = 0
    capture_count = 0

    while not should_stop():
        target_alt, target_az = compute_altaz(target_info)
        target_alt = max(0.0, target_alt)
        imu = read_imu(imu_bus, samples=3, interval=0.03)
        gpitch = gravity_pitch(imu_bus)

        az_error = angle_diff(target_az, imu['heading'])
        alt_error = target_alt - gpitch
        total_error = math.sqrt(az_error**2 + alt_error**2)

        now = time.strftime("%H:%M:%S")
        cal = calib_str(imu, active_mode)

        print(f"  {now}  Az={target_az:7.2f}° Alt={target_alt:7.2f}°  "
              f"H={imu['heading']:7.2f} P={gpitch:7.2f}  "
              f"{total_error:4.1f}°  {cal}", end="")

        if status_cb is not None:
            status_cb({
                'phase': 'track',
                'target_alt': target_alt, 'target_az': target_az,
                'imu_heading': imu['heading'], 'imu_pitch': gpitch,
                'az_err': az_error, 'alt_err': alt_error,
                'total_err': total_error, 'calib': cal,
                'corrections': correction_count, 'captures': capture_count,
            })

        if total_error > TOLERANCE_DEG * 2:
            gain = min(0.4, total_error / 30.0 + 0.1)
            strategy.apply_correction(az_error, alt_error, gain, TRACK_SPEED,
                                       ph, pkt, imu_bus)
            correction_count += 1
            print(f"  <- correcting (#{correction_count}, gain={gain:.2f})")
            wait_all_stopped(ph, pkt, timeout=1.5)
            time.sleep(TRACK_SETTLE)
        elif total_error > TOLERANCE_DEG:
            strategy.apply_correction(az_error, alt_error, 0.1, TRACK_SPEED,
                                       ph, pkt, imu_bus)
            correction_count += 1
            print(f"  <- nudge (#{correction_count})")
            wait_all_stopped(ph, pkt, timeout=1.0)
            time.sleep(TRACK_SETTLE)
        else:
            if pipeline:
                capture_count += 1
                elapsed = run_capture(pipeline, ph, pkt, imu_bus,
                                       target_info, target_name, active_mode)
                if elapsed is not None:
                    print(f"  OK  [burst #{capture_count} in {elapsed:.1f}s]")
                else:
                    print(f"  OK  [skipped — not yet stable]")
            else:
                print(f"  OK")

        time.sleep(TRACK_INTERVAL)

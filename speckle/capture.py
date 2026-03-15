"""
SpeckleCapture — rapid burst capture with stability gating.

Does NOT own camera or hardware lifecycle.  Receives initialized
ArducamQuadCapture + config, captures N short-exposure frames as fast
as possible (no disk I/O during burst).
"""

import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional


@dataclass
class BurstResult:
    """Result of a single burst capture."""
    frames_by_cam: dict          # {cam_idx: list[np.ndarray]}
    timestamps: list             # per-frame timestamps (time.time())
    burst_count: int             # actual frames captured
    duration: float              # total burst time (seconds)
    imu_before: Optional[dict] = None
    imu_after: Optional[dict] = None
    servo_positions: Optional[dict] = None
    context: dict = field(default_factory=dict)  # target name, alt/az, etc.
    stable: bool = True


class SpeckleCapture:
    """Burst capture engine for speckle interferometry."""

    def __init__(self, cam, capture_config, stability_config):
        """
        Args:
            cam: Initialized ArducamQuadCapture instance.
            capture_config: CaptureConfig dataclass.
            stability_config: StabilityConfig dataclass.
        """
        self.cam = cam
        self.cc = capture_config
        self.sc = stability_config

    def set_exposure(self, exposure_us):
        """Change camera exposure between bursts."""
        self.cc.exposure_us = exposure_us
        if self.cam.picam2:
            self.cam.picam2.set_controls({"ExposureTime": exposure_us})

    def capture_burst(self, imu_bus=None, servo_hw=None, context=None):
        """Capture a burst of frames from all 4 cameras.

        Args:
            imu_bus: smbus2.SMBus instance (optional, for stability gating).
            servo_hw: (port_handler, packet_handler) tuple (optional).
            context: dict of metadata to attach (target name, alt/az, etc.).

        Returns:
            BurstResult dataclass.
        """
        context = context or {}
        ph, pkt = servo_hw if servo_hw else (None, None)

        # --- Pre-check: servos stopped ---
        if ph and pkt and self.sc.require_stable:
            print("    [capture] Waiting for servos to stop...")
            if not _wait_all_stopped(ph, pkt,
                                     speed_threshold=self.sc.servo_speed_threshold,
                                     timeout=self.sc.servo_timeout):
                print("    [capture] SKIP — servos still moving after "
                      f"{self.sc.servo_timeout}s timeout")
                return BurstResult(
                    frames_by_cam={}, timestamps=[], burst_count=0,
                    duration=0.0, context=context, stable=False,
                )
            print("    [capture] Servos stopped")

        # --- Pre-check: IMU stable ---
        imu_before = None
        if imu_bus and self.sc.require_stable:
            print("    [capture] Waiting for IMU to stabilize...")
            imu_before = _wait_imu_stable(
                imu_bus,
                threshold=self.sc.imu_threshold,
                samples=self.sc.imu_samples,
                interval=self.sc.imu_interval,
                timeout=self.sc.imu_timeout,
            )
            if imu_before is None:
                print("    [capture] SKIP — IMU not stable after "
                      f"{self.sc.imu_timeout}s timeout")
                return BurstResult(
                    frames_by_cam={}, timestamps=[], burst_count=0,
                    duration=0.0, context=context, stable=False,
                )
            print(f"    [capture] IMU stable (H={imu_before['heading']:.1f} "
                  f"P={imu_before['pitch']:.1f} "
                  f"spread H={imu_before['h_spread']:.3f} P={imu_before['p_spread']:.3f})")

        # --- Read servo positions ---
        servo_positions = None
        if ph and pkt:
            servo_positions = {}
            for sid in range(1, 6):
                pos, result, _ = pkt.read2ByteTxRx(ph, sid, 56)  # ADDR_PRESENT_POSITION
                if result == 0:  # COMM_SUCCESS
                    servo_positions[sid] = pos

        # --- Tight burst loop (no I/O) ---
        print(f"    [capture] Starting burst: {self.cc.burst_count} frames, "
              f"exposure={self.cc.exposure_us}us")
        all_frames = {i: [] for i in range(4)}
        timestamps = []
        t0 = time.time()

        for frame_i in range(self.cc.burst_count):
            combined = self.cam.capture_combined()
            ts = time.time()
            sub_images = self.cam.split_cameras(combined)
            for cam_idx, img in enumerate(sub_images):
                all_frames[cam_idx].append(img)
            timestamps.append(ts)
            if self.cc.burst_interval > 0:
                time.sleep(self.cc.burst_interval)
            # Progress every 25%
            if self.cc.burst_count >= 20 and (frame_i + 1) % (self.cc.burst_count // 4) == 0:
                elapsed = time.time() - t0
                fps = (frame_i + 1) / elapsed if elapsed > 0 else 0
                print(f"    [capture] {frame_i+1}/{self.cc.burst_count} "
                      f"({fps:.1f} fps)")

        duration = time.time() - t0
        fps = len(timestamps) / duration if duration > 0 else 0
        print(f"    [capture] Burst complete: {len(timestamps)} frames "
              f"in {duration:.2f}s ({fps:.1f} fps)")

        # --- Post-burst IMU reading ---
        imu_after = None
        if imu_bus:
            imu_after = _read_imu_quick(imu_bus)
            if imu_before:
                drift_h = abs(imu_after['heading'] - imu_before['heading'])
                drift_p = abs(imu_after['pitch'] - imu_before['pitch'])
                print(f"    [capture] IMU drift during burst: "
                      f"H={drift_h:.2f} P={drift_p:.2f} deg")

        return BurstResult(
            frames_by_cam=all_frames,
            timestamps=timestamps,
            burst_count=len(timestamps),
            duration=duration,
            imu_before=imu_before,
            imu_after=imu_after,
            servo_positions=servo_positions,
            context=context,
            stable=True,
        )


# ── Stability helpers (self-contained, mirror goto.py logic) ──

# BNO055 registers
_QUA_DATA_W_LSB = 0x20
_CALIB_STAT = 0x35
_IMU_ADDR = 0x28

# Camera forward in IMU body frame (must match goto.py)
_CAM_FORWARD = (0.0, 1.0, 0.0)

import struct
import math


def _quat_rotate(q, v):
    w, x, y, z = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def _read_quat(bus):
    data = bus.read_i2c_block_data(_IMU_ADDR, _QUA_DATA_W_LSB, 8)
    w = struct.unpack('<h', bytes(data[0:2]))[0] / 16384.0
    x = struct.unpack('<h', bytes(data[2:4]))[0] / 16384.0
    y = struct.unpack('<h', bytes(data[4:6]))[0] / 16384.0
    z = struct.unpack('<h', bytes(data[6:8]))[0] / 16384.0
    return (w, x, y, z)


def _quat_to_hp(q):
    wx, wy, wz = _quat_rotate(q, _CAM_FORWARD)
    horiz = math.sqrt(wx * wx + wy * wy)
    pitch = math.degrees(math.atan2(wz, horiz)) if horiz > 1e-6 else (90.0 if wz > 0 else -90.0)
    heading = math.degrees(math.atan2(wx, wy)) % 360.0
    return heading, pitch


def _wait_all_stopped(ph, pkt, speed_threshold=5, timeout=2.0):
    """Poll servo speed registers, return True when all near zero."""
    ADDR_PRESENT_SPEED = 58
    start = time.time()
    while time.time() - start < timeout:
        all_stopped = True
        for sid in list(range(1, 6)) + [7, 8, 9]:
            try:
                speed, result, _ = pkt.read2ByteTxRx(ph, sid, ADDR_PRESENT_SPEED)
                if result != 0 or speed > speed_threshold:
                    all_stopped = False
                    break
            except Exception:
                all_stopped = False
                break
        if all_stopped:
            return True
        time.sleep(0.05)
    return False


def _wait_imu_stable(bus, threshold=0.15, samples=6, interval=0.05, timeout=3.0):
    """Wait until IMU readings stabilize (quaternion-based, roll-invariant)."""
    start = time.time()
    while time.time() - start < timeout:
        headings, pitches = [], []
        for _ in range(samples):
            h, p = _quat_to_hp(_read_quat(bus))
            headings.append(h)
            pitches.append(p)
            time.sleep(interval)

        h_spread = max(headings) - min(headings)
        if h_spread > 180:
            shifted = [(h + 180) % 360 for h in headings]
            h_spread = max(shifted) - min(shifted)
        p_spread = max(pitches) - min(pitches)

        if h_spread < threshold and p_spread < threshold:
            hx = sum(math.cos(math.radians(h)) for h in headings) / len(headings)
            hy = sum(math.sin(math.radians(h)) for h in headings) / len(headings)
            avg_h = math.degrees(math.atan2(hy, hx)) % 360.0

            calib = bus.read_byte_data(_IMU_ADDR, _CALIB_STAT)
            return {
                'heading': avg_h,
                'pitch': sum(pitches) / len(pitches),
                'calib_sys': (calib >> 6) & 0x03,
                'calib_gyro': (calib >> 4) & 0x03,
                'calib_accel': (calib >> 2) & 0x03,
                'calib_mag': calib & 0x03,
                'h_spread': h_spread,
                'p_spread': p_spread,
            }
        time.sleep(0.1)
    return None


def _read_imu_quick(bus):
    """Single fast IMU reading (quaternion-based)."""
    h, p = _quat_to_hp(_read_quat(bus))
    calib = bus.read_byte_data(_IMU_ADDR, _CALIB_STAT)
    return {
        'heading': h,
        'pitch': p,
        'calib_sys': (calib >> 6) & 0x03,
        'calib_gyro': (calib >> 4) & 0x03,
        'calib_accel': (calib >> 2) & 0x03,
        'calib_mag': calib & 0x03,
    }

"""Motion strategy abstraction: defines how pointing errors map to motor commands."""

import time
import math
import json
import os
from abc import ABC, abstractmethod

from .config import (
    SHOULDER_ROT, SHOULDER_PITCH, WRIST_PITCH, WRIST_ROLL,
    JOINT_LIMITS, PITCH_MAX_STEP, POSES_DIR,
    TICKS_PER_DEG_ROTATION, TICKS_PER_DEG_PITCH, MOVE_SPEED,
    get_min_ticks,
)
from .servos import (
    read_servo, move_servo, move_base_az,
    wait_all_stopped, stop_wheels,
)
from .imu import read_imu


class MotionStrategy(ABC):
    """Base class for motion strategies.

    Subclass this to define how azimuth and altitude errors
    map to motor movements. The tracker calls apply_correction()
    each iteration without knowing which motors are involved.
    """

    @abstractmethod
    def apply_correction(self, az_error, alt_error, gain, speed,
                         ph, pkt, imu_bus):
        """Apply a single correction step given current errors in degrees."""
        ...

    @abstractmethod
    def get_status(self, ph, pkt):
        """Return dict of current servo positions relevant to this strategy."""
        ...


class WristOnlyStrategy(MotionStrategy):
    """Shoulder rotation (1) for azimuth, wrist pitch (4) for altitude.

    - Azimuth: shoulder rotation is the primary actuator.
      Falls back to base wheels when shoulder is near its limits.
    - Altitude: wrist pitch only. Motors 2,3 are locked.
    """

    def __init__(self, shoulder_margin=100, wrist_pitch_dir=None):
        """
        Args:
            shoulder_margin: ticks of margin from joint limits before
                             falling back to wheels.
            wrist_pitch_dir: +1 if +ticks = pitch up, -1 if +ticks = pitch down.
                             If None, must call calibrate_pitch_dir() before use.
        """
        self.shoulder_margin = shoulder_margin
        self.wrist_pitch_dir = wrist_pitch_dir
        self.ticks_per_deg_pitch = TICKS_PER_DEG_PITCH  # default, overridden by calibration
        self.pitch_slope = None  # deg/tick from calibration
        self.pitch_intercept = None

    def calibrate_pitch_dir(self, ph, pkt, imu_bus):
        """Load pitch calibration from pitch_cal.json (from --calibrate).

        Falls back to auto-detect if no calibration file exists.
        """
        cal_path = os.path.join(POSES_DIR, 'pitch_cal.json')
        if os.path.exists(cal_path):
            with open(cal_path) as f:
                cal = json.load(f)
            self.wrist_pitch_dir = cal['pitch_dir']
            self.ticks_per_deg_pitch = cal['ticks_per_deg']
            self.pitch_slope = cal['slope_deg_per_tick']
            self.pitch_intercept = cal['intercept']
            print(f"  [cal] Loaded pitch_cal.json: dir={self.wrist_pitch_dir:+d}, "
                  f"{self.ticks_per_deg_pitch:.1f} ticks/deg")
            return

        # Fallback: auto-detect with nudge
        print(f"  [cal] No pitch_cal.json found, run --calibrate for better results")
        self._auto_calibrate_pitch_dir(ph, pkt, imu_bus)

    def _auto_calibrate_pitch_dir(self, ph, pkt, imu_bus):
        """Auto-detect pitch direction by nudging wrist pitch."""
        NUDGE = 200
        current_wp = read_servo(ph, pkt, WRIST_PITCH)
        if current_wp is None:
            print("  [cal] Can't read wrist pitch, defaulting pitch_dir=+1")
            self.wrist_pitch_dir = 1
            return

        lo, hi = JOINT_LIMITS[WRIST_PITCH]
        mid = (lo + hi) // 2

        if current_wp < mid:
            nudge_dir = +1
        else:
            nudge_dir = -1
        test_pos = max(lo, min(hi, current_wp + NUDGE * nudge_dir))
        actual_nudge = test_pos - current_wp

        if abs(actual_nudge) < 30:
            print(f"  [cal] Wrist pitch at limit ({current_wp}), not enough room to calibrate")
            self.wrist_pitch_dir = 1
            return

        time.sleep(0.5)
        before = read_imu(imu_bus, samples=8, interval=0.03)
        print(f"  [cal] Baseline pitch: {before['pitch']:.1f}°, WPt={current_wp}")

        move_servo(ph, pkt, WRIST_PITCH, test_pos, MOVE_SPEED)
        time.sleep(0.8)
        wait_all_stopped(ph, pkt, timeout=2.0)
        time.sleep(0.3)

        arrived = read_servo(ph, pkt, WRIST_PITCH)
        after = read_imu(imu_bus, samples=8, interval=0.03)
        pitch_change = after['pitch'] - before['pitch']
        ticks_moved = (arrived or test_pos) - current_wp

        print(f"  [cal] After: pitch={after['pitch']:.1f}°, WPt={arrived}, "
              f"moved {ticks_moved:+d} ticks, pitch changed {pitch_change:+.1f}°")

        move_servo(ph, pkt, WRIST_PITCH, current_wp, MOVE_SPEED)
        time.sleep(0.8)
        wait_all_stopped(ph, pkt, timeout=2.0)

        if abs(pitch_change) < 0.5:
            print(f"  [cal] WARNING: pitch barely changed ({pitch_change:+.1f}°) "
                  f"after {ticks_moved:+d} ticks, defaulting dir=+1")
            self.wrist_pitch_dir = 1
        else:
            self.wrist_pitch_dir = 1 if (pitch_change * ticks_moved) > 0 else -1
            print(f"  [cal] wrist_pitch_dir={self.wrist_pitch_dir:+d} "
                  f"({ticks_moved:+d} ticks -> {pitch_change:+.1f}° pitch)")

    def apply_correction(self, az_error, alt_error, gain, speed,
                         ph, pkt, imu_bus):
        # Use full errors — ticks_per_deg provides natural proportionality
        desired_pitch = alt_error

        # ── Azimuth: shoulder rotation, fallback to wheels ──
        if abs(az_error) > 0.2:
            self._correct_azimuth(az_error, speed, ph, pkt, imu_bus)

        # ── Altitude: wrist pitch only ──
        if abs(desired_pitch) > 0.1:
            self._correct_altitude(desired_pitch, speed, ph, pkt)
        else:
            print(f"  [pitch OK]")

    def _correct_azimuth(self, az_deg, speed, ph, pkt, imu_bus):
        """Move shoulder rotation for azimuth. Fall back to wheels if needed."""
        current_rot = read_servo(ph, pkt, SHOULDER_ROT)
        if current_rot is None:
            print(f"    [az] Can't read shoulder rotation, using wheels")
            actual = move_base_az(ph, pkt, imu_bus, az_deg,
                                   speed=max(300, min(1000, int(abs(az_deg) * 50 + 200))))
            print(f"    Wheels: {az_deg:+.1f}° (actual {actual:+.1f}°)")
            return

        lo, hi = JOINT_LIMITS[SHOULDER_ROT]
        needed_ticks = int(az_deg * TICKS_PER_DEG_ROTATION)
        new_pos = current_rot + needed_ticks
        min_t = get_min_ticks(SHOULDER_ROT, speed)

        # If move is too small for servo to register, use wheels instead
        if abs(needed_ticks) < min_t and abs(az_deg) > 0.3:
            print(f"    [az] Move too small for shoulder ({needed_ticks} ticks < {min_t}), using wheels")
            actual = move_base_az(ph, pkt, imu_bus, az_deg,
                                   speed=max(300, min(1000, int(abs(az_deg) * 50 + 200))))
            print(f"    Wheels: {az_deg:+.1f}° (actual {actual:+.1f}°)")
            return

        # Check if shoulder can absorb this move with margin
        if (lo + self.shoulder_margin) <= new_pos <= (hi - self.shoulder_margin):
            move_servo(ph, pkt, SHOULDER_ROT, new_pos, speed)
            time.sleep(0.3)
            wait_all_stopped(ph, pkt, timeout=1.5)
            after = read_servo(ph, pkt, SHOULDER_ROT)
            print(f"    ShRot: {current_rot}->{after} ({az_deg:+.1f}°)")
        else:
            # Shoulder near limit — use wheels for the full move,
            # then re-center shoulder rotation
            print(f"    [az] Shoulder near limit ({current_rot}), using wheels")
            actual = move_base_az(ph, pkt, imu_bus, az_deg,
                                   speed=max(300, min(1000, int(abs(az_deg) * 50 + 200))))
            print(f"    Wheels: {az_deg:+.1f}° (actual {actual:+.1f}°)")

            # Re-center shoulder rotation to middle of range
            mid = (lo + hi) // 2
            if abs(current_rot - mid) > 200:
                move_servo(ph, pkt, SHOULDER_ROT, mid, speed)
                time.sleep(0.3)
                print(f"    ShRot re-centered to {mid}")

    def _correct_altitude(self, desired_pitch, speed, ph, pkt):
        """Move shoulder pitch (2) + wrist pitch (4) together for altitude.

        Both motors tilt the same axis in the same tick direction.
        Motor 2 has more leverage so it does the heavy lifting.
        """
        if self.wrist_pitch_dir is None:
            print(f"    [alt] pitch_dir not calibrated, skipping")
            return

        current_wp = read_servo(ph, pkt, WRIST_PITCH)
        current_sp = read_servo(ph, pkt, SHOULDER_PITCH)
        if current_wp is None or current_sp is None:
            print(f"    [alt] Can't read servos (sp={current_sp}, wp={current_wp})")
            return

        wp_lo, wp_hi = JOINT_LIMITS[WRIST_PITCH]
        sp_lo, sp_hi = JOINT_LIMITS[SHOULDER_PITCH]

        # Both use same direction — calibrated from wrist pitch
        pitch_dir = self.wrist_pitch_dir

        # Shoulder pitch (motor 2): primary, ~11.3 ticks/deg
        sp_delta = int(desired_pitch * TICKS_PER_DEG_PITCH) * pitch_dir
        sp_delta = max(-PITCH_MAX_STEP, min(PITCH_MAX_STEP, sp_delta))

        # Wrist pitch (motor 4): calibrated ticks/deg
        wp_delta = int(desired_pitch * self.ticks_per_deg_pitch) * pitch_dir
        wp_delta = max(-PITCH_MAX_STEP, min(PITCH_MAX_STEP, wp_delta))

        sp_min = get_min_ticks(SHOULDER_PITCH, speed)
        wp_min = get_min_ticks(WRIST_PITCH, speed)

        # Enforce minimum effective step — servos need ~25+ ticks to
        # overcome static friction under load
        MIN_EFFECTIVE = 25
        if 0 < abs(sp_delta) < MIN_EFFECTIVE:
            sp_delta = MIN_EFFECTIVE * (1 if sp_delta > 0 else -1)
        if 0 < abs(wp_delta) < MIN_EFFECTIVE:
            wp_delta = MIN_EFFECTIVE * (1 if wp_delta > 0 else -1)

        sp_target = max(sp_lo, min(sp_hi, current_sp + sp_delta))
        wp_target = max(wp_lo, min(wp_hi, current_wp + wp_delta))

        print(f"  ShPit: {current_sp}->{sp_target} ({sp_target-current_sp:+d})  "
              f"WPt: {current_wp}->{wp_target} ({wp_target-current_wp:+d})"
              f"  [pitch={desired_pitch:+.1f}°]")

        if sp_delta != 0:
            move_servo(ph, pkt, SHOULDER_PITCH, sp_target, speed)
        if wp_delta != 0:
            move_servo(ph, pkt, WRIST_PITCH, wp_target, speed)
        time.sleep(0.3)
        wait_all_stopped(ph, pkt, timeout=1.5)
        after_sp = read_servo(ph, pkt, SHOULDER_PITCH)
        after_wp = read_servo(ph, pkt, WRIST_PITCH)
        print(f"    Actual: ShPit={after_sp} WPt={after_wp}")

    def get_status(self, ph, pkt):
        """Return positions of motors this strategy controls."""
        return {
            'shoulder_rot': read_servo(ph, pkt, SHOULDER_ROT),
            'shoulder_pitch': read_servo(ph, pkt, SHOULDER_PITCH),
            'wrist_pitch': read_servo(ph, pkt, WRIST_PITCH),
            'wrist_roll': read_servo(ph, pkt, WRIST_ROLL),
        }

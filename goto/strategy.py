"""Motion strategy abstraction: defines how pointing errors map to motor commands."""

import time
import math
import json
import os
from abc import ABC, abstractmethod

from .config import (
    SHOULDER_ROT, SHOULDER_PITCH, ELBOW, WRIST_PITCH, WRIST_ROLL,
    JOINT_LIMITS, PITCH_MAX_STEP, POSES_DIR, PITCH_DIR,
    TICKS_PER_DEG_ROTATION, TICKS_PER_DEG_PITCH, MOVE_SPEED,
    PITCH_TICKS_PER_DEG, AZ_DEADBAND_DEG, AZ_WHEEL_MIN_DEG, ALT_DEADBAND_DEG,
    get_min_ticks,
)
from .servos import (
    read_servo, move_servo, move_base_az,
    wait_all_stopped, stop_wheels, clear_overload,
)
from .imu import read_imu


# Pitch joints in fixed iteration order
_PITCH_JOINTS = (SHOULDER_PITCH, ELBOW, WRIST_PITCH)
_PITCH_LABELS = {SHOULDER_PITCH: 'ShPit', ELBOW: 'Elb', WRIST_PITCH: 'WPt'}

# Cascade-stage ordering for "scorpion tail" pitch allocation. Each
# stage tries to absorb the residual demand; remaining demand spills to
# the next stage only when the current joint hits its hardware limit.
# Putting wrist first keeps the upper arm folded and the elbow tucked,
# rather than extending the arm horizontally to reach a low target.
_PITCH_CASCADE = (WRIST_PITCH, SHOULDER_PITCH, ELBOW)


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

    def reset_slew(self):
        """Optional hook called by the slew loop before a new slew starts.

        Use this to clear per-slew state (stall counts, saturation flags).
        Default: no-op.
        """
        return


class WristOnlyStrategy(MotionStrategy):
    """Coordinated 3-joint pitch + shoulder-rot/wheels azimuth.

    Pitch is distributed across shoulder_pitch (2), elbow (3),
    wrist_pitch (4) using per-joint ticks-per-degree leverages. Saturated
    joints — those that hit a hardware limit or stall — are excluded and
    their share is redistributed to the remaining joints.

    Azimuth is driven by shoulder rotation (motor 1). The base wheels
    are used only when the shoulder runs out of range and the requested
    move is large enough (≥AZ_WHEEL_MIN_DEG) to be worth the chassis
    wobble they introduce.
    """

    # Stall classification: actual movement below this fraction of the
    # commanded delta counts as a stall.
    STALL_RATIO = 0.25
    # Consecutive stalls before excluding a joint from further commands.
    STALL_GIVE_UP = 2

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
        self.ticks_per_deg = dict(PITCH_TICKS_PER_DEG)
        self.pitch_slope = None
        self.pitch_intercept = None
        # Per-slew transient state (reset by reset_slew()).
        self._stall_count = {sid: 0 for sid in _PITCH_JOINTS}
        self._saturated = set()       # joints to skip until reset
        self._last_cmd = {}           # sid → (start_pos, commanded_delta)

    def reset_slew(self):
        """Clear stall/saturation tracking — call at the start of a slew."""
        for sid in self._stall_count:
            self._stall_count[sid] = 0
        self._saturated.clear()
        self._last_cmd.clear()

    # ── Calibration ──

    def calibrate_pitch_dir(self, ph, pkt, imu_bus):
        """Load pitch calibration from pitch_cal.json (from --calibrate).

        Falls back to auto-detect if no calibration file exists.
        """
        cal_path = os.path.join(POSES_DIR, 'pitch_cal.json')
        if os.path.exists(cal_path):
            with open(cal_path) as f:
                cal = json.load(f)
            self.wrist_pitch_dir = cal['pitch_dir']
            self.ticks_per_deg[WRIST_PITCH] = cal['ticks_per_deg']
            self.pitch_slope = cal['slope_deg_per_tick']
            self.pitch_intercept = cal['intercept']
            print(f"  [cal] Loaded pitch_cal.json: dir={self.wrist_pitch_dir:+d}, "
                  f"WPt={self.ticks_per_deg[WRIST_PITCH]:.1f} ticks/deg")
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
        nudge_dir = +1 if current_wp < mid else -1
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

    # ── Public correction step ──

    def apply_correction(self, az_error, alt_error, gain, speed,
                         ph, pkt, imu_bus):
        """Issue one coordinated correction step.

        Az and pitch commands are dispatched together; we wait once for
        all motors to stop and then verify movement to detect saturation.
        Sub-deadband errors are skipped entirely — chasing them just
        causes wobble.
        """
        self._last_cmd.clear()

        az_active = abs(az_error) >= AZ_DEADBAND_DEG
        alt_active = abs(alt_error) >= ALT_DEADBAND_DEG

        if not az_active and not alt_active:
            print(f"  [within deadband] az_err={az_error:+.2f}° "
                  f"alt_err={alt_error:+.2f}°")
            return

        moved_any = False
        if az_active:
            moved_any |= self._command_azimuth(az_error * gain, speed,
                                                ph, pkt, imu_bus)
        if alt_active:
            moved_any |= self._command_altitude(alt_error * gain, speed,
                                                 ph, pkt)

        if not moved_any:
            return

        # Single coordinated settle. Previously we waited twice (once
        # after az, once after pitch); doing it once cuts ~1.5s per iter.
        time.sleep(0.25)
        wait_all_stopped(ph, pkt, timeout=2.0)
        self._verify_movement(ph, pkt)

    def get_status(self, ph, pkt):
        return {
            'shoulder_rot': read_servo(ph, pkt, SHOULDER_ROT),
            'shoulder_pitch': read_servo(ph, pkt, SHOULDER_PITCH),
            'wrist_pitch': read_servo(ph, pkt, WRIST_PITCH),
            'wrist_roll': read_servo(ph, pkt, WRIST_ROLL),
        }

    # ── Azimuth ──

    def _command_azimuth(self, az_deg, speed, ph, pkt, imu_bus):
        """Issue an azimuth command. Returns True if anything was driven.

        Shoulder rotation is the primary actuator. Wheels are used only
        when shoulder is at a limit AND the requested move is large
        enough to be worth the chassis wobble.
        """
        current_rot = read_servo(ph, pkt, SHOULDER_ROT)
        if current_rot is None:
            if abs(az_deg) < AZ_WHEEL_MIN_DEG:
                print(f"    [az] Can't read shoulder, |{az_deg:+.2f}°| "
                      f"< {AZ_WHEEL_MIN_DEG}° (wheel min); skipping")
                return False
            print(f"    [az] Can't read shoulder rotation, using wheels")
            actual = move_base_az(ph, pkt, imu_bus, az_deg,
                                   speed=_wheel_speed(az_deg))
            print(f"    Wheels: {az_deg:+.1f}° (actual {actual:+.1f}°)")
            return True

        lo, hi = JOINT_LIMITS[SHOULDER_ROT]
        needed_ticks = int(az_deg * TICKS_PER_DEG_ROTATION)
        min_t = get_min_ticks(SHOULDER_ROT, speed)

        # Round sub-min-tick demands up to min-tick. Otherwise low-gain
        # track-mode corrections deadlock: the gained command is too
        # small for the servo and also too small for the wheel fallback,
        # so nothing happens iteration after iteration. If we've stalled
        # before, double the floor each time so the command grows until
        # the servo overcomes stiction or we mark it saturated.
        stall_boost = 1 << min(self._stall_count.get(SHOULDER_ROT, 0), 4)
        floor = min(min_t * stall_boost, 60)
        if 0 < abs(needed_ticks) < floor:
            needed_ticks = floor * (1 if needed_ticks >= 0 else -1)

        new_pos = current_rot + needed_ticks
        in_range = (lo + self.shoulder_margin) <= new_pos <= (hi - self.shoulder_margin)

        # Shoulder rotation is fine — issue the move and let the shared
        # wait/verify pass handle settling.
        if in_range and abs(needed_ticks) >= min_t:
            move_servo(ph, pkt, SHOULDER_ROT, new_pos, speed)
            self._last_cmd[SHOULDER_ROT] = (current_rot, needed_ticks)
            print(f"    ShRot: {current_rot}->{new_pos} ({az_deg:+.1f}°)")
            return True

        # Shoulder can't take the move. Wheels only for sizeable errors —
        # spinning the chassis for sub-degree moves shakes the IMU and
        # creates the oscillation we used to see.
        if abs(az_deg) < AZ_WHEEL_MIN_DEG:
            reason = "near limit" if not in_range else "below min-tick"
            print(f"    [az] Shoulder {reason} ({current_rot}); "
                  f"|{az_deg:+.2f}°| < {AZ_WHEEL_MIN_DEG}° wheel min; skipping")
            return False

        why = "near limit" if not in_range else "below min-tick"
        print(f"    [az] Shoulder {why} ({current_rot}), using wheels")
        actual = move_base_az(ph, pkt, imu_bus, az_deg, speed=_wheel_speed(az_deg))
        print(f"    Wheels: {az_deg:+.1f}° (actual {actual:+.1f}°)")

        # Re-center shoulder to leave headroom for the next correction.
        mid = (lo + hi) // 2
        if abs(current_rot - mid) > 200:
            move_servo(ph, pkt, SHOULDER_ROT, mid, speed)
            self._last_cmd[SHOULDER_ROT] = (current_rot, mid - current_rot)
            print(f"    ShRot re-centered to {mid}")
        return True

    # ── Altitude ──

    def _command_altitude(self, desired_pitch, speed, ph, pkt):
        """Wrist-first pitch allocation ("scorpion tail" posture).

        The cascade is wrist → shoulder → elbow. Each stage absorbs as
        much of the residual demand as it can without spilling, then
        passes the leftover only when its joint hits a hardware limit.
        Demand the per-iteration cap can't move *yet* is deferred to the
        next iteration on the same joint — not spilled — so the arm
        doesn't unfold prematurely.

        Net effect: routine altitude moves stay on the wrist while
        shoulder/elbow remain near their folded/upright positions.
        Shoulder pitches in only when wrist saturates; elbow extends
        only when both wrist and shoulder are saturated.
        """
        if self.wrist_pitch_dir is None:
            print(f"    [alt] pitch_dir not calibrated, skipping")
            return False

        positions = {sid: read_servo(ph, pkt, sid) for sid in _PITCH_JOINTS}
        if any(v is None for v in positions.values()):
            print(f"    [alt] Can't read servos: {positions}")
            return False

        pitch_dir = self.wrist_pitch_dir
        # Elbow folds opposite to shoulder/wrist in this kinematic chain.
        motor_dir = {
            SHOULDER_PITCH: pitch_dir,
            ELBOW: -pitch_dir,
            WRIST_PITCH: pitch_dir,
        }

        commanded = {sid: (positions[sid], 0) for sid in _PITCH_JOINTS}
        cascade_path = []
        remaining_deg = desired_pitch

        def try_absorb(sid, demand_deg):
            """Try to absorb demand_deg on joint `sid`.

            Returns leftover demand in degrees. Zero means the joint
            took the demand (or the per-iter cap deferred it to next
            iter — same outcome from the cascade's perspective).
            Non-zero means the joint hit a hardware limit and we should
            spill into the next stage.
            """
            if sid in self._saturated:
                return demand_deg
            tpd = self.ticks_per_deg.get(sid, TICKS_PER_DEG_PITCH)
            lo, hi = JOINT_LIMITS[sid]

            demand_ticks = int(demand_deg * tpd) * motor_dir[sid]
            # Per-iteration ramp limit; bigger demand is split over iters.
            step_ticks = max(-PITCH_MAX_STEP, min(PITCH_MAX_STEP, demand_ticks))

            # Round sub-min-tick demands up to min-tick before clamping.
            # Without this, low-gain corrections crush below servo
            # resolution and the loop deadlocks. Each consecutive stall
            # doubles the floor so the command grows until the servo
            # breaks stiction or we mark it saturated.
            min_t = get_min_ticks(sid, speed)
            stall_boost = 1 << min(self._stall_count.get(sid, 0), 4)
            floor = min(min_t * stall_boost, PITCH_MAX_STEP)
            if 0 < abs(step_ticks) < floor:
                step_ticks = floor * (1 if step_ticks >= 0 else -1)

            target = max(lo, min(hi, positions[sid] + step_ticks))
            actual_ticks = target - positions[sid]

            # If joint-limit clamping drove the actual move below
            # min-tick, the joint is effectively saturated — zero it
            # out so the cascade spills to the next stage.
            if 0 < abs(actual_ticks) < min_t:
                actual_ticks = 0
                target = positions[sid]

            commanded[sid] = (target, actual_ticks)
            if actual_ticks != 0:
                cascade_path.append(_PITCH_LABELS[sid])

            # If joint limits clamped the step, spill the unfittable
            # remainder. If the per-iter cap is the only thing holding us
            # back, future iterations will keep absorbing on this joint —
            # don't unfold the next stage.
            clamped_by_limit = abs(actual_ticks) < abs(step_ticks)
            if clamped_by_limit and tpd > 0:
                consumed_deg = (actual_ticks / tpd) * motor_dir[sid]
                return demand_deg - consumed_deg
            return 0.0

        for sid in _PITCH_CASCADE:
            if abs(remaining_deg) < 0.1:
                break
            remaining_deg = try_absorb(sid, remaining_deg)

        # One-line summary
        parts = []
        for sid in _PITCH_JOINTS:
            tgt, d = commanded[sid]
            if d != 0:
                parts.append(f"{_PITCH_LABELS[sid]}: {positions[sid]}->{tgt} ({d:+d})")
        sat = sorted(self._saturated)
        sat_str = f"  [saturated: {','.join(str(s) for s in sat)}]" if sat else ""
        path_str = f" [{'→'.join(cascade_path)}]" if cascade_path else ""
        if parts:
            print(f"  {'  '.join(parts)}  [pitch={desired_pitch:+.1f}°]"
                  f"{path_str}{sat_str}")
        else:
            print(f"  [alt] All deltas below min-tick "
                  f"(pitch={desired_pitch:+.2f}°){sat_str}")
            return False

        # Send commands. A joint that stalled previously gets a speed bump
        # — friction and overload both yield to a faster command.
        any_moved = False
        for sid in _PITCH_JOINTS:
            tgt, d = commanded[sid]
            if d == 0:
                continue
            s = min(speed * 2, 500) if self._stall_count[sid] >= 1 else speed
            move_servo(ph, pkt, sid, tgt, s)
            self._last_cmd[sid] = (positions[sid], d)
            any_moved = True

        return any_moved

    # ── Post-move verification (saturation/stall detection) ──

    def _verify_movement(self, ph, pkt):
        """Read back and compare actual vs commanded; flag saturation."""
        if not self._last_cmd:
            return

        actuals = {}
        for sid in self._last_cmd:
            actuals[sid] = read_servo(ph, pkt, sid)

        # Actuals print for the pitch joints
        actual_parts = []
        for sid in _PITCH_JOINTS:
            if sid in actuals and actuals[sid] is not None:
                actual_parts.append(f"{_PITCH_LABELS[sid]}={actuals[sid]}")
        if actual_parts:
            print(f"    Actual: {'  '.join(actual_parts)}")

        # Stall classification per touched joint
        for sid, (start_pos, cmd_delta) in self._last_cmd.items():
            if cmd_delta == 0 or actuals.get(sid) is None:
                continue
            actual = actuals[sid] - start_pos
            ratio = abs(actual) / max(1, abs(cmd_delta))
            if ratio < self.STALL_RATIO and abs(cmd_delta) >= 5:
                self._stall_count[sid] = self._stall_count.get(sid, 0) + 1
                # First stall: try clearing overload — it might just be
                # the servo's protection latch.
                try:
                    clear_overload(ph, pkt, sid)
                except Exception:
                    pass
                if self._stall_count[sid] >= self.STALL_GIVE_UP:
                    if sid not in self._saturated:
                        self._saturated.add(sid)
                        print(f"    [stall] Motor {sid} saturated "
                              f"(cmd {cmd_delta:+d}, moved {actual:+d}); "
                              f"excluded for this slew")
                else:
                    print(f"    [stall] Motor {sid} stuck "
                          f"(cmd {cmd_delta:+d}, moved {actual:+d}, "
                          f"attempt {self._stall_count[sid]})")
            else:
                # Joint moved enough — clear any prior stall count
                if self._stall_count.get(sid, 0):
                    self._stall_count[sid] = 0


# ── helpers ──

def _wheel_speed(az_deg):
    return max(300, min(1000, int(abs(az_deg) * 50 + 200)))

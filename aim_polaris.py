#!/usr/bin/env python3
"""
Aim at Polaris using BNO055 IMU feedback and SO-100 servos.

Uses the IMU heading (magnetometer-based) as azimuth reference
and IMU pitch as altitude reference. Iteratively moves servos
until the IMU reads the target alt/az.

No ROS2 required — direct I2C + serial.
"""

import smbus2 as smbus
import scservo_sdk as sdk
import struct
import time
import sys
import math

from astropy.coordinates import EarthLocation, AltAz, SkyCoord
from astropy.time import Time
import astropy.units as u

# ── Config ──
I2C_BUS = 1
IMU_ADDR = 0x28
EULER_H_LSB = 0x1A
CALIB_STAT = 0x35
OPR_MODE_REG = 0x3D
MODE_NDOF = 0x0C
MODE_CONFIG = 0x00

SERVO_PORT = '/dev/ttyACM0'
SERVO_BAUD = 1000000
ADDR_GOAL_POSITION = 42
ADDR_MOVING_SPEED = 46
ADDR_PRESENT_POSITION = 56

# Safe servo range
# Per-joint mechanical limits (discovered by probing)
JOINT_LIMITS = {
    1: (742, 3494),   # Shoulder Rotation: 242°
    2: (900, 2305),   # Shoulder Pitch: 123°
    3: (896, 1916),   # Elbow: 90°
    4: (924, 2859),   # Wrist Pitch: 170°
    5: (7, 2004),     # Wrist Roll: 176°
}

# Servo IDs
SHOULDER_ROT = 1
SHOULDER_PITCH = 2
ELBOW = 3
WRIST_PITCH = 4
WRIST_ROLL = 5

# Control parameters
MOVE_SPEED = 150          # Slow servo speed
TOLERANCE_DEG = 2.0       # Acceptable pointing error
MAX_ITERATIONS = 50       # Max correction iterations
SETTLE_TIME = 1.5         # Seconds to wait after each move

# How many servo ticks per degree of IMU change (from our test: 150 ticks ≈ 13°)
TICKS_PER_DEG_ROTATION = 150 / 12.9   # ~11.6 ticks/degree for shoulder rotation -> heading
TICKS_PER_DEG_PITCH = 150 / 13.3      # ~11.3 ticks/degree for shoulder pitch -> pitch

# Observer location (Toronto area — update for your location)
OBSERVER_LAT = 43.65
OBSERVER_LON = -79.38
OBSERVER_HEIGHT = 76  # meters


def init_imu():
    bus = smbus.SMBus(I2C_BUS)
    chip_id = bus.read_byte_data(IMU_ADDR, 0x00)
    assert chip_id == 0xA0, f"Bad chip ID: 0x{chip_id:02x}"
    bus.write_byte_data(IMU_ADDR, OPR_MODE_REG, MODE_CONFIG)
    time.sleep(0.025)
    bus.write_byte_data(IMU_ADDR, OPR_MODE_REG, MODE_NDOF)
    time.sleep(0.5)
    print("BNO055 initialized")
    return bus


def read_imu(bus, samples=10, interval=0.05):
    headings, rolls, pitches = [], [], []
    for _ in range(samples):
        data = bus.read_i2c_block_data(IMU_ADDR, EULER_H_LSB, 6)
        h = struct.unpack('<h', bytes(data[0:2]))[0] / 16.0
        r = struct.unpack('<h', bytes(data[2:4]))[0] / 16.0
        p = struct.unpack('<h', bytes(data[4:6]))[0] / 16.0
        headings.append(h)
        rolls.append(r)
        pitches.append(p)
        time.sleep(interval)

    calib = bus.read_byte_data(IMU_ADDR, CALIB_STAT)

    return {
        'heading': sum(headings) / len(headings),
        'roll': sum(rolls) / len(rolls),
        'pitch': sum(pitches) / len(pitches),
        'calib_sys': (calib >> 6) & 0x03,
        'calib_gyro': (calib >> 4) & 0x03,
        'calib_accel': (calib >> 2) & 0x03,
        'calib_mag': calib & 0x03,
    }


def init_servos():
    port_handler = sdk.PortHandler(SERVO_PORT)
    packet_handler = sdk.PacketHandler(0)
    assert port_handler.openPort(), "Failed to open servo port"
    assert port_handler.setBaudRate(SERVO_BAUD), "Failed to set baudrate"
    print(f"Servos connected on {SERVO_PORT}")
    return port_handler, packet_handler


def read_servo(ph, pkt, servo_id):
    pos, result, _ = pkt.read2ByteTxRx(ph, servo_id, ADDR_PRESENT_POSITION)
    return pos if result == sdk.COMM_SUCCESS else None


def move_servo(ph, pkt, servo_id, target, speed=MOVE_SPEED):
    lo, hi = JOINT_LIMITS.get(servo_id, (200, 3800))
    target = max(lo, min(hi, int(target)))
    pkt.write2ByteTxRx(ph, servo_id, ADDR_MOVING_SPEED, speed)
    pkt.write2ByteTxRx(ph, servo_id, ADDR_GOAL_POSITION, target)
    return target


def wait_servo(ph, pkt, servo_id, target, timeout=5.0):
    start = time.time()
    while time.time() - start < timeout:
        pos = read_servo(ph, pkt, servo_id)
        if pos is not None and abs(pos - target) < 20:
            return True
        time.sleep(0.1)
    return False


def compute_polaris_altaz():
    """Compute current Alt/Az of Polaris for observer location."""
    obs = EarthLocation(
        lat=OBSERVER_LAT * u.deg,
        lon=OBSERVER_LON * u.deg,
        height=OBSERVER_HEIGHT * u.m,
    )
    now = Time.now()
    altaz_frame = AltAz(obstime=now, location=obs)
    polaris = SkyCoord(ra='2h31m49.09s', dec='+89d15m50.8s', frame='icrs')
    polaris_altaz = polaris.transform_to(altaz_frame)
    return polaris_altaz.alt.deg, polaris_altaz.az.deg


def angle_diff(target, current):
    """Signed shortest angular difference (handles wraparound)."""
    diff = target - current
    while diff > 180:
        diff -= 360
    while diff < -180:
        diff += 360
    return diff


def main():
    print("=" * 60)
    print("  AIM AT POLARIS")
    print("=" * 60)

    # Compute target
    target_alt, target_az = compute_polaris_altaz()
    print(f"\nPolaris position (computed):")
    print(f"  Altitude: {target_alt:.2f}°")
    print(f"  Azimuth:  {target_az:.2f}° (0°=North)")
    print(f"  Observer: {OBSERVER_LAT:.2f}°N, {OBSERVER_LON:.2f}°E")

    # Initialize hardware
    imu_bus = init_imu()
    port_handler, packet_handler = init_servos()

    # Read initial state
    imu = read_imu(imu_bus)
    print(f"\nInitial IMU reading:")
    print(f"  Heading: {imu['heading']:.1f}°  Roll: {imu['roll']:.1f}°  Pitch: {imu['pitch']:.1f}°")
    print(f"  Calib: S{imu['calib_sys']} G{imu['calib_gyro']} A{imu['calib_accel']} M{imu['calib_mag']}")

    # Current servo positions
    servo_pos = {}
    for sid in [SHOULDER_ROT, SHOULDER_PITCH, ELBOW, WRIST_PITCH, WRIST_ROLL]:
        servo_pos[sid] = read_servo(port_handler, packet_handler, sid)
        print(f"  Servo {sid}: {servo_pos[sid]}")

    # The IMU heading maps to azimuth, IMU pitch maps to altitude.
    # We need to figure out the current pointing direction from IMU,
    # then calculate how much to move.
    #
    # From our tests:
    #   - Shoulder Rotation (servo 1): +150 ticks -> +12.9° heading
    #   - Shoulder Pitch (servo 2): +150 ticks -> -13.3° pitch
    #     (positive ticks = negative pitch, so pitch gain is negative)
    #   - Wrist Pitch (servo 4): +150 ticks -> -13.2° pitch
    #     (same sign as shoulder pitch)

    print(f"\n--- Starting iterative aim ---")
    print(f"Target: Az={target_az:.1f}°, Alt={target_alt:.1f}°")
    print(f"Tolerance: {TOLERANCE_DEG}°")

    for iteration in range(MAX_ITERATIONS):
        # Read current IMU
        imu = read_imu(imu_bus)
        current_heading = imu['heading']
        current_pitch = imu['pitch']

        # Compute errors
        # Heading -> Azimuth: IMU heading is compass bearing (0=North, 90=East)
        az_error = angle_diff(target_az, current_heading)
        # Pitch -> Altitude: IMU pitch represents elevation
        alt_error = target_alt - current_pitch

        total_error = math.sqrt(az_error**2 + alt_error**2)

        print(f"\n  Iteration {iteration + 1}:")
        print(f"    IMU:   Heading={current_heading:.1f}°  Pitch={current_pitch:.1f}°")
        print(f"    Error: Az={az_error:+.1f}°  Alt={alt_error:+.1f}°  Total={total_error:.1f}°")

        if total_error < TOLERANCE_DEG:
            print(f"\n  CONVERGED! Within {TOLERANCE_DEG}° tolerance.")
            break

        # Calculate servo corrections
        gain = 0.6

        # Azimuth correction via Shoulder Rotation
        rot_correction = az_error * TICKS_PER_DEG_ROTATION * gain

        # Altitude correction: blend across shoulder pitch, elbow, and wrist pitch
        # weighted by available range in the needed direction
        total_pitch_ticks = -alt_error * TICKS_PER_DEG_PITCH * gain

        current_rot = read_servo(port_handler, packet_handler, SHOULDER_ROT)
        current_sp = read_servo(port_handler, packet_handler, SHOULDER_PITCH)
        current_elbow = read_servo(port_handler, packet_handler, ELBOW)
        current_wp = read_servo(port_handler, packet_handler, WRIST_PITCH)

        pitch_joints = [
            (SHOULDER_PITCH, current_sp),
            (ELBOW, current_elbow),
            (WRIST_PITCH, current_wp),
        ]

        direction = 1 if total_pitch_ticks > 0 else -1
        ranges = []
        for sid, pos in pitch_joints:
            lo, hi = JOINT_LIMITS[sid]
            r = (hi - pos) if direction > 0 else (pos - lo)
            ranges.append(max(0, r))

        total_range = sum(ranges)
        if total_range > 0:
            weights = [r / total_range for r in ranges]
            corrections = [total_pitch_ticks * w for w in weights]
        else:
            corrections = [0, 0, 0]

        targets = []
        for (sid, pos), corr in zip(pitch_joints, corrections):
            lo, hi = JOINT_LIMITS[sid]
            targets.append(max(lo, min(hi, pos + corr)))

        sp_clamped, elbow_clamped, wp_clamped = targets
        rot_target = current_rot + rot_correction

        print(f"    Corrections: Rot={rot_correction:+.0f}, SP={sp_clamped-current_sp:+.0f}, Elbow={elbow_clamped-current_elbow:+.0f}, WP={wp_clamped-current_wp:+.0f}")
        print(f"    Targets: Rot={rot_target:.0f}, SP={sp_clamped:.0f}, Elbow={elbow_clamped:.0f}, WP={wp_clamped:.0f}")

        # Move all servos
        actual_rot = move_servo(port_handler, packet_handler, SHOULDER_ROT, rot_target)
        move_servo(port_handler, packet_handler, SHOULDER_PITCH, sp_clamped)
        move_servo(port_handler, packet_handler, ELBOW, elbow_clamped)
        move_servo(port_handler, packet_handler, WRIST_PITCH, wp_clamped)

        # Wait for movement + settling
        wait_servo(port_handler, packet_handler, SHOULDER_ROT, actual_rot)
        wait_servo(port_handler, packet_handler, SHOULDER_PITCH, int(sp_clamped))
        wait_servo(port_handler, packet_handler, ELBOW, int(elbow_clamped))
        wait_servo(port_handler, packet_handler, WRIST_PITCH, int(wp_clamped))
        time.sleep(SETTLE_TIME)

        # Bail early if all joints are at limits and error remains
        at_limit = (
            abs(sp_clamped - sp_lo) < 5 or abs(sp_clamped - sp_hi) < 5
        ) and (
            abs(elbow_clamped - el_lo) < 5 or abs(elbow_clamped - el_hi) < 5
        ) and (
            abs(wp_clamped - wp_lo) < 5 or abs(wp_clamped - wp_hi) < 5
        )
        if at_limit and total_error > TOLERANCE_DEG:
            print(f"\n  All joints at mechanical limits. Cannot reach target altitude.")
            break

    else:
        print(f"\n  Did not converge within {MAX_ITERATIONS} iterations.")

    # Final reading
    imu = read_imu(imu_bus)
    final_az_error = angle_diff(target_az, imu['heading'])
    final_alt_error = target_alt - imu['pitch']
    final_total = math.sqrt(final_az_error**2 + final_alt_error**2)

    print(f"\n{'='*60}")
    print(f"FINAL STATE")
    print(f"{'='*60}")
    print(f"  Target:  Az={target_az:.1f}°  Alt={target_alt:.1f}°")
    print(f"  IMU:     Heading={imu['heading']:.1f}°  Pitch={imu['pitch']:.1f}°")
    print(f"  Error:   Az={final_az_error:+.1f}°  Alt={final_alt_error:+.1f}°  Total={final_total:.1f}°")
    print(f"  Calib:   S{imu['calib_sys']} G{imu['calib_gyro']} A{imu['calib_accel']} M{imu['calib_mag']}")

    for sid in [SHOULDER_ROT, SHOULDER_PITCH, ELBOW, WRIST_PITCH, WRIST_ROLL]:
        pos = read_servo(port_handler, packet_handler, sid)
        print(f"  Servo {sid}: {pos}")

    port_handler.closePort()
    imu_bus.close()
    print("\nDone.")


if __name__ == '__main__':
    main()

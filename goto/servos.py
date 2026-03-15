"""Servo and wheel driver: init, read, move, wait, base wheels."""

import scservo_sdk as sdk
import time

from .config import (
    SERVO_PORT, SERVO_BAUD,
    ADDR_GOAL_POSITION, ADDR_MOVING_SPEED,
    ADDR_PRESENT_POSITION, ADDR_PRESENT_SPEED,
    ADDR_TORQUE_ENABLE, ADDR_TORQUE_LIMIT,
    ADDR_MODE, ADDR_LOCK, ADDR_ACC,
    MODE_SERVO, MODE_WHEEL_CLOSED, WHEEL_DEFAULT_ACC,
    SHOULDER_ROT, SHOULDER_PITCH, ELBOW, WRIST_PITCH, WRIST_ROLL,
    WHEEL_IDS, JOINT_LIMITS, CAL_POSES, CAL_SPEED,
    MOVE_SPEED, angle_diff,
)
from .imu import read_imu


def init_servos(locked_pose=None):
    """Initialize servo port and all arm + wheel servos.

    Args:
        locked_pose: optional dict {servo_id: tick_position} for motors
                     that should be locked at specific positions (e.g. {2: 2146, 3: 887})
    """
    ph = sdk.PortHandler(SERVO_PORT)
    pkt = sdk.PacketHandler(0)
    assert ph.openPort(), "Failed to open servo port"
    assert ph.setBaudRate(SERVO_BAUD), "Failed to set baudrate"

    # Toggle torque off/on to clear overload protection
    for sid in range(1, 6):
        pkt.write1ByteTxRx(ph, sid, ADDR_TORQUE_ENABLE, 0)
    time.sleep(0.2)
    for sid in range(1, 6):
        pkt.write1ByteTxRx(ph, sid, ADDR_TORQUE_ENABLE, 1)
    time.sleep(0.05)

    # Set base wheels to closed-loop wheel mode
    for sid in WHEEL_IDS:
        pkt.write1ByteTxRx(ph, sid, ADDR_TORQUE_ENABLE, 0)
        time.sleep(0.05)
        pkt.write1ByteTxRx(ph, sid, ADDR_LOCK, 0)
        pkt.write1ByteTxRx(ph, sid, ADDR_MODE, MODE_WHEEL_CLOSED)
        pkt.write1ByteTxRx(ph, sid, ADDR_LOCK, 1)
        pkt.write1ByteTxRx(ph, sid, ADDR_TORQUE_ENABLE, 1)
        time.sleep(0.05)
    print("Base wheels set to closed-loop wheel mode")

    # Move to home position
    home = CAL_POSES['home']
    sids = [SHOULDER_ROT, SHOULDER_PITCH, ELBOW, WRIST_PITCH, WRIST_ROLL]
    for sid, target in zip(sids, home):
        lo, hi = JOINT_LIMITS.get(sid, (200, 3800))
        target = max(lo, min(hi, int(target)))
        pkt.write2ByteTxRx(ph, sid, ADDR_MOVING_SPEED, 200)
        pkt.write2ByteTxRx(ph, sid, ADDR_GOAL_POSITION, target)
    time.sleep(1.0)
    for sid, target in zip(sids, home):
        lo, hi = JOINT_LIMITS.get(sid, (200, 3800))
        target = max(lo, min(hi, int(target)))
        wait_servo(ph, pkt, sid, target, timeout=8)
    time.sleep(0.5)

    # Lock specified motors at their recorded positions
    if locked_pose:
        for sid, pos in locked_pose.items():
            move_servo(ph, pkt, sid, pos, speed=200)
        time.sleep(0.5)
        for sid, pos in locked_pose.items():
            wait_servo(ph, pkt, sid, pos, timeout=8)
        locked_str = ', '.join(f'{sid}={pos}' for sid, pos in locked_pose.items())
        print(f"Locked motors at: {locked_str}")

    return ph, pkt


def read_servo(ph, pkt, sid):
    """Read current position of a servo. Returns tick value or None."""
    pos, result, _ = pkt.read2ByteTxRx(ph, sid, ADDR_PRESENT_POSITION)
    return pos if result == sdk.COMM_SUCCESS else None


def move_servo(ph, pkt, sid, target, speed=MOVE_SPEED):
    """Move a servo to target position, clamped to joint limits."""
    lo, hi = JOINT_LIMITS.get(sid, (200, 3800))
    target = max(lo, min(hi, int(target)))
    pkt.write2ByteTxRx(ph, sid, ADDR_MOVING_SPEED, speed)
    pkt.write2ByteTxRx(ph, sid, ADDR_GOAL_POSITION, target)
    return target


def wait_servo(ph, pkt, sid, target, timeout=5.0):
    """Wait until servo reaches target position. Returns True on success."""
    start = time.time()
    while time.time() - start < timeout:
        pos = read_servo(ph, pkt, sid)
        if pos is not None and abs(pos - target) < 20:
            return True
        time.sleep(0.1)
    return False


def wait_all_stopped(ph, pkt, timeout=2.0):
    """Wait until all arm servos and wheels report speed ~0."""
    all_ids = list(range(1, 6)) + WHEEL_IDS
    start = time.time()
    while time.time() - start < timeout:
        all_stopped = True
        for sid in all_ids:
            try:
                speed, result, _ = pkt.read2ByteTxRx(ph, sid, ADDR_PRESENT_SPEED)
                if result != sdk.COMM_SUCCESS or speed > 5:
                    all_stopped = False
                    break
            except Exception:
                all_stopped = False
                break
        if all_stopped:
            return True
        time.sleep(0.05)
    return False


def move_to_pose(ph, pkt, pose, speed=CAL_SPEED):
    """Move all 5 joints to a pose (tuple of 5 tick values) and wait."""
    sids = [SHOULDER_ROT, SHOULDER_PITCH, ELBOW, WRIST_PITCH, WRIST_ROLL]
    targets = []
    for sid, target in zip(sids, pose):
        actual = move_servo(ph, pkt, sid, target, speed)
        targets.append(actual)
    for sid, target in zip(sids, targets):
        wait_servo(ph, pkt, sid, target, timeout=10)
    time.sleep(0.5)


def clear_overload(ph, pkt, sid):
    """Toggle torque off/on to clear overload protection lockout."""
    pkt.write1ByteTxRx(ph, sid, ADDR_TORQUE_ENABLE, 0)
    time.sleep(0.1)
    pkt.write1ByteTxRx(ph, sid, ADDR_TORQUE_ENABLE, 1)
    time.sleep(0.05)


# ── Base wheel helpers ──

def set_wheel_speed(ph, pkt, speed, acc=WHEEL_DEFAULT_ACC):
    """Set all base wheels to a speed in closed-loop wheel mode.

    Args:
        speed: steps/sec, positive = increase heading, negative = decrease.
        acc: acceleration (0-254)
    """
    servo_speed = -speed  # Invert: +heading needs negative servo direction
    if servo_speed < 0:
        encoded = (-servo_speed) | (1 << 15)
    else:
        encoded = servo_speed

    for sid in WHEEL_IDS:
        txdata = [
            acc,
            0, 0,                         # Goal Position (ignored in wheel mode)
            0, 0,                         # Goal Time (ignored)
            encoded & 0xFF,
            (encoded >> 8) & 0xFF,
        ]
        pkt.writeTxRx(ph, sid, ADDR_ACC, len(txdata), txdata)
        time.sleep(0.01)


def stop_wheels(ph, pkt):
    """Stop all base wheels."""
    set_wheel_speed(ph, pkt, 0)


def move_base_az(ph, pkt, imu_bus, az_degrees, speed=500, timeout=10.0):
    """Rotate base by az_degrees using wheels with IMU feedback.

    Returns actual degrees rotated.
    """
    if abs(az_degrees) < 0.2:
        return 0.0

    start_imu = read_imu(imu_bus, samples=3, interval=0.02)
    start_heading = start_imu['heading']
    target_heading = (start_heading + az_degrees) % 360

    direction = 1 if az_degrees > 0 else -1
    set_wheel_speed(ph, pkt, direction * speed)

    t0 = time.time()
    while time.time() - t0 < timeout:
        current = read_imu(imu_bus, samples=2, interval=0.01)
        remaining = angle_diff(target_heading, current['heading'])

        if abs(remaining) < 1.5:
            stop_wheels(ph, pkt)
            time.sleep(0.2)
            final = read_imu(imu_bus, samples=3, interval=0.02)
            return angle_diff(final['heading'], start_heading)

        if abs(remaining) < 10:
            proportional = max(80, int(speed * abs(remaining) / 10))
            set_wheel_speed(ph, pkt, (1 if remaining > 0 else -1) * proportional)

        time.sleep(0.05)

    # Timeout
    stop_wheels(ph, pkt)
    time.sleep(0.2)
    final = read_imu(imu_bus, samples=3, interval=0.02)
    return angle_diff(final['heading'], start_heading)


def wait_wheels_stopped(ph, pkt, timeout=3.0):
    """Wait until all base wheel servos report speed ~0."""
    start = time.time()
    while time.time() - start < timeout:
        all_stopped = True
        for sid in WHEEL_IDS:
            try:
                spd, result, _ = pkt.read2ByteTxRx(ph, sid, ADDR_PRESENT_SPEED)
                if result != sdk.COMM_SUCCESS or spd > 5:
                    all_stopped = False
                    break
            except Exception:
                all_stopped = False
                break
        if all_stopped:
            return True
        time.sleep(0.05)
    return False

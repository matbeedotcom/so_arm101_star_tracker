#!/usr/bin/env python3
"""Determine BNO055 axis orientation by moving servos and reading raw accel.

Moves shoulder rotation and wrist pitch through several positions,
printing raw accelerometer (ax, ay, az) at each. This reveals which
IMU body axis corresponds to heading rotation vs pitch tilt.
"""

import time
import struct
import scservo_sdk as sdk
from goto.config import (
    SERVO_PORT, SERVO_BAUD, JOINT_LIMITS,
    SHOULDER_ROT, SHOULDER_PITCH, ELBOW, WRIST_PITCH, WRIST_ROLL,
    ADDR_GOAL_POSITION, ADDR_MOVING_SPEED, ADDR_TORQUE_ENABLE,
    I2C_BUS, IMU_ADDR, OPR_MODE_REG, MODE_NDOF, MODE_CONFIG,
)
from goto.imu import init_imu, read_imu_quat, quat_to_pointing
from goto.servos import move_servo, wait_servo, read_servo


def read_raw_accel(bus):
    data = bus.read_i2c_block_data(IMU_ADDR, 0x08, 6)
    ax = struct.unpack('<h', bytes(data[0:2]))[0] / 100.0
    ay = struct.unpack('<h', bytes(data[2:4]))[0] / 100.0
    az = struct.unpack('<h', bytes(data[4:6]))[0] / 100.0
    return ax, ay, az


def read_gravity(bus):
    data = bus.read_i2c_block_data(IMU_ADDR, 0x2E, 6)
    gx = struct.unpack('<h', bytes(data[0:2]))[0] / 100.0
    gy = struct.unpack('<h', bytes(data[2:4]))[0] / 100.0
    gz = struct.unpack('<h', bytes(data[4:6]))[0] / 100.0
    return gx, gy, gz


def main():
    # Init IMU
    imu_bus = init_imu(mode=MODE_NDOF)
    print("BNO055 initialized (NDOF)")
    time.sleep(1.0)

    # Init servos
    ph = sdk.PortHandler(SERVO_PORT)
    pkt = sdk.PacketHandler(0)
    assert ph.openPort()
    assert ph.setBaudRate(SERVO_BAUD)
    for sid in range(1, 6):
        pkt.write1ByteTxRx(ph, sid, ADDR_TORQUE_ENABLE, 0)
    time.sleep(0.2)
    for sid in range(1, 6):
        pkt.write1ByteTxRx(ph, sid, ADDR_TORQUE_ENABLE, 1)
    time.sleep(0.1)

    # Load tracking pose for motors 2,3
    from goto.config import load_pose
    try:
        pose = load_pose('tracking')
        move_servo(ph, pkt, SHOULDER_PITCH, pose[2], 200)
        move_servo(ph, pkt, ELBOW, pose[3], 200)
        move_servo(ph, pkt, WRIST_ROLL, pose.get(5, 1204), 200)
        time.sleep(1.5)
        print(f"Locked M2={pose[2]}, M3={pose[3]}")
    except:
        print("No tracking pose, using current positions")

    # Start wrist pitch at mid-range
    wp_mid = 1900
    move_servo(ph, pkt, WRIST_PITCH, wp_mid, 200)
    move_servo(ph, pkt, SHOULDER_ROT, 2100, 200)
    time.sleep(2.0)

    print("\n" + "=" * 80)
    print("TEST 1: Rotate shoulder (azimuth) — wrist pitch fixed")
    print("=" * 80)
    print(f"{'ShRot':>6}  {'ax':>6} {'ay':>6} {'az':>6}  |  {'gx':>6} {'gy':>6} {'gz':>6}  |  {'heading':>7} {'pitch':>6}")
    print("-" * 80)

    for sh_pos in [1400, 1700, 2000, 2300, 2600, 2900]:
        lo, hi = JOINT_LIMITS[SHOULDER_ROT]
        sh_pos = max(lo, min(hi, sh_pos))
        move_servo(ph, pkt, SHOULDER_ROT, sh_pos, 200)
        time.sleep(1.5)
        ax, ay, az = read_raw_accel(imu_bus)
        gx, gy, gz = read_gravity(imu_bus)
        q = read_imu_quat(imu_bus)
        h, p = quat_to_pointing(q)
        actual = read_servo(ph, pkt, SHOULDER_ROT)
        print(f"{actual:>6}  {ax:>6.1f} {ay:>6.1f} {az:>6.1f}  |  {gx:>6.1f} {gy:>6.1f} {gz:>6.1f}  |  {h:>7.1f} {p:>6.1f}")

    # Return to center
    move_servo(ph, pkt, SHOULDER_ROT, 2100, 200)
    time.sleep(1.5)

    print("\n" + "=" * 80)
    print("TEST 2: Tilt wrist pitch — shoulder rotation fixed")
    print("=" * 80)
    print(f"{'WPit':>6}  {'ax':>6} {'ay':>6} {'az':>6}  |  {'gx':>6} {'gy':>6} {'gz':>6}  |  {'heading':>7} {'pitch':>6}")
    print("-" * 80)

    for wp_pos in [2800, 2400, 2000, 1600, 1200, 924]:
        move_servo(ph, pkt, WRIST_PITCH, wp_pos, 200)
        time.sleep(1.5)
        ax, ay, az = read_raw_accel(imu_bus)
        gx, gy, gz = read_gravity(imu_bus)
        q = read_imu_quat(imu_bus)
        h, p = quat_to_pointing(q)
        actual = read_servo(ph, pkt, WRIST_PITCH)
        print(f"{actual:>6}  {ax:>6.1f} {ay:>6.1f} {az:>6.1f}  |  {gx:>6.1f} {gy:>6.1f} {gz:>6.1f}  |  {h:>7.1f} {p:>6.1f}")

    # Return to mid
    move_servo(ph, pkt, WRIST_PITCH, wp_mid, 200)
    time.sleep(1.0)

    print("\nDone. Look for:")
    print("  TEST 1: Which accel axes change with rotation (those are horizontal)")
    print("  TEST 2: Which accel axes change with tilt (that's the pitch axis)")

    ph.closePort()
    imu_bus.close()


if __name__ == '__main__':
    main()

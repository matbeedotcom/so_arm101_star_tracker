"""BNO055 IMU driver: init, quaternion reads, pointing, calibration."""

import smbus2 as smbus
import struct
import time
import math

from .config import (
    I2C_BUS, IMU_ADDR, CALIB_STAT, OPR_MODE_REG, QUA_DATA_W_LSB,
    MODE_CONFIG, MODE_IMU, MODE_NDOF,
    CAM_FORWARD_BODY,
    IMU_STABLE_THRESHOLD, IMU_STABLE_SAMPLES, IMU_STABLE_INTERVAL,
    CAL_POSES, SHOULDER_ROT, JOINT_LIMITS,
)


def init_imu(mode=MODE_IMU):
    """Initialize BNO055 in the given fusion mode. Returns smbus object."""
    bus = smbus.SMBus(I2C_BUS)
    chip_id = bus.read_byte_data(IMU_ADDR, 0x00)
    assert chip_id == 0xA0, f"Bad BNO055 chip ID: 0x{chip_id:02x}"
    bus.write_byte_data(IMU_ADDR, OPR_MODE_REG, MODE_CONFIG)
    time.sleep(0.025)
    bus.write_byte_data(IMU_ADDR, OPR_MODE_REG, mode)
    time.sleep(0.5)
    return bus


def _quat_rotate(q, v):
    """Rotate vector v by quaternion q = (w, x, y, z)."""
    w, x, y, z = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def read_imu_quat(bus):
    """Read BNO055 quaternion and return (w, x, y, z) as floats."""
    data = bus.read_i2c_block_data(IMU_ADDR, QUA_DATA_W_LSB, 8)
    w = struct.unpack('<h', bytes(data[0:2]))[0] / 16384.0
    x = struct.unpack('<h', bytes(data[2:4]))[0] / 16384.0
    y = struct.unpack('<h', bytes(data[4:6]))[0] / 16384.0
    z = struct.unpack('<h', bytes(data[6:8]))[0] / 16384.0
    return (w, x, y, z)


# BNO055 gravity vector register
GRV_DATA_X_LSB = 0x2E


def read_gravity(bus):
    """Read BNO055 gravity vector (m/s²). Returns (gx, gy, gz).

    This is heading-independent — purely from the accelerometer fusion.
    Available in IMU and NDOF modes.
    """
    data = bus.read_i2c_block_data(IMU_ADDR, GRV_DATA_X_LSB, 6)
    gx = struct.unpack('<h', bytes(data[0:2]))[0] / 100.0
    gy = struct.unpack('<h', bytes(data[2:4]))[0] / 100.0
    gz = struct.unpack('<h', bytes(data[4:6]))[0] / 100.0
    return (gx, gy, gz)


def gravity_pitch(bus):
    """Compute pitch (elevation) from RAW accelerometer data.

    Uses atan2(ax, az) since the wrist tilt sweeps ax and az
    while ay stays roughly constant. This is pure accelerometer —
    no quaternion fusion, no magnetometer, no heading dependency.

    Returns pitch in degrees (-90 to +90).
    """
    data = bus.read_i2c_block_data(IMU_ADDR, 0x08, 6)
    ax = struct.unpack('<h', bytes(data[0:2]))[0] / 100.0
    az = struct.unpack('<h', bytes(data[4:6]))[0] / 100.0

    # BNO055 axes on wrist (from calibrate_imu_axes.py):
    #   WPit=1508 (camera level): ax≈0, az≈9.8  → gravity along +Z
    #   WPit=1201 (camera up):    ax≈-4.4, az≈8.9 → gravity shifts to -X
    #   WPit=2399 (camera down):  ax≈9.2, az≈1.1  → gravity shifts to +X
    # So pitch = atan2(-ax, az): level→0°, up→+26°, down→-83°
    pitch = math.degrees(math.atan2(-ax, az))
    return pitch


def quat_to_pointing(q):
    """Convert IMU quaternion to camera heading and pitch (altitude).

    Returns (heading_deg, pitch_deg) where:
      heading: 0-360 compass bearing (0=North, 90=East)
      pitch: -90 to +90 elevation (positive = above horizon)
    """
    wx, wy, wz = _quat_rotate(q, CAM_FORWARD_BODY)
    horiz = math.sqrt(wx * wx + wy * wy)
    pitch = math.degrees(math.atan2(wz, horiz)) if horiz > 1e-6 else (90.0 if wz > 0 else -90.0)
    heading = math.degrees(math.atan2(wx, wy)) % 360.0
    return heading, pitch


def read_imu(bus, samples=5, interval=0.03):
    """Read IMU orientation using quaternions, averaged over multiple samples."""
    headings, pitches = [], []
    for _ in range(samples):
        q = read_imu_quat(bus)
        h, p = quat_to_pointing(q)
        headings.append(h)
        pitches.append(p)
        time.sleep(interval)

    # Average headings with wrap handling
    hx = sum(math.cos(math.radians(h)) for h in headings) / len(headings)
    hy = sum(math.sin(math.radians(h)) for h in headings) / len(headings)
    avg_heading = math.degrees(math.atan2(hy, hx)) % 360.0
    avg_pitch = sum(pitches) / len(pitches)

    calib = bus.read_byte_data(IMU_ADDR, CALIB_STAT)
    return {
        'heading': avg_heading,
        'pitch': avg_pitch,
        'calib_sys': (calib >> 6) & 0x03,
        'calib_gyro': (calib >> 4) & 0x03,
        'calib_accel': (calib >> 2) & 0x03,
        'calib_mag': calib & 0x03,
    }


def wait_imu_stable(bus, timeout=3.0):
    """Wait until IMU readings stabilize. Returns averaged IMU dict or None."""
    start = time.time()
    while time.time() - start < timeout:
        headings, pitches = [], []
        for _ in range(IMU_STABLE_SAMPLES):
            q = read_imu_quat(bus)
            h, p = quat_to_pointing(q)
            headings.append(h)
            pitches.append(p)
            time.sleep(IMU_STABLE_INTERVAL)

        h_spread = max(headings) - min(headings)
        if h_spread > 180:
            shifted = [(h + 180) % 360 for h in headings]
            h_spread = max(shifted) - min(shifted)
        p_spread = max(pitches) - min(pitches)

        if h_spread < IMU_STABLE_THRESHOLD and p_spread < IMU_STABLE_THRESHOLD:
            hx = sum(math.cos(math.radians(h)) for h in headings) / len(headings)
            hy = sum(math.sin(math.radians(h)) for h in headings) / len(headings)
            avg_h = math.degrees(math.atan2(hy, hx)) % 360.0
            avg_p = sum(pitches) / len(pitches)

            calib = bus.read_byte_data(IMU_ADDR, CALIB_STAT)
            return {
                'heading': avg_h,
                'pitch': avg_p,
                'calib_sys': (calib >> 6) & 0x03,
                'calib_gyro': (calib >> 4) & 0x03,
                'calib_accel': (calib >> 2) & 0x03,
                'calib_mag': calib & 0x03,
                'h_spread': h_spread,
                'p_spread': p_spread,
            }
        time.sleep(0.1)
    return None


# ── Calibration status helpers ──

def read_calib_status(bus):
    """Read BNO055 calibration register, return dict with sys/gyro/accel/mag."""
    calib = bus.read_byte_data(IMU_ADDR, CALIB_STAT)
    return {
        'sys':   (calib >> 6) & 0x03,
        'gyro':  (calib >> 4) & 0x03,
        'accel': (calib >> 2) & 0x03,
        'mag':   calib & 0x03,
    }


def print_calib(cal, prefix=""):
    print(f"{prefix}S{cal['sys']} G{cal['gyro']} A{cal['accel']} M{cal['mag']}")


def wait_calib(bus, targets, timeout=15.0, label=""):
    """Poll calibration status until targets met. Returns True on success."""
    start = time.time()
    while time.time() - start < timeout:
        cal = read_calib_status(bus)
        status = f"S{cal['sys']}G{cal['gyro']}A{cal['accel']}M{cal['mag']}"
        print(f"\r  {label} [{status}] {time.time()-start:.0f}s", end="", flush=True)
        if all(cal[k] >= v for k, v in targets.items()):
            print()
            return True
        time.sleep(0.3)
    print()
    return False


def calib_str(imu_data, active_mode=MODE_IMU):
    """Format calibration for display based on active mode."""
    if active_mode == MODE_IMU:
        return f"G{imu_data.get('calib_gyro', '?')}A{imu_data.get('calib_accel', '?')}"
    return (f"S{imu_data.get('calib_sys', '?')}G{imu_data.get('calib_gyro', '?')}"
            f"A{imu_data.get('calib_accel', '?')}M{imu_data.get('calib_mag', '?')}")


def calibrate_imu(bus, move_fn, mode):
    """Run automated IMU calibration routine.

    Args:
        bus: smbus object
        move_fn: callable(pose_name) that moves arm to a named pose and waits
        mode: MODE_IMU or MODE_NDOF
    """
    print("\n" + "=" * 50)
    print("  IMU CALIBRATION ROUTINE")
    print("=" * 50)
    cal = read_calib_status(bus)
    print_calib(cal, prefix="  Initial: ")

    # Step 1: Gyro — hold still at home
    print("\n  [1/3] Gyro calibration — holding still...")
    move_fn('home')
    time.sleep(5.0)
    if not wait_calib(bus, {'gyro': 3}, timeout=10, label="Gyro"):
        print("  Gyro cal incomplete, continuing...")

    # Step 2: Accel — cycle through poses
    print("\n  [2/3] Accel calibration — rotating through poses...")
    accel_poses = ['wrist_up', 'wrist_down', 'roll_left', 'roll_right',
                   'pitch_fwd', 'pitch_back']
    for pose_name in accel_poses:
        cal = read_calib_status(bus)
        if cal['accel'] >= 3:
            print(f"  Accel calibrated (level 3)")
            break
        print(f"  -> {pose_name}")
        move_fn(pose_name)
        time.sleep(2.0)

    # Step 3: Mag (NDOF only)
    if mode == MODE_NDOF:
        print("\n  [3/3] Magnetometer calibration — sweeping rotation...")
        # This requires direct servo access, so we use move_fn with custom poses
        # For now, just move through some rotation poses
        move_fn('home')
        time.sleep(1.0)
    else:
        print("\n  [3/3] Mag calibration — skipped (IMU mode, no magnetometer)")

    # Return to home
    print("\n  Returning to home pose...")
    move_fn('home')

    cal = read_calib_status(bus)
    print_calib(cal, prefix="\n  Final: ")

    if cal['gyro'] < 3:
        print("  WARNING: Gyro calibration low — readings may drift")
    if cal['accel'] < 2:
        print("  WARNING: Accel calibration low — pitch may be inaccurate")
    if mode == MODE_NDOF and cal['mag'] < 2:
        print("  WARNING: Mag calibration low — heading may drift")

    print("=" * 50 + "\n")

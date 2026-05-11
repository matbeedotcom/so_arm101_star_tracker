"""Constants, calibration data, and pose management."""

import os
import json
import math
from datetime import datetime

# ── I2C / IMU ──
I2C_BUS = 1
IMU_ADDR = 0x28
EULER_H_LSB = 0x1A
CALIB_STAT = 0x35
OPR_MODE_REG = 0x3D
MODE_NDOF = 0x0C
MODE_IMU = 0x08
MODE_CONFIG = 0x00

# BNO055 quaternion registers
QUA_DATA_W_LSB = 0x20

# Camera forward vector in IMU body frame (BNO055 +X = camera boresight)
# Determined empirically: wrist tilt sweeps ax from +9 to -5, az from +1 to +9
CAM_FORWARD_BODY = (1.0, 0.0, 0.0)

# ── Servo communication ──
SERVO_PORT = '/dev/ttyACM0'
SERVO_BAUD = 1000000
ADDR_GOAL_POSITION = 42
ADDR_MOVING_SPEED = 46
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_SPEED = 58
ADDR_TORQUE_ENABLE = 40
ADDR_TORQUE_LIMIT = 34

# STS3215 mode register
ADDR_MODE = 33
ADDR_LOCK = 55
ADDR_ACC = 41
MODE_SERVO = 0
MODE_WHEEL_CLOSED = 1
WHEEL_DEFAULT_ACC = 50

# ── Servo IDs ──
SHOULDER_ROT = 1
SHOULDER_PITCH = 2
ELBOW = 3
WRIST_PITCH = 4
WRIST_ROLL = 5

# Base wheel servo IDs (omniwheels for azimuth rotation)
WHEEL_IDS = [7, 8, 9]

# ── Mechanical limits ──
JOINT_LIMITS = {
    1: (742, 3494),    # Shoulder Rotation: 242°
    2: (900, 2305),    # Shoulder Pitch
    3: (896, 2500),    # Elbow
    4: (924, 2859),    # Wrist Pitch: 170°
    5: (7, 2004),      # Wrist Roll: 176°
}

# Servo tick values at URDF joint angle 0
ZERO_TICKS = {
    SHOULDER_ROT: 2132,
    SHOULDER_PITCH: 936,
    ELBOW: 2945,
    WRIST_PITCH: 2670,
    WRIST_ROLL: 1205,
}

# ── Control parameters ──
MOVE_SPEED = 200
TRACK_SPEED = 80
CAL_SPEED = 150
TOLERANCE_DEG = 0.5
SLEW_SETTLE = 0.3
TRACK_INTERVAL = 1.0
TRACK_SETTLE = 0.3
PITCH_MAX_STEP = 200

# IMU-to-servo gains (measured)
TICKS_PER_DEG_ROTATION = 150 / 12.9   # ~11.6 ticks/deg
TICKS_PER_DEG_PITCH = 150 / 13.3      # ~11.3 ticks/deg — legacy single-joint
WHEEL_TICKS_PER_DEG = 35.0

# Per-joint pitch leverage (ticks per degree of camera-pitch contribution).
# Each pitch joint has a different mechanical advantage on the camera angle,
# so a single constant blew up the inverse map. Shoulder pitch has the
# biggest lever (small ticks per degree); wrist pitch the smallest.
# These are empirical defaults — calibrate with a per-joint sweep for
# better accuracy. The WRIST_PITCH entry is overridden by pitch_cal.json.
PITCH_TICKS_PER_DEG = {
    SHOULDER_PITCH: 5.0,
    ELBOW:          8.0,
    WRIST_PITCH:    TICKS_PER_DEG_PITCH,
}

# Control deadbands — sub-degree corrections aren't worth the chatter.
AZ_DEADBAND_DEG   = 1.0    # Skip azimuth correction below this error.
AZ_WHEEL_MIN_DEG  = 3.0    # Only fall back to wheels for moves ≥ this.
ALT_DEADBAND_DEG  = 0.5    # Skip altitude correction below this error.

# Achievable altitude envelope (degrees). Targets outside this range
# are rejected before the slew loop — the arm can't physically reach
# them, so iterating burns time and grinds servos against limits. Tune
# for your actual mechanical reach; raise the floor if your linkage
# can't get below ~10° elevation.
ACHIEVABLE_ALT_MIN = 10.0
ACHIEVABLE_ALT_MAX = 80.0

# Slew convergence — stop iterating when error plateaus.
SLEW_MAX_ITERATIONS    = 18    # Hard cap; loop usually exits earlier.
SLEW_NO_PROGRESS_LIMIT = 4     # Iters without ≥SLEW_MIN_IMPROVEMENT → bail.
SLEW_MIN_IMPROVEMENT   = 0.5   # Degrees of total-error reduction per iter.

# ── Camera optics ──
# Field of view (degrees). Wide-field lenses stay forgiving at
# degree-scale pointing errors; long focal lengths need arc-minute
# precision. Used by the frontend to render "% of FOV" feedback and
# decide whether a target is comfortably framed.
CAMERA_HFOV_DEG = 75.0
CAMERA_VFOV_DEG = 50.0

# Approximate angular sizes for known celestial bodies (degrees).
# Stars are point sources (0.0). Planets vary with Earth distance — the
# values below are typical peaks at favourable oppositions; treat as
# rough framing hints rather than ephemeris-grade truth.
TARGET_ANGULAR_SIZE = {
    'sun':     0.53,
    'moon':    0.52,
    'mercury': 0.003,
    'venus':   0.017,
    'mars':    0.014,
    'jupiter': 0.040,
    'saturn':  0.046,   # rings included
    'uranus':  0.001,
    'neptune': 0.0007,
}

# IMU stability thresholds
IMU_STABLE_THRESHOLD = 0.15
IMU_STABLE_SAMPLES = 6
IMU_STABLE_INTERVAL = 0.05

# ── Calibration poses ──
CAL_POSES = {
    'home':       (2118, 950,  2200, 950,  1005),
    'wrist_up':   (2118, 950,  2200, 924,  1005),
    'wrist_down': (2118, 950,  2200, 2859, 1005),
    'roll_left':  (2118, 950,  2200, 950,  7),
    'roll_right': (2118, 950,  2200, 950,  2004),
    'pitch_fwd':  (2118, 950,  1600, 924,  1005),
    'pitch_back': (2118, 950,  2200, 2859, 1005),
}

# Per-joint direction for pitch: +1 means +ticks = pitch up
UP_POSE = {SHOULDER_PITCH: 2146, ELBOW: 2500, WRIST_PITCH: 924}
HOME_POSE = {
    SHOULDER_PITCH: CAL_POSES['home'][1],
    ELBOW: CAL_POSES['home'][2],
    WRIST_PITCH: CAL_POSES['home'][3],
}
PITCH_DIR = {
    sid: (1 if UP_POSE[sid] > HOME_POSE[sid] else -1)
    for sid in [SHOULDER_PITCH, ELBOW, WRIST_PITCH]
}

# Per-joint minimum tick thresholds (from calibrate_min_ticks.py)
MIN_TICKS_BY_SPEED = {
    1: [(40, 5),  (80, 14), (9999, 13)],
    2: [(40, 23), (80, 24), (9999, 10)],
    3: [(40, 8),  (80, 12), (9999, 12)],
    4: [(40, 10), (80, 9),  (9999, 10)],
    5: [(40, 10), (80, 10), (9999, 10)],
}

# ── Observer location ──
OBSERVER_LAT = 43.65
OBSERVER_LON = -79.38
OBSERVER_HEIGHT = 76

# ── Star catalog ──
STAR_CATALOG = {
    'polaris':      ('2h31m49.09s',  '+89d15m50.8s'),
    'sirius':       ('6h45m08.92s',  '-16d42m58.0s'),
    'vega':         ('18h36m56.34s', '+38d47m01.3s'),
    'arcturus':     ('14h15m39.67s', '+19d10m56.7s'),
    'betelgeuse':   ('5h55m10.31s',  '+7d24m25.4s'),
    'rigel':        ('5h14m32.27s',  '-8d12m05.9s'),
    'capella':      ('5h16m41.36s',  '+45d59m52.8s'),
    'procyon':      ('7h39m18.12s',  '+5d13m30.0s'),
    'altair':       ('19h50m47.00s', '+8d52m06.0s'),
    'deneb':        ('20h41m25.91s', '+45d16m49.2s'),
    'antares':      ('16h29m24.46s', '-26d25m55.2s'),
    'spica':        ('13h25m11.58s', '-11d09m40.8s'),
    'aldebaran':    ('4h35m55.24s',  '+16d30m33.5s'),
    'regulus':      ('10h08m22.31s', '+11d58m01.9s'),
    'castor':       ('7h34m35.87s',  '+31d53m17.8s'),
    'pollux':       ('7h45m18.95s',  '+28d01m34.3s'),
    'fomalhaut':    ('22h57m39.05s', '-29d37m20.1s'),
    'canopus':      ('6h23m57.11s',  '-52d41m44.4s'),
}

SOLAR_SYSTEM = [
    'sun', 'moon', 'mercury', 'venus', 'mars',
    'jupiter', 'saturn', 'uranus', 'neptune',
]


# ── Utility ──

def angle_diff(target, current):
    """Signed shortest-path difference between two angles in degrees."""
    diff = target - current
    while diff > 180:
        diff -= 360
    while diff < -180:
        diff += 360
    return diff


def get_min_ticks(servo_id, speed):
    """Look up minimum tick threshold for a servo at a given speed."""
    tiers = MIN_TICKS_BY_SPEED.get(servo_id)
    if not tiers:
        return 10
    for speed_threshold, min_ticks in tiers:
        if speed <= speed_threshold:
            return min_ticks
    return tiers[-1][1]


# ── Pose save/load ──

_WORKSPACE = os.path.dirname(os.path.dirname(__file__))
POSES_DIR = os.path.join(_WORKSPACE, 'poses')


def save_pose(name, positions, metadata=None):
    """Save servo positions to poses/<name>.json."""
    os.makedirs(POSES_DIR, exist_ok=True)
    data = {'positions': {str(k): v for k, v in positions.items()}}
    if metadata:
        data.update(metadata)
    data['timestamp'] = datetime.now().isoformat()
    path = os.path.join(POSES_DIR, f'{name}.json')
    with open(path, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"Saved pose '{name}' to {path}")
    return path


def load_pose(name):
    """Load servo positions from poses/<name>.json or workspace root."""
    for path in [
        os.path.join(POSES_DIR, f'{name}.json'),
        os.path.join(_WORKSPACE, f'{name}.json'),
        name,
    ]:
        if os.path.exists(path):
            with open(path) as f:
                data = json.load(f)
            return {int(k): v for k, v in data['positions'].items()}
    raise FileNotFoundError(f"Pose '{name}' not found")

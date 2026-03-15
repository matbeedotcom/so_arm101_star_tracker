"""goto — Modular celestial tracking for the SO-100 arm."""

from .config import (
    SHOULDER_ROT, SHOULDER_PITCH, ELBOW, WRIST_PITCH, WRIST_ROLL,
    WHEEL_IDS, JOINT_LIMITS,
    save_pose, load_pose,
)
from .strategy import MotionStrategy, WristOnlyStrategy

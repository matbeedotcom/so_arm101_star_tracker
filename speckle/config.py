"""
Configuration dataclasses for speckle interferometry pipeline.
"""

from dataclasses import dataclass, field


@dataclass
class CaptureConfig:
    """Camera capture parameters."""
    resolution: tuple = (5120, 800)
    bit_depth: int = 8
    exposure_us: int = 5000
    burst_count: int = 100
    burst_interval: float = 0.0  # seconds between frames (0 = as fast as possible)


@dataclass
class StabilityConfig:
    """IMU/servo stability gating for burst capture."""
    imu_threshold: float = 0.15       # max heading/pitch spread (degrees)
    imu_samples: int = 6              # readings to check
    imu_interval: float = 0.05        # seconds between readings
    servo_speed_threshold: int = 5    # max servo speed register value
    servo_timeout: float = 2.0        # seconds to wait for servos to stop
    imu_timeout: float = 3.0          # seconds to wait for IMU to stabilize
    require_stable: bool = True       # skip burst if not stable


@dataclass
class ProcessingConfig:
    """Speckle processing parameters."""
    image_size: int = 512
    pixel_size_um: float = 3.45       # OV9281 pixel size
    wavelength_nm: float = 550.0      # observation wavelength (green)
    # Triangular mount baselines (mm) — center + 3 outer at 95mm radius
    camera_baselines: dict = field(default_factory=lambda: {
        (0, 1): 95.0,
        (0, 2): 95.0,
        (0, 3): 95.0,
        (1, 2): 164.5,  # 95 * sqrt(3) ≈ 164.5
        (1, 3): 164.5,
        (2, 3): 164.5,
    })
    max_bispectrum_triangles: int = 5000  # random UV triangles for Pi5 performance

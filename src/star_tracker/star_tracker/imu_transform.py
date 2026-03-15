#!/usr/bin/env python3

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R
from typing import Tuple, Optional


class IMUTransform:
    """Handles transformation between IMU coordinates and telescope pointing."""

    def __init__(self, config_file: Optional[str] = None):
        """Initialize IMU transform with configuration."""
        self.config = self.load_config(config_file) if config_file else self.default_config()

        # Extract configuration
        self.rotation_matrix = np.array(self.config['imu_to_telescope']['rotation_matrix'])
        self.euler_offsets = self.config['imu_to_telescope']['euler_offsets']
        self.mounting_offset = self.config['mounting_offset']
        self.coordinate_convention = self.config['coordinate_convention']
        self.magnetic_declination = self.config['compensation']['magnetic_declination']

        # Convert euler offsets to radians
        self.roll_offset = np.radians(self.euler_offsets['roll_offset'])
        self.pitch_offset = np.radians(self.euler_offsets['pitch_offset'])
        self.yaw_offset = np.radians(self.euler_offsets['yaw_offset'])

        # Filtering parameters
        self.use_filter = self.config['filtering']['use_kalman_filter']
        self.alpha = self.config['filtering']['complementary_filter_alpha']

        # State for complementary filter
        self.filtered_orientation = None

    def default_config(self) -> dict:
        """Return default configuration if no config file provided."""
        return {
            'imu_to_telescope': {
                'rotation_matrix': [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                'euler_offsets': {'roll_offset': 0, 'pitch_offset': 0, 'yaw_offset': 0}
            },
            'mounting_offset': {'x': 0, 'y': 0, 'z': 0},
            'coordinate_convention': 'NED',
            'compensation': {'magnetic_declination': 0, 'temperature_coefficient': 0},
            'filtering': {
                'use_kalman_filter': False,
                'complementary_filter_alpha': 0.98,
                'lpf_cutoff': 5.0
            }
        }

    def load_config(self, config_file: str) -> dict:
        """Load configuration from YAML file."""
        try:
            with open(config_file, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            print(f"Error loading IMU config: {e}, using defaults")
            return self.default_config()

    def imu_to_telescope(self, imu_euler: np.ndarray) -> Tuple[float, float]:
        """
        Transform IMU euler angles to telescope pointing (altitude, azimuth).

        Args:
            imu_euler: [roll, pitch, yaw] in radians from IMU

        Returns:
            (altitude, azimuth) in radians for telescope pointing
        """
        # Apply coordinate convention transformation
        if self.coordinate_convention == 'NED':
            # North-East-Down convention
            roll, pitch, yaw = imu_euler
        else:  # ENU
            # East-North-Up convention - swap and negate as needed
            roll, pitch, yaw = imu_euler[1], imu_euler[0], -imu_euler[2]

        # Create rotation from euler angles
        imu_rotation = R.from_euler('xyz', [roll, pitch, yaw])

        # Apply custom rotation matrix transformation
        transformed = imu_rotation.as_matrix() @ self.rotation_matrix.T

        # Extract telescope pointing angles
        # This depends on how the IMU is mounted relative to telescope
        telescope_rotation = R.from_matrix(transformed)
        telescope_euler = telescope_rotation.as_euler('xyz')

        # Apply calibration offsets
        telescope_euler[0] += self.roll_offset
        telescope_euler[1] += self.pitch_offset
        telescope_euler[2] += self.yaw_offset

        # Apply magnetic declination to azimuth
        telescope_euler[2] += np.radians(self.magnetic_declination)

        # Convert to altitude/azimuth
        # Altitude: angle above horizon (pitch in telescope frame)
        # Azimuth: compass bearing (yaw in telescope frame)
        altitude = telescope_euler[1]  # Pitch represents pointing up/down
        azimuth = telescope_euler[2]    # Yaw represents compass direction

        # Ensure altitude is in correct range
        altitude = np.clip(altitude, -np.pi/2, np.pi/2)

        # Normalize azimuth to [0, 2π]
        azimuth = azimuth % (2 * np.pi)

        return altitude, azimuth

    def telescope_to_imu(self, altitude: float, azimuth: float) -> np.ndarray:
        """
        Inverse transformation: telescope pointing to expected IMU readings.

        Args:
            altitude: Telescope altitude in radians
            azimuth: Telescope azimuth in radians

        Returns:
            Expected IMU euler angles [roll, pitch, yaw] in radians
        """
        # Remove magnetic declination from azimuth
        azimuth -= np.radians(self.magnetic_declination)

        # Create telescope euler angles (assuming roll=0 for telescope)
        telescope_euler = np.array([0, altitude, azimuth])

        # Remove calibration offsets
        telescope_euler[0] -= self.roll_offset
        telescope_euler[1] -= self.pitch_offset
        telescope_euler[2] -= self.yaw_offset

        # Apply inverse rotation matrix
        telescope_rotation = R.from_euler('xyz', telescope_euler)
        transformed = telescope_rotation.as_matrix() @ np.linalg.inv(self.rotation_matrix.T)

        # Convert back to IMU euler angles
        imu_rotation = R.from_matrix(transformed)
        imu_euler = imu_rotation.as_euler('xyz')

        # Apply coordinate convention inverse
        if self.coordinate_convention == 'ENU':
            imu_euler = np.array([imu_euler[1], imu_euler[0], -imu_euler[2]])

        return imu_euler

    def apply_complementary_filter(self,
                                   gyro_orientation: np.ndarray,
                                   accel_orientation: np.ndarray) -> np.ndarray:
        """
        Apply complementary filter to combine gyro and accelerometer data.

        Args:
            gyro_orientation: Orientation from gyroscope integration
            accel_orientation: Orientation from accelerometer

        Returns:
            Filtered orientation
        """
        if self.filtered_orientation is None:
            self.filtered_orientation = accel_orientation
            return accel_orientation

        # Complementary filter: trust gyro short-term, accel long-term
        self.filtered_orientation = (self.alpha * gyro_orientation +
                                     (1 - self.alpha) * accel_orientation)

        return self.filtered_orientation

    def compensate_mounting_offset(self, altitude: float, azimuth: float,
                                   distance: float = 0.0) -> Tuple[float, float]:
        """
        Compensate for IMU mounting position offset from optical axis.

        Args:
            altitude: Measured altitude in radians
            azimuth: Measured azimuth in radians
            distance: Distance to target (for parallax correction)

        Returns:
            Corrected (altitude, azimuth)
        """
        if distance <= 0 or np.allclose(self.mounting_offset.values(), 0):
            return altitude, azimuth

        # Calculate angular offset due to mounting position
        # This is simplified - full correction would need target distance
        x_offset = self.mounting_offset['x']
        y_offset = self.mounting_offset['y']
        z_offset = self.mounting_offset['z']

        # Angular corrections (small angle approximation)
        if distance > 0:
            alt_correction = np.arctan(z_offset / distance)
            az_correction = np.arctan(y_offset / distance)
        else:
            alt_correction = 0
            az_correction = 0

        return altitude + alt_correction, azimuth + az_correction

    def calibrate_from_known_target(self,
                                     measured_imu: np.ndarray,
                                     true_altitude: float,
                                     true_azimuth: float):
        """
        Calibrate transformation using known target position.

        Args:
            measured_imu: IMU euler angles when pointing at target
            true_altitude: Known altitude of target
            true_azimuth: Known azimuth of target
        """
        # Calculate what IMU should read for this target
        expected_imu = self.telescope_to_imu(true_altitude, true_azimuth)

        # Calculate correction needed
        error = expected_imu - measured_imu

        # Update offsets (simple approach - could use more sophisticated methods)
        self.roll_offset += error[0] * 0.1  # Gradual adjustment
        self.pitch_offset += error[1] * 0.1
        self.yaw_offset += error[2] * 0.1

        return error


class IMUCalibrationProcedure:
    """Automated calibration procedure for IMU-telescope alignment."""

    def __init__(self, transform: IMUTransform):
        self.transform = transform
        self.calibration_points = []

    def add_calibration_point(self,
                              imu_reading: np.ndarray,
                              true_alt: float,
                              true_az: float,
                              target_name: str = ""):
        """Add a calibration point."""
        self.calibration_points.append({
            'imu': imu_reading,
            'altitude': true_alt,
            'azimuth': true_az,
            'target': target_name
        })

    def calculate_transformation_matrix(self) -> np.ndarray:
        """
        Calculate optimal transformation matrix from calibration points.
        Uses least squares fitting if more than 3 points available.
        """
        if len(self.calibration_points) < 3:
            raise ValueError("Need at least 3 calibration points")

        # Build matrices for least squares
        imu_points = np.array([p['imu'] for p in self.calibration_points])
        telescope_points = np.array([[p['altitude'], p['azimuth'], 0]
                                     for p in self.calibration_points])

        # Solve for transformation matrix using SVD
        # T * IMU = Telescope
        transformation, residuals, rank, s = np.linalg.lstsq(
            imu_points, telescope_points, rcond=None
        )

        return transformation

    def validate_calibration(self) -> float:
        """
        Validate calibration by checking residual errors.
        Returns RMS error in degrees.
        """
        if not self.calibration_points:
            return float('inf')

        errors = []
        for point in self.calibration_points:
            # Transform IMU to telescope using current calibration
            calc_alt, calc_az = self.transform.imu_to_telescope(point['imu'])

            # Calculate error
            alt_error = calc_alt - point['altitude']
            az_error = calc_az - point['azimuth']

            # RMS error in degrees
            error_deg = np.degrees(np.sqrt(alt_error**2 + az_error**2))
            errors.append(error_deg)

        return np.mean(errors)
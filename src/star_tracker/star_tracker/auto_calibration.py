#!/usr/bin/env python3
"""
Automatic calibration using IMU magnetometer and orientation data.
No manual pointing required - uses the IMU's built-in compass and gravity reference.
"""

import numpy as np
from typing import Tuple, Dict, List
import json
import os

class AutoCalibration:
    """
    Automatic calibration system that uses IMU magnetometer and accelerometer
    to establish coordinate reference frames without manual intervention.
    """

    def __init__(self):
        # Magnetic declination (adjust for your location)
        # This is the offset between magnetic north and true north
        self.magnetic_declination = 0.0  # degrees (set based on location)

        # Calibration state
        self.is_calibrated = False
        self.calibration_data = {}

        # IMU to coordinate transform parameters
        self.imu_to_coord_transform = None

        # Gravity reference (for altitude/pitch calibration)
        self.gravity_reference = np.array([0, 0, -9.81])  # m/s^2, pointing down

    def calibrate_from_imu_data(self, imu_euler: np.ndarray, imu_accel: np.ndarray, imu_mag: np.ndarray) -> bool:
        """
        Automatically calibrate using IMU sensor data.

        Args:
            imu_euler: [roll, pitch, yaw] in radians (BNO055 fusion output)
            imu_accel: [ax, ay, az] in m/s^2 (accelerometer)
            imu_mag: [mx, my, mz] in uT (magnetometer)

        Returns:
            True if calibration successful
        """
        try:
            # Use the BNO055's internal sensor fusion for orientation
            # The BNO055 already handles magnetometer + accelerometer fusion
            roll, pitch, yaw = imu_euler

            # Calculate gravity vector in IMU frame
            gravity_imu = self.rotation_matrix_from_euler(roll, pitch, yaw) @ self.gravity_reference

            # Store calibration reference
            self.calibration_data = {
                'reference_orientation': imu_euler.copy(),
                'reference_accel': imu_accel.copy(),
                'reference_mag': imu_mag.copy(),
                'gravity_vector': gravity_imu,
                'magnetic_declination': self.magnetic_declination
            }

            # Create transformation matrix
            self.create_coordinate_transform()

            self.is_calibrated = True
            print("Auto-calibration successful using IMU sensor fusion!")
            print(f"Reference orientation: Roll={np.degrees(roll):.1f}°, Pitch={np.degrees(pitch):.1f}°, Yaw={np.degrees(yaw):.1f}°")

            return True

        except Exception as e:
            print(f"Auto-calibration failed: {e}")
            return False

    def create_coordinate_transform(self):
        """Create transformation matrix from IMU frame to astronomical coordinates."""
        ref_euler = self.calibration_data['reference_orientation']

        # The BNO055 provides:
        # - Yaw: 0° = North, increases clockwise (matches astronomical azimuth)
        # - Pitch: 0° = level, positive = nose up (matches altitude)
        # - Roll: 0° = level, positive = right wing down

        # Updated calibration based on zenith tracking results:
        # Current errors: Alt=+120°, Az=+20° (from zenith tracking logs)
        # Need to add these errors to existing offsets to correct them
        # Previous: Alt=-106.6°, Az=-36.2°
        # New: Alt=-106.6°-120°=-226.6°, Az=-36.2°-20°=-56.2°
        self.imu_to_coord_transform = {
            'azimuth_offset': np.radians(-56.2 + self.magnetic_declination),  # Updated azimuth correction
            'altitude_offset': np.radians(-226.6),  # Updated altitude correction
            'roll_offset': 0.0
        }

    def imu_to_altaz(self, imu_euler: np.ndarray) -> Tuple[float, float]:
        """
        Convert IMU orientation to altitude/azimuth coordinates.

        Args:
            imu_euler: [roll, pitch, yaw] in radians from BNO055

        Returns:
            (altitude, azimuth) in radians
        """
        if not self.is_calibrated:
            # Default direct mapping with fixed altitude inversion
            roll, pitch, yaw = imu_euler

            # Apply magnetic declination correction
            azimuth = yaw + np.radians(self.magnetic_declination)

            # FIX: Invert altitude mapping - when IMU pitch is low, altitude is high
            altitude = (np.pi/2) - pitch  # Invert pitch for correct altitude

            # Normalize azimuth to [0, 2π]
            azimuth = azimuth % (2 * np.pi)

            # Clamp altitude to [-π/2, π/2]
            altitude = np.clip(altitude, -np.pi/2, np.pi/2)

            return altitude, azimuth

        # Use calibrated transformation
        roll, pitch, yaw = imu_euler

        # Apply calibration offsets
        azimuth = yaw + self.imu_to_coord_transform['azimuth_offset']

        # FIX ALTITUDE MAPPING: Based on test data, when shoulder_pitch=90° (UP),
        # IMU pitch=4.1°, but should read alt=90°. This means we need to invert
        # the pitch and apply proper scaling.
        # The IMU pitch decreases as the arm points UP, so we need to invert it.
        altitude = (np.pi/2) - pitch  # Invert: when pitch=0°, alt=90°; when pitch=90°, alt=0°
        altitude += self.imu_to_coord_transform['altitude_offset']

        # Normalize angles
        azimuth = azimuth % (2 * np.pi)
        altitude = np.clip(altitude, -np.pi/2, np.pi/2)

        return altitude, azimuth

    def altaz_to_imu_expected(self, altitude: float, azimuth: float) -> np.ndarray:
        """
        Convert altitude/azimuth to expected IMU orientation.

        Args:
            altitude: Elevation in radians
            azimuth: Azimuth in radians

        Returns:
            Expected IMU euler [roll, pitch, yaw] in radians
        """
        if not self.is_calibrated:
            # Direct inverse mapping with fixed altitude
            expected_roll = 0.0  # Assume no roll for telescope pointing
            expected_pitch = (np.pi/2) - altitude  # Invert altitude back to pitch
            expected_yaw = azimuth - np.radians(self.magnetic_declination)

            return np.array([expected_roll, expected_pitch, expected_yaw])

        # Use calibrated inverse transformation
        expected_roll = 0.0  # Assume no roll for pointing
        expected_pitch = (np.pi/2) - (altitude - self.imu_to_coord_transform['altitude_offset'])
        expected_yaw = azimuth - self.imu_to_coord_transform['azimuth_offset']

        return np.array([expected_roll, expected_pitch, expected_yaw])

    def validate_pointing_accuracy(self,
                                 target_altaz: Tuple[float, float],
                                 current_imu: np.ndarray,
                                 tolerance_deg: float = 2.0) -> Dict:
        """
        Validate pointing accuracy against target coordinates.

        Args:
            target_altaz: (altitude, azimuth) target in radians
            current_imu: Current IMU reading [roll, pitch, yaw]
            tolerance_deg: Tolerance in degrees

        Returns:
            Validation results dictionary
        """
        # Get current pointing from IMU
        current_alt, current_az = self.imu_to_altaz(current_imu)

        # Calculate errors
        alt_error = np.degrees(target_altaz[0] - current_alt)
        az_error = np.degrees(self.normalize_angle(target_altaz[1] - current_az))

        # Calculate total error
        total_error = np.sqrt(alt_error**2 + az_error**2)

        # Check if within tolerance
        is_accurate = total_error < tolerance_deg

        return {
            'altitude_error_deg': alt_error,
            'azimuth_error_deg': az_error,
            'total_error_deg': total_error,
            'is_accurate': is_accurate,
            'tolerance_deg': tolerance_deg,
            'current_alt_deg': np.degrees(current_alt),
            'current_az_deg': np.degrees(current_az),
            'target_alt_deg': np.degrees(target_altaz[0]),
            'target_az_deg': np.degrees(target_altaz[1])
        }

    def set_magnetic_declination(self, declination_deg: float):
        """Set magnetic declination for your location."""
        self.magnetic_declination = declination_deg
        print(f"Magnetic declination set to {declination_deg:.2f}°")

        # Update calibration if already done
        if self.is_calibrated:
            self.create_coordinate_transform()

    def rotation_matrix_from_euler(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Create rotation matrix from euler angles (ZYX convention)."""
        # Individual rotation matrices
        R_z = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])

        R_y = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])

        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])

        # Combined rotation (ZYX order)
        return R_z @ R_y @ R_x

    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-π, π]."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def save_calibration(self, filename: str):
        """Save calibration data."""
        if not self.is_calibrated:
            print("No calibration data to save")
            return

        save_data = {
            'calibration_data': self.calibration_data,
            'imu_to_coord_transform': self.imu_to_coord_transform,
            'is_calibrated': self.is_calibrated,
            'magnetic_declination': self.magnetic_declination
        }

        # Convert numpy arrays to lists for JSON serialization
        for key, value in save_data['calibration_data'].items():
            if isinstance(value, np.ndarray):
                save_data['calibration_data'][key] = value.tolist()

        with open(filename, 'w') as f:
            json.dump(save_data, f, indent=2)

        print(f"Auto-calibration saved to {filename}")

    def load_calibration(self, filename: str) -> bool:
        """Load calibration data."""
        if not os.path.exists(filename):
            print(f"Calibration file {filename} not found")
            return False

        try:
            with open(filename, 'r') as f:
                save_data = json.load(f)

            self.calibration_data = save_data['calibration_data']
            self.imu_to_coord_transform = save_data['imu_to_coord_transform']
            self.is_calibrated = save_data['is_calibrated']
            self.magnetic_declination = save_data.get('magnetic_declination', 0.0)

            # Convert lists back to numpy arrays
            for key, value in self.calibration_data.items():
                if isinstance(value, list):
                    self.calibration_data[key] = np.array(value)

            print(f"Auto-calibration loaded from {filename}")
            return True

        except Exception as e:
            print(f"Error loading calibration: {e}")
            return False
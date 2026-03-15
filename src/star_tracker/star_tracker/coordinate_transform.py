#!/usr/bin/env python3

import numpy as np
from typing import Tuple, Optional, Dict, List
import json
import os

class CoordinateTransform:
    """
    Handles transformations between:
    - Astronomical coordinates (Alt/Az, RA/Dec)
    - ARM joint positions
    - IMU orientation readings

    This replaces hardcoded calibration with a learned transformation system.
    """

    def __init__(self):
        # ARM joint limits (radians)
        self.joint_limits = {
            'shoulder_rotation': (-np.pi, np.pi),
            'shoulder_pitch': (-np.pi/2, np.pi/2),
            'elbow': (-np.pi, np.pi),
            'wrist_pitch': (-np.pi/2, np.pi/2),
            'wrist_roll': (-np.pi, np.pi)
        }

        # Calibration data storage
        self.calibration_points = []
        self.transform_matrix = None
        self.is_calibrated = False

        # ARM kinematics parameters (adjust based on your robot)
        self.arm_config = {
            'base_height': 0.0,  # Height of base from ground
            'shoulder_length': 0.15,  # Length from base to shoulder joint
            'upper_arm_length': 0.20,  # Length of upper arm
            'forearm_length': 0.18,   # Length of forearm
            'end_effector_offset': 0.05  # Camera/IMU offset from wrist
        }

    def altaz_to_joint_positions(self, altitude: float, azimuth: float) -> List[float]:
        """
        Convert altitude/azimuth coordinates to ARM joint positions.

        Args:
            altitude: Elevation angle in radians (0 = horizon, π/2 = zenith)
            azimuth: Azimuth angle in radians (0 = North, π/2 = East)

        Returns:
            List of joint positions [shoulder_rot, shoulder_pitch, elbow, wrist_pitch, wrist_roll]
        """
        # Simple kinematic approach - can be refined

        # Shoulder rotation directly controls azimuth
        shoulder_rotation = azimuth

        # For altitude control, use shoulder pitch as primary
        # Altitude 0° (horizon) -> shoulder_pitch = 0°
        # Altitude 90° (zenith) -> shoulder_pitch = 90°
        shoulder_pitch = altitude

        # Keep elbow straight for simplicity
        elbow = 0.0

        # Wrist pitch compensates to keep end-effector pointing correctly
        # This keeps the camera/IMU level with the target
        wrist_pitch = -shoulder_pitch

        # Wrist roll for camera orientation (0 for now)
        wrist_roll = 0.0

        # Apply joint limits
        joint_positions = [
            np.clip(shoulder_rotation, *self.joint_limits['shoulder_rotation']),
            np.clip(shoulder_pitch, *self.joint_limits['shoulder_pitch']),
            np.clip(elbow, *self.joint_limits['elbow']),
            np.clip(wrist_pitch, *self.joint_limits['wrist_pitch']),
            np.clip(wrist_roll, *self.joint_limits['wrist_roll'])
        ]

        return joint_positions

    def joint_positions_to_altaz(self, joint_positions: List[float]) -> Tuple[float, float]:
        """
        Calculate expected altitude/azimuth from joint positions (forward kinematics).

        Args:
            joint_positions: [shoulder_rot, shoulder_pitch, elbow, wrist_pitch, wrist_roll]

        Returns:
            (altitude, azimuth) in radians
        """
        shoulder_rot, shoulder_pitch, elbow, wrist_pitch, wrist_roll = joint_positions

        # Simple forward kinematics
        # The end-effector pointing direction is determined by:
        # - Azimuth from shoulder rotation
        # - Altitude from shoulder pitch + wrist pitch compensation

        azimuth = shoulder_rot
        altitude = shoulder_pitch + wrist_pitch  # Combined effect

        return altitude, azimuth

    def joint_positions_to_imu_expected(self, joint_positions: List[float]) -> np.ndarray:
        """
        Calculate expected IMU orientation from joint positions.

        Args:
            joint_positions: [shoulder_rot, shoulder_pitch, elbow, wrist_pitch, wrist_roll]

        Returns:
            Expected IMU euler angles [roll, pitch, yaw] in radians
        """
        shoulder_rot, shoulder_pitch, elbow, wrist_pitch, wrist_roll = joint_positions

        # Simplified approach: direct mapping from joint angles to expected IMU
        # This assumes a specific mounting orientation and can be refined with calibration

        # The end-effector orientation is primarily determined by:
        # - Yaw (heading) from shoulder rotation
        # - Pitch (elevation) from shoulder pitch + wrist pitch
        # - Roll from wrist roll

        expected_yaw = shoulder_rot  # Shoulder rotation controls heading
        expected_pitch = shoulder_pitch + wrist_pitch  # Combined pitch effect
        expected_roll = wrist_roll  # Direct roll mapping

        return np.array([expected_roll, expected_pitch, expected_yaw])

    def imu_to_altaz(self, imu_euler: np.ndarray) -> Tuple[float, float]:
        """
        Convert IMU orientation to altitude/azimuth (if calibrated).

        Args:
            imu_euler: [roll, pitch, yaw] in radians from IMU

        Returns:
            (altitude, azimuth) in radians
        """
        if not self.is_calibrated:
            # Fallback: direct mapping (BNO055 convention: x=yaw, y=roll, z=pitch)
            return imu_euler[2], imu_euler[0]  # pitch->alt, yaw->az

        # Apply calibration correction
        corrected_imu = imu_euler + self.imu_correction

        # Convert to altitude/azimuth
        # This mapping can be refined based on actual IMU mounting
        altitude = corrected_imu[2]  # pitch -> altitude
        azimuth = corrected_imu[0]   # yaw -> azimuth

        return altitude, azimuth

    def add_calibration_point(self,
                            joint_positions: List[float],
                            imu_reading: np.ndarray,
                            true_altaz: Tuple[float, float],
                            target_name: str = ""):
        """
        Add a calibration point for learning the transformation.

        Args:
            joint_positions: Current joint positions
            imu_reading: Current IMU reading [roll, pitch, yaw]
            true_altaz: Known true altitude/azimuth of target
            target_name: Name of calibration target
        """
        calib_point = {
            'joint_positions': joint_positions.copy(),
            'imu_reading': imu_reading.copy(),
            'true_altitude': true_altaz[0],
            'true_azimuth': true_altaz[1],
            'target': target_name,
            'timestamp': np.datetime64('now').astype(str)
        }

        self.calibration_points.append(calib_point)

        print(f"Added calibration point for {target_name}: "
              f"Alt={np.degrees(true_altaz[0]):.1f}°, Az={np.degrees(true_altaz[1]):.1f}°")

    def calculate_imu_joint_relationship(self) -> bool:
        """
        Calculate the relationship between IMU readings and joint positions.

        Returns:
            True if successful calibration
        """
        if len(self.calibration_points) < 3:
            print(f"Need at least 3 calibration points, have {len(self.calibration_points)}")
            return False

        # Extract data matrices
        joint_data = np.array([point['joint_positions'] for point in self.calibration_points])
        imu_data = np.array([point['imu_reading'] for point in self.calibration_points])
        altaz_data = np.array([[point['true_altitude'], point['true_azimuth']]
                              for point in self.calibration_points])

        # For now, use a simple approach: learn mapping from joints to expected IMU
        # More sophisticated methods could use machine learning

        # Calculate expected IMU readings from joint positions
        expected_imu = np.array([self.joint_positions_to_imu_expected(joints)
                                for joints in joint_data])

        # Calculate errors between expected and actual IMU readings
        imu_errors = imu_data - expected_imu

        # Store average correction factors
        self.imu_correction = np.mean(imu_errors, axis=0)

        # Validate calibration quality
        rms_error = np.sqrt(np.mean(np.sum((expected_imu + self.imu_correction - imu_data)**2, axis=1)))

        print(f"IMU calibration complete. RMS error: {np.degrees(rms_error):.2f}°")
        print(f"IMU correction factors: roll={np.degrees(self.imu_correction[0]):.2f}°, "
              f"pitch={np.degrees(self.imu_correction[1]):.2f}°, "
              f"yaw={np.degrees(self.imu_correction[2]):.2f}°")

        self.is_calibrated = True
        return True

    def save_calibration(self, filename: str):
        """Save calibration data to file."""
        calib_data = {
            'calibration_points': self.calibration_points,
            'imu_correction': self.imu_correction.tolist() if self.is_calibrated else None,
            'is_calibrated': self.is_calibrated,
            'arm_config': self.arm_config
        }

        with open(filename, 'w') as f:
            json.dump(calib_data, f, indent=2)

        print(f"Calibration saved to {filename}")

    def load_calibration(self, filename: str) -> bool:
        """Load calibration data from file."""
        if not os.path.exists(filename):
            print(f"Calibration file {filename} not found")
            return False

        try:
            with open(filename, 'r') as f:
                calib_data = json.load(f)

            self.calibration_points = calib_data['calibration_points']
            self.is_calibrated = calib_data['is_calibrated']

            if self.is_calibrated and calib_data['imu_correction']:
                self.imu_correction = np.array(calib_data['imu_correction'])

            if 'arm_config' in calib_data:
                self.arm_config.update(calib_data['arm_config'])

            print(f"Calibration loaded from {filename}")
            print(f"Loaded {len(self.calibration_points)} calibration points")

            return True

        except Exception as e:
            print(f"Error loading calibration: {e}")
            return False

    def validate_position(self,
                         joint_positions: List[float],
                         imu_reading: np.ndarray,
                         tolerance_deg: float = 2.0) -> Tuple[bool, Dict]:
        """
        Validate if current position matches expected based on calibration.

        Args:
            joint_positions: Current joint positions
            imu_reading: Current IMU reading
            tolerance_deg: Tolerance in degrees for validation

        Returns:
            (is_valid, error_info)
        """
        if not self.is_calibrated:
            return True, {"message": "No calibration available"}

        # Calculate expected IMU reading from joint positions
        expected_imu = self.joint_positions_to_imu_expected(joint_positions)
        expected_imu += self.imu_correction  # Apply calibration correction

        # Calculate error
        error = imu_reading - expected_imu
        error_deg = np.degrees(error)

        # Check if within tolerance
        is_valid = np.all(np.abs(error_deg) < tolerance_deg)

        error_info = {
            'roll_error': error_deg[0],
            'pitch_error': error_deg[1],
            'yaw_error': error_deg[2],
            'max_error': np.max(np.abs(error_deg)),
            'is_valid': is_valid
        }

        return is_valid, error_info
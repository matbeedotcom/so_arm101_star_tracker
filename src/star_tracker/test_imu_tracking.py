#!/usr/bin/env python3

"""
Test script for IMU-based star tracking
Verifies that IMU euler angles are correctly mapped to telescope pointing
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
import numpy as np


class IMUTrackingTest(Node):
    def __init__(self):
        super().__init__('imu_tracking_test')

        # Subscribe to IMU euler angles
        self.euler_sub = self.create_subscription(
            Vector3, 'imu/euler', self.euler_callback, 10
        )

        # Track received data
        self.euler_data = None
        self.test_timer = self.create_timer(1.0, self.run_tests)

        self.get_logger().info('IMU Tracking Test started')
        self.get_logger().info('Move the end-effector/IMU to test tracking:')
        self.get_logger().info('1. Point North (Az=0°) at horizon (Alt=0°)')
        self.get_logger().info('2. Point East (Az=90°)')
        self.get_logger().info('3. Point up (Alt=90°)')

    def euler_callback(self, msg):
        """Store euler angles from IMU."""
        self.euler_data = msg

    def run_tests(self):
        """Display current IMU readings and expected telescope pointing."""
        if self.euler_data is None:
            self.get_logger().warn('Waiting for IMU data...')
            return

        # Extract euler angles (BNO055 convention)
        heading_deg = np.degrees(self.euler_data.x)  # Yaw/heading
        roll_deg = np.degrees(self.euler_data.y)     # Roll
        pitch_deg = np.degrees(self.euler_data.z)    # Pitch

        # Calculate expected telescope pointing
        # Based on corrected mapping:
        azimuth_deg = heading_deg  # Heading maps to azimuth
        altitude_deg = pitch_deg   # Pitch maps to altitude

        # Normalize azimuth to 0-360
        if azimuth_deg < 0:
            azimuth_deg += 360

        # Display results
        self.get_logger().info('-' * 50)
        self.get_logger().info('IMU Euler Angles:')
        self.get_logger().info(f'  Heading/Yaw: {heading_deg:6.1f}° (x-axis)')
        self.get_logger().info(f'  Roll:        {roll_deg:6.1f}° (y-axis)')
        self.get_logger().info(f'  Pitch:       {pitch_deg:6.1f}° (z-axis)')
        self.get_logger().info('Telescope Pointing:')
        self.get_logger().info(f'  Azimuth:  {azimuth_deg:6.1f}° (N=0, E=90, S=180, W=270)')
        self.get_logger().info(f'  Altitude: {altitude_deg:6.1f}° (Horizon=0, Zenith=90)')

        # Check for common orientations
        self.check_orientation(azimuth_deg, altitude_deg)

    def check_orientation(self, az, alt):
        """Check if current orientation matches known positions."""
        orientations = []

        # Check cardinal directions
        if abs(az - 0) < 10 or abs(az - 360) < 10:
            orientations.append('North')
        elif abs(az - 90) < 10:
            orientations.append('East')
        elif abs(az - 180) < 10:
            orientations.append('South')
        elif abs(az - 270) < 10:
            orientations.append('West')

        # Check altitude
        if abs(alt) < 10:
            orientations.append('Horizon level')
        elif abs(alt - 45) < 10:
            orientations.append('45° elevation')
        elif abs(alt - 90) < 10:
            orientations.append('Pointing at Zenith')
        elif abs(alt + 90) < 10:
            orientations.append('Pointing at Nadir')

        if orientations:
            self.get_logger().info(f'  Position: {", ".join(orientations)}')


def main(args=None):
    rclpy.init(args=args)
    node = IMUTrackingTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
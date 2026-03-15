#!/usr/bin/env python3
"""
Test to find the actual IMU range by manually moving the arm
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import numpy as np
import time

class IMURangeTest(Node):
    def __init__(self):
        super().__init__('imu_range_test')

        # IMU data subscription
        self.euler_sub = self.create_subscription(
            Vector3, '/imu/euler', self.euler_callback, 10
        )

        self.current_euler = None
        self.readings = []

        self.get_logger().info('IMU Range Test - Manual Arm Movement')
        self.get_logger().info('=' * 60)
        self.get_logger().info('INSTRUCTIONS:')
        self.get_logger().info('1. Manually move the arm to point at the HORIZON')
        self.get_logger().info('2. Press ENTER to record horizon reading')
        self.get_logger().info('3. Manually move the arm to point STRAIGHT UP')
        self.get_logger().info('4. Press ENTER to record zenith reading')
        self.get_logger().info('5. The test will calculate the actual range')
        self.get_logger().info('=' * 60)

    def euler_callback(self, msg):
        """Store current IMU euler angles."""
        self.current_euler = {
            'heading': np.degrees(msg.x),
            'roll': np.degrees(msg.y),
            'pitch': np.degrees(msg.z)
        }

    def get_current_imu(self):
        """Get current IMU reading with some averaging."""
        readings = []
        for i in range(20):  # Collect 20 readings
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_euler is not None:
                readings.append((
                    self.current_euler['heading'],
                    self.current_euler['roll'],
                    self.current_euler['pitch']
                ))
            time.sleep(0.05)

        if not readings:
            return None

        # Average the readings
        readings_array = np.array(readings)
        avg = np.mean(readings_array, axis=0)
        std = np.std(readings_array, axis=0)

        return {
            'heading': avg[0],
            'roll': avg[1],
            'pitch': avg[2],
            'heading_std': std[0],
            'roll_std': std[1],
            'pitch_std': std[2]
        }

    def run_test(self):
        """Run manual range finding test."""

        # Wait for IMU data
        self.get_logger().info('Waiting for IMU data...')
        while self.current_euler is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('IMU data available!')

        # Show live readings
        self.get_logger().info('\nCurrent live IMU readings:')
        for i in range(50):  # Show 5 seconds of readings
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_euler is not None:
                print(f'H={self.current_euler["heading"]:6.1f}° R={self.current_euler["roll"]:6.1f}° P={self.current_euler["pitch"]:6.1f}°', end='\r')
            time.sleep(0.1)

        print()  # New line

        # Horizon measurement
        self.get_logger().info('\n--- STEP 1: HORIZON MEASUREMENT ---')
        self.get_logger().info('Position the arm to point at the HORIZON (level, 0° elevation)')
        input('Press ENTER when ready to record horizon reading...')

        horizon_imu = self.get_current_imu()
        if horizon_imu is None:
            self.get_logger().error('Failed to get horizon reading')
            return False

        self.get_logger().info(f'Horizon IMU: H={horizon_imu["heading"]:.1f}° R={horizon_imu["roll"]:.1f}° P={horizon_imu["pitch"]:.1f}°')
        self.get_logger().info(f'Stability:   H±{horizon_imu["heading_std"]:.1f}° R±{horizon_imu["roll_std"]:.1f}° P±{horizon_imu["pitch_std"]:.1f}°')

        # Zenith measurement
        self.get_logger().info('\n--- STEP 2: ZENITH MEASUREMENT ---')
        self.get_logger().info('Position the arm to point STRAIGHT UP (90° elevation)')
        input('Press ENTER when ready to record zenith reading...')

        zenith_imu = self.get_current_imu()
        if zenith_imu is None:
            self.get_logger().error('Failed to get zenith reading')
            return False

        self.get_logger().info(f'Zenith IMU:  H={zenith_imu["heading"]:.1f}° R={zenith_imu["roll"]:.1f}° P={zenith_imu["pitch"]:.1f}°')
        self.get_logger().info(f'Stability:   H±{zenith_imu["heading_std"]:.1f}° R±{zenith_imu["roll_std"]:.1f}° P±{zenith_imu["pitch_std"]:.1f}°')

        # Analysis
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('CALIBRATION ANALYSIS')
        self.get_logger().info('=' * 60)

        pitch_range = zenith_imu['pitch'] - horizon_imu['pitch']
        heading_change = zenith_imu['heading'] - horizon_imu['heading']

        self.get_logger().info(f'Horizon pitch: {horizon_imu["pitch"]:.1f}°')
        self.get_logger().info(f'Zenith pitch:  {zenith_imu["pitch"]:.1f}°')
        self.get_logger().info(f'Pitch range:   {pitch_range:.1f}° (should map to 90° altitude)')

        if abs(heading_change) > 180:
            if heading_change > 0:
                heading_change -= 360
            else:
                heading_change += 360

        self.get_logger().info(f'Heading change: {heading_change:.1f}° (should be ~0° for vertical movement)')

        # Generate new calibration
        self.get_logger().info('\n--- RECOMMENDED CALIBRATION VALUES ---')
        self.get_logger().info(f'imu_horizon_pitch = {horizon_imu["pitch"]:.1f}')
        self.get_logger().info(f'imu_zenith_pitch = {zenith_imu["pitch"]:.1f}')

        # Test current position
        self.get_logger().info('\n--- CURRENT POSITION TEST ---')
        current_imu = self.get_current_imu()
        if current_imu is not None:
            # Calculate altitude using new calibration
            calculated_alt = (current_imu['pitch'] - horizon_imu['pitch']) * (90.0 / pitch_range)

            self.get_logger().info(f'Current IMU pitch: {current_imu["pitch"]:.1f}°')
            self.get_logger().info(f'Calculated altitude: {calculated_alt:.1f}°')

            # Compare with old calibration
            old_horizon = -95.5
            old_zenith = 175.0
            old_range = old_zenith - old_horizon
            old_alt = (current_imu['pitch'] - old_horizon) * (90.0 / old_range)

            self.get_logger().info(f'Old calibration would give: {old_alt:.1f}°')
            self.get_logger().info(f'Difference: {calculated_alt - old_alt:.1f}°')

        return True


def main():
    rclpy.init()
    node = IMURangeTest()

    try:
        node.run_test()
    except KeyboardInterrupt:
        node.get_logger().info('Test interrupted')
    except Exception as e:
        node.get_logger().error(f'Test failed: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
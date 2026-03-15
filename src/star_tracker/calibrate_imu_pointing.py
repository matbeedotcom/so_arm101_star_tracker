#!/usr/bin/env python3

"""
IMU Calibration Script for Star Tracker
Helps determine the correct IMU-to-telescope transformation
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import numpy as np
import yaml
import os


class IMUCalibrationNode(Node):
    def __init__(self):
        super().__init__('imu_calibration')

        # Subscribe to IMU euler angles
        self.euler_sub = self.create_subscription(
            Vector3, '/imu/euler', self.euler_callback, 10
        )

        self.current_euler = None
        self.calibration_points = []

        # Timer for instructions
        self.instruction_timer = self.create_timer(2.0, self.show_instructions)
        self.instruction_step = 0

        self.get_logger().info('IMU Calibration Node Started')
        self.get_logger().info('Follow the instructions to calibrate your IMU orientation')

    def euler_callback(self, msg):
        """Store current IMU euler angles."""
        # Convert from radians to degrees for easier reading
        self.current_euler = {
            'heading': np.degrees(msg.x),
            'roll': np.degrees(msg.y),
            'pitch': np.degrees(msg.z)
        }

    def show_instructions(self):
        """Show calibration instructions step by step."""
        if self.current_euler is None:
            self.get_logger().warn('Waiting for IMU data...')
            return

        # Display current IMU readings
        euler = self.current_euler
        self.get_logger().info('-' * 60)
        self.get_logger().info(f'Current IMU: H={euler["heading"]:6.1f}° R={euler["roll"]:6.1f}° P={euler["pitch"]:6.1f}°')

        if self.instruction_step == 0:
            self.get_logger().info('STEP 1: Point the telescope/camera LEVEL at the HORIZON')
            self.get_logger().info('        The optical axis should be pointing straight ahead (0° elevation)')
            self.get_logger().info('        Press Enter when positioned...')
            self.instruction_step = 1

        elif self.instruction_step == 1:
            self.get_logger().info('Position the arm to point LEVEL at HORIZON, then press ENTER')
            # Wait for user input (this would need to be handled externally)

        elif self.instruction_step == 2:
            self.get_logger().info('STEP 2: Point the telescope/camera straight UP (toward zenith)')
            self.get_logger().info('        The optical axis should be pointing at 90° elevation')
            self.get_logger().info('        Press Enter when positioned...')
            self.instruction_step = 3

        elif self.instruction_step == 3:
            self.get_logger().info('Position the arm to point UP at ZENITH, then press ENTER')

        elif self.instruction_step == 4:
            self.get_logger().info('STEP 3: Point the telescope/camera due NORTH')
            self.get_logger().info('        The optical axis should be pointing at 0° azimuth')
            self.get_logger().info('        Press Enter when positioned...')
            self.instruction_step = 5

        elif self.instruction_step == 5:
            self.get_logger().info('Position the arm to point NORTH, then press ENTER')

    def capture_calibration_point(self, position_name, expected_alt, expected_az):
        """Capture a calibration point."""
        if self.current_euler is None:
            self.get_logger().error('No IMU data available')
            return

        point = {
            'name': position_name,
            'imu_heading': self.current_euler['heading'],
            'imu_roll': self.current_euler['roll'],
            'imu_pitch': self.current_euler['pitch'],
            'expected_altitude': expected_alt,
            'expected_azimuth': expected_az
        }

        self.calibration_points.append(point)
        self.get_logger().info(f'Captured {position_name}: IMU({point["imu_heading"]:.1f}°, {point["imu_roll"]:.1f}°, {point["imu_pitch"]:.1f}°) -> Target({expected_alt}°, {expected_az}°)')

    def calculate_calibration(self):
        """Calculate calibration offsets from captured points."""
        if len(self.calibration_points) < 2:
            self.get_logger().error('Need at least 2 calibration points')
            return

        self.get_logger().info('\nCalculating calibration offsets...')

        # Find the best offsets to map IMU readings to expected telescope pointing
        pitch_offsets = []
        yaw_offsets = []

        for point in self.calibration_points:
            # For altitude: telescope elevation should map to corrected IMU pitch
            pitch_offset = point['expected_altitude'] - point['imu_pitch']
            pitch_offsets.append(pitch_offset)

            # For azimuth: telescope azimuth should map to IMU heading
            yaw_offset = point['expected_azimuth'] - point['imu_heading']
            # Handle angle wrapping
            if yaw_offset > 180:
                yaw_offset -= 360
            elif yaw_offset < -180:
                yaw_offset += 360
            yaw_offsets.append(yaw_offset)

        # Average the offsets
        avg_pitch_offset = float(np.mean(pitch_offsets))
        avg_yaw_offset = float(np.mean(yaw_offsets))

        self.get_logger().info(f'Calculated offsets:')
        self.get_logger().info(f'  Pitch offset: {avg_pitch_offset:.1f}° (altitude correction)')
        self.get_logger().info(f'  Yaw offset: {avg_yaw_offset:.1f}° (azimuth correction)')

        # Generate calibration config
        self.generate_config(avg_pitch_offset, avg_yaw_offset)

    def generate_config(self, pitch_offset, yaw_offset):
        """Generate updated calibration configuration."""
        config = {
            'imu_to_telescope': {
                'rotation_matrix': [
                    [1.0, 0.0, 0.0],  # Roll mapping
                    [0.0, 1.0, 0.0],  # Pitch mapping
                    [0.0, 0.0, 1.0]   # Yaw mapping
                ],
                'euler_offsets': {
                    'roll_offset': 0.0,
                    'pitch_offset': pitch_offset,
                    'yaw_offset': yaw_offset
                }
            },
            'mounting_offset': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'coordinate_convention': 'NED',
            'compensation': {'magnetic_declination': 0.0, 'temperature_coefficient': 0.01},
            'filtering': {'use_kalman_filter': False, 'complementary_filter_alpha': 0.98, 'lpf_cutoff': 5.0},
            'validation': {'max_pointing_error': 1.0, 'hold_time': 3.0, 'max_std_dev': 0.5}
        }

        # Save to file
        config_path = '/ros2_ws/src/so_100_arm/star_tracker/config/imu_calibration.yaml'
        try:
            with open(config_path, 'w') as f:
                yaml.safe_dump(config, f, default_flow_style=False, indent=2)
            self.get_logger().info(f'Calibration saved to {config_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to save calibration: {e}')


def main():
    rclpy.init()
    node = IMUCalibrationNode()

    print("\n" + "="*60)
    print("IMU CALIBRATION PROCEDURE")
    print("="*60)
    print("This will help calibrate your IMU orientation for star tracking.")
    print("You'll position the telescope at known orientations.")
    print("")
    print("Commands:")
    print("  h = Capture HORIZON pointing (0° elevation)")
    print("  z = Capture ZENITH pointing (90° elevation)")
    print("  n = Capture NORTH pointing (0° azimuth)")
    print("  c = Calculate calibration")
    print("  q = Quit")
    print("="*60)

    import threading
    import time

    # Start ROS spinning in background thread
    def spin_thread():
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

    spinner = threading.Thread(target=spin_thread, daemon=True)
    spinner.start()

    # Wait a moment for initial data
    time.sleep(1.0)

    try:
        while rclpy.ok():
            # Simple command interface
            try:
                cmd = input("\nEnter command (h/z/n/c/q): ").lower().strip()

                if cmd == 'h':
                    node.capture_calibration_point('HORIZON', 0.0, 0.0)
                elif cmd == 'z':
                    node.capture_calibration_point('ZENITH', 90.0, 0.0)
                elif cmd == 'n':
                    node.capture_calibration_point('NORTH', 0.0, 0.0)
                elif cmd == 'c':
                    node.calculate_calibration()
                elif cmd == 'q':
                    break
                else:
                    print("Invalid command. Use h/z/n/c/q")

            except (EOFError, KeyboardInterrupt):
                break

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
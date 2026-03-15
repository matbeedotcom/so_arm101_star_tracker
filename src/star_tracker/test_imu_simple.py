#!/usr/bin/env python3
"""
Simplified IMU Verification Test
Uses direct joint trajectory commands instead of MoveIt2 planning
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
import numpy as np
import time

class SimpleIMUTest(Node):
    def __init__(self):
        super().__init__('simple_imu_test')

        # IMU data subscription
        self.euler_sub = self.create_subscription(
            Vector3, '/imu/euler', self.euler_callback, 10
        )

        # Joint state subscription
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )

        # Joint trajectory action client
        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory, '/so_100_arm_controller/follow_joint_trajectory'
        )

        self.current_euler = None
        self.current_joints = None
        self.test_results = []

        # Simple test poses
        self.test_poses = {
            'current_position': {
                'description': 'Current arm position',
                'joints': None,  # Will be filled with current position
                'expected_altitude': None,  # Will calculate from current IMU
                'expected_azimuth': None
            },
            'small_move_up': {
                'description': 'Move shoulder pitch up by 10°',
                'joints': None,  # Will be current + [0, -0.175, 0, 0.175, 0]
                'expected_altitude': None,  # Should increase by ~10°
                'expected_azimuth': None
            },
            'small_move_down': {
                'description': 'Move shoulder pitch down by 10°',
                'joints': None,  # Will be current + [0, +0.175, 0, -0.175, 0]
                'expected_altitude': None,  # Should decrease by ~10°
                'expected_azimuth': None
            }
        }

        self.get_logger().info('Simple IMU Test initialized')

    def euler_callback(self, msg):
        """Store current IMU euler angles."""
        self.current_euler = {
            'heading': np.degrees(msg.x),
            'roll': np.degrees(msg.y),
            'pitch': np.degrees(msg.z)
        }

    def joint_callback(self, msg):
        """Store current joint positions."""
        if len(msg.position) >= 5:
            self.current_joints = list(msg.position[:5])

    def calculate_telescope_pointing(self, euler_angles):
        """Convert IMU readings to telescope pointing using current calibration."""
        imu_horizon_pitch = -95.5
        imu_zenith_pitch = 175.0
        yaw_offset_deg = -288.2

        # Calculate altitude from pitch
        imu_pitch = euler_angles['pitch']
        altitude = (imu_pitch - imu_horizon_pitch) * (90.0 / (imu_zenith_pitch - imu_horizon_pitch))

        # Calculate azimuth from heading
        corrected_heading = euler_angles['heading'] + yaw_offset_deg
        if corrected_heading < 0:
            corrected_heading += 360.0
        elif corrected_heading >= 360:
            corrected_heading -= 360.0
        azimuth = corrected_heading

        return altitude, azimuth

    def wait_for_action_server(self):
        """Wait for joint trajectory action server."""
        self.get_logger().info('Waiting for joint trajectory action server...')
        if not self.trajectory_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Joint trajectory action server not available!')
            return False
        self.get_logger().info('Joint trajectory action server available')
        return True

    def move_to_joint_positions(self, joint_positions, duration=3.0):
        """Move arm to specified joint positions."""
        if self.current_joints is None:
            self.get_logger().error('No current joint positions available')
            return False

        # Create trajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'Shoulder_Rotation', 'Shoulder_Pitch', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll'
        ]

        # Add current position as starting point
        start_point = JointTrajectoryPoint()
        start_point.positions = self.current_joints
        start_point.time_from_start.sec = 0
        start_point.time_from_start.nanosec = 0
        trajectory.points.append(start_point)

        # Add target position
        target_point = JointTrajectoryPoint()
        target_point.positions = joint_positions
        target_point.time_from_start.sec = int(duration)
        target_point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        trajectory.points.append(target_point)

        # Create goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        # Send goal
        self.get_logger().info(f'Moving to: {[f"{j:.3f}" for j in joint_positions]}')
        future = self.trajectory_client.send_goal_async(goal)

        # Wait for acceptance
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            self.get_logger().error('Failed to send trajectory goal')
            return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal rejected')
            return False

        self.get_logger().info('Trajectory goal accepted, waiting for completion...')

        # Wait for execution
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 2.0)

        if result_future.result() is None:
            self.get_logger().error('Failed to get trajectory result')
            return False

        result = result_future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('Trajectory completed successfully')
            return True
        else:
            self.get_logger().error(f'Trajectory failed with error: {result.error_code}')
            return False

    def get_stable_imu_reading(self, duration=2.0):
        """Get stable IMU reading."""
        self.get_logger().info(f'Collecting IMU readings for {duration}s...')

        readings = []
        start_time = time.time()

        while (time.time() - start_time) < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_euler is not None:
                readings.append((
                    self.current_euler['heading'],
                    self.current_euler['roll'],
                    self.current_euler['pitch']
                ))
            time.sleep(0.1)

        if not readings:
            self.get_logger().error('No IMU readings collected')
            return None

        # Return average
        readings_array = np.array(readings)
        avg_readings = np.mean(readings_array, axis=0)
        std_devs = np.std(readings_array, axis=0)

        self.get_logger().info(f'IMU averages: H={avg_readings[0]:.1f}° R={avg_readings[1]:.1f}° P={avg_readings[2]:.1f}°')
        self.get_logger().info(f'IMU std devs: H={std_devs[0]:.1f}° R={std_devs[1]:.1f}° P={std_devs[2]:.1f}°')

        return {
            'heading': avg_readings[0],
            'roll': avg_readings[1],
            'pitch': avg_readings[2]
        }

    def run_tests(self):
        """Run simplified IMU tests."""
        self.get_logger().info('Starting Simple IMU Verification Tests')
        self.get_logger().info('=' * 60)

        if not self.wait_for_action_server():
            return False

        # Wait for initial data
        self.get_logger().info('Waiting for initial data...')
        timeout = time.time() + 10.0
        while (self.current_euler is None or self.current_joints is None) and time.time() < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.current_euler is None or self.current_joints is None:
            self.get_logger().error('Failed to get initial data')
            return False

        # Test 1: Current position
        self.get_logger().info('\n--- TEST 1: Current Position Baseline ---')
        baseline_imu = self.get_stable_imu_reading()
        if baseline_imu is None:
            return False

        baseline_alt, baseline_az = self.calculate_telescope_pointing(baseline_imu)
        self.get_logger().info(f'Current telescope pointing: Alt={baseline_alt:.1f}°, Az={baseline_az:.1f}°')

        # Test 2: Move up by 10°
        self.get_logger().info('\n--- TEST 2: Move Shoulder Pitch UP by 10° ---')
        up_joints = self.current_joints.copy()
        up_joints[1] -= np.radians(10)  # Shoulder pitch up
        up_joints[3] += np.radians(10)  # Wrist pitch compensation

        if not self.move_to_joint_positions(up_joints):
            self.get_logger().error('Failed to move up')
            return False

        time.sleep(1.0)  # Let arm settle

        up_imu = self.get_stable_imu_reading()
        if up_imu is None:
            return False

        up_alt, up_az = self.calculate_telescope_pointing(up_imu)
        alt_change_up = up_alt - baseline_alt

        self.get_logger().info(f'After moving UP: Alt={up_alt:.1f}°, Az={up_az:.1f}°')
        self.get_logger().info(f'Altitude change: {alt_change_up:.1f}° (expected: ~+10°)')

        # Test 3: Move down by 20° (10° below baseline)
        self.get_logger().info('\n--- TEST 3: Move Shoulder Pitch DOWN by 20° from UP position ---')
        down_joints = up_joints.copy()
        down_joints[1] += np.radians(20)  # Shoulder pitch down by 20°
        down_joints[3] -= np.radians(20)  # Wrist pitch compensation

        if not self.move_to_joint_positions(down_joints):
            self.get_logger().error('Failed to move down')
            return False

        time.sleep(1.0)  # Let arm settle

        down_imu = self.get_stable_imu_reading()
        if down_imu is None:
            return False

        down_alt, down_az = self.calculate_telescope_pointing(down_imu)
        alt_change_down = down_alt - up_alt

        self.get_logger().info(f'After moving DOWN: Alt={down_alt:.1f}°, Az={down_az:.1f}°')
        self.get_logger().info(f'Altitude change: {alt_change_down:.1f}° (expected: ~-20°)')

        # Analysis
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('ANALYSIS')
        self.get_logger().info('=' * 60)

        self.get_logger().info(f'Baseline altitude: {baseline_alt:.1f}°')
        self.get_logger().info(f'After +10° joint move: {up_alt:.1f}° (change: {alt_change_up:.1f}°)')
        self.get_logger().info(f'After -20° joint move: {down_alt:.1f}° (change: {alt_change_down:.1f}°)')

        # Check if movement direction is correct
        if alt_change_up > 5.0:
            self.get_logger().info('✓ GOOD: Moving shoulder pitch UP increases altitude')
        elif alt_change_up < -5.0:
            self.get_logger().warn('✗ INVERTED: Moving shoulder pitch UP decreases altitude!')
        else:
            self.get_logger().warn('? UNCLEAR: Small altitude change, may need larger movement')

        if alt_change_down < -10.0:
            self.get_logger().info('✓ GOOD: Moving shoulder pitch DOWN decreases altitude')
        elif alt_change_down > 10.0:
            self.get_logger().warn('✗ INVERTED: Moving shoulder pitch DOWN increases altitude!')
        else:
            self.get_logger().warn('? UNCLEAR: Small altitude change, may need larger movement')

        # Return to baseline
        self.get_logger().info('\n--- Returning to baseline position ---')
        self.move_to_joint_positions(self.current_joints)

        return True


def main():
    rclpy.init()
    node = SimpleIMUTest()

    try:
        success = node.run_tests()
        if success:
            node.get_logger().info('Tests completed successfully')
        else:
            node.get_logger().error('Tests failed')
    except KeyboardInterrupt:
        node.get_logger().info('Test interrupted')
    except Exception as e:
        node.get_logger().error(f'Test failed: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
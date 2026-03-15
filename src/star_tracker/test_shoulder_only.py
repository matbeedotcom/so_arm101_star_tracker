#!/usr/bin/env python3
"""
Test ONLY shoulder pitch movement to isolate the joint direction
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

class ShoulderOnlyTest(Node):
    def __init__(self):
        super().__init__('shoulder_only_test')

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

        self.get_logger().info('Shoulder Only Test initialized')

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

    def wait_for_action_server(self):
        """Wait for joint trajectory action server."""
        self.get_logger().info('Waiting for joint trajectory action server...')
        if not self.trajectory_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Joint trajectory action server not available!')
            return False
        self.get_logger().info('Joint trajectory action server available')
        return True

    def move_shoulder_pitch(self, delta_radians, duration=2.0):
        """Move ONLY the shoulder pitch by delta_radians."""
        if self.current_joints is None:
            self.get_logger().error('No current joint positions available')
            return False

        # Create new target with only shoulder pitch changed
        target_joints = self.current_joints.copy()
        target_joints[1] += delta_radians  # Only change shoulder pitch
        # Keep all other joints the same

        # Create trajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'Shoulder_Rotation', 'Shoulder_Pitch', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll'
        ]

        # Add target position
        target_point = JointTrajectoryPoint()
        target_point.positions = target_joints
        target_point.time_from_start.sec = int(duration)
        target_point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        trajectory.points.append(target_point)

        # Create goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        # Send goal
        delta_degrees = np.degrees(delta_radians)
        self.get_logger().info(f'Moving shoulder pitch by {delta_degrees:.1f}°')
        self.get_logger().info(f'From: {[f"{j:.3f}" for j in self.current_joints]}')
        self.get_logger().info(f'To:   {[f"{j:.3f}" for j in target_joints]}')

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

    def get_imu_reading(self):
        """Get current IMU reading."""
        for _ in range(10):  # Try a few times
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_euler is not None:
                return self.current_euler.copy()
        return None

    def run_test(self):
        """Run shoulder pitch test."""
        self.get_logger().info('Starting Shoulder Pitch Direction Test')
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

        # Get baseline
        baseline_imu = self.get_imu_reading()
        if baseline_imu is None:
            self.get_logger().error('Failed to get baseline IMU reading')
            return False

        baseline_joints = self.current_joints.copy()

        self.get_logger().info('\n--- BASELINE ---')
        self.get_logger().info(f'IMU: H={baseline_imu["heading"]:.1f}° R={baseline_imu["roll"]:.1f}° P={baseline_imu["pitch"]:.1f}°')
        self.get_logger().info(f'Shoulder Pitch: {baseline_joints[1]:.3f} rad ({np.degrees(baseline_joints[1]):.1f}°)')

        # Test 1: Move shoulder pitch DOWN (positive direction)
        self.get_logger().info('\n--- TEST 1: Move Shoulder Pitch DOWN (+15°) ---')

        if not self.move_shoulder_pitch(np.radians(15)):  # +15° = DOWN
            return False

        time.sleep(1.0)  # Let settle

        down_imu = self.get_imu_reading()
        if down_imu is None:
            return False

        pitch_change_1 = down_imu['pitch'] - baseline_imu['pitch']
        self.get_logger().info(f'IMU: H={down_imu["heading"]:.1f}° R={down_imu["roll"]:.1f}° P={down_imu["pitch"]:.1f}°')
        self.get_logger().info(f'IMU Pitch change: {pitch_change_1:.1f}° (joint moved +15° DOWN)')

        # Test 2: Move shoulder pitch UP (negative direction) by 30°
        self.get_logger().info('\n--- TEST 2: Move Shoulder Pitch UP (-30°) from current position ---')

        if not self.move_shoulder_pitch(np.radians(-30)):  # -30° = UP
            return False

        time.sleep(1.0)  # Let settle

        up_imu = self.get_imu_reading()
        if up_imu is None:
            return False

        pitch_change_2 = up_imu['pitch'] - down_imu['pitch']
        self.get_logger().info(f'IMU: H={up_imu["heading"]:.1f}° R={up_imu["roll"]:.1f}° P={up_imu["pitch"]:.1f}°')
        self.get_logger().info(f'IMU Pitch change: {pitch_change_2:.1f}° (joint moved -30° UP)')

        # Analysis
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('ANALYSIS - Joint Direction Mapping')
        self.get_logger().info('=' * 60)

        self.get_logger().info(f'Joint +15° (DOWN): IMU pitch changed by {pitch_change_1:.1f}°')
        self.get_logger().info(f'Joint -30° (UP):   IMU pitch changed by {pitch_change_2:.1f}°')

        if pitch_change_1 > 5:
            self.get_logger().info('✓ Moving joint DOWN (+) increases IMU pitch')
        elif pitch_change_1 < -5:
            self.get_logger().info('✗ Moving joint DOWN (+) decreases IMU pitch')
        else:
            self.get_logger().warn('? Small change - unclear')

        if pitch_change_2 < -10:
            self.get_logger().info('✓ Moving joint UP (-) decreases IMU pitch')
        elif pitch_change_2 > 10:
            self.get_logger().info('✗ Moving joint UP (-) increases IMU pitch')
        else:
            self.get_logger().warn('? Small change - unclear')

        # Return to baseline
        self.get_logger().info('\n--- Returning to baseline ---')
        if not self.move_shoulder_pitch(np.radians(15)):  # Move back +15° to return
            return False

        return True


def main():
    rclpy.init()
    node = ShoulderOnlyTest()

    try:
        success = node.run_test()
        if success:
            node.get_logger().info('Test completed successfully')
        else:
            node.get_logger().error('Test failed')
    except KeyboardInterrupt:
        node.get_logger().info('Test interrupted')
    except Exception as e:
        node.get_logger().error(f'Test failed: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
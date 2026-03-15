#!/usr/bin/env python3
"""
Automated IMU Verification Test Suite
Uses MoveIt2 to move arm to known orientations and verify IMU readings
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Pose, PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint
)
from rclpy.action import ActionClient
import numpy as np
import time

class IMUVerificationTest(Node):
    def __init__(self):
        super().__init__('imu_verification_test')

        # IMU data subscription
        self.euler_sub = self.create_subscription(
            Vector3, '/imu/euler', self.euler_callback, 10
        )

        # Joint state subscription
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )

        # MoveIt2 action client
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')

        self.current_euler = None
        self.current_joints = None
        self.test_results = []

        # Test poses - joint configurations for different orientations
        self.test_poses = {
            'horizon_north': {
                'description': 'Pointing at horizon toward North (0° elevation, 0° azimuth)',
                'joints': [0.0, 0.0, 0.0, 0.0, 0.0],  # Shoulder_Rotation, Shoulder_Pitch, Elbow, Wrist_Pitch, Wrist_Roll
                'expected_altitude': 0.0,
                'expected_azimuth': 0.0
            },
            'zenith': {
                'description': 'Pointing straight up (90° elevation)',
                'joints': [0.0, -np.pi/2, 0.0, np.pi/2, 0.0],  # Point up
                'expected_altitude': 90.0,
                'expected_azimuth': 0.0  # Azimuth undefined at zenith
            },
            'horizon_east': {
                'description': 'Pointing at horizon toward East (0° elevation, 90° azimuth)',
                'joints': [np.pi/2, 0.0, 0.0, 0.0, 0.0],  # Rotate 90° to point East
                'expected_altitude': 0.0,
                'expected_azimuth': 90.0
            },
            'horizon_south': {
                'description': 'Pointing at horizon toward South (0° elevation, 180° azimuth)',
                'joints': [np.pi, 0.0, 0.0, 0.0, 0.0],  # Rotate 180° to point South
                'expected_altitude': 0.0,
                'expected_azimuth': 180.0
            },
            'horizon_west': {
                'description': 'Pointing at horizon toward West (0° elevation, 270° azimuth)',
                'joints': [-np.pi/2, 0.0, 0.0, 0.0, 0.0],  # Rotate -90° to point West
                'expected_altitude': 0.0,
                'expected_azimuth': 270.0
            },
            'mid_elevation_north': {
                'description': 'Pointing 45° up toward North',
                'joints': [0.0, -np.pi/4, 0.0, np.pi/4, 0.0],  # 45° elevation
                'expected_altitude': 45.0,
                'expected_azimuth': 0.0
            }
        }

        self.get_logger().info('IMU Verification Test Suite initialized')
        self.get_logger().info(f'Will test {len(self.test_poses)} different orientations')

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
        """Wait for MoveIt2 action server to be available."""
        self.get_logger().info('Waiting for MoveIt2 action server...')
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveIt2 action server not available!')
            return False
        self.get_logger().info('MoveIt2 action server available')
        return True

    def move_to_joint_positions(self, joint_positions, timeout=10.0):
        """Move arm to specified joint positions using MoveIt2."""

        # Create motion plan request
        req = MotionPlanRequest()
        req.group_name = 'so_100_arm'  # Planning group name
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0

        # Set joint constraints
        joint_names = ['Shoulder_Rotation', 'Shoulder_Pitch', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll']

        # Create a single constraint with all joint constraints
        goal_constraint = Constraints()

        for name, position in zip(joint_names, joint_positions):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = position
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            goal_constraint.joint_constraints.append(joint_constraint)

        req.goal_constraints.append(goal_constraint)

        # Create MoveGroup goal
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False  # Execute the plan

        # Send goal
        self.get_logger().info(f'Moving to joint positions: {[f"{j:.2f}" for j in joint_positions]}')
        future = self.move_group_client.send_goal_async(goal)

        # Wait for result
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

        if future.result() is not None:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info('Motion goal accepted')

                # Wait for execution to complete
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)

                if result_future.result() is not None:
                    result = result_future.result().result
                    if result.error_code.val == result.error_code.SUCCESS:
                        self.get_logger().info('Motion completed successfully')
                        return True
                    else:
                        self.get_logger().error(f'Motion failed with error: {result.error_code.val}')
                        return False
                else:
                    self.get_logger().error('Failed to get motion result')
                    return False
            else:
                self.get_logger().error('Motion goal rejected')
                return False
        else:
            self.get_logger().error('Failed to send motion goal')
            return False

    def wait_for_stable_readings(self, duration=3.0, tolerance=1.0):
        """Wait for IMU readings to stabilize."""
        self.get_logger().info(f'Waiting {duration}s for stable IMU readings...')

        start_time = time.time()
        readings = []

        while (time.time() - start_time) < duration:
            # Spin once to get fresh data
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.current_euler is not None:
                readings.append((
                    self.current_euler['heading'],
                    self.current_euler['roll'],
                    self.current_euler['pitch']
                ))
            time.sleep(0.1)

        if not readings:
            self.get_logger().error('No IMU readings received')
            return None

        # Calculate standard deviation to check stability
        readings_array = np.array(readings)
        std_devs = np.std(readings_array, axis=0)

        self.get_logger().info(f'IMU stability - H_std: {std_devs[0]:.2f}°, R_std: {std_devs[1]:.2f}°, P_std: {std_devs[2]:.2f}°')

        if np.max(std_devs) > tolerance:
            self.get_logger().warn('IMU readings not stable - continuing anyway')

        # Return average readings
        avg_readings = np.mean(readings_array, axis=0)
        return {
            'heading': avg_readings[0],
            'roll': avg_readings[1],
            'pitch': avg_readings[2]
        }

    def calculate_telescope_pointing(self, euler_angles):
        """Convert IMU readings to telescope pointing using current calibration."""
        # This uses the same logic as the star tracker
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

    def run_test_pose(self, pose_name, pose_data):
        """Run test for a single pose."""
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Testing: {pose_name}')
        self.get_logger().info(f'Description: {pose_data["description"]}')
        self.get_logger().info(f'Expected: Alt={pose_data["expected_altitude"]:.1f}°, Az={pose_data["expected_azimuth"]:.1f}°')

        # Move to target pose
        success = self.move_to_joint_positions(pose_data['joints'])
        if not success:
            self.get_logger().error(f'Failed to move to pose: {pose_name}')
            return False

        # Wait for movement to settle
        time.sleep(2.0)

        # Get stable IMU readings
        stable_euler = self.wait_for_stable_readings()
        if stable_euler is None:
            self.get_logger().error('Failed to get stable IMU readings')
            return False

        # Convert to telescope pointing
        calculated_alt, calculated_az = self.calculate_telescope_pointing(stable_euler)

        # Calculate errors
        alt_error = calculated_alt - pose_data['expected_altitude']
        az_error = calculated_az - pose_data['expected_azimuth']

        # Handle azimuth wrap-around
        if abs(az_error) > 180:
            if az_error > 0:
                az_error -= 360
            else:
                az_error += 360

        # Store results
        result = {
            'pose_name': pose_name,
            'description': pose_data['description'],
            'expected_alt': pose_data['expected_altitude'],
            'expected_az': pose_data['expected_azimuth'],
            'imu_heading': stable_euler['heading'],
            'imu_roll': stable_euler['roll'],
            'imu_pitch': stable_euler['pitch'],
            'calculated_alt': calculated_alt,
            'calculated_az': calculated_az,
            'alt_error': alt_error,
            'az_error': az_error,
            'success': abs(alt_error) < 10.0 and abs(az_error) < 10.0  # 10° tolerance
        }

        self.test_results.append(result)

        # Log results
        self.get_logger().info(f'IMU readings: H={stable_euler["heading"]:.1f}°, R={stable_euler["roll"]:.1f}°, P={stable_euler["pitch"]:.1f}°')
        self.get_logger().info(f'Calculated: Alt={calculated_alt:.1f}°, Az={calculated_az:.1f}°')
        self.get_logger().info(f'Errors: Alt={alt_error:.1f}°, Az={az_error:.1f}°')

        if result['success']:
            self.get_logger().info('✓ PASS - Within tolerance')
        else:
            self.get_logger().warn('✗ FAIL - Outside tolerance')

        return result['success']

    def run_all_tests(self):
        """Run all verification tests."""
        self.get_logger().info('Starting IMU Verification Test Suite')
        self.get_logger().info('=' * 60)

        if not self.wait_for_action_server():
            return False

        # Wait for initial data
        self.get_logger().info('Waiting for initial IMU and joint data...')
        timeout = time.time() + 10.0
        while (self.current_euler is None or self.current_joints is None) and time.time() < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.current_euler is None or self.current_joints is None:
            self.get_logger().error('Failed to receive initial data')
            return False

        # Run each test pose
        passed_tests = 0
        for pose_name, pose_data in self.test_poses.items():
            success = self.run_test_pose(pose_name, pose_data)
            if success:
                passed_tests += 1

            # Small delay between tests
            time.sleep(1.0)

        # Generate summary report
        self.generate_report()

        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Test Summary: {passed_tests}/{len(self.test_poses)} tests passed')

        return passed_tests == len(self.test_poses)

    def generate_report(self):
        """Generate detailed test report."""
        self.get_logger().info('\n' + '=' * 80)
        self.get_logger().info('IMU VERIFICATION TEST REPORT')
        self.get_logger().info('=' * 80)

        for result in self.test_results:
            status = '✓ PASS' if result['success'] else '✗ FAIL'
            self.get_logger().info(f"\n{result['pose_name']}: {status}")
            self.get_logger().info(f"  Description: {result['description']}")
            self.get_logger().info(f"  Expected:    Alt={result['expected_alt']:6.1f}°  Az={result['expected_az']:6.1f}°")
            self.get_logger().info(f"  Calculated:  Alt={result['calculated_alt']:6.1f}°  Az={result['calculated_az']:6.1f}°")
            self.get_logger().info(f"  Error:       Alt={result['alt_error']:6.1f}°  Az={result['az_error']:6.1f}°")
            self.get_logger().info(f"  IMU Raw:     H={result['imu_heading']:6.1f}°  R={result['imu_roll']:6.1f}°  P={result['imu_pitch']:6.1f}°")

        # Calculate calibration recommendations
        self.analyze_calibration_errors()

    def analyze_calibration_errors(self):
        """Analyze systematic errors and suggest calibration improvements."""
        if len(self.test_results) < 2:
            return

        alt_errors = [r['alt_error'] for r in self.test_results]
        az_errors = [r['az_error'] for r in self.test_results]

        avg_alt_error = np.mean(alt_errors)
        avg_az_error = np.mean(az_errors)

        self.get_logger().info('\n' + '-' * 60)
        self.get_logger().info('CALIBRATION ANALYSIS')
        self.get_logger().info('-' * 60)
        self.get_logger().info(f'Average altitude error: {avg_alt_error:.2f}°')
        self.get_logger().info(f'Average azimuth error:  {avg_az_error:.2f}°')

        if abs(avg_alt_error) > 2.0:
            self.get_logger().info(f'Recommendation: Adjust pitch calibration by {-avg_alt_error:.1f}°')

        if abs(avg_az_error) > 2.0:
            self.get_logger().info(f'Recommendation: Adjust yaw offset by {-avg_az_error:.1f}°')

        self.get_logger().info('-' * 60)


def main():
    rclpy.init()
    node = IMUVerificationTest()

    try:
        # Run the test suite
        success = node.run_all_tests()

        if success:
            node.get_logger().info('All tests passed! IMU calibration is good.')
        else:
            node.get_logger().warn('Some tests failed. Check calibration values.')

    except KeyboardInterrupt:
        node.get_logger().info('Test interrupted by user')
    except Exception as e:
        node.get_logger().error(f'Test failed with exception: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
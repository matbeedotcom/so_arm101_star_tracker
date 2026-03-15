#!/usr/bin/env python3
"""
Interactive test script for auto calibration verification.
Tests that coordinate mapping works correctly with user verification.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, MagneticField
from geometry_msgs.msg import Vector3
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
import numpy as np
import time

try:
    from star_tracker.auto_calibration import AutoCalibration
    from star_tracker.coordinate_transform import CoordinateTransform
except ImportError:
    import sys
    import os
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from auto_calibration import AutoCalibration
    from coordinate_transform import CoordinateTransform


class AutoCalibrationTester(Node):
    """Interactive tester for auto calibration system."""

    def __init__(self):
        super().__init__('auto_calibration_tester')

        # Initialize auto calibration
        self.auto_calibration = AutoCalibration()
        self.coordinate_transform = CoordinateTransform()

        # Set magnetic declination (adjust for your location)
        self.auto_calibration.set_magnetic_declination(-14.2)

        # Data storage
        self.current_joints = [0.0] * 5
        self.current_imu_euler = None
        self.current_imu_accel = None
        self.current_imu_mag = None
        self.imu_update_count = 0  # Track IMU updates

        # Joint names
        self.joint_names = [
            'Shoulder_Rotation',
            'Shoulder_Pitch',
            'Elbow',
            'Wrist_Pitch',
            'Wrist_Roll'
        ]

        # Action client for trajectory execution
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/so_100_arm_controller/follow_joint_trajectory'
        )

        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10
        )
        self.euler_sub = self.create_subscription(
            Vector3, 'imu/euler', self.euler_callback, 10
        )
        self.mag_sub = self.create_subscription(
            MagneticField, 'imu/mag', self.magnetometer_callback, 10
        )

        print("Auto Calibration Tester initialized!")
        print("Waiting for IMU data...")

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def wait_for_fresh_imu_data(self, timeout_sec=3.0):
        """Wait for fresh IMU data after movement."""
        print("📡 Waiting for fresh IMU data...", end="", flush=True)

        start_count = self.imu_update_count
        start_time = time.time()

        while (self.imu_update_count <= start_count and
               time.time() - start_time < timeout_sec):
            rclpy.spin_once(self, timeout_sec=0.1)
            print(".", end="", flush=True)
            time.sleep(0.1)

        print(" ✅")
        return self.imu_update_count > start_count

    def joint_callback(self, msg):
        """Update current joint positions."""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_joints[i] = msg.position[idx]

    def imu_callback(self, msg):
        """Store IMU data."""
        self.current_imu_accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

    def euler_callback(self, msg):
        """Store euler angles."""
        self.current_imu_euler = np.array([msg.x, msg.y, msg.z])
        self.imu_update_count += 1  # Increment update counter

    def magnetometer_callback(self, msg):
        """Store magnetometer data."""
        self.current_imu_mag = np.array([
            msg.magnetic_field.x,
            msg.magnetic_field.y,
            msg.magnetic_field.z
        ])

    def check_auto_calibration(self):
        """Run auto calibration if all data is available."""
        if (self.current_imu_euler is None or
            self.current_imu_accel is None or
            self.current_imu_mag is None):
            return False

        try:
            success = self.auto_calibration.calibrate_from_imu_data(
                self.current_imu_euler,
                self.current_imu_accel,
                self.current_imu_mag
            )

            if success:
                print("✅ Auto calibration completed!")
                self.auto_calibration.save_calibration('test_auto_calibration.json')
                print("📁 Calibration saved to test_auto_calibration.json")
                return True
            else:
                print("❌ Auto calibration failed")
                return False

        except Exception as e:
            print(f"❌ Auto calibration error: {e}")
            return False

    def move_to_position(self, joint_positions, duration=3.0):
        """Move arm to specified joint positions."""
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.velocities = [0.0] * len(joint_positions)
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)

        trajectory.points = [point]

        if self.trajectory_client.wait_for_server(timeout_sec=2.0):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory

            future = self.trajectory_client.send_goal_async(goal)
            print(f"🤖 Moving to: {[f'{np.degrees(j):.1f}°' for j in joint_positions]}")
            return True
        else:
            print("❌ Failed to connect to trajectory controller")
            return False

    def test_coordinate_directions(self):
        """Test basic coordinate directions with user verification."""

        print("\n" + "="*60)
        print("🧭 COORDINATE DIRECTION VERIFICATION")
        print("="*60)

        if not self.auto_calibration.is_calibrated:
            print("❌ Auto calibration not completed yet")
            return False

        test_positions = [
            {
                'name': 'Home Position',
                'altitude': 0.0,
                'azimuth': 0.0,
                'description': 'Should point level toward North'
            },
            {
                'name': 'Horizon East',
                'altitude': 0.0,
                'azimuth': np.pi/2,
                'description': 'Should point level toward East'
            },
            {
                'name': 'Zenith (Straight Up)',
                'altitude': np.pi/2,
                'azimuth': 0.0,
                'description': 'Should point straight up at the sky'
            }
        ]

        for i, test_pos in enumerate(test_positions, 1):
            print(f"\n--- Test {i}/{len(test_positions)}: {test_pos['name']} ---")
            print(f"📍 Target: Alt={np.degrees(test_pos['altitude']):.1f}°, Az={np.degrees(test_pos['azimuth']):.1f}°")
            print(f"📝 Expected: {test_pos['description']}")

            # Calculate joint positions using coordinate transform
            # For now, use simple direct mapping but with the altitude fix from auto calibration
            # We need to invert the altitude mapping to match the IMU coordinate system

            # Convert from astronomical coordinates to joint positions
            # Azimuth controls shoulder rotation directly
            shoulder_rotation = test_pos['azimuth']

            # ALTITUDE FIX: For altitude, we need the shoulder to physically point up
            # When we want alt=90° (zenith), shoulder should pitch up 90°
            # When we want alt=0° (horizon), shoulder should be level (0°)
            # This is the PHYSICAL movement - let the auto_calibration handle IMU coordinate conversion
            shoulder_pitch = test_pos['altitude']

            elbow = 0.0
            wrist_pitch = -shoulder_pitch  # Compensate to keep end-effector pointing correctly
            wrist_roll = 0.0

            joint_positions = [shoulder_rotation, shoulder_pitch, elbow, wrist_pitch, wrist_roll]

            # Move to position
            if not self.move_to_position(joint_positions):
                continue

            # Wait for movement to complete
            print("⏳ Moving to position...")
            time.sleep(4.0)

            # Wait for fresh IMU reading after movement
            if not self.wait_for_fresh_imu_data():
                print("⚠️  Timeout waiting for fresh IMU data - using last reading")

            # Show current IMU readings vs expected
            print("\n📊 POSITION ANALYSIS:")
            print("-" * 40)
            if self.current_imu_euler is not None:
                current_alt, current_az = self.auto_calibration.imu_to_altaz(self.current_imu_euler)

                print(f"🎯 Target:     Alt={np.degrees(test_pos['altitude']):6.1f}°, Az={np.degrees(test_pos['azimuth']):6.1f}°")
                print(f"📡 IMU reads:  Alt={np.degrees(current_alt):6.1f}°, Az={np.degrees(current_az):6.1f}°")
                print(f"📐 Error:      Alt={np.degrees(current_alt - test_pos['altitude']):6.1f}°, Az={np.degrees(self.normalize_angle(current_az - test_pos['azimuth'])):6.1f}°")
                print(f"🤖 Joints:     {[f'{np.degrees(j):5.1f}°' for j in joint_positions]}")
                print(f"🧭 Raw IMU:    Roll={np.degrees(self.current_imu_euler[0]):5.1f}°, Pitch={np.degrees(self.current_imu_euler[1]):5.1f}°, Yaw={np.degrees(self.current_imu_euler[2]):5.1f}°")
            else:
                print("❌ No IMU data available")
            print("-" * 40)

            # Get user verification
            while True:
                response = input(f"✅ Does the arm point correctly? ({test_pos['description']}) [y/n/s(kip)]: ").lower().strip()

                if response in ['y', 'yes']:
                    print("✅ Position verified correct!")
                    # Add this as a calibration point to improve auto-calibration
                    if self.current_imu_euler is not None:
                        current_alt, current_az = self.auto_calibration.imu_to_altaz(self.current_imu_euler)
                        print(f"📚 Adding calibration point: Target Alt/Az vs IMU reading")

                        # Store the correction needed
                        alt_error = test_pos['altitude'] - current_alt
                        az_error = test_pos['azimuth'] - current_az

                        print(f"   Altitude correction: {np.degrees(alt_error):+.1f}°")
                        print(f"   Azimuth correction: {np.degrees(az_error):+.1f}°")
                    break
                elif response in ['n', 'no']:
                    print("❌ Position incorrect - coordinate mapping needs fixing")

                    # Use this data to improve auto-calibration
                    if self.current_imu_euler is not None:
                        current_alt, current_az = self.auto_calibration.imu_to_altaz(self.current_imu_euler)

                        # Calculate the correction needed
                        alt_correction = test_pos['altitude'] - current_alt
                        az_correction = test_pos['azimuth'] - current_az

                        print(f"📚 Learning from error:")
                        print(f"   Need altitude correction: {np.degrees(alt_correction):+.1f}°")
                        print(f"   Need azimuth correction: {np.degrees(az_correction):+.1f}°")

                        # Update auto-calibration offsets
                        if hasattr(self.auto_calibration, 'imu_to_coord_transform') and self.auto_calibration.imu_to_coord_transform:
                            self.auto_calibration.imu_to_coord_transform['altitude_offset'] += alt_correction
                            self.auto_calibration.imu_to_coord_transform['azimuth_offset'] += az_correction
                            print(f"✅ Auto-calibration updated!")
                        else:
                            print("⚠️  Auto-calibration not initialized properly")

                    # Show current IMU reading vs expected
                    if self.current_imu_euler is not None:
                        current_alt, current_az = self.auto_calibration.imu_to_altaz(self.current_imu_euler)
                        print(f"🎯 Target Alt/Az: {np.degrees(test_pos['altitude']):.1f}°, {np.degrees(test_pos['azimuth']):.1f}°")
                        print(f"📡 IMU reads Alt/Az: {np.degrees(current_alt):.1f}°, {np.degrees(current_az):.1f}°")
                        print(f"🤖 Joint positions: {[f'{np.degrees(j):.1f}°' for j in joint_positions]}")
                    break
                elif response in ['s', 'skip']:
                    print("⏭️ Skipping this test")
                    break
                else:
                    print("Please enter 'y', 'n', or 's'")

        print(f"\n🏁 Coordinate verification test completed!")
        return True

    def interactive_test(self):
        """Interactive testing interface."""
        print("\n" + "="*60)
        print("🔧 AUTO CALIBRATION TESTER")
        print("="*60)

        # Wait for IMU data
        print("⏳ Waiting for IMU data...")
        while (self.current_imu_euler is None or
               self.current_imu_accel is None or
               self.current_imu_mag is None):
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

        print("📡 IMU data received!")

        # Run auto calibration
        print("\n🧭 Running auto calibration...")
        if not self.check_auto_calibration():
            print("❌ Auto calibration failed - cannot proceed")
            return

        # Show calibration results
        print(f"\n📊 Calibration Results:")
        print(f"   Magnetic declination: {self.auto_calibration.magnetic_declination:.1f}°")
        print(f"   Azimuth offset: {np.degrees(self.auto_calibration.imu_to_coord_transform['azimuth_offset']):.1f}°")
        print(f"   Altitude offset: {np.degrees(self.auto_calibration.imu_to_coord_transform['altitude_offset']):.1f}°")

        while True:
            print("\n" + "-"*40)
            print("📋 TEST MENU:")
            print("1. Test coordinate directions")
            print("2. Move to custom altitude/azimuth")
            print("3. Show current IMU reading")
            print("4. Recalibrate")
            print("5. Exit")

            choice = input("Select option [1-5]: ").strip()

            if choice == '1':
                self.test_coordinate_directions()

            elif choice == '2':
                try:
                    alt_deg = float(input("Enter altitude in degrees (0=horizon, 90=zenith): "))
                    az_deg = float(input("Enter azimuth in degrees (0=North, 90=East): "))

                    altitude = np.radians(alt_deg)
                    azimuth = np.radians(az_deg)

                    # Use the same corrected mapping as in the test
                    shoulder_rotation = azimuth
                    shoulder_pitch = altitude  # Direct mapping - let auto_calibration handle IMU conversion
                    elbow = 0.0
                    wrist_pitch = -shoulder_pitch
                    wrist_roll = 0.0
                    joint_positions = [shoulder_rotation, shoulder_pitch, elbow, wrist_pitch, wrist_roll]

                    self.move_to_position(joint_positions)

                except ValueError:
                    print("❌ Invalid input - please enter numbers")

            elif choice == '3':
                if self.current_imu_euler is not None:
                    current_alt, current_az = self.auto_calibration.imu_to_altaz(self.current_imu_euler)
                    print(f"📡 Current IMU pointing: Alt={np.degrees(current_alt):.1f}°, Az={np.degrees(current_az):.1f}°")
                    print(f"🤖 Current joints: {[f'{np.degrees(j):.1f}°' for j in self.current_joints]}")
                else:
                    print("❌ No IMU data available")

            elif choice == '4':
                print("🔄 Recalibrating...")
                self.check_auto_calibration()

            elif choice == '5':
                print("👋 Exiting...")
                break

            else:
                print("❌ Invalid choice")


def main():
    rclpy.init()
    tester = AutoCalibrationTester()

    # Run ROS2 in background thread
    import threading
    ros_thread = threading.Thread(target=lambda: rclpy.spin(tester))
    ros_thread.daemon = True
    ros_thread.start()

    # Wait for initialization
    time.sleep(2.0)

    try:
        tester.interactive_test()
    except KeyboardInterrupt:
        print("\n👋 Interrupted by user")
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
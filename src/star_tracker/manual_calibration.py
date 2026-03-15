#!/usr/bin/env python3
"""
Manual calibration script for star tracker.
Prompts user to manually move arm to specific positions and records IMU readings.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, MagneticField
from geometry_msgs.msg import Vector3
import numpy as np
import time
import json

class ManualCalibration(Node):
    """Manual calibration for star tracker IMU."""

    def __init__(self):
        super().__init__('manual_calibration')

        # Data storage
        self.current_imu_euler = None
        self.current_imu_accel = None
        self.current_imu_mag = None
        self.current_joints = [0.0] * 5
        self.imu_update_count = 0

        self.calibration_data = {}

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

        print("Manual Calibration initialized!")
        print("Waiting for IMU data...")

    def joint_callback(self, msg):
        """Update current joint positions."""
        joint_names = [
            'Shoulder_Rotation',
            'Shoulder_Pitch',
            'Elbow',
            'Wrist_Pitch',
            'Wrist_Roll'
        ]

        for i, name in enumerate(joint_names):
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
        self.imu_update_count += 1

    def magnetometer_callback(self, msg):
        """Store magnetometer data."""
        self.current_imu_mag = np.array([
            msg.magnetic_field.x,
            msg.magnetic_field.y,
            msg.magnetic_field.z
        ])

    def wait_for_fresh_imu_data(self, timeout_sec=3.0):
        """Wait for fresh IMU data."""
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

    def record_position(self, position_name, target_alt_deg, target_az_deg):
        """Record IMU reading for a manually positioned arm."""
        print(f"\n📍 Recording: {position_name}")
        print(f"   Target: Alt={target_alt_deg}°, Az={target_az_deg}°")

        # Wait for user to position arm
        input(f"🤖 Manually move the arm to point {position_name.lower()}. Press Enter when ready...")

        # Wait for fresh IMU reading
        self.wait_for_fresh_imu_data()

        if self.current_imu_euler is None:
            print("❌ No IMU data available")
            return False

        # Record the data
        self.calibration_data[position_name] = {
            'target_altitude_deg': target_alt_deg,
            'target_azimuth_deg': target_az_deg,
            'target_altitude_rad': np.radians(target_alt_deg),
            'target_azimuth_rad': np.radians(target_az_deg),
            'imu_euler': self.current_imu_euler.copy(),
            'imu_accel': self.current_imu_accel.copy() if self.current_imu_accel is not None else None,
            'imu_mag': self.current_imu_mag.copy() if self.current_imu_mag is not None else None,
            'joint_positions': self.current_joints.copy()
        }

        # Show what was recorded
        print(f"✅ Recorded IMU: Roll={np.degrees(self.current_imu_euler[0]):.1f}°, "
              f"Pitch={np.degrees(self.current_imu_euler[1]):.1f}°, "
              f"Yaw={np.degrees(self.current_imu_euler[2]):.1f}°")
        print(f"   Joint positions: {[f'{np.degrees(j):.1f}°' for j in self.current_joints]}")

        return True

    def calculate_calibration(self):
        """Calculate calibration offsets from recorded data."""
        if len(self.calibration_data) < 2:
            print("❌ Need at least 2 calibration points")
            return False

        print("\n📊 CALIBRATION ANALYSIS:")
        print("=" * 50)

        # Show all recorded positions
        for name, data in self.calibration_data.items():
            target_alt = data['target_altitude_deg']
            target_az = data['target_azimuth_deg']
            imu_euler = data['imu_euler']

            # Apply current auto-calibration formula: altitude = (π/2) - pitch
            imu_alt_deg = np.degrees((np.pi/2) - imu_euler[1])
            imu_az_deg = np.degrees(imu_euler[2])

            alt_error = target_alt - imu_alt_deg
            az_error = target_az - imu_az_deg

            print(f"\n{name}:")
            print(f"  🎯 Target:    Alt={target_alt:6.1f}°, Az={target_az:6.1f}°")
            print(f"  📡 IMU reads: Alt={imu_alt_deg:6.1f}°, Az={imu_az_deg:6.1f}°")
            print(f"  📐 Error:     Alt={alt_error:+6.1f}°, Az={az_error:+6.1f}°")
            print(f"  🧭 Raw IMU:   Roll={np.degrees(imu_euler[0]):5.1f}°, Pitch={np.degrees(imu_euler[1]):5.1f}°, Yaw={np.degrees(imu_euler[2]):5.1f}°")

        # Calculate average corrections
        alt_corrections = []
        az_corrections = []

        for name, data in self.calibration_data.items():
            target_alt = data['target_altitude_deg']
            target_az = data['target_azimuth_deg']
            imu_euler = data['imu_euler']

            imu_alt_deg = np.degrees((np.pi/2) - imu_euler[1])
            imu_az_deg = np.degrees(imu_euler[2])

            alt_corrections.append(target_alt - imu_alt_deg)
            az_corrections.append(target_az - imu_az_deg)

        avg_alt_correction = np.mean(alt_corrections)
        avg_az_correction = np.mean(az_corrections)

        print(f"\n📚 CALCULATED CORRECTIONS:")
        print(f"  Altitude offset: {avg_alt_correction:+.1f}°")
        print(f"  Azimuth offset:  {avg_az_correction:+.1f}°")

        # Save calibration
        calibration_result = {
            'altitude_offset_deg': avg_alt_correction,
            'azimuth_offset_deg': avg_az_correction,
            'altitude_offset_rad': np.radians(avg_alt_correction),
            'azimuth_offset_rad': np.radians(avg_az_correction),
            'calibration_points': self.calibration_data
        }

        with open('manual_calibration_result.json', 'w') as f:
            # Convert numpy arrays to lists for JSON serialization
            def convert_numpy(obj):
                if isinstance(obj, np.ndarray):
                    return obj.tolist()
                elif isinstance(obj, np.float64):
                    return float(obj)
                return obj

            import json
            json.dump(calibration_result, f, indent=2, default=convert_numpy)

        print(f"📁 Calibration saved to manual_calibration_result.json")
        return True

    def run_manual_calibration(self):
        """Run the manual calibration process."""
        print("\n" + "=" * 60)
        print("🔧 MANUAL STAR TRACKER CALIBRATION")
        print("=" * 60)
        print("\nThis will guide you through manually positioning the arm")
        print("and recording IMU readings for calibration.\n")

        # Wait for IMU data
        print("⏳ Waiting for IMU data...")
        while (self.current_imu_euler is None or
               self.current_imu_accel is None or
               self.current_imu_mag is None):
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

        print("📡 IMU data received!")

        # Record calibration positions
        positions = [
            ("Horizon North", 0, 0),
            ("Horizon East", 0, 90),
            ("Zenith (Straight Up)", 90, 0)
        ]

        for position_name, alt_deg, az_deg in positions:
            if not self.record_position(position_name, alt_deg, az_deg):
                print("❌ Failed to record position")
                return False

        # Calculate calibration
        return self.calculate_calibration()


def main():
    rclpy.init()
    calibrator = ManualCalibration()

    # Run ROS2 in background thread
    import threading
    ros_thread = threading.Thread(target=lambda: rclpy.spin(calibrator))
    ros_thread.daemon = True
    ros_thread.start()

    # Wait for initialization
    time.sleep(2.0)

    try:
        calibrator.run_manual_calibration()
    except KeyboardInterrupt:
        print("\n👋 Interrupted by user")
    finally:
        calibrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
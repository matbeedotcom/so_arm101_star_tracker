#!/usr/bin/env python3
"""
Calibration script for learning IMU <-> ARM joint transformations.

Usage:
1. Point the arm at known targets (stars, landmarks)
2. Record joint positions and IMU readings
3. Build transformation model
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Vector3
import numpy as np
import sys
import time
from star_tracker.coordinate_transform import CoordinateTransform

# Import astronomical libraries
try:
    from astropy.coordinates import EarthLocation, AltAz, get_sun, get_moon
    from astropy.time import Time
    from astropy import units as u
    import astropy.coordinates as coord
    ASTROPY_AVAILABLE = True
except ImportError:
    ASTROPY_AVAILABLE = False
    print("Warning: astropy not available. Manual coordinate entry required.")


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('coordinate_calibration')

        # Location for astronomical calculations
        self.lat = 40.7128  # Default NYC
        self.lon = -74.0060
        self.alt = 10.0

        if ASTROPY_AVAILABLE:
            self.location = EarthLocation(lat=self.lat*u.deg, lon=self.lon*u.deg, height=self.alt*u.m)

        # Initialize coordinate transformer
        self.transformer = CoordinateTransform()

        # Data storage
        self.current_joints = [0.0] * 5
        self.current_imu_euler = np.array([0.0, 0.0, 0.0])

        # ROS2 subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )

        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10
        )

        self.euler_sub = self.create_subscription(
            Vector3, 'imu/euler', self.euler_callback, 10
        )

        print("Calibration node ready!")
        print("Commands:")
        print("  'point <target>' - Point at celestial target (sun, moon, polaris, sirius)")
        print("  'manual <alt> <az>' - Record manual coordinates (degrees)")
        print("  'record <name>' - Record current position")
        print("  'calibrate' - Calculate transformation")
        print("  'save <file>' - Save calibration")
        print("  'load <file>' - Load calibration")
        print("  'validate' - Test current position against calibration")
        print("  'quit' - Exit")

    def joint_callback(self, msg):
        """Update current joint positions."""
        joint_names = ['Shoulder_Rotation', 'Shoulder_Pitch', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll']
        for i, name in enumerate(joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_joints[i] = msg.position[idx]

    def imu_callback(self, msg):
        """Update IMU data."""
        # Convert quaternion to euler if needed
        pass

    def euler_callback(self, msg):
        """Update euler angles from IMU."""
        self.current_imu_euler = np.array([msg.x, msg.y, msg.z])

    def get_target_coordinates(self, target_name: str):
        """Get current coordinates of celestial target."""
        if not ASTROPY_AVAILABLE:
            print("Astropy not available - use manual coordinates")
            return None, None

        current_time = Time.now()
        altaz_frame = AltAz(obstime=current_time, location=self.location)

        try:
            if target_name.lower() == 'sun':
                obj_coord = get_sun(current_time)
            elif target_name.lower() == 'moon':
                obj_coord = get_moon(current_time)
            elif target_name.lower() == 'polaris':
                obj_coord = coord.SkyCoord(ra='02h31m49s', dec='+89d15m51s')
            elif target_name.lower() == 'sirius':
                obj_coord = coord.SkyCoord(ra='06h45m09s', dec='-16d42m58s')
            else:
                print(f"Unknown target: {target_name}")
                return None, None

            obj_altaz = obj_coord.transform_to(altaz_frame)

            if obj_altaz.alt.rad < 0:
                print(f"Warning: {target_name} is below horizon")

            return obj_altaz.alt.rad, obj_altaz.az.rad

        except Exception as e:
            print(f"Error calculating {target_name} position: {e}")
            return None, None

    def record_calibration_point(self, altitude, azimuth, target_name):
        """Record current position as calibration point."""
        self.transformer.add_calibration_point(
            self.current_joints.copy(),
            self.current_imu_euler.copy(),
            (altitude, azimuth),
            target_name
        )

        print(f"Recorded calibration point:")
        print(f"  Target: {target_name}")
        print(f"  Coordinates: Alt={np.degrees(altitude):.2f}°, Az={np.degrees(azimuth):.2f}°")
        print(f"  Joints: {[f'{np.degrees(j):.1f}°' for j in self.current_joints]}")
        print(f"  IMU: Roll={np.degrees(self.current_imu_euler[0]):.1f}°, "
              f"Pitch={np.degrees(self.current_imu_euler[1]):.1f}°, "
              f"Yaw={np.degrees(self.current_imu_euler[2]):.1f}°")

    def interactive_calibration(self):
        """Interactive calibration interface."""
        try:
            while True:
                command = input("\nCalibration> ").strip().split()

                if not command:
                    continue

                cmd = command[0].lower()

                if cmd == 'quit':
                    break

                elif cmd == 'point':
                    if len(command) < 2:
                        print("Usage: point <target>")
                        continue

                    target = command[1]
                    alt, az = self.get_target_coordinates(target)

                    if alt is not None and az is not None:
                        print(f"{target.capitalize()} coordinates: Alt={np.degrees(alt):.2f}°, Az={np.degrees(az):.2f}°")
                        print("Point the arm at this target, then use 'record' command")
                        self.pending_target = (alt, az, target)
                    else:
                        print(f"Could not calculate coordinates for {target}")

                elif cmd == 'manual':
                    if len(command) < 3:
                        print("Usage: manual <altitude_deg> <azimuth_deg>")
                        continue

                    try:
                        alt_deg = float(command[1])
                        az_deg = float(command[2])
                        alt = np.radians(alt_deg)
                        az = np.radians(az_deg)

                        print(f"Manual coordinates: Alt={alt_deg:.2f}°, Az={az_deg:.2f}°")
                        print("Point the arm at this position, then use 'record' command")
                        self.pending_target = (alt, az, f"manual_{alt_deg}_{az_deg}")

                    except ValueError:
                        print("Invalid coordinates")

                elif cmd == 'record':
                    if not hasattr(self, 'pending_target'):
                        print("No target set. Use 'point' or 'manual' first")
                        continue

                    name = command[1] if len(command) > 1 else self.pending_target[2]
                    self.record_calibration_point(
                        self.pending_target[0],
                        self.pending_target[1],
                        name
                    )

                    print(f"Total calibration points: {len(self.transformer.calibration_points)}")

                elif cmd == 'calibrate':
                    success = self.transformer.calculate_imu_joint_relationship()
                    if success:
                        print("Calibration successful!")
                    else:
                        print("Calibration failed - need more points")

                elif cmd == 'save':
                    filename = command[1] if len(command) > 1 else 'coordinate_calibration.json'
                    self.transformer.save_calibration(filename)

                elif cmd == 'load':
                    filename = command[1] if len(command) > 1 else 'coordinate_calibration.json'
                    self.transformer.load_calibration(filename)

                elif cmd == 'validate':
                    if not self.transformer.is_calibrated:
                        print("No calibration loaded")
                        continue

                    is_valid, error_info = self.transformer.validate_position(
                        self.current_joints,
                        self.current_imu_euler
                    )

                    print(f"Position validation: {'VALID' if is_valid else 'INVALID'}")
                    print(f"  Roll error: {error_info['roll_error']:.2f}°")
                    print(f"  Pitch error: {error_info['pitch_error']:.2f}°")
                    print(f"  Yaw error: {error_info['yaw_error']:.2f}°")
                    print(f"  Max error: {error_info['max_error']:.2f}°")

                elif cmd == 'status':
                    print(f"Current joint positions: {[f'{np.degrees(j):.1f}°' for j in self.current_joints]}")
                    print(f"Current IMU: Roll={np.degrees(self.current_imu_euler[0]):.1f}°, "
                          f"Pitch={np.degrees(self.current_imu_euler[1]):.1f}°, "
                          f"Yaw={np.degrees(self.current_imu_euler[2]):.1f}°")
                    print(f"Calibration points: {len(self.transformer.calibration_points)}")

                else:
                    print(f"Unknown command: {cmd}")

        except KeyboardInterrupt:
            print("\nExiting calibration...")


def main():
    rclpy.init()
    node = CalibrationNode()

    # Run ROS2 in background thread
    import threading
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node))
    ros_thread.daemon = True
    ros_thread.start()

    # Wait for initial data
    print("Waiting for joint states and IMU data...")
    time.sleep(2.0)

    # Start interactive calibration
    node.interactive_calibration()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
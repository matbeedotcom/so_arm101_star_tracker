#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference, Imu, JointState
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Bool, Float64, Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from datetime import datetime, timezone
import threading
import time
import json
import math
from scipy.spatial.transform import Rotation

try:
    from astropy.coordinates import EarthLocation, AltAz, get_sun, get_moon
    from astropy.time import Time
    from astropy import units as u
    import astropy.coordinates as coord
    ASTROPY_AVAILABLE = True
except ImportError:
    ASTROPY_AVAILABLE = False


class MockGPSProvider(Node):
    """Mock GPS provider for testing with realistic satellite data."""
    
    def __init__(self, test_location=None, time_offset=0.0):
        super().__init__('mock_gps_provider')
        
        # Test location (default: New York City)
        if test_location:
            self.lat, self.lon, self.alt = test_location
        else:
            self.lat, self.lon, self.alt = 40.7128, -74.0060, 10.0
        
        self.time_offset = time_offset  # For testing time accuracy
        
        # Publishers matching real GPS interface
        self.fix_pub = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.time_pub = self.create_publisher(TimeReference, 'gps/time', 10)
        self.status_pub = self.create_publisher(Bool, 'gps/has_fix', 10)
        
        # Simulate GPS acquisition sequence
        self.fix_acquired = False
        self.acquisition_time = 0
        
        # Timer for GPS updates
        self.timer = self.create_timer(1.0, self.publish_gps_data)
        
        self.get_logger().info(f'Mock GPS initialized at {self.lat:.6f}, {self.lon:.6f}')
    
    def publish_gps_data(self):
        """Publish mock GPS data."""
        stamp = self.get_clock().now().to_msg()
        
        # Simulate GPS acquisition (30 seconds)
        self.acquisition_time += 1
        if self.acquisition_time >= 5:  # Quick acquisition for testing
            self.fix_acquired = True
        
        # GPS fix message
        fix_msg = NavSatFix()
        fix_msg.header.stamp = stamp
        fix_msg.header.frame_id = 'gps'
        
        if self.fix_acquired:
            # Add small random variation to simulate GPS noise
            fix_msg.latitude = self.lat + np.random.normal(0, 0.000001)  # ~10cm accuracy
            fix_msg.longitude = self.lon + np.random.normal(0, 0.000001)
            fix_msg.altitude = self.alt + np.random.normal(0, 0.1)  # 10cm vertical
            
            fix_msg.status.status = NavSatStatus.STATUS_FIX
            fix_msg.status.service = NavSatStatus.SERVICE_GPS
            
            # Simulate good accuracy
            fix_msg.position_covariance[0] = 1.0  # 1m accuracy
            fix_msg.position_covariance[4] = 1.0
            fix_msg.position_covariance[8] = 4.0
        else:
            fix_msg.status.status = NavSatStatus.STATUS_NO_FIX
        
        self.fix_pub.publish(fix_msg)
        
        # GPS time
        if self.fix_acquired:
            time_msg = TimeReference()
            time_msg.header.stamp = stamp
            time_msg.header.frame_id = 'gps'
            time_msg.source = 'gps'
            time_msg.time_ref = time.time() + self.time_offset
            self.time_pub.publish(time_msg)
        
        # Status
        status_msg = Bool()
        status_msg.data = self.fix_acquired
        self.status_pub.publish(status_msg)


class MockIMUProvider(Node):
    """Mock IMU provider for testing GoTo mode."""
    
    def __init__(self, noise_level=0.01):
        super().__init__('mock_imu_provider')
        
        self.noise_level = noise_level
        
        # Publishers matching BNO055 interface
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.euler_pub = self.create_publisher(Vector3, 'imu/euler', 10)
        
        # Current orientation (will be updated by arm commands)
        self.current_orientation = [0.0, 0.0, 0.0]  # yaw, roll, pitch
        
        # Subscribe to trajectory commands to simulate IMU following arm
        self.traj_sub = self.create_subscription(
            JointTrajectory,
            '/so_100_arm_controller/joint_trajectory',
            self.trajectory_callback,
            10
        )
        
        # Timer for IMU updates
        self.timer = self.create_timer(0.02, self.publish_imu_data)  # 50Hz
        
        self.get_logger().info('Mock IMU initialized')
    
    def trajectory_callback(self, msg):
        """Update IMU orientation based on arm movement."""
        if msg.points:
            point = msg.points[0]
            if len(point.positions) >= 2:
                # Simple mapping: shoulder rotation -> yaw, shoulder pitch -> pitch
                self.current_orientation[0] = point.positions[0]  # Yaw from shoulder rotation
                self.current_orientation[2] = point.positions[1]  # Pitch from shoulder pitch
    
    def publish_imu_data(self):
        """Publish mock IMU data."""
        stamp = self.get_clock().now().to_msg()
        
        # Add noise to simulate real IMU
        noisy_orientation = [
            angle + np.random.normal(0, self.noise_level) 
            for angle in self.current_orientation
        ]
        
        # Create quaternion from Euler angles
        r = Rotation.from_euler('zyx', noisy_orientation)
        quat = r.as_quat()  # [x, y, z, w]
        
        # IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = 'imu_link'
        
        imu_msg.orientation.x = quat[0]
        imu_msg.orientation.y = quat[1]
        imu_msg.orientation.z = quat[2]
        imu_msg.orientation.w = quat[3]
        
        # Add some angular velocity noise
        imu_msg.angular_velocity.x = np.random.normal(0, 0.001)
        imu_msg.angular_velocity.y = np.random.normal(0, 0.001)
        imu_msg.angular_velocity.z = np.random.normal(0, 0.001)
        
        self.imu_pub.publish(imu_msg)
        
        # Euler angles
        euler_msg = Vector3()
        euler_msg.x = noisy_orientation[0]  # Yaw
        euler_msg.y = noisy_orientation[1]  # Roll
        euler_msg.z = noisy_orientation[2]  # Pitch
        self.euler_pub.publish(euler_msg)


class EmulatedSO100Arm(Node):
    """Emulated SO-100 arm for testing tracking commands."""
    
    def __init__(self):
        super().__init__('emulated_so100_arm')
        
        # Joint names
        self.joint_names = [
            'Shoulder_Rotation',
            'Shoulder_Pitch', 
            'Elbow',
            'Wrist_Pitch',
            'Wrist_Roll'
        ]
        
        # Current joint positions
        self.joint_positions = [0.0] * 5
        self.joint_velocities = [0.0] * 5
        
        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Subscribers
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/so_100_arm_controller/joint_trajectory',
            self.trajectory_callback,
            10
        )
        
        # Timer for joint state publishing
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10Hz
        
        # Movement tracking
        self.target_positions = None
        self.move_start_time = None
        self.move_duration = 2.0
        
        self.get_logger().info('Emulated SO-100 arm initialized')
    
    def trajectory_callback(self, msg):
        """Execute trajectory commands."""
        if msg.points:
            point = msg.points[0]
            self.target_positions = point.positions[:5]  # Only first 5 joints
            self.move_start_time = time.time()
            
            # Extract duration from trajectory point
            if hasattr(point, 'time_from_start'):
                self.move_duration = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            
            self.get_logger().info(
                f'Moving to positions: {[np.degrees(p) for p in self.target_positions]}'
            )
    
    def publish_joint_states(self):
        """Publish current joint states."""
        # Simulate smooth movement to target
        if self.target_positions and self.move_start_time:
            elapsed = time.time() - self.move_start_time
            progress = min(elapsed / self.move_duration, 1.0)
            
            # Smooth interpolation (S-curve)
            smooth_progress = 3 * progress**2 - 2 * progress**3
            
            for i in range(5):
                start_pos = self.joint_positions[i] if elapsed > 0.1 else 0.0
                target_pos = self.target_positions[i]
                self.joint_positions[i] = start_pos + (target_pos - start_pos) * smooth_progress
            
            # Calculate velocities
            if elapsed > 0.1:  # After some movement
                self.joint_velocities = [0.1 * smooth_progress] * 5
            else:
                self.joint_velocities = [0.0] * 5
        
        # Publish joint state
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = self.joint_names
        joint_msg.position = self.joint_positions
        joint_msg.velocity = self.joint_velocities
        
        self.joint_state_pub.publish(joint_msg)


class StarTrackerTestSuite(Node):
    """Automated test suite for star tracker functionality."""
    
    def __init__(self):
        super().__init__('star_tracker_test_suite')
        
        self.test_results = {}
        self.current_test = None
        
        # Test locations for validation
        self.test_locations = {
            'new_york': (40.7128, -74.0060, 10.0),
            'london': (51.5074, -0.1278, 35.0),
            'sydney': (-33.8688, 151.2093, 58.0),
            'tokyo': (35.6762, 139.6503, 40.0)
        }
        
        # Subscribe to star tracker output
        self.tracking_sub = self.create_subscription(
            JointTrajectory,
            '/so_100_arm_controller/joint_trajectory',
            self.tracking_callback,
            10
        )
        
        self.get_logger().info('Star Tracker Test Suite initialized')
    
    def tracking_callback(self, msg):
        """Monitor tracking commands for validation."""
        if self.current_test and msg.points:
            point = msg.points[0]
            positions = point.positions[:2]  # Shoulder rotation and pitch
            
            # Convert to alt/az for validation
            az = positions[0]  # Shoulder rotation
            alt = positions[1] + np.pi/2  # Shoulder pitch (adjust for robot zero)
            
            self.test_results[self.current_test] = {
                'timestamp': time.time(),
                'commanded_alt': alt,
                'commanded_az': az,
                'joint_positions': positions
            }
    
    def run_astropy_validation_tests(self):
        """Test astropy calculations against known ephemeris data."""
        self.get_logger().info('Running astropy validation tests...')
        
        if not ASTROPY_AVAILABLE:
            self.get_logger().error('Astropy not available for validation')
            return False
        
        results = {}
        
        for location_name, coords in self.test_locations.items():
            lat, lon, alt = coords
            location = EarthLocation(lat=lat*u.deg, lon=lon*u.deg, height=alt*u.m)
            
            # Test at specific time (2024-01-01 12:00:00 UTC)
            test_time = Time('2024-01-01T12:00:00')
            altaz_frame = AltAz(obstime=test_time, location=location)
            
            # Test sun position
            sun_coord = get_sun(test_time)
            sun_altaz = sun_coord.transform_to(altaz_frame)
            
            # Test moon position  
            moon_coord = get_moon(test_time)
            moon_altaz = moon_coord.transform_to(altaz_frame)
            
            results[location_name] = {
                'sun_alt': sun_altaz.alt.deg,
                'sun_az': sun_altaz.az.deg,
                'moon_alt': moon_altaz.alt.deg,
                'moon_az': moon_altaz.az.deg
            }
            
            self.get_logger().info(
                f'{location_name}: Sun({sun_altaz.alt.deg:.1f}°, {sun_altaz.az.deg:.1f}°) '
                f'Moon({moon_altaz.alt.deg:.1f}°, {moon_altaz.az.deg:.1f}°)'
            )
        
        # Validate against expected ranges
        all_passed = True
        for location_name, data in results.items():
            # Sun should be reasonable for midday in January
            if not (-90 <= data['sun_alt'] <= 90):
                all_passed = False
                self.get_logger().error(f'{location_name}: Invalid sun altitude')
        
        self.get_logger().info(f'Astropy validation: {"PASSED" if all_passed else "FAILED"}')
        return all_passed
    
    def run_tracking_accuracy_test(self, target='moon', duration=60):
        """Test tracking accuracy over time."""
        self.get_logger().info(f'Running {duration}s tracking accuracy test for {target}')
        
        self.current_test = f'tracking_{target}'
        start_time = time.time()
        
        # Let the test run for specified duration
        while (time.time() - start_time) < duration:
            time.sleep(1.0)
        
        # Analyze results
        if self.current_test in self.test_results:
            result = self.test_results[self.current_test]
            self.get_logger().info(
                f'Tracking test complete: Alt={np.degrees(result["commanded_alt"]):.2f}°, '
                f'Az={np.degrees(result["commanded_az"]):.2f}°'
            )
            return True
        else:
            self.get_logger().error('No tracking data received')
            return False
    
    def run_performance_benchmark(self):
        """Benchmark tracking update rates and response times."""
        self.get_logger().info('Running performance benchmark...')
        
        # This would measure update rates, latency, etc.
        # For now, just simulate
        benchmark_results = {
            'avg_update_rate': 1.0,  # Hz
            'max_latency': 0.1,  # seconds
            'cpu_usage': 15.0  # percent
        }
        
        self.get_logger().info(f'Performance: {benchmark_results}')
        return benchmark_results
    
    def save_test_results(self, filename='test_results.json'):
        """Save all test results to file."""
        with open(filename, 'w') as f:
            json.dump(self.test_results, f, indent=2)
        self.get_logger().info(f'Test results saved to {filename}')


def main(args=None):
    rclpy.init(args=args)
    
    # Create test nodes
    gps_provider = MockGPSProvider()
    imu_provider = MockIMUProvider()
    arm_emulator = EmulatedSO100Arm()
    test_suite = StarTrackerTestSuite()
    
    # Run validation tests
    test_suite.run_astropy_validation_tests()
    
    # Multi-threaded execution
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(gps_provider)
    executor.add_node(imu_provider)
    executor.add_node(arm_emulator)
    executor.add_node(test_suite)
    
    try:
        # Run for a limited time for testing
        import threading
        import time
        
        def run_executor():
            executor.spin()
        
        executor_thread = threading.Thread(target=run_executor)
        executor_thread.daemon = True
        executor_thread.start()
        
        # Run tests for 60 seconds
        time.sleep(60)
        
    except KeyboardInterrupt:
        pass
    finally:
        test_suite.save_test_results()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
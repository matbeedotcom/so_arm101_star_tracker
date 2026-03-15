#!/usr/bin/env python3

"""
Mock providers for GPS, IMU, and arm emulation for testing star tracker.
Separated into individual nodes for better modularity and Docker testing.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference, Imu, JointState
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Bool, Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
from datetime import datetime, timezone
import time
import math
from scipy.spatial.transform import Rotation


class TorontoGPSProvider(Node):
    """Mock GPS provider configured for Toronto location."""
    
    def __init__(self):
        super().__init__('toronto_gps_provider')
        
        # Declare parameters
        self.declare_parameter('location_lat', 43.6532)
        self.declare_parameter('location_lon', -79.3832)
        self.declare_parameter('location_alt', 76.0)
        self.declare_parameter('gps_noise_level', 0.000001)
        self.declare_parameter('acquisition_delay', 5.0)
        self.declare_parameter('update_rate', 1.0)
        
        # Get parameters
        self.lat = self.get_parameter('location_lat').value
        self.lon = self.get_parameter('location_lon').value
        self.alt = self.get_parameter('location_alt').value
        self.noise_level = self.get_parameter('gps_noise_level').value
        self.acquisition_delay = self.get_parameter('acquisition_delay').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # Publishers
        self.fix_pub = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.time_pub = self.create_publisher(TimeReference, 'gps/time', 10)
        self.status_pub = self.create_publisher(Bool, 'gps/has_fix', 10)
        
        # GPS state
        self.start_time = time.time()
        self.fix_acquired = False
        self.satellites_visible = 0
        
        # Timer for updates
        self.timer = self.create_timer(1.0 / self.update_rate, self.publish_gps_data)
        
        self.get_logger().info(
            f'Toronto GPS Provider initialized at {self.lat:.6f}°N, {self.lon:.6f}°W, {self.alt:.1f}m'
        )
    
    def simulate_satellite_acquisition(self):
        """Simulate gradual satellite acquisition."""
        elapsed = time.time() - self.start_time
        
        if elapsed < self.acquisition_delay:
            # Gradually acquire satellites
            self.satellites_visible = min(int(elapsed * 2), 12)
            if self.satellites_visible >= 4:
                self.fix_acquired = True
        else:
            self.satellites_visible = np.random.randint(8, 13)
            self.fix_acquired = True
    
    def publish_gps_data(self):
        """Publish realistic GPS data with Toronto coordinates."""
        self.simulate_satellite_acquisition()
        
        stamp = self.get_clock().now().to_msg()
        
        # GPS fix message
        fix_msg = NavSatFix()
        fix_msg.header.stamp = stamp
        fix_msg.header.frame_id = 'gps'
        
        if self.fix_acquired:
            # Add realistic GPS noise
            lat_noise = np.random.normal(0, self.noise_level)
            lon_noise = np.random.normal(0, self.noise_level)
            alt_noise = np.random.normal(0, self.noise_level * 100)  # Altitude less accurate
            
            fix_msg.latitude = self.lat + lat_noise
            fix_msg.longitude = self.lon + lon_noise
            fix_msg.altitude = self.alt + alt_noise
            
            fix_msg.status.status = NavSatStatus.STATUS_FIX
            fix_msg.status.service = (
                NavSatStatus.SERVICE_GPS | 
                NavSatStatus.SERVICE_GLONASS |
                NavSatStatus.SERVICE_GALILEO
            )
            
            # Covariance based on satellite count
            accuracy = 10.0 / max(self.satellites_visible, 1)
            fix_msg.position_covariance_type = 2  # Diagonal known
            fix_msg.position_covariance[0] = accuracy  # East
            fix_msg.position_covariance[4] = accuracy  # North
            fix_msg.position_covariance[8] = accuracy * 2  # Vertical
            
        else:
            fix_msg.status.status = NavSatStatus.STATUS_NO_FIX
            fix_msg.latitude = 0.0
            fix_msg.longitude = 0.0
            fix_msg.altitude = 0.0
        
        self.fix_pub.publish(fix_msg)
        
        # GPS time reference
        if self.fix_acquired:
            time_msg = TimeReference()
            time_msg.header = fix_msg.header
            time_msg.source = 'gps'
            # Simulate atomic clock precision
            time_msg.time_ref = stamp
            self.time_pub.publish(time_msg)
        
        # Status
        status_msg = Bool()
        status_msg.data = self.fix_acquired
        self.status_pub.publish(status_msg)
        
        if self.fix_acquired:
            self.get_logger().debug(
                f'GPS Fix: {fix_msg.latitude:.8f}, {fix_msg.longitude:.8f}, '
                f'Satellites: {self.satellites_visible}'
            )


class MockBNO055IMU(Node):
    """Mock BNO055 IMU for GoTo mode testing."""
    
    def __init__(self):
        super().__init__('mock_bno055_imu')
        
        # Declare parameters
        self.declare_parameter('noise_level', 0.01)
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('simulate_drift', True)
        self.declare_parameter('drift_rate', 0.001)
        
        # Get parameters
        self.noise_level = self.get_parameter('noise_level').value
        self.update_rate = self.get_parameter('update_rate').value
        self.simulate_drift = self.get_parameter('simulate_drift').value
        self.drift_rate = self.get_parameter('drift_rate').value
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.euler_pub = self.create_publisher(Vector3, 'imu/euler', 10)
        
        # IMU state
        self.orientation = np.array([0.0, 0.0, 0.0])  # yaw, roll, pitch
        self.angular_velocity = np.array([0.0, 0.0, 0.0])
        self.linear_acceleration = np.array([0.0, 0.0, 9.81])  # Gravity
        
        # Drift simulation
        self.drift_offset = np.array([0.0, 0.0, 0.0])
        
        # Subscribe to arm trajectory to simulate IMU following arm movement
        self.traj_sub = self.create_subscription(
            JointTrajectory,
            '/so_100_arm_controller/joint_trajectory',
            self.trajectory_callback,
            10
        )
        
        # Timer for IMU updates
        self.timer = self.create_timer(1.0 / self.update_rate, self.publish_imu_data)
        
        self.get_logger().info(f'Mock BNO055 IMU initialized at {self.update_rate}Hz')
    
    def trajectory_callback(self, msg):
        """Update IMU orientation based on arm movement."""
        if msg.points:
            point = msg.points[0]
            if len(point.positions) >= 2:
                # Map arm joints to IMU orientation
                # Shoulder rotation -> yaw
                # Shoulder pitch -> pitch
                target_yaw = point.positions[0]
                target_pitch = point.positions[1]
                
                # Smooth transition
                alpha = 0.1  # Smoothing factor
                self.orientation[0] = (1 - alpha) * self.orientation[0] + alpha * target_yaw
                self.orientation[2] = (1 - alpha) * self.orientation[2] + alpha * target_pitch
                
                self.get_logger().debug(
                    f'IMU tracking arm: Yaw={np.degrees(self.orientation[0]):.1f}°, '
                    f'Pitch={np.degrees(self.orientation[2]):.1f}°'
                )
    
    def publish_imu_data(self):
        """Publish realistic IMU data."""
        stamp = self.get_clock().now().to_msg()
        
        # Apply drift if enabled
        if self.simulate_drift:
            self.drift_offset += np.random.normal(0, self.drift_rate, 3)
        
        # Add noise to orientation
        noisy_orientation = self.orientation + self.drift_offset
        noisy_orientation += np.random.normal(0, self.noise_level, 3)
        
        # Convert to quaternion
        r = Rotation.from_euler('zyx', noisy_orientation)
        quat = r.as_quat()  # [x, y, z, w]
        
        # IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = 'imu_link'
        
        # Orientation
        imu_msg.orientation.x = quat[0]
        imu_msg.orientation.y = quat[1]
        imu_msg.orientation.z = quat[2]
        imu_msg.orientation.w = quat[3]
        imu_msg.orientation_covariance[0] = self.noise_level ** 2
        imu_msg.orientation_covariance[4] = self.noise_level ** 2
        imu_msg.orientation_covariance[8] = self.noise_level ** 2
        
        # Angular velocity (with noise)
        imu_msg.angular_velocity.x = self.angular_velocity[0] + np.random.normal(0, 0.001)
        imu_msg.angular_velocity.y = self.angular_velocity[1] + np.random.normal(0, 0.001)
        imu_msg.angular_velocity.z = self.angular_velocity[2] + np.random.normal(0, 0.001)
        
        # Linear acceleration (gravity + noise)
        imu_msg.linear_acceleration.x = self.linear_acceleration[0] + np.random.normal(0, 0.01)
        imu_msg.linear_acceleration.y = self.linear_acceleration[1] + np.random.normal(0, 0.01)
        imu_msg.linear_acceleration.z = self.linear_acceleration[2] + np.random.normal(0, 0.01)
        
        self.imu_pub.publish(imu_msg)
        
        # Euler angles (for debugging)
        euler_msg = Vector3()
        euler_msg.x = np.degrees(noisy_orientation[0])  # Yaw
        euler_msg.y = np.degrees(noisy_orientation[1])  # Roll
        euler_msg.z = np.degrees(noisy_orientation[2])  # Pitch
        self.euler_pub.publish(euler_msg)


class SO100ArmEmulator(Node):
    """Emulated SO-100 arm for testing."""
    
    def __init__(self):
        super().__init__('so100_arm_emulator')
        
        # Declare parameters
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('simulate_backlash', True)
        self.declare_parameter('backlash_amount', 0.005)
        self.declare_parameter('max_velocity', 1.0)
        
        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.simulate_backlash = self.get_parameter('simulate_backlash').value
        self.backlash_amount = self.get_parameter('backlash_amount').value
        self.max_velocity = self.get_parameter('max_velocity').value
        
        # Joint configuration
        self.joint_names = [
            'Shoulder_Rotation',
            'Shoulder_Pitch',
            'Elbow',
            'Wrist_Pitch',
            'Wrist_Roll'
        ]
        
        # Joint states
        self.current_positions = np.array([0.0] * 5)
        self.target_positions = np.array([0.0] * 5)
        self.joint_velocities = np.array([0.0] * 5)
        
        # Movement control
        self.movement_start_time = None
        self.movement_duration = 2.0
        self.is_moving = False
        
        # Backlash simulation
        self.backlash_state = np.array([0.0] * 5)
        
        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Subscribers
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/so_100_arm_controller/joint_trajectory',
            self.trajectory_callback,
            10
        )
        
        # Timer for joint state updates
        self.timer = self.create_timer(1.0 / self.update_rate, self.publish_joint_states)
        
        self.get_logger().info('SO-100 Arm Emulator initialized')
    
    def trajectory_callback(self, msg):
        """Handle trajectory commands."""
        if not msg.points:
            return
        
        point = msg.points[0]
        
        # Extract target positions
        self.target_positions = np.array(point.positions[:5])
        
        # Extract duration
        if hasattr(point, 'time_from_start'):
            self.movement_duration = (
                point.time_from_start.sec + 
                point.time_from_start.nanosec * 1e-9
            )
        
        # Start movement
        self.movement_start_time = time.time()
        self.is_moving = True
        
        self.get_logger().info(
            f'Moving to: Az={np.degrees(self.target_positions[0]):.1f}°, '
            f'Alt={np.degrees(self.target_positions[1]):.1f}° '
            f'in {self.movement_duration:.1f}s'
        )
    
    def simulate_movement(self):
        """Simulate smooth arm movement with realistic characteristics."""
        if not self.is_moving or self.movement_start_time is None:
            return
        
        elapsed = time.time() - self.movement_start_time
        
        if elapsed >= self.movement_duration:
            # Movement complete
            self.current_positions = self.target_positions.copy()
            self.joint_velocities = np.zeros(5)
            self.is_moving = False
            return
        
        # S-curve motion profile
        progress = elapsed / self.movement_duration
        smooth_progress = 3 * progress**2 - 2 * progress**3
        
        # Update positions
        for i in range(5):
            start_pos = self.current_positions[i]
            target_pos = self.target_positions[i]
            
            # Interpolate position
            new_pos = start_pos + (target_pos - start_pos) * smooth_progress
            
            # Calculate velocity
            if elapsed > 0:
                self.joint_velocities[i] = (new_pos - self.current_positions[i]) * self.update_rate
            
            # Apply backlash simulation
            if self.simulate_backlash:
                if abs(self.joint_velocities[i]) > 0.001:
                    # Add backlash when changing direction
                    direction = np.sign(self.joint_velocities[i])
                    if direction != np.sign(self.backlash_state[i]):
                        new_pos += direction * self.backlash_amount
                    self.backlash_state[i] = direction
            
            self.current_positions[i] = new_pos
    
    def publish_joint_states(self):
        """Publish current joint states."""
        self.simulate_movement()
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        msg.name = self.joint_names
        msg.position = self.current_positions.tolist()
        msg.velocity = self.joint_velocities.tolist()
        msg.effort = [0.0] * 5  # Not simulating torque
        
        self.joint_state_pub.publish(msg)


def main_gps(args=None):
    """Run Toronto GPS provider."""
    rclpy.init(args=args)
    node = TorontoGPSProvider()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_imu(args=None):
    """Run mock IMU provider."""
    rclpy.init(args=args)
    node = MockBNO055IMU()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_arm(args=None):
    """Run arm emulator."""
    rclpy.init(args=args)
    node = SO100ArmEmulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        if sys.argv[1] == 'gps':
            main_gps()
        elif sys.argv[1] == 'imu':
            main_imu()
        elif sys.argv[1] == 'arm':
            main_arm()
    else:
        print("Usage: mock_providers.py [gps|imu|arm]")
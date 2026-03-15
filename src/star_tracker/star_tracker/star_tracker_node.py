#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import JointState, Imu, NavSatFix, TimeReference, MagneticField
from std_msgs.msg import String, Bool, Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
import numpy as np
from datetime import datetime, timezone
import os
import json
try:
    from .coordinate_transform import CoordinateTransform
    from .auto_calibration import AutoCalibration
except ImportError:
    # Fallback for direct execution
    import sys
    import os
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from coordinate_transform import CoordinateTransform
    from auto_calibration import AutoCalibration

# Import astronomical libraries
try:
    from astropy.coordinates import EarthLocation, AltAz, get_sun, get_moon
    from astropy.time import Time
    from astropy import units as u
    import astropy.coordinates as coord
    ASTROPY_AVAILABLE = True
except ImportError:
    ASTROPY_AVAILABLE = False
    print("Warning: astropy not available. Using basic calculations.")


class StarTrackerNode(Node):
    def __init__(self):
        super().__init__('star_tracker_node')
        
        # Parameters
        self.declare_parameter('update_rate', 1.0)  # Hz
        self.declare_parameter('location_lat', 40.7128)  # degrees (fallback if no GPS)
        self.declare_parameter('location_lon', -74.0060)  # degrees (fallback if no GPS)
        self.declare_parameter('location_alt', 10.0)  # meters (fallback if no GPS)
        self.declare_parameter('target_object', 'polaris')
        self.declare_parameter('tracking_mode', 'continuous')
        self.declare_parameter('use_imu', False)
        self.declare_parameter('use_gps', True)  # Use GPS for location and time
        self.declare_parameter('goto_mode', False)
        self.declare_parameter('alignment_file', 'star_alignment.json')
        self.declare_parameter('gps_timeout', 30.0)  # Seconds to wait for GPS fix
        self.declare_parameter('imu_config_file', 'config/imu_calibration.yaml')  # IMU calibration config
        
        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.fallback_lat = self.get_parameter('location_lat').value
        self.fallback_lon = self.get_parameter('location_lon').value
        self.fallback_alt = self.get_parameter('location_alt').value
        self.target = self.get_parameter('target_object').value
        self.tracking_mode = self.get_parameter('tracking_mode').value
        self.use_imu = self.get_parameter('use_imu').value
        self.use_gps = self.get_parameter('use_gps').value
        self.goto_mode = self.get_parameter('goto_mode').value
        self.alignment_file = self.get_parameter('alignment_file').value
        self.gps_timeout = self.get_parameter('gps_timeout').value
        self.imu_config_file = self.get_parameter('imu_config_file').value

        # Debug parameter values
        print(f"DEBUG: use_imu={self.use_imu}, goto_mode={self.goto_mode}")
        print(f"DEBUG: target_object parameter = '{self.target}'")
        
        # Current location (will be updated by GPS if available)
        self.lat = self.fallback_lat
        self.lon = self.fallback_lon
        self.alt = self.fallback_alt
        
        # GPS state
        self.gps_fix = False
        self.gps_time = None
        self.last_gps_update = None
        
        # Initialize location
        if ASTROPY_AVAILABLE:
            self.location = EarthLocation(lat=self.lat*u.deg, 
                                         lon=self.lon*u.deg, 
                                         height=self.alt*u.m)
        
        # Joint names for SO-100 arm
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
        
        # Publishers
        self.joint_pub = self.create_publisher(
            JointTrajectory,
            '/so_100_arm_controller/joint_trajectory',
            10
        )
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # GPS subscribers
        if self.use_gps:
            self.gps_fix_sub = self.create_subscription(
                NavSatFix, 'gps/fix', self.gps_fix_callback, 10
            )
            self.gps_time_sub = self.create_subscription(
                TimeReference, 'gps/time', self.gps_time_callback, 10
            )
            self.gps_status_sub = self.create_subscription(
                Bool, 'gps/has_fix', self.gps_status_callback, 10
            )

        # IMU subscribers (for GoTo mode)
        if self.use_imu:
            self.imu_sub = self.create_subscription(
                Imu, 'imu/data', self.imu_callback, 10
            )
            self.euler_sub = self.create_subscription(
                Vector3, 'imu/euler', self.euler_callback, 10
            )
            # Subscribe to magnetometer data for auto-calibration
            # Note: BNO055 interface publishes /imu/mag as MagneticField message
            # Accelerometer data is available in the /imu/data Imu message
            self.mag_sub = self.create_subscription(
                MagneticField, 'imu/mag', self.magnetometer_callback, 10
            )
            self.alignment_status_sub = self.create_subscription(
                Bool, 'alignment/is_aligned', self.alignment_status_callback, 10
            )
        
        # State
        self.current_joint_positions = [0.0] * 5
        self.target_alt_az = (0.0, 0.0)  # altitude, azimuth in radians
        self.current_imu_orientation = None
        self.current_euler = None
        self.current_imu_accel = None
        self.current_imu_mag = None
        self.is_aligned = False
        self.auto_calibration_done = False

        # Initialize automatic calibration system
        self.auto_calibration = AutoCalibration()

        # Set magnetic declination for your location (look up online for your lat/lon)
        # Default to 0 - user can set this via parameter if needed
        self.declare_parameter('magnetic_declination', 0.0)
        mag_declination = self.get_parameter('magnetic_declination').value
        self.auto_calibration.set_magnetic_declination(mag_declination)

        # Try to load existing auto-calibration
        auto_calib_file = os.path.expanduser('~/auto_calibration.json')
        if os.path.exists(auto_calib_file):
            if self.auto_calibration.load_calibration(auto_calib_file):
                self.get_logger().info('Loaded automatic calibration')
                self.is_aligned = True
            else:
                self.get_logger().warn('Failed to load automatic calibration')
        else:
            self.get_logger().info('No automatic calibration found - will auto-calibrate on first IMU data')

        # Keep coordinate transformer for fallback
        self.coordinate_transform = CoordinateTransform()

        # Setup automatic calibration
        if self.use_imu:
            self.get_logger().info('IMU enabled - automatic calibration will run when IMU data is received')
            if self.auto_calibration.is_calibrated:
                self.is_aligned = True
                self.get_logger().info('Using existing automatic calibration')
        else:
            self.get_logger().info('IMU disabled - using coordinate-only mode')
        
        # Timer for tracking updates
        self.timer = self.create_timer(1.0 / self.update_rate, self.tracking_callback)
        
        self.get_logger().info(f'Star Tracker Node initialized')
        self.get_logger().info(f'Location: Lat={self.lat}, Lon={self.lon}, Alt={self.alt}')
        self.get_logger().info(f'Tracking target: {self.target}')
        if self.use_gps:
            self.get_logger().info(f'GPS integration enabled - waiting for fix...')
        if self.use_imu:
            self.get_logger().info(f'IMU integration enabled - GoTo mode: {self.goto_mode}')
            self.get_logger().info(f'use_imu parameter: {self.use_imu}')
            self.get_logger().info(f'imu_config_file parameter: {self.imu_config_file}')
    
    def joint_state_callback(self, msg):
        """Update current joint positions from joint states."""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_joint_positions[i] = msg.position[idx]
    
    def gps_fix_callback(self, msg):
        """Update location from GPS fix."""
        if msg.status.status >= 0:  # Valid fix
            self.lat = msg.latitude
            self.lon = msg.longitude
            self.alt = msg.altitude
            self.last_gps_update = self.get_clock().now()
            
            # Update astropy location if available
            if ASTROPY_AVAILABLE:
                self.location = EarthLocation(
                    lat=self.lat*u.deg,
                    lon=self.lon*u.deg,
                    height=self.alt*u.m
                )
            
            self.get_logger().info(
                f'GPS location updated: Lat={self.lat:.6f}, Lon={self.lon:.6f}, Alt={self.alt:.1f}m'
            )
    
    def gps_time_callback(self, msg):
        """Update time reference from GPS."""
        self.gps_time = msg.time_ref
        
        # Log time synchronization
        if self.gps_time:
            gps_datetime = datetime.fromtimestamp(self.gps_time, tz=timezone.utc)
            self.get_logger().debug(f'GPS time: {gps_datetime.isoformat()}')
    
    def gps_status_callback(self, msg):
        """Update GPS fix status."""
        prev_status = self.gps_fix
        self.gps_fix = msg.data
        
        if self.gps_fix and not prev_status:
            self.get_logger().info('GPS fix acquired')
        elif not self.gps_fix and prev_status:
            self.get_logger().warn('GPS fix lost - using last known position')
    
    def imu_callback(self, msg):
        """Store current IMU orientation and accelerometer data for GoTo calculations."""
        self.current_imu_orientation = msg.orientation
        # Extract accelerometer data for auto-calibration
        self.current_imu_accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
    
    def euler_callback(self, msg):
        """Store current Euler angles from IMU.
        BNO055 publishes as: x=heading/yaw, y=roll, z=pitch
        """
        self.current_euler = np.array([msg.x, msg.y, msg.z])

        # Perform automatic calibration if not done yet
        self.try_auto_calibration()

    def magnetometer_callback(self, msg):
        """Store current magnetometer reading from MagneticField message."""
        self.current_imu_mag = np.array([
            msg.magnetic_field.x,
            msg.magnetic_field.y,
            msg.magnetic_field.z
        ])

    def try_auto_calibration(self):
        """Attempt automatic calibration if all IMU data is available."""
        if (self.auto_calibration_done or
            self.current_euler is None or
            self.current_imu_accel is None or
            self.current_imu_mag is None):
            return

        try:
            # Perform automatic calibration using current IMU data
            success = self.auto_calibration.calibrate_from_imu_data(
                self.current_euler,
                self.current_imu_accel,
                self.current_imu_mag
            )

            if success:
                self.auto_calibration_done = True
                self.is_aligned = True
                self.get_logger().info('Automatic IMU calibration completed!')

                # Save calibration for future use
                auto_calib_file = os.path.expanduser('~/auto_calibration.json')
                self.auto_calibration.save_calibration(auto_calib_file)
                self.get_logger().info(f'Auto-calibration saved to {auto_calib_file}')

                print(f"DEBUG: After auto-calibration, self.target = '{self.target}'")

        except Exception as e:
            self.get_logger().error(f'Auto-calibration failed: {e}')
    
    def alignment_status_callback(self, msg):
        """Update alignment status from calibration node."""
        self.is_aligned = msg.data
        if self.is_aligned and self.goto_mode:
            self.get_logger().info('Alignment confirmed - GoTo mode active')
    
    def tracking_callback(self):
        """Main tracking loop - calculate target position and move arm."""
        if self.goto_mode and self.use_imu:
            # GoTo mode: Use IMU feedback for closed-loop control
            if not self.is_aligned:
                self.get_logger().warn('Waiting for alignment...')
                return
            
            # Get target position
            alt, az = self.calculate_target_position()
            if alt is None or az is None:
                return
            
            # Get current pointing from IMU
            current_alt, current_az = self.get_current_pointing()
            
            # Calculate error
            alt_error = alt - current_alt
            az_error = self.normalize_angle(az - current_az)
            
            # Only move if error is significant
            if abs(alt_error) > np.radians(1) or abs(az_error) > np.radians(1):
                # Calculate corrective joint positions
                joint_positions = self.calculate_goto_trajectory(
                    current_alt, current_az, alt, az
                )
                self.send_trajectory(joint_positions)
                
                self.get_logger().info(
                    f'GoTo {self.target}: Target Alt={np.degrees(alt):.2f}°, Az={np.degrees(az):.2f}° | '
                    f'Error: Alt={np.degrees(alt_error):.2f}°, Az={np.degrees(az_error):.2f}°'
                )
        else:
            # Standard tracking mode (open-loop)
            alt, az = self.calculate_target_position()
            
            if alt is None or az is None:
                return
            
            # Convert to joint angles
            joint_positions = self.altaz_to_joint_angles(alt, az)
            
            # Send trajectory command
            self.send_trajectory(joint_positions)
            
            # Log tracking info
            self.get_logger().info(
                f'Tracking {self.target}: Alt={np.degrees(alt):.2f}°, '
                f'Az={np.degrees(az):.2f}°'
            )
    
    def calculate_target_position(self):
        """Calculate altitude and azimuth of target object."""
        if not ASTROPY_AVAILABLE:
            # Simple calculation for testing without astropy
            if self.use_gps and self.gps_time:
                current_time = datetime.fromtimestamp(self.gps_time, tz=timezone.utc)
            else:
                current_time = datetime.utcnow()
            
            hour_angle = (current_time.hour + current_time.minute/60.0) * 15.0

            print(f"DEBUG: calculate_target_position() called with self.target = '{self.target}'")

            if self.target == 'polaris':
                # Polaris is approximately at celestial north pole
                alt = np.radians(self.lat)  # Altitude equals latitude
                az = 0.0  # North
            elif self.target == 'zenith':
                # Zenith: straight up (90° altitude, any azimuth)
                alt = np.radians(90.0)  # Straight up
                az = 0.0  # Point north for consistency
            else:
                # Simple sun approximation
                alt = np.radians(45.0)  # Fixed altitude for testing
                az = np.radians(hour_angle)
            
            return alt, az
        
        # Use astropy for accurate calculations
        # Use GPS time if available, otherwise system time
        if self.use_gps and self.gps_time:
            current_time = Time(self.gps_time, format='unix')
        else:
            current_time = Time.now()
        
        altaz_frame = AltAz(obstime=current_time, location=self.location)
        
        if self.target == 'sun':
            obj_coord = get_sun(current_time)
        elif self.target == 'moon':
            obj_coord = get_moon(current_time)
        elif self.target == 'polaris':
            # Polaris coordinates (RA: 2h 31m 49s, Dec: +89° 15' 51")
            obj_coord = coord.SkyCoord(ra='02h31m49s', dec='+89d15m51s')
        elif self.target == 'sirius':
            # Sirius coordinates
            obj_coord = coord.SkyCoord(ra='06h45m09s', dec='-16d42m58s')
        elif self.target == 'zenith':
            # Zenith: straight up (90° altitude, any azimuth)
            # Skip astropy transformation, return directly
            return np.radians(90.0), 0.0
        else:
            self.get_logger().warn(f'Unknown target: {self.target}')
            return None, None
        
        # Transform to altitude-azimuth
        obj_altaz = obj_coord.transform_to(altaz_frame)
        
        # Check if object is above horizon
        if obj_altaz.alt.rad < 0:
            self.get_logger().info(f'{self.target} is below horizon')
            return None, None
        
        return obj_altaz.alt.rad, obj_altaz.az.rad
    
    def altaz_to_joint_angles(self, alt, az):
        """Convert altitude/azimuth to robot joint angles using coordinate transformer."""
        return self.coordinate_transform.altaz_to_joint_positions(alt, az)
    
    def send_trajectory(self, target_positions, duration=2.0):
        """Send joint trajectory command to robot."""
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.velocities = [0.0] * len(target_positions)
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        
        trajectory.points = [point]
        
        # Publish trajectory
        self.joint_pub.publish(trajectory)
        
        # Optionally use action client for feedback
        if self.trajectory_client.wait_for_server(timeout_sec=1.0):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory
            self.trajectory_client.send_goal_async(goal)
    
    def get_current_pointing(self):
        """Get current telescope pointing from IMU data using automatic calibration."""
        if self.current_euler is None:
            return 0.0, 0.0

        if self.use_imu:
            # Use automatic calibration system (works immediately with IMU magnetometer/accelerometer)
            current_alt, current_az = self.auto_calibration.imu_to_altaz(self.current_euler)

            # Log pointing info occasionally for debugging
            if hasattr(self, '_last_pointing_log'):
                if (self.get_clock().now().nanoseconds - self._last_pointing_log) > 5e9:  # 5 seconds
                    self.get_logger().info(f'Current pointing: Alt={np.degrees(current_alt):.1f}°, Az={np.degrees(current_az):.1f}°')
                    self._last_pointing_log = self.get_clock().now().nanoseconds
            else:
                self._last_pointing_log = self.get_clock().now().nanoseconds

            return current_alt, current_az

        else:
            # Fallback: direct mapping for initial operation
            # BNO055 euler message convention: x=heading/yaw, y=roll, z=pitch
            current_az = self.current_euler[0]   # x = heading/yaw for compass bearing
            current_alt = self.current_euler[2]  # z = pitch for elevation

            return current_alt, current_az
    
    def calculate_goto_trajectory(self, current_alt, current_az, target_alt, target_az):
        """Calculate joint positions for GoTo movement with IMU feedback."""
        # Calculate absolute joint positions needed to point at target
        # This replaces the incremental approach with absolute positioning

        # Convert target altitude/azimuth directly to joint angles
        target_joint_positions = self.altaz_to_joint_angles(target_alt, target_az)

        # Calculate error for logging
        error_alt = target_alt - current_alt
        error_az = self.normalize_angle(target_az - current_az)

        # Apply proportional control to smooth movement
        gain = 0.1  # Lower gain for stability - prevent oscillation

        # Interpolate between current and target positions
        current_joints = np.array(self.current_joint_positions)
        target_joints = np.array(target_joint_positions)

        # Calculate the difference and apply gain
        joint_diff = target_joints - current_joints

        # Normalize shoulder rotation difference to shortest path
        joint_diff[0] = self.normalize_angle(joint_diff[0])

        # Apply gain to difference
        corrected_joints = current_joints + gain * joint_diff

        # Clamp to joint limits
        for i in range(len(corrected_joints)):
            corrected_joints[i] = np.clip(corrected_joints[i], -np.pi, np.pi)

        # Log joint positions for debugging
        joint_names = ['Shoulder_Rotation', 'Shoulder_Pitch', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll']
        current_degrees = [np.degrees(j) for j in current_joints]
        target_degrees = [np.degrees(j) for j in target_joints]
        corrected_degrees = [np.degrees(j) for j in corrected_joints]

        self.get_logger().info(f'Current joints: {joint_names[0]}={current_degrees[0]:.1f}° {joint_names[1]}={current_degrees[1]:.1f}°')
        self.get_logger().info(f'Target joints:  {joint_names[0]}={target_degrees[0]:.1f}° {joint_names[1]}={target_degrees[1]:.1f}°')
        self.get_logger().info(f'Moving to:      {joint_names[0]}={corrected_degrees[0]:.1f}° {joint_names[1]}={corrected_degrees[1]:.1f}°')

        return corrected_joints.tolist()
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def apply_alignment_transform(self, euler_angles):
        """Apply alignment transformation to IMU data.
        Input: euler_angles[0]=heading/yaw, [1]=roll, [2]=pitch
        Output: (altitude, azimuth) for telescope pointing
        """
        if not self.alignment_transform:
            # Default mapping: altitude=pitch, azimuth=heading
            return euler_angles[2], euler_angles[0]

        # Apply transformation matrix from alignment calibration
        transformed = self.alignment_transform @ euler_angles

        # Return altitude, azimuth
        return transformed[2], transformed[0]
    
    def load_alignment(self):
        """Load alignment calibration for GoTo mode."""
        filepath = os.path.expanduser(f'~/{self.alignment_file}')
        
        if not os.path.exists(filepath):
            self.get_logger().warn('No alignment file found - GoTo mode will use direct mapping')
            return
        
        try:
            with open(filepath, 'r') as f:
                calib_data = json.load(f)
            
            if calib_data.get('is_aligned', False):
                self.is_aligned = True
                self.alignment_transform = np.array(calib_data.get('alignment_matrix', np.eye(3).tolist()))
                self.get_logger().info('Alignment loaded successfully')
            else:
                self.get_logger().warn('Alignment file exists but system is not aligned')
                
        except Exception as e:
            self.get_logger().error(f'Failed to load alignment: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    # Get environment variables
    lat = float(os.environ.get('LOCATION_LAT', '40.7128'))
    lon = float(os.environ.get('LOCATION_LON', '-74.0060'))
    alt = float(os.environ.get('LOCATION_ALT', '10.0'))
    target = os.environ.get('TARGET_OBJECT', 'polaris')
    
    node = StarTrackerNode()
    
    # Override location parameters from environment
    node.lat = lat
    node.lon = lon
    node.alt = alt
    # Don't override target - use ROS parameter value
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
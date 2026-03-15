#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import String, Bool, Float32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from scipy.spatial.transform import Rotation
import json
import os
from datetime import datetime

try:
    from astropy.coordinates import EarthLocation, AltAz, get_body
    from astropy.time import Time
    from astropy import units as u
    import astropy.coordinates as coord
    ASTROPY_AVAILABLE = True
except ImportError:
    ASTROPY_AVAILABLE = False


class AlignmentCalibration(Node):
    """
    Performs alignment calibration for star tracker using IMU data.
    Implements 2-star or 3-star alignment for automated GoTo functionality.
    """
    
    def __init__(self):
        super().__init__('alignment_calibration')
        
        # Parameters
        self.declare_parameter('calibration_file', 'star_alignment.json')
        self.declare_parameter('location_lat', 40.7128)
        self.declare_parameter('location_lon', -74.0060)
        self.declare_parameter('location_alt', 10.0)
        self.declare_parameter('alignment_method', '2star')  # '1star', '2star', '3star'
        
        # Get parameters
        self.calib_file = self.get_parameter('calibration_file').value
        self.lat = self.get_parameter('location_lat').value
        self.lon = self.get_parameter('location_lon').value
        self.alt = self.get_parameter('location_alt').value
        self.alignment_method = self.get_parameter('alignment_method').value
        
        # Initialize location
        if ASTROPY_AVAILABLE:
            self.location = EarthLocation(
                lat=self.lat*u.deg,
                lon=self.lon*u.deg,
                height=self.alt*u.m
            )
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10
        )
        self.euler_sub = self.create_subscription(
            Vector3, 'imu/euler', self.euler_callback, 10
        )
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )
        
        # Publishers
        self.status_pub = self.create_publisher(
            String, 'alignment/status', 10
        )
        self.aligned_pub = self.create_publisher(
            Bool, 'alignment/is_aligned', 10
        )
        self.offset_pub = self.create_publisher(
            Vector3, 'alignment/offset', 10
        )
        
        # Services for alignment commands
        self.create_service_callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        
        # State
        self.current_imu_orientation = None
        self.current_euler = None
        self.current_joint_positions = {}
        self.alignment_points = []
        self.is_aligned = False
        self.alignment_matrix = np.eye(3)
        self.alignment_offset = np.zeros(3)
        
        # Load existing calibration
        self.load_calibration()
        
        self.get_logger().info('Alignment calibration node initialized')
    
    def imu_callback(self, msg):
        """Store current IMU orientation."""
        self.current_imu_orientation = msg.orientation
    
    def euler_callback(self, msg):
        """Store current Euler angles from IMU."""
        self.current_euler = np.array([msg.x, msg.y, msg.z])
    
    def joint_callback(self, msg):
        """Store current joint positions."""
        for i, name in enumerate(msg.name):
            self.current_joint_positions[name] = msg.position[i]
    
    def start_alignment(self, star_name=None):
        """Start alignment procedure with specified star."""
        if not star_name:
            # Auto-select bright stars based on visibility
            star_name = self.select_alignment_star()
        
        self.get_logger().info(f'Starting alignment with {star_name}')
        
        # Get theoretical position of star
        alt, az = self.calculate_star_position(star_name)
        
        if alt is None:
            self.get_logger().error(f'{star_name} not visible')
            return False
        
        # Store alignment point
        alignment_point = {
            'star': star_name,
            'theoretical_altaz': [alt, az],
            'imu_orientation': self.quaternion_to_array(self.current_imu_orientation),
            'euler_angles': self.current_euler.tolist() if self.current_euler is not None else None,
            'joint_positions': dict(self.current_joint_positions),
            'timestamp': datetime.utcnow().isoformat()
        }
        
        self.alignment_points.append(alignment_point)
        
        # Check if we have enough points for alignment
        if self.alignment_method == '1star' and len(self.alignment_points) >= 1:
            self.perform_1star_alignment()
        elif self.alignment_method == '2star' and len(self.alignment_points) >= 2:
            self.perform_2star_alignment()
        elif self.alignment_method == '3star' and len(self.alignment_points) >= 3:
            self.perform_3star_alignment()
        
        return True
    
    def perform_1star_alignment(self):
        """
        Simple 1-star alignment using compass heading.
        Assumes IMU is level and uses magnetometer for azimuth.
        """
        point = self.alignment_points[-1]
        
        if point['euler_angles'] is None:
            self.get_logger().error('No IMU data available')
            return
        
        # Calculate offset between theoretical and actual
        theoretical_az = point['theoretical_altaz'][1]
        actual_az = point['euler_angles'][0]  # Heading from IMU
        
        self.alignment_offset[0] = theoretical_az - actual_az
        
        self.is_aligned = True
        self.save_calibration()
        
        self.get_logger().info(f'1-star alignment complete. Azimuth offset: {np.degrees(self.alignment_offset[0]):.2f}°')
        
        # Publish status
        self.publish_alignment_status()
    
    def perform_2star_alignment(self):
        """
        2-star alignment to determine mount orientation.
        Calculates transformation matrix between IMU and celestial coordinates.
        """
        if len(self.alignment_points) < 2:
            return
        
        points = self.alignment_points[-2:]
        
        # Build matrices for least squares solution
        theoretical_vectors = []
        actual_vectors = []
        
        for point in points:
            # Convert alt/az to unit vectors
            alt, az = point['theoretical_altaz']
            theoretical_vec = self.altaz_to_vector(alt, az)
            theoretical_vectors.append(theoretical_vec)
            
            # Get IMU orientation as vector
            if point['euler_angles'] is not None:
                euler = point['euler_angles']
                actual_vec = self.euler_to_vector(euler)
                actual_vectors.append(actual_vec)
        
        # Calculate transformation matrix using SVD
        theoretical_mat = np.array(theoretical_vectors).T
        actual_mat = np.array(actual_vectors).T
        
        # Solve for rotation matrix R where: theoretical = R * actual
        H = actual_mat @ theoretical_mat.T
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        
        # Ensure proper rotation matrix
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        
        self.alignment_matrix = R
        self.is_aligned = True
        self.save_calibration()
        
        self.get_logger().info('2-star alignment complete')
        self.publish_alignment_status()
    
    def perform_3star_alignment(self):
        """
        3-star alignment for highest accuracy.
        Accounts for mechanical errors and flexure.
        """
        if len(self.alignment_points) < 3:
            return
        
        # Use last 3 alignment points
        points = self.alignment_points[-3:]
        
        # Build overdetermined system for robust solution
        A = []
        b = []
        
        for point in points:
            alt, az = point['theoretical_altaz']
            theoretical_vec = self.altaz_to_vector(alt, az)
            
            if point['euler_angles'] is not None:
                euler = point['euler_angles']
                actual_vec = self.euler_to_vector(euler)
                
                A.append(actual_vec)
                b.append(theoretical_vec)
        
        A = np.array(A)
        b = np.array(b)
        
        # Solve using least squares with regularization
        # This accounts for measurement errors
        ATA = A.T @ A + 0.001 * np.eye(3)  # Regularization
        ATb = A.T @ b
        
        # Each column of solution is transformation for that axis
        transform = np.linalg.solve(ATA, ATb)
        
        self.alignment_matrix = transform
        self.is_aligned = True
        self.save_calibration()
        
        # Calculate RMS error
        errors = []
        for i, point in enumerate(points):
            predicted = transform @ A[i]
            error = np.linalg.norm(predicted - b[i])
            errors.append(error)
        
        rms_error = np.sqrt(np.mean(np.array(errors)**2))
        
        self.get_logger().info(f'3-star alignment complete. RMS error: {np.degrees(rms_error):.3f}°')
        self.publish_alignment_status()
    
    def transform_to_celestial(self, imu_euler):
        """Transform IMU orientation to celestial coordinates using alignment."""
        if not self.is_aligned:
            return None
        
        # Convert IMU Euler to vector
        imu_vec = self.euler_to_vector(imu_euler)
        
        # Apply transformation
        celestial_vec = self.alignment_matrix @ imu_vec
        
        # Add offset for 1-star alignment
        if self.alignment_method == '1star':
            celestial_vec[2] += self.alignment_offset[0]  # Azimuth offset
        
        # Convert back to alt/az
        alt, az = self.vector_to_altaz(celestial_vec)
        
        return alt, az
    
    def calculate_star_position(self, star_name):
        """Calculate current position of alignment star."""
        if not ASTROPY_AVAILABLE:
            # Return test positions
            test_positions = {
                'polaris': (np.radians(self.lat), 0.0),
                'vega': (np.radians(60), np.radians(45)),
                'arcturus': (np.radians(45), np.radians(90))
            }
            return test_positions.get(star_name, (None, None))
        
        current_time = Time.now()
        altaz_frame = AltAz(obstime=current_time, location=self.location)
        
        # Define bright alignment stars
        stars = {
            'polaris': coord.SkyCoord(ra='02h31m49s', dec='+89d15m51s'),
            'vega': coord.SkyCoord(ra='18h36m56s', dec='+38d47m01s'),
            'arcturus': coord.SkyCoord(ra='14h15m40s', dec='+19d10m57s'),
            'capella': coord.SkyCoord(ra='05h16m41s', dec='+45d59m53s'),
            'rigel': coord.SkyCoord(ra='05h14m32s', dec='-08d12m06s'),
            'procyon': coord.SkyCoord(ra='07h39m18s', dec='+05d13m30s'),
            'betelgeuse': coord.SkyCoord(ra='05h55m10s', dec='+07d24m26s'),
            'sirius': coord.SkyCoord(ra='06h45m09s', dec='-16d42m58s'),
            'altair': coord.SkyCoord(ra='19h50m47s', dec='+08d52m06s'),
            'deneb': coord.SkyCoord(ra='20h41m26s', dec='+45d16m49s')
        }
        
        if star_name not in stars:
            return None, None
        
        star_coord = stars[star_name]
        star_altaz = star_coord.transform_to(altaz_frame)
        
        if star_altaz.alt.rad < 0:
            return None, None  # Below horizon
        
        return star_altaz.alt.rad, star_altaz.az.rad
    
    def select_alignment_star(self):
        """Auto-select best star for alignment based on visibility and position."""
        if not ASTROPY_AVAILABLE:
            return 'polaris'
        
        current_time = Time.now()
        altaz_frame = AltAz(obstime=current_time, location=self.location)
        
        # Check bright stars
        candidates = []
        star_names = ['vega', 'arcturus', 'capella', 'sirius', 'altair']
        
        for star_name in star_names:
            alt, az = self.calculate_star_position(star_name)
            if alt and alt > np.radians(20):  # At least 20° above horizon
                candidates.append((star_name, alt))
        
        if candidates:
            # Choose highest star
            candidates.sort(key=lambda x: x[1], reverse=True)
            return candidates[0][0]
        
        return 'polaris'  # Default fallback
    
    def altaz_to_vector(self, alt, az):
        """Convert altitude/azimuth to unit vector."""
        x = np.cos(alt) * np.sin(az)
        y = np.cos(alt) * np.cos(az)
        z = np.sin(alt)
        return np.array([x, y, z])
    
    def vector_to_altaz(self, vec):
        """Convert unit vector to altitude/azimuth."""
        vec = vec / np.linalg.norm(vec)  # Normalize
        alt = np.arcsin(vec[2])
        az = np.arctan2(vec[0], vec[1])
        return alt, az
    
    def euler_to_vector(self, euler):
        """Convert Euler angles to pointing vector."""
        # Assuming euler = [yaw, roll, pitch]
        yaw, roll, pitch = euler
        
        # Create rotation matrix
        R = Rotation.from_euler('zyx', [yaw, roll, pitch])
        
        # Forward vector
        forward = np.array([0, 1, 0])  # Assuming Y is forward
        pointing = R.apply(forward)
        
        return pointing
    
    def quaternion_to_array(self, quat):
        """Convert quaternion message to array."""
        if quat:
            return [quat.w, quat.x, quat.y, quat.z]
        return [1.0, 0.0, 0.0, 0.0]
    
    def publish_alignment_status(self):
        """Publish current alignment status."""
        status_msg = String()
        if self.is_aligned:
            status_msg.data = f"Aligned ({self.alignment_method}): {len(self.alignment_points)} stars"
        else:
            status_msg.data = f"Not aligned: {len(self.alignment_points)}/{self.get_required_stars()} stars"
        self.status_pub.publish(status_msg)
        
        aligned_msg = Bool()
        aligned_msg.data = self.is_aligned
        self.aligned_pub.publish(aligned_msg)
        
        if self.is_aligned and self.alignment_method == '1star':
            offset_msg = Vector3()
            offset_msg.x = self.alignment_offset[0]
            offset_msg.y = self.alignment_offset[1]
            offset_msg.z = self.alignment_offset[2]
            self.offset_pub.publish(offset_msg)
    
    def get_required_stars(self):
        """Get number of stars required for current alignment method."""
        return {'1star': 1, '2star': 2, '3star': 3}.get(self.alignment_method, 2)
    
    def save_calibration(self):
        """Save alignment calibration to file."""
        calib_data = {
            'method': self.alignment_method,
            'is_aligned': self.is_aligned,
            'alignment_matrix': self.alignment_matrix.tolist(),
            'alignment_offset': self.alignment_offset.tolist(),
            'alignment_points': self.alignment_points,
            'location': {
                'latitude': self.lat,
                'longitude': self.lon,
                'altitude': self.alt
            },
            'timestamp': datetime.utcnow().isoformat()
        }
        
        filepath = os.path.expanduser(f'~/{self.calib_file}')
        with open(filepath, 'w') as f:
            json.dump(calib_data, f, indent=2)
        
        self.get_logger().info(f'Calibration saved to {filepath}')
    
    def load_calibration(self):
        """Load alignment calibration from file."""
        filepath = os.path.expanduser(f'~/{self.calib_file}')
        
        if not os.path.exists(filepath):
            self.get_logger().info('No calibration file found')
            return
        
        try:
            with open(filepath, 'r') as f:
                calib_data = json.load(f)
            
            self.alignment_method = calib_data.get('method', '2star')
            self.is_aligned = calib_data.get('is_aligned', False)
            self.alignment_matrix = np.array(calib_data.get('alignment_matrix', np.eye(3).tolist()))
            self.alignment_offset = np.array(calib_data.get('alignment_offset', [0, 0, 0]))
            self.alignment_points = calib_data.get('alignment_points', [])
            
            self.get_logger().info(f'Calibration loaded from {filepath}')
            self.publish_alignment_status()
            
        except Exception as e:
            self.get_logger().error(f'Failed to load calibration: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = AlignmentCalibration()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
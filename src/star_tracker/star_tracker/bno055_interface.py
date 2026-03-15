#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Header, Float32MultiArray
import numpy as np
import serial
import struct
import time

try:
    import smbus2 as smbus
    SMBUS_AVAILABLE = True
except ImportError:
    try:
        import smbus
        SMBUS_AVAILABLE = True
    except ImportError:
        SMBUS_AVAILABLE = False
        print("Warning: smbus/smbus2 not available")


class BNO055Interface(Node):
    """ROS2 interface for BNO055 9-DOF IMU sensor."""
    
    # BNO055 Registers
    CHIP_ID = 0xA0
    
    # Operation modes
    OPERATION_MODE_CONFIG = 0x00
    OPERATION_MODE_NDOF = 0x0C
    
    # Output registers
    EULER_H_LSB = 0x1A
    QUATERNION_W_LSB = 0x20
    LINEAR_ACCEL_X_LSB = 0x28
    GRAVITY_X_LSB = 0x2E
    TEMP = 0x34
    CALIB_STAT = 0x35
    
    def __init__(self):
        super().__init__('bno055_interface')
        
        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x28)  # or 0x29
        self.declare_parameter('serial_port', '')
        self.declare_parameter('update_rate', 50.0)  # Hz
        self.declare_parameter('use_magnetometer', True)
        self.declare_parameter('calibration_file', '')
        
        # Get parameters
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.serial_port = self.get_parameter('serial_port').value
        self.update_rate = self.get_parameter('update_rate').value
        self.use_mag = self.get_parameter('use_magnetometer').value
        self.calib_file = self.get_parameter('calibration_file').value
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.temp_pub = self.create_publisher(Temperature, 'imu/temperature', 10)
        self.euler_pub = self.create_publisher(Vector3, 'imu/euler', 10)
        self.calib_pub = self.create_publisher(Float32MultiArray, 'imu/calibration_status', 10)
        
        # Initialize sensor
        if not self.init_sensor():
            self.get_logger().error('Failed to initialize BNO055')
            return
        
        # Load calibration if available
        if self.calib_file:
            self.load_calibration(self.calib_file)
        
        # Timer for sensor updates
        self.timer = self.create_timer(1.0 / self.update_rate, self.sensor_callback)
        
        self.get_logger().info('BNO055 interface initialized')
    
    def init_sensor(self):
        """Initialize BNO055 sensor via I2C or serial."""
        try:
            if SMBUS_AVAILABLE and not self.serial_port:
                # Use direct I2C via smbus
                self.bus = smbus.SMBus(self.i2c_bus)

                # Check chip ID
                chip_id = self.bus.read_byte_data(self.i2c_address, 0x00)
                if chip_id != self.CHIP_ID:
                    self.get_logger().warn(f'Unexpected chip ID: 0x{chip_id:02x}, expected 0x{self.CHIP_ID:02x}')

                # Reset the sensor
                self.bus.write_byte_data(self.i2c_address, 0x3F, 0x20)  # SYS_TRIGGER reset
                time.sleep(0.7)  # Wait for reset

                # Set to config mode
                self.bus.write_byte_data(self.i2c_address, 0x3D, self.OPERATION_MODE_CONFIG)
                time.sleep(0.02)

                # Set to NDOF mode (9-DOF fusion)
                self.bus.write_byte_data(self.i2c_address, 0x3D, self.OPERATION_MODE_NDOF)
                time.sleep(0.02)

                self.get_logger().info('BNO055 initialized via I2C')
                return True
            elif self.serial_port:
                # Use serial interface
                self.serial = serial.Serial(self.serial_port, 115200, timeout=1)
                # Add serial initialization commands here
                return True
            else:
                self.get_logger().error('No valid interface available')
                return False
        except Exception as e:
            self.get_logger().error(f'Sensor init failed: {e}')
            return False
    
    def sensor_callback(self):
        """Read sensor data and publish to ROS topics."""
        try:
            # Read all sensor data
            quaternion = self.read_quaternion()
            euler = self.read_euler()
            linear_accel = self.read_linear_acceleration()
            angular_vel = self.read_gyroscope()
            magnetic = self.read_magnetometer()
            temperature = self.read_temperature()
            calibration = self.read_calibration_status()
            
            # Create timestamp
            stamp = self.get_clock().now().to_msg()
            
            # Publish IMU data
            imu_msg = Imu()
            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = 'imu_link'
            
            imu_msg.orientation.x = quaternion[1]
            imu_msg.orientation.y = quaternion[2]
            imu_msg.orientation.z = quaternion[3]
            imu_msg.orientation.w = quaternion[0]
            
            imu_msg.angular_velocity.x = angular_vel[0]
            imu_msg.angular_velocity.y = angular_vel[1]
            imu_msg.angular_velocity.z = angular_vel[2]
            
            imu_msg.linear_acceleration.x = linear_accel[0]
            imu_msg.linear_acceleration.y = linear_accel[1]
            imu_msg.linear_acceleration.z = linear_accel[2]
            
            # Set covariance based on calibration status
            calib_level = min(calibration)
            if calib_level == 3:  # Fully calibrated
                orientation_cov = 0.001
                angular_vel_cov = 0.001
                linear_accel_cov = 0.01
            elif calib_level == 2:  # Partially calibrated
                orientation_cov = 0.01
                angular_vel_cov = 0.01
                linear_accel_cov = 0.1
            else:  # Poor calibration
                orientation_cov = 0.1
                angular_vel_cov = 0.1
                linear_accel_cov = 1.0
            
            imu_msg.orientation_covariance[0] = orientation_cov
            imu_msg.orientation_covariance[4] = orientation_cov
            imu_msg.orientation_covariance[8] = orientation_cov
            
            self.imu_pub.publish(imu_msg)
            
            # Publish Euler angles
            euler_msg = Vector3()
            euler_msg.x = euler[0]  # Heading/Yaw
            euler_msg.y = euler[1]  # Roll
            euler_msg.z = euler[2]  # Pitch
            self.euler_pub.publish(euler_msg)
            
            # Publish magnetometer data
            if self.use_mag:
                mag_msg = MagneticField()
                mag_msg.header.stamp = stamp
                mag_msg.header.frame_id = 'imu_link'
                mag_msg.magnetic_field.x = magnetic[0] * 1e-6  # Convert to Tesla
                mag_msg.magnetic_field.y = magnetic[1] * 1e-6
                mag_msg.magnetic_field.z = magnetic[2] * 1e-6
                self.mag_pub.publish(mag_msg)
            
            # Publish temperature
            temp_msg = Temperature()
            temp_msg.header.stamp = stamp
            temp_msg.header.frame_id = 'imu_link'
            temp_msg.temperature = temperature
            self.temp_pub.publish(temp_msg)
            
            # Publish calibration status
            calib_msg = Float32MultiArray()
            calib_msg.data = [float(c) for c in calibration]
            self.calib_pub.publish(calib_msg)
            
        except Exception as e:
            self.get_logger().error(f'Sensor read failed: {e}')
    
    def read_quaternion(self):
        """Read quaternion orientation."""
        if SMBUS_AVAILABLE and hasattr(self, 'bus'):
            try:
                # Read 8 bytes starting from QUATERNION_W_LSB (0x20)
                data = self.bus.read_i2c_block_data(self.i2c_address, self.QUATERNION_W_LSB, 8)
                # Convert to quaternion (w, x, y, z)
                w = struct.unpack('<h', bytes(data[0:2]))[0] / 16384.0
                x = struct.unpack('<h', bytes(data[2:4]))[0] / 16384.0
                y = struct.unpack('<h', bytes(data[4:6]))[0] / 16384.0
                z = struct.unpack('<h', bytes(data[6:8]))[0] / 16384.0
                return [w, x, y, z]
            except Exception as e:
                self.get_logger().debug(f'Failed to read quaternion: {e}')
        return [1.0, 0.0, 0.0, 0.0]  # Identity quaternion
    
    def read_euler(self):
        """Read Euler angles (heading, roll, pitch) in radians."""
        if SMBUS_AVAILABLE and hasattr(self, 'bus'):
            try:
                # Read 6 bytes starting from EULER_H_LSB (0x1A)
                data = self.bus.read_i2c_block_data(self.i2c_address, self.EULER_H_LSB, 6)
                # Convert to Euler angles (heading, roll, pitch) in degrees
                heading = struct.unpack('<h', bytes(data[0:2]))[0] / 16.0
                roll = struct.unpack('<h', bytes(data[2:4]))[0] / 16.0
                pitch = struct.unpack('<h', bytes(data[4:6]))[0] / 16.0
                # Convert to radians
                return [np.radians(heading), np.radians(roll), np.radians(pitch)]
            except Exception as e:
                self.get_logger().debug(f'Failed to read euler: {e}')
        return [0.0, 0.0, 0.0]
    
    def read_linear_acceleration(self):
        """Read linear acceleration (gravity compensated) in m/s^2."""
        if SMBUS_AVAILABLE and hasattr(self, 'bus'):
            try:
                # Read 6 bytes starting from LINEAR_ACCEL_X_LSB (0x28)
                data = self.bus.read_i2c_block_data(self.i2c_address, self.LINEAR_ACCEL_X_LSB, 6)
                # Convert to m/s^2 (100 LSB = 1 m/s^2)
                x = struct.unpack('<h', bytes(data[0:2]))[0] / 100.0
                y = struct.unpack('<h', bytes(data[2:4]))[0] / 100.0
                z = struct.unpack('<h', bytes(data[4:6]))[0] / 100.0
                return [x, y, z]
            except Exception as e:
                self.get_logger().debug(f'Failed to read linear acceleration: {e}')
        return [0.0, 0.0, 0.0]
    
    def read_gyroscope(self):
        """Read angular velocity in rad/s."""
        if SMBUS_AVAILABLE and hasattr(self, 'bus'):
            try:
                # Read 6 bytes starting from GYRO_DATA_X_LSB (0x14)
                data = self.bus.read_i2c_block_data(self.i2c_address, 0x14, 6)
                # Convert to degrees/s (16 LSB = 1 degree/s)
                x = struct.unpack('<h', bytes(data[0:2]))[0] / 16.0
                y = struct.unpack('<h', bytes(data[2:4]))[0] / 16.0
                z = struct.unpack('<h', bytes(data[4:6]))[0] / 16.0
                # Convert to rad/s
                return [np.radians(x), np.radians(y), np.radians(z)]
            except Exception as e:
                self.get_logger().debug(f'Failed to read gyroscope: {e}')
        return [0.0, 0.0, 0.0]
    
    def read_magnetometer(self):
        """Read magnetic field in microTesla."""
        if SMBUS_AVAILABLE and hasattr(self, 'bus'):
            try:
                # Read 6 bytes starting from MAG_DATA_X_LSB (0x0E)
                data = self.bus.read_i2c_block_data(self.i2c_address, 0x0E, 6)
                # Convert to microTesla (16 LSB = 1 microTesla)
                x = struct.unpack('<h', bytes(data[0:2]))[0] / 16.0
                y = struct.unpack('<h', bytes(data[2:4]))[0] / 16.0
                z = struct.unpack('<h', bytes(data[4:6]))[0] / 16.0
                return [x, y, z]
            except Exception as e:
                self.get_logger().debug(f'Failed to read magnetometer: {e}')
        return [0.0, 0.0, 0.0]
    
    def read_temperature(self):
        """Read temperature in Celsius."""
        if SMBUS_AVAILABLE and hasattr(self, 'bus'):
            try:
                # Read temperature byte
                temp = self.bus.read_byte_data(self.i2c_address, self.TEMP)
                return float(temp)  # Already in Celsius
            except Exception as e:
                self.get_logger().debug(f'Failed to read temperature: {e}')
        return 25.0
    
    def read_calibration_status(self):
        """Read calibration status (sys, gyro, accel, mag) 0-3."""
        if SMBUS_AVAILABLE and hasattr(self, 'bus'):
            try:
                # Read calibration status byte
                calib = self.bus.read_byte_data(self.i2c_address, self.CALIB_STAT)
                # Extract individual calibration levels (2 bits each)
                sys_calib = (calib >> 6) & 0x03
                gyro_calib = (calib >> 4) & 0x03
                accel_calib = (calib >> 2) & 0x03
                mag_calib = calib & 0x03
                return [sys_calib, gyro_calib, accel_calib, mag_calib]
            except Exception as e:
                self.get_logger().debug(f'Failed to read calibration: {e}')
        return [0, 0, 0, 0]
    
    def save_calibration(self, filename):
        """Save calibration data to file."""
        if SMBUS_AVAILABLE and hasattr(self, 'bus'):
            try:
                # Read calibration data (22 bytes starting at 0x55)
                calib_data = self.bus.read_i2c_block_data(self.i2c_address, 0x55, 22)
                with open(filename, 'wb') as f:
                    f.write(bytearray(calib_data))
                self.get_logger().info(f'Calibration saved to {filename}')
            except Exception as e:
                self.get_logger().error(f'Failed to save calibration: {e}')

    def load_calibration(self, filename):
        """Load calibration data from file."""
        try:
            with open(filename, 'rb') as f:
                calib_data = f.read()
            if SMBUS_AVAILABLE and hasattr(self, 'bus') and len(calib_data) == 22:
                # Switch to config mode
                self.bus.write_byte_data(self.i2c_address, 0x3D, self.OPERATION_MODE_CONFIG)
                time.sleep(0.02)
                # Write calibration data
                for i, byte in enumerate(calib_data):
                    self.bus.write_byte_data(self.i2c_address, 0x55 + i, byte)
                # Switch back to NDOF mode
                self.bus.write_byte_data(self.i2c_address, 0x3D, self.OPERATION_MODE_NDOF)
                time.sleep(0.02)
            self.get_logger().info(f'Calibration loaded from {filename}')
        except Exception as e:
            self.get_logger().warn(f'Failed to load calibration: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = BNO055Interface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import smbus
import struct
import numpy as np


class SimpleBNO055Node(Node):
    def __init__(self):
        super().__init__('simple_bno055_test')

        # I2C setup
        self.i2c_bus = 1
        self.i2c_address = 0x28

        # Publisher
        self.euler_pub = self.create_publisher(Vector3, '/imu/euler', 10)

        # Initialize I2C
        try:
            self.bus = smbus.SMBus(self.i2c_bus)
            self.get_logger().info(f'I2C bus {self.i2c_bus} opened successfully')

            # Check chip ID
            chip_id = self.bus.read_byte_data(self.i2c_address, 0x00)
            self.get_logger().info(f'BNO055 Chip ID: 0x{chip_id:02x}')

            # Set to NDOF mode
            self.bus.write_byte_data(self.i2c_address, 0x3D, 0x0C)
            self.get_logger().info('BNO055 set to NDOF mode')

        except Exception as e:
            self.get_logger().error(f'Failed to initialize I2C: {e}')
            return

        # Create timer for publishing
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        self.get_logger().info('Simple BNO055 test node started')

    def timer_callback(self):
        """Read and publish euler angles."""
        try:
            # Read 6 bytes of euler data from register 0x1A
            data = self.bus.read_i2c_block_data(self.i2c_address, 0x1A, 6)

            # Convert to degrees
            heading = struct.unpack('<h', bytes(data[0:2]))[0] / 16.0
            roll = struct.unpack('<h', bytes(data[2:4]))[0] / 16.0
            pitch = struct.unpack('<h', bytes(data[4:6]))[0] / 16.0

            # Convert to radians
            heading_rad = np.radians(heading)
            roll_rad = np.radians(roll)
            pitch_rad = np.radians(pitch)

            # Publish
            euler_msg = Vector3()
            euler_msg.x = heading_rad  # Yaw/heading
            euler_msg.y = roll_rad      # Roll
            euler_msg.z = pitch_rad     # Pitch

            self.euler_pub.publish(euler_msg)

            # Log every 10th message (1Hz)
            if int(self.get_clock().now().nanoseconds / 1e8) % 10 == 0:
                self.get_logger().info(
                    f'Euler: H={heading:.1f}° R={roll:.1f}° P={pitch:.1f}°'
                )

        except Exception as e:
            self.get_logger().error(f'Error reading BNO055: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleBNO055Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
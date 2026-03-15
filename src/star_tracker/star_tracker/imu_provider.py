#!/usr/bin/env python3

"""BNO055 IMU Provider for testing."""

import rclpy
from star_tracker.mock_providers import MockBNO055IMU

def main():
    rclpy.init()
    node = MockBNO055IMU()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
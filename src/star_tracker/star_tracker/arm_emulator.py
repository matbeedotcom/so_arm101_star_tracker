#!/usr/bin/env python3

"""SO-100 Arm Emulator for testing."""

import rclpy
from star_tracker.mock_providers import SO100ArmEmulator

def main():
    rclpy.init()
    node = SO100ArmEmulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
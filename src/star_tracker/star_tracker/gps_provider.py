#!/usr/bin/env python3

"""Toronto GPS Provider for testing."""

import rclpy
from star_tracker.mock_providers import TorontoGPSProvider

def main():
    rclpy.init()
    node = TorontoGPSProvider()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
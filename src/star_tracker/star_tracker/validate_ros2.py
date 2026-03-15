#!/usr/bin/env python3

"""
ROS2 message compatibility validation for Star Tracker.
Checks that all used message types are available and compatible.
"""

import sys

def validate_ros2_messages():
    """Validate all ROS2 message types used in the star tracker."""
    
    print("=== ROS2 Message Compatibility Validation ===\n")
    
    # Define all message types used in the project
    message_imports = [
        # sensor_msgs
        ('sensor_msgs.msg', 'NavSatFix', 'GPS position data'),
        ('sensor_msgs.msg', 'NavSatStatus', 'GPS status information'),
        ('sensor_msgs.msg', 'TimeReference', 'GPS time synchronization'),
        ('sensor_msgs.msg', 'Imu', 'IMU orientation and motion data'),
        ('sensor_msgs.msg', 'JointState', 'Robot joint positions and velocities'),
        ('sensor_msgs.msg', 'MagneticField', 'Magnetometer data'),
        ('sensor_msgs.msg', 'Temperature', 'Temperature sensor data'),
        
        # geometry_msgs  
        ('geometry_msgs.msg', 'Vector3', 'XYZ vector data'),
        ('geometry_msgs.msg', 'Vector3Stamped', 'Timestamped XYZ vector'),
        ('geometry_msgs.msg', 'Quaternion', 'Rotation quaternion'),
        ('geometry_msgs.msg', 'Twist', 'Linear and angular velocity'),
        
        # std_msgs
        ('std_msgs.msg', 'String', 'Text messages'),
        ('std_msgs.msg', 'Bool', 'Boolean values'),
        ('std_msgs.msg', 'Float64', 'Double precision numbers'),
        ('std_msgs.msg', 'Header', 'Message headers with timestamps'),
        ('std_msgs.msg', 'Float32MultiArray', 'Arrays of float32 values'),
        
        # trajectory_msgs
        ('trajectory_msgs.msg', 'JointTrajectory', 'Robot trajectory commands'),
        ('trajectory_msgs.msg', 'JointTrajectoryPoint', 'Individual trajectory points'),
        
        # control_msgs
        ('control_msgs.action', 'FollowJointTrajectory', 'Trajectory execution action')
    ]
    
    results = {'available': [], 'missing': []}
    
    for package, message_type, description in message_imports:
        try:
            # Attempt to import the message type
            if '.action' in package:
                # Handle action messages differently
                module_path = package.replace('.action', '.action')
                module = __import__(module_path, fromlist=[message_type])
                getattr(module, message_type)
            else:
                module = __import__(package, fromlist=[message_type])
                getattr(module, message_type)
            
            print(f"✓ {package}.{message_type} - {description}")
            results['available'].append((package, message_type, description))
            
        except ImportError as e:
            print(f"✗ {package}.{message_type} - {description} (MISSING)")
            print(f"  Error: {e}")
            results['missing'].append((package, message_type, description))
        except AttributeError as e:
            print(f"✗ {package}.{message_type} - {description} (NOT FOUND)")
            print(f"  Error: {e}")
            results['missing'].append((package, message_type, description))
    
    return results

def validate_ros2_packages():
    """Validate ROS2 package dependencies."""
    
    print(f"\n=== ROS2 Package Dependencies ===\n")
    
    ros2_packages = [
        ('rclpy', 'ROS2 Python client library'),
        ('rclpy.node', 'ROS2 node base class'),
        ('rclpy.action', 'ROS2 action client/server'),
        ('rclpy.executors', 'ROS2 execution management'),
        ('rclpy.callback_groups', 'ROS2 callback management'),
        ('launch', 'ROS2 launch system'),
        ('launch.actions', 'Launch actions'),
        ('launch.substitutions', 'Launch parameter substitutions'),
        ('launch_ros.actions', 'ROS2-specific launch actions'),
        ('launch.conditions', 'Launch conditional logic'),
    ]
    
    results = {'available': [], 'missing': []}
    
    for package, description in ros2_packages:
        try:
            __import__(package)
            print(f"✓ {package} - {description}")
            results['available'].append((package, description))
        except ImportError as e:
            print(f"✗ {package} - {description} (MISSING)")
            print(f"  Error: {e}")
            results['missing'].append((package, description))
    
    return results

def validate_message_structure():
    """Validate expected message field structure."""
    
    print(f"\n=== Message Structure Validation ===\n")
    
    try:
        # Test NavSatFix structure
        from sensor_msgs.msg import NavSatFix, NavSatStatus
        
        nav_fix = NavSatFix()
        expected_fields = ['header', 'status', 'latitude', 'longitude', 'altitude', 'position_covariance']
        
        for field in expected_fields:
            if hasattr(nav_fix, field):
                print(f"✓ NavSatFix.{field}")
            else:
                print(f"✗ NavSatFix.{field} (MISSING)")
                return False
        
        # Test NavSatStatus constants
        status_constants = ['STATUS_NO_FIX', 'STATUS_FIX', 'SERVICE_GPS']
        for constant in status_constants:
            if hasattr(NavSatStatus, constant):
                print(f"✓ NavSatStatus.{constant}")
            else:
                print(f"✗ NavSatStatus.{constant} (MISSING)")
                return False
        
        # Test JointTrajectory structure
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        
        traj = JointTrajectory()
        traj_fields = ['header', 'joint_names', 'points']
        
        for field in traj_fields:
            if hasattr(traj, field):
                print(f"✓ JointTrajectory.{field}")
            else:
                print(f"✗ JointTrajectory.{field} (MISSING)")
                return False
        
        # Test JointTrajectoryPoint structure
        point = JointTrajectoryPoint()
        point_fields = ['positions', 'velocities', 'accelerations', 'effort', 'time_from_start']
        
        for field in point_fields:
            if hasattr(point, field):
                print(f"✓ JointTrajectoryPoint.{field}")
            else:
                print(f"✗ JointTrajectoryPoint.{field} (MISSING)")
                return False
        
        print("✓ All message structures validated")
        return True
        
    except ImportError as e:
        print(f"✗ Cannot validate message structures - missing packages: {e}")
        return False
    except Exception as e:
        print(f"✗ Message structure validation error: {e}")
        return False

def validate_action_interfaces():
    """Validate action message interfaces."""
    
    print(f"\n=== Action Interface Validation ===\n")
    
    try:
        from control_msgs.action import FollowJointTrajectory
        
        # Check action structure
        action_parts = ['Goal', 'Result', 'Feedback']
        
        for part in action_parts:
            if hasattr(FollowJointTrajectory, part):
                print(f"✓ FollowJointTrajectory.{part}")
            else:
                print(f"✗ FollowJointTrajectory.{part} (MISSING)")
                return False
        
        # Test goal structure
        goal = FollowJointTrajectory.Goal()
        if hasattr(goal, 'trajectory'):
            print(f"✓ FollowJointTrajectory.Goal.trajectory")
        else:
            print(f"✗ FollowJointTrajectory.Goal.trajectory (MISSING)")
            return False
        
        print("✓ Action interfaces validated")
        return True
        
    except ImportError as e:
        print(f"✗ Cannot validate action interfaces - missing packages: {e}")
        return False
    except Exception as e:
        print(f"✗ Action interface validation error: {e}")
        return False

def main():
    """Run all ROS2 compatibility validations."""
    
    print("ROS2 Compatibility Validation for Star Tracker")
    print("=" * 60)
    
    # Run all validation tests
    message_results = validate_ros2_messages()
    package_results = validate_ros2_packages()
    structure_valid = validate_message_structure()
    action_valid = validate_action_interfaces()
    
    # Summary
    print(f"\n" + "=" * 60)
    print("ROS2 COMPATIBILITY SUMMARY")
    print("=" * 60)
    
    print(f"Messages Available: {len(message_results['available'])}")
    print(f"Messages Missing:   {len(message_results['missing'])}")
    print(f"Packages Available: {len(package_results['available'])}")
    print(f"Packages Missing:   {len(package_results['missing'])}")
    print(f"Message Structures: {'✓ Valid' if structure_valid else '✗ Invalid'}")
    print(f"Action Interfaces:  {'✓ Valid' if action_valid else '✗ Invalid'}")
    
    # Overall assessment
    messages_ok = len(message_results['missing']) == 0
    packages_ok = len(package_results['missing']) == 0
    overall_success = messages_ok and packages_ok and structure_valid and action_valid
    
    print(f"\nOVERALL ROS2 COMPATIBILITY: {'✓ COMPATIBLE' if overall_success else '✗ ISSUES FOUND'}")
    
    if not overall_success:
        print("\nRECOMMENDATIONS:")
        if not packages_ok:
            print("- Install missing ROS2 packages:")
            print("  sudo apt install ros-humble-sensor-msgs ros-humble-geometry-msgs")
            print("  sudo apt install ros-humble-trajectory-msgs ros-humble-control-msgs")
        
        if message_results['missing']:
            print("- Some message types may require additional packages")
        
        if not structure_valid:
            print("- Message structures may be incompatible with ROS2 Humble")
        
        if not action_valid:
            print("- Action interfaces may need ros-humble-control-msgs package")
    
    return overall_success

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)
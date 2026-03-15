#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os


def generate_launch_description():
    # Test configuration arguments
    test_location_arg = DeclareLaunchArgument(
        'test_location',
        default_value='new_york',
        description='Test location: new_york, london, sydney, tokyo'
    )
    
    target_object_arg = DeclareLaunchArgument(
        'target_object',
        default_value='moon',
        description='Target object for testing: sun, moon, polaris, sirius'
    )
    
    test_duration_arg = DeclareLaunchArgument(
        'test_duration',
        default_value='120',
        description='Test duration in seconds'
    )
    
    enable_gps_test_arg = DeclareLaunchArgument(
        'enable_gps_test',
        default_value='true',
        description='Enable GPS mock provider for testing'
    )
    
    enable_imu_test_arg = DeclareLaunchArgument(
        'enable_imu_test',
        default_value='true',
        description='Enable IMU mock provider for testing'
    )
    
    enable_arm_emulator_arg = DeclareLaunchArgument(
        'enable_arm_emulator',
        default_value='true',
        description='Enable SO-100 arm emulator'
    )
    
    gps_noise_level_arg = DeclareLaunchArgument(
        'gps_noise_level',
        default_value='0.000001',
        description='GPS coordinate noise level for realism'
    )
    
    imu_noise_level_arg = DeclareLaunchArgument(
        'imu_noise_level',
        default_value='0.01',
        description='IMU orientation noise level in radians'
    )
    
    # Test coordinates for different locations
    test_locations = {
        'new_york': {'lat': 40.7128, 'lon': -74.0060, 'alt': 10.0},
        'london': {'lat': 51.5074, 'lon': -0.1278, 'alt': 35.0},
        'sydney': {'lat': -33.8688, 'lon': 151.2093, 'alt': 58.0},
        'tokyo': {'lat': 35.6762, 'lon': 139.6503, 'alt': 40.0}
    }
    
    # Test Framework Node (runs all mock providers)
    test_framework_node = Node(
        package='star_tracker',
        executable='test_framework',
        name='test_framework',
        parameters=[{
            'test_location': LaunchConfiguration('test_location'),
            'gps_noise_level': LaunchConfiguration('gps_noise_level'),
            'imu_noise_level': LaunchConfiguration('imu_noise_level'),
            'enable_gps_test': LaunchConfiguration('enable_gps_test'),
            'enable_imu_test': LaunchConfiguration('enable_imu_test'),
            'enable_arm_emulator': LaunchConfiguration('enable_arm_emulator')
        }],
        output='screen'
    )
    
    # Star Tracker Node (under test)
    star_tracker_node = Node(
        package='star_tracker',
        executable='star_tracker_node',
        name='star_tracker_node',
        parameters=[{
            'update_rate': 2.0,  # Faster for testing
            'location_lat': 40.7128,  # Will be overridden by GPS
            'location_lon': -74.0060,
            'location_alt': 10.0,
            'target_object': LaunchConfiguration('target_object'),
            'tracking_mode': 'continuous',
            'use_imu': LaunchConfiguration('enable_imu_test'),
            'use_gps': LaunchConfiguration('enable_gps_test'),
            'goto_mode': LaunchConfiguration('enable_imu_test'),
            'gps_timeout': 10.0,  # Shorter timeout for testing
            'alignment_file': 'test_star_alignment.json'
        }],
        output='screen'
    )
    
    # Test Suite Node
    test_suite_node = Node(
        package='star_tracker',
        executable='test_framework',
        name='star_tracker_test_suite',
        parameters=[{
            'test_duration': LaunchConfiguration('test_duration'),
            'target_object': LaunchConfiguration('target_object')
        }],
        output='screen'
    )
    
    # Delayed start for test suite (let other nodes initialize)
    delayed_test_suite = TimerAction(
        period=5.0,
        actions=[test_suite_node]
    )
    
    # Launch info
    launch_info = LogInfo(
        msg=[
            '\\n=== Star Tracker Automated Test Suite ===\\n',
            'Test Location: ', LaunchConfiguration('test_location'), '\\n',
            'Target Object: ', LaunchConfiguration('target_object'), '\\n',
            'Test Duration: ', LaunchConfiguration('test_duration'), ' seconds\\n',
            'GPS Testing: ', LaunchConfiguration('enable_gps_test'), '\\n',
            'IMU Testing: ', LaunchConfiguration('enable_imu_test'), '\\n',
            'Arm Emulation: ', LaunchConfiguration('enable_arm_emulator'), '\\n',
            '==========================================\\n'
        ]
    )
    
    return LaunchDescription([
        # Arguments
        test_location_arg,
        target_object_arg,
        test_duration_arg,
        enable_gps_test_arg,
        enable_imu_test_arg,
        enable_arm_emulator_arg,
        gps_noise_level_arg,
        imu_noise_level_arg,
        
        # Launch info
        launch_info,
        
        # Test framework
        test_framework_node,
        
        # System under test
        star_tracker_node,
        
        # Test suite (delayed start)
        delayed_test_suite,
    ])
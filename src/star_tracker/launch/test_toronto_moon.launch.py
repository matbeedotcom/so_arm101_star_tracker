#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os


def generate_launch_description():
    """
    Launch file for testing star tracker with moon tracking from Toronto.
    Includes GPS and IMU emulation for GoTo mode testing.
    """
    
    # Toronto coordinates
    toronto_lat = 43.6532
    toronto_lon = -79.3832
    toronto_alt = 76.0  # meters above sea level
    
    # Test parameters
    target_object_arg = DeclareLaunchArgument(
        'target_object',
        default_value='moon',
        description='Celestial target (sun, moon, polaris, sirius)'
    )
    
    test_duration_arg = DeclareLaunchArgument(
        'test_duration',
        default_value='300',  # 5 minutes
        description='Test duration in seconds'
    )
    
    use_gps_arg = DeclareLaunchArgument(
        'use_gps',
        default_value='true',
        description='Use GPS emulation'
    )
    
    use_imu_arg = DeclareLaunchArgument(
        'use_imu',
        default_value='true',
        description='Use IMU emulation for GoTo mode'
    )
    
    goto_mode_arg = DeclareLaunchArgument(
        'goto_mode',
        default_value='true',
        description='Enable GoTo mode (requires IMU)'
    )
    
    gps_noise_arg = DeclareLaunchArgument(
        'gps_noise_level',
        default_value='0.000001',
        description='GPS coordinate noise (degrees)'
    )
    
    imu_noise_arg = DeclareLaunchArgument(
        'imu_noise_level',
        default_value='0.01',
        description='IMU orientation noise (radians)'
    )
    
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='2.0',
        description='Tracking update rate (Hz)'
    )
    
    # Mock GPS Provider (Toronto location)
    gps_provider = Node(
        package='star_tracker',
        executable='test_framework',
        name='mock_gps_toronto',
        parameters=[{
            'node_type': 'gps',
            'test_location_lat': toronto_lat,
            'test_location_lon': toronto_lon,
            'test_location_alt': toronto_alt,
            'gps_noise_level': LaunchConfiguration('gps_noise_level'),
            'acquisition_delay': 5.0,  # seconds to acquire fix
            'update_rate': 1.0  # 1 Hz GPS updates
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_gps'))
    )
    
    # Mock IMU Provider
    imu_provider = Node(
        package='star_tracker',
        executable='test_framework',
        name='mock_imu_bno055',
        parameters=[{
            'node_type': 'imu',
            'noise_level': LaunchConfiguration('imu_noise_level'),
            'update_rate': 50.0,  # 50 Hz IMU updates
            'simulate_drift': True,
            'drift_rate': 0.001  # radians per second
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_imu'))
    )
    
    # Emulated SO-100 Arm
    arm_emulator = Node(
        package='star_tracker',
        executable='test_framework',
        name='emulated_so100_arm',
        parameters=[{
            'node_type': 'arm',
            'joint_names': ['Shoulder_Rotation', 'Shoulder_Pitch', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll'],
            'max_velocities': [1.0, 1.0, 1.0, 1.0, 1.0],  # rad/s
            'update_rate': 10.0,  # 10 Hz joint state updates
            'simulate_backlash': True,
            'backlash_amount': 0.005  # radians
        }],
        output='screen'
    )
    
    # Star Tracker Node (System Under Test)
    star_tracker_node = Node(
        package='star_tracker',
        executable='star_tracker_node',
        name='star_tracker_toronto',
        parameters=[{
            # Location (will be overridden by GPS if enabled)
            'location_lat': toronto_lat,
            'location_lon': toronto_lon,
            'location_alt': toronto_alt,
            
            # Target and tracking
            'target_object': LaunchConfiguration('target_object'),
            'update_rate': LaunchConfiguration('update_rate'),
            'tracking_mode': 'continuous',
            
            # GPS/IMU integration
            'use_gps': LaunchConfiguration('use_gps'),
            'use_imu': LaunchConfiguration('use_imu'),
            'goto_mode': LaunchConfiguration('goto_mode'),
            'gps_timeout': 10.0,
            'imu_timeout': 5.0,
            
            # Tracking parameters
            'min_elevation': 10.0,  # degrees
            'max_slew_rate': 10.0,  # degrees/second
            'alignment_file': '/tmp/toronto_star_alignment.json',
            
            # Logging
            'verbose_logging': True,
            'log_tracking_data': True,
            'tracking_log_file': '/tmp/toronto_moon_tracking.log'
        }],
        output='screen'
    )
    
    # Test Monitor and Validator
    test_monitor = Node(
        package='star_tracker',
        executable='test_framework',
        name='test_monitor',
        parameters=[{
            'node_type': 'monitor',
            'test_duration': LaunchConfiguration('test_duration'),
            'target_object': LaunchConfiguration('target_object'),
            'expected_location': 'toronto',
            'validate_tracking': True,
            'validate_gps': LaunchConfiguration('use_gps'),
            'validate_imu': LaunchConfiguration('use_imu'),
            'save_results': True,
            'results_file': '/tmp/toronto_test_results.json',
            'performance_metrics': True,
            'measure_latency': True,
            'measure_accuracy': True
        }],
        output='screen'
    )
    
    # Delayed test start (let nodes initialize)
    delayed_test_start = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg=[
                '\\n',
                '=' * 60, '\\n',
                'STARTING MOON TRACKING TEST FROM TORONTO\\n',
                '=' * 60, '\\n',
                'Location: Toronto, Canada (', str(toronto_lat), '°N, ', str(toronto_lon), '°W)\\n',
                'Target: ', LaunchConfiguration('target_object'), '\\n',
                'GPS Enabled: ', LaunchConfiguration('use_gps'), '\\n',
                'IMU Enabled: ', LaunchConfiguration('use_imu'), '\\n',
                'GoTo Mode: ', LaunchConfiguration('goto_mode'), '\\n',
                'Test Duration: ', LaunchConfiguration('test_duration'), ' seconds\\n',
                '=' * 60, '\\n'
            ])
        ]
    )
    
    # Test completion notification
    test_completion = TimerAction(
        period=PythonExpression([
            LaunchConfiguration('test_duration'),
            ' + 15.0'  # Extra time for cleanup
        ]),
        actions=[
            LogInfo(msg=[
                '\\n',
                '=' * 60, '\\n',
                'TEST COMPLETE\\n',
                'Results saved to /tmp/toronto_test_results.json\\n',
                'Tracking log: /tmp/toronto_moon_tracking.log\\n',
                '=' * 60, '\\n'
            ])
        ]
    )
    
    # Initial info
    initial_info = LogInfo(
        msg=[
            '\\n',
            '=' * 60, '\\n',
            'TORONTO MOON TRACKING TEST\\n',
            '=' * 60, '\\n',
            'Initializing test environment...\\n',
            '- Mock GPS Provider (Toronto coordinates)\\n',
            '- Mock IMU Provider (BNO055 emulation)\\n',
            '- Emulated SO-100 Arm\\n',
            '- Star Tracker with GoTo Mode\\n',
            '=' * 60, '\\n'
        ]
    )
    
    return LaunchDescription([
        # Arguments
        target_object_arg,
        test_duration_arg,
        use_gps_arg,
        use_imu_arg,
        goto_mode_arg,
        gps_noise_arg,
        imu_noise_arg,
        update_rate_arg,
        
        # Initial info
        initial_info,
        
        # Mock providers
        gps_provider,
        imu_provider,
        arm_emulator,
        
        # Star tracker (delayed start to let mocks initialize)
        TimerAction(
            period=3.0,
            actions=[star_tracker_node]
        ),
        
        # Test monitor
        TimerAction(
            period=5.0,
            actions=[test_monitor]
        ),
        
        # Test notifications
        delayed_test_start,
        test_completion
    ])
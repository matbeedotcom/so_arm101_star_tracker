#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import EqualsSubstitution
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Launch arguments
    location_lat_arg = DeclareLaunchArgument(
        'location_lat',
        default_value='40.7128',
        description='Fallback latitude in degrees (if GPS not available)'
    )
    
    location_lon_arg = DeclareLaunchArgument(
        'location_lon', 
        default_value='-74.0060',
        description='Fallback longitude in degrees (if GPS not available)'
    )
    
    location_alt_arg = DeclareLaunchArgument(
        'location_alt',
        default_value='10.0',
        description='Fallback altitude in meters (if GPS not available)'
    )
    
    target_object_arg = DeclareLaunchArgument(
        'target_object',
        default_value='polaris',
        description='Celestial object to track (sun, moon, polaris, sirius, etc.)'
    )
    
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='1.0',
        description='Tracking update rate in Hz'
    )
    
    use_imu_arg = DeclareLaunchArgument(
        'use_imu',
        default_value='false',
        description='Enable IMU integration for GoTo mode'
    )
    
    use_gps_arg = DeclareLaunchArgument(
        'use_gps',
        default_value='true',
        description='Enable GPS integration for precise location and timing'
    )
    
    goto_mode_arg = DeclareLaunchArgument(
        'goto_mode',
        default_value='false',
        description='Enable GoTo mode with IMU feedback'
    )
    
    # GPS parameters
    gps_serial_port_arg = DeclareLaunchArgument(
        'gps_serial_port',
        default_value='/dev/ttyAMA0',
        description='GPS serial port (default UART on Raspberry Pi)'
    )
    
    gps_baud_rate_arg = DeclareLaunchArgument(
        'gps_baud_rate',
        default_value='9600',
        description='GPS baud rate'
    )
    
    gps_timeout_arg = DeclareLaunchArgument(
        'gps_timeout',
        default_value='30.0',
        description='Timeout in seconds to wait for GPS fix'
    )
    
    # IMU parameters (if using BNO055)
    imu_serial_port_arg = DeclareLaunchArgument(
        'imu_serial_port',
        default_value='',
        description='IMU serial port (empty for I2C)'
    )
    
    imu_i2c_bus_arg = DeclareLaunchArgument(
        'imu_i2c_bus',
        default_value='1',
        description='IMU I2C bus number'
    )
    
    imu_update_rate_arg = DeclareLaunchArgument(
        'imu_update_rate',
        default_value='50.0',
        description='IMU update rate in Hz'
    )
    
    # GPS Interface Node
    gps_node = Node(
        package='star_tracker',
        executable='gps_interface',
        name='gps_interface',
        parameters=[{
            'serial_port': LaunchConfiguration('gps_serial_port'),
            'baud_rate': LaunchConfiguration('gps_baud_rate'),
            'update_rate': 1.0,
            'timeout': 2.0,
            'enable_pps': False
        }],
        output='screen',
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('use_gps'), 'true'))
    )
    
    # BNO055 IMU Interface Node (optional)
    imu_node = Node(
        package='star_tracker',
        executable='bno055_interface',
        name='bno055_interface',
        parameters=[{
            'i2c_bus': LaunchConfiguration('imu_i2c_bus'),
            'i2c_address': 0x28,
            'serial_port': LaunchConfiguration('imu_serial_port'),
            'update_rate': LaunchConfiguration('imu_update_rate'),
            'use_magnetometer': True,
            'calibration_file': ''
        }],
        output='screen',
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('use_imu'), 'true'))
    )
    
    # Star Tracker Node with GPS integration
    star_tracker_node = Node(
        package='star_tracker',
        executable='star_tracker_node',
        name='star_tracker_node',
        parameters=[{
            'update_rate': LaunchConfiguration('update_rate'),
            'location_lat': LaunchConfiguration('location_lat'),
            'location_lon': LaunchConfiguration('location_lon'),
            'location_alt': LaunchConfiguration('location_alt'),
            'target_object': LaunchConfiguration('target_object'),
            'tracking_mode': 'continuous',
            'use_imu': LaunchConfiguration('use_imu'),
            'use_gps': LaunchConfiguration('use_gps'),
            'goto_mode': LaunchConfiguration('goto_mode'),
            'gps_timeout': LaunchConfiguration('gps_timeout'),
            'alignment_file': 'star_alignment.json'
        }],
        output='screen'
    )
    
    # Alignment Calibration Node (optional for GoTo mode)
    alignment_node = Node(
        package='star_tracker',
        executable='alignment_calibration',
        name='alignment_calibration',
        parameters=[{
            'calibration_file': 'star_alignment.json',
            'location_lat': LaunchConfiguration('location_lat'),
            'location_lon': LaunchConfiguration('location_lon'), 
            'location_alt': LaunchConfiguration('location_alt'),
            'alignment_method': '2star'
        }],
        output='screen',
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('goto_mode'), 'true'))
    )
    
    # Launch info
    launch_info = LogInfo(
        msg=[
            'Starting Star Tracker with GPS integration\\n',
            'Target: ', LaunchConfiguration('target_object'), '\\n',
            'GPS enabled: ', LaunchConfiguration('use_gps'), '\\n',
            'IMU enabled: ', LaunchConfiguration('use_imu'), '\\n',
            'GoTo mode: ', LaunchConfiguration('goto_mode'), '\\n',
            'Update rate: ', LaunchConfiguration('update_rate'), ' Hz\\n',
            'Fallback location: Lat=', LaunchConfiguration('location_lat'),
            ', Lon=', LaunchConfiguration('location_lon'),
            ', Alt=', LaunchConfiguration('location_alt'), 'm'
        ]
    )
    
    return LaunchDescription([
        # Arguments
        location_lat_arg,
        location_lon_arg,
        location_alt_arg,
        target_object_arg,
        update_rate_arg,
        use_imu_arg,
        use_gps_arg,
        goto_mode_arg,
        gps_serial_port_arg,
        gps_baud_rate_arg,
        gps_timeout_arg,
        imu_serial_port_arg,
        imu_i2c_bus_arg,
        imu_update_rate_arg,
        
        # Launch info
        launch_info,
        
        # Nodes
        gps_node,
        imu_node,
        star_tracker_node,
        alignment_node,
    ])
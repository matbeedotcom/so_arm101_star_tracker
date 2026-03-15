from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Declare launch arguments
    i2c_bus_arg = DeclareLaunchArgument(
        'i2c_bus', default_value='1',
        description='I2C bus number for BNO055'
    )
    
    i2c_address_arg = DeclareLaunchArgument(
        'i2c_address', default_value='0x28',
        description='I2C address for BNO055 (0x28 or 0x29)'
    )
    
    location_lat_arg = DeclareLaunchArgument(
        'location_lat', default_value='40.7128',
        description='Observer latitude in degrees'
    )
    
    location_lon_arg = DeclareLaunchArgument(
        'location_lon', default_value='-74.0060',
        description='Observer longitude in degrees'
    )
    
    location_alt_arg = DeclareLaunchArgument(
        'location_alt', default_value='10.0',
        description='Observer altitude in meters'
    )
    
    target_object_arg = DeclareLaunchArgument(
        'target_object', default_value='polaris',
        description='Initial target object'
    )
    
    goto_mode_arg = DeclareLaunchArgument(
        'goto_mode', default_value='true',
        description='Enable GoTo mode with IMU feedback'
    )
    
    alignment_method_arg = DeclareLaunchArgument(
        'alignment_method', default_value='2star',
        description='Alignment method: 1star, 2star, or 3star'
    )
    
    # BNO055 IMU interface node
    bno055_node = Node(
        package='star_tracker',
        executable='bno055_interface.py',
        name='bno055_interface',
        output='screen',
        parameters=[{
            'i2c_bus': LaunchConfiguration('i2c_bus'),
            'i2c_address': LaunchConfiguration('i2c_address'),
            'update_rate': 50.0,
            'use_magnetometer': True,
        }]
    )
    
    # Alignment calibration node
    alignment_node = Node(
        package='star_tracker',
        executable='alignment_calibration.py',
        name='alignment_calibration',
        output='screen',
        parameters=[{
            'location_lat': LaunchConfiguration('location_lat'),
            'location_lon': LaunchConfiguration('location_lon'),
            'location_alt': LaunchConfiguration('location_alt'),
            'alignment_method': LaunchConfiguration('alignment_method'),
            'calibration_file': 'star_alignment.json',
        }]
    )
    
    # Star tracker node with IMU integration
    star_tracker_node = Node(
        package='star_tracker',
        executable='star_tracker_node.py',
        name='star_tracker',
        output='screen',
        parameters=[{
            'location_lat': LaunchConfiguration('location_lat'),
            'location_lon': LaunchConfiguration('location_lon'),
            'location_alt': LaunchConfiguration('location_alt'),
            'target_object': LaunchConfiguration('target_object'),
            'use_imu': True,
            'goto_mode': LaunchConfiguration('goto_mode'),
            'update_rate': 2.0,
            'alignment_file': 'star_alignment.json',
        }]
    )
    
    return LaunchDescription([
        # Arguments
        i2c_bus_arg,
        i2c_address_arg,
        location_lat_arg,
        location_lon_arg,
        location_alt_arg,
        target_object_arg,
        goto_mode_arg,
        alignment_method_arg,
        # Nodes
        bno055_node,
        alignment_node,
        star_tracker_node,
    ])
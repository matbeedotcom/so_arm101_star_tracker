from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Declare launch arguments
    location_lat_arg = DeclareLaunchArgument(
        'location_lat',
        default_value=EnvironmentVariable('LOCATION_LAT', default_value='40.7128'),
        description='Observer latitude in degrees'
    )
    
    location_lon_arg = DeclareLaunchArgument(
        'location_lon',
        default_value=EnvironmentVariable('LOCATION_LON', default_value='-74.0060'),
        description='Observer longitude in degrees'
    )
    
    location_alt_arg = DeclareLaunchArgument(
        'location_alt',
        default_value=EnvironmentVariable('LOCATION_ALT', default_value='10.0'),
        description='Observer altitude in meters'
    )
    
    target_object_arg = DeclareLaunchArgument(
        'target_object',
        default_value=EnvironmentVariable('TARGET_OBJECT', default_value='polaris'),
        description='Celestial object to track (sun, moon, polaris, sirius, etc.)'
    )
    
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='1.0',
        description='Tracking update rate in Hz'
    )
    
    # Star tracker node
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
            'update_rate': LaunchConfiguration('update_rate'),
        }]
    )
    
    return LaunchDescription([
        location_lat_arg,
        location_lon_arg,
        location_alt_arg,
        target_object_arg,
        update_rate_arg,
        star_tracker_node
    ])
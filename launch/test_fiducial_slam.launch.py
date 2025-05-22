from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'map_frame', 
            default_value='map',
            description='The name of the map frame'
        ),
        DeclareLaunchArgument(
            'odom_frame', 
            default_value='odom',
            description='The name of the odom frame'
        ),
        DeclareLaunchArgument(
            'base_frame', 
            default_value='base_link',
            description='The name of the base frame'
        ),
        
        # Launch the fiducial_slam node
        Node(
            package='fiducial_slam_ros2',
            executable='fiducial_slam_node',
            name='fiducial_slam',
            output='screen',
            parameters=[{
                'map_frame': LaunchConfiguration('map_frame'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
            }]
        )
    ])

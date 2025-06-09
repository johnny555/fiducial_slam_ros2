from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fiducial_slam_ros2',
            executable='f_slam',
            name='f_slam',
            output='screen',
        parameters=[{
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'camera_frame': 'realsense_optical_frame',
            'mapping_mode': True,  # Set to False for localization mode
            'fiducial_map_file': 'fiducial_map.txt',
            'optimization_frequency': 2.0,
            'use_sim_time': False
        }],
        )
    ])

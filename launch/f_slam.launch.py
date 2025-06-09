from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fiducial_slam_ros2',
            executable='f_slam',
            name='f_slam',
            output='screen',
            parameters=[
                {'map_frame': 'map'},
                {'odom_frame': 'odom'},
                {'base_frame': 'base_link'},
                {'camera_frame': 'camera_link'},
                {'mapping_mode': True},
                {'fiducial_map_file': '/tmp/fiducial_map.txt'},
                {'optimization_frequency': 2.0}
            ]
        )
    ])

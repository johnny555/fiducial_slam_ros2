from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fiducial_slam_ros2',
            executable='fiducial_slam_node',
            name='fiducial_slam',
            output='screen',
            parameters=[]
        )
    ])

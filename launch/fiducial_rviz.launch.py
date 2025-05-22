\
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '$(find fiducial_slam_ros2)/rviz/fiducial_slam.rviz'] 
            # Note: You'll need to create/save an rviz2 config file, 
            # for example, at /workspace/src/fiducial_slam_ros2/rviz/fiducial_slam.rviz
            # If you don't have a specific config yet, you can remove the arguments list
            # or remove the '-d' argument and the path to start with a default RViz2 setup.
        )
    ])

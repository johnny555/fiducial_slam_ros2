from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get the user's home directory
    home_dir = os.path.expanduser('~')
    default_map_file = os.path.join(home_dir, '.ros', 'slam', 'map.txt')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            default_value=default_map_file,
            description='Path to the map file'
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='The frame ID for the map'
        ),
        DeclareLaunchArgument(
            'odom_frame',
            default_value='odom',
            description='The frame ID for odom'
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='The frame ID for the robot base'
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='Whether to publish the map to odom TF transform'
        ),
        DeclareLaunchArgument(
            'tf_publish_interval',
            default_value='0.2',
            description='Interval in seconds for publishing TF transforms'
        ),
        DeclareLaunchArgument(
            'future_date_transforms',
            default_value='0.0',
            description='Future date transforms'
        ),
        DeclareLaunchArgument(
            'publish_6dof_pose',
            default_value='false',
            description='Publish 6DOF pose'
        ),
        # Note: pose_publish_rate is not directly used by the C++ node in the same way,
        # publishing is typically handled by the node's internal logic/timers.
        # The closest equivalent might be managed by tf_publish_interval or internal timers.
        DeclareLaunchArgument(
            'systematic_error',
            default_value='0.01',
            description='Systematic error for pose estimation'
        ),
        DeclareLaunchArgument(
            'covariance_diagonal',
            default_value='', # Expecting a string of space-separated doubles, e.g., "0.1 0.1 0.1 0.1 0.1 0.1"
            description='Diagonal elements of the covariance matrix for published poses. ' \
                        'If empty, internal variance is used. Example: "0.1 0.1 0.1 0.1 0.1 0.1"'
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/camera_info',
            description='Camera info topic to remap'
        ),

        Node(
            package='fiducial_slam_ros2',
            executable='fiducial_slam_node',
            name='fiducial_slam',
            output='screen',
            parameters=[
                {'map_file': LaunchConfiguration('map_file')},
                {'map_frame': LaunchConfiguration('map_frame')},
                {'odom_frame': LaunchConfiguration('odom_frame')},
                {'base_frame': LaunchConfiguration('base_frame')},
                {'publish_tf': LaunchConfiguration('publish_tf')},
                {'tf_publish_interval': LaunchConfiguration('tf_publish_interval')},
                {'future_date_transforms': LaunchConfiguration('future_date_transforms')},
                {'publish_6dof_pose': LaunchConfiguration('publish_6dof_pose')},
                {'systematic_error': LaunchConfiguration('systematic_error')},
                # Convert covariance_diagonal string to a list of doubles if not empty
                # This requires a bit more complex handling if we want to pass it as a list of doubles directly.
                # For simplicity, the C++ node can parse the string.
                {'covariance_diagonal_str': LaunchConfiguration('covariance_diagonal')}
            ],
            remappings=[
                ('/camera_info', LaunchConfiguration('camera_info_topic'))
            ]
        )
    ])

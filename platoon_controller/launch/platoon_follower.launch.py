"""Launch scan_filter and platoon_follower for a single or namespaced robot."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create the launch description for the platoon follower stack."""
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    target_distance = LaunchConfiguration('target_distance')
    max_track_range = LaunchConfiguration('max_track_range')
    invert_linear = LaunchConfiguration('invert_linear')
    invert_angular = LaunchConfiguration('invert_angular')
    log_csv = LaunchConfiguration('log_csv')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Optional robot namespace (e.g. tb3_2).',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock simulation time when running in Gazebo.',
        ),
        DeclareLaunchArgument(
            'target_distance',
            default_value='0.30',
            description='Desired following gap in metres.',
        ),
        DeclareLaunchArgument(
            'max_track_range',
            default_value='1.2',
            description='Maximum LiDAR tracking range in metres.',
        ),
        DeclareLaunchArgument(
            'invert_linear',
            default_value='false',
            description='Flip linear.x sign for reversed drive motors.',
        ),
        DeclareLaunchArgument(
            'invert_angular',
            default_value='false',
            description='Flip angular.z sign for reversed steering motors.',
        ),
        DeclareLaunchArgument(
            'log_csv',
            default_value='',
            description='Optional CSV output file path for calibration logs.',
        ),
        Node(
            package='platoon_controller',
            executable='scan_filter',
            namespace=namespace,
            name='scan_filter',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'max_track_range': max_track_range,
            }],
        ),
        Node(
            package='platoon_controller',
            executable='platoon_follower',
            namespace=namespace,
            name='platoon_follower',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_distance': target_distance,
                'invert_linear': invert_linear,
                'invert_angular': invert_angular,
                'log_csv': log_csv,
            }],
        ),
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share_dir = get_package_share_directory('passable_area')
    params_file = PathJoinSubstitution([package_share_dir, 'config', 'nav_params.yaml'])
    log_level = LaunchConfiguration('log_level')

    return LaunchDescription([
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level for the passable_area node'
        ),
        Node(
            package='passable_area',
            executable='passable_area',
            name='passable_area',
            output='screen',
            parameters=[params_file],
             arguments=['--ros-args', '--log-level', log_level],
            remappings=[
                ('cloud_topic', '/accumulate_cloud/cloud_gravity'),
                ('imu', '/IMU_YESENSE')
            ]
        )
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    passable_share_dir = get_package_share_directory('passable_area')
    passable_params = PathJoinSubstitution([passable_share_dir, 'config', 'nav_params.yaml'])
    log_level = LaunchConfiguration('log_level')

    accu_share_dir = get_package_share_directory('accumulate_cloud')
    accu_params = PathJoinSubstitution([accu_share_dir, 'config', 'accumulate_cloud.yaml'])

    accumulate_cloud_node = Node(
        package='accumulate_cloud',
        executable='accumulate_cloud',
        name='accumulate_cloud',
        output='screen',
        parameters=[accu_params]
    )

    passable_area_node = Node(
        package='passable_area',
        executable='passable_area',
        name='passable_area',
        output='screen',
        parameters=[passable_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[
            ('cloud_topic', '/accumulate_cloud/cloud_gravity'),
            ('imu', '/IMU_YESENSE')
        ]
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level for the passable_area node'
        ),
        accumulate_cloud_node,
        passable_area_node
    ])

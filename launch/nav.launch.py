from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share_dir = get_package_share_directory('passable_node')
    params_file = PathJoinSubstitution([package_share_dir, 'config', 'nav_params.yaml'])
    # print(f"package_share_dir: {package_share_dir}")
    return LaunchDescription([
        Node(
            package='passable_node',
            executable='passable_node',
            name='passable_node',
            output='screen',
            parameters=[params_file],
            remappings=[
                ('cloud_topic', '/cloud_cur'),
                ('imu', '/imu/data')
            ]
        )
    ])

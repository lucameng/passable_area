from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory("passable_area")
    params_file = PathJoinSubstitution([share_dir, "config", "passable_area.yaml"])
    sensors_file = PathJoinSubstitution([share_dir, "config", "sensors.yaml"])
    debug_file = PathJoinSubstitution([share_dir, "config", "debug.yaml"])
    log_level = LaunchConfiguration("log_level")

    return LaunchDescription(
        [
            DeclareLaunchArgument("log_level", default_value="info"),
            Node(
                package="passable_area",
                executable="passable_area_node",
                name="passable_area",
                output="screen",
                parameters=[params_file, sensors_file, debug_file],
                arguments=["--ros-args", "--log-level", log_level],
            ),
        ]
    )

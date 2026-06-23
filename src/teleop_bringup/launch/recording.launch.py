"""L7: multi-modal LeRobot recorder."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    output_dir = LaunchConfiguration("output_dir")
    return LaunchDescription([
        DeclareLaunchArgument("output_dir", default_value="data/episodes"),
        Node(
            package="lerobot_recorder",
            executable="lerobot_recorder_node",
            name="lerobot_recorder",
            output="screen",
            parameters=[{"output_dir": output_dir}],
        ),
    ])

"""Launch the passive M7 grasp monitor."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("grasp_monitor"),
                "config",
                "grasp_monitor_params.yaml",
            ]),
        ),
        Node(
            package="grasp_monitor",
            executable="grasp_monitor_node",
            name="grasp_monitor",
            output="screen",
            prefix="nice -n 19 ionice -c 3",
            parameters=[params_file],
        ),
    ])

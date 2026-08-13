# Copyright 2026 ros2-arm-teleoperation-suite contributors
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("out_dir", default_value="/tmp/geometry_stage1"),
            DeclareLaunchArgument("random_count", default_value="5"),
            DeclareLaunchArgument("seed", default_value="20260813"),
            Node(
                package="teleop_diagnostics",
                executable="geometry_diagnostics_node",
                name="geometry_diagnostics",
                output="screen",
                parameters=[
                    {
                        "out_dir": LaunchConfiguration("out_dir"),
                        "random_count": LaunchConfiguration("random_count"),
                        "seed": LaunchConfiguration("seed"),
                    }
                ],
            ),
        ]
    )

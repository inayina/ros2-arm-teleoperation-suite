# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Launch a one-shot live-TF Stage-1 closeout report (observer-only)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    out_dir = LaunchConfiguration("out_dir")
    domain_id = LaunchConfiguration("domain_id")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "out_dir",
                default_value="/tmp/geometry_stage1_live_tf",
            ),
            DeclareLaunchArgument("domain_id", default_value="91"),
            # The CLI itself starts an isolated RSP on domain_id; this launch
            # just runs the report process with a wall-clock bound via timeout.
            ExecuteProcess(
                cmd=[
                    "timeout",
                    "120s",
                    "ros2",
                    "run",
                    "teleop_diagnostics",
                    "geometry_live_tf_report",
                    "--out-dir",
                    out_dir,
                    "--domain-id",
                    domain_id,
                ],
                output="screen",
            ),
        ]
    )

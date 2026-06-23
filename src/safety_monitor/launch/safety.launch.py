"""Bring up the L1 safety layer: safety_monitor + diagnostic_aggregator."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    safety_params = PathJoinSubstitution(
        [FindPackageShare("safety_monitor"), "config", "safety_limits.yaml"]
    )
    return LaunchDescription([
        Node(
            package="safety_monitor",
            executable="safety_monitor_node",
            name="safety_monitor",
            output="screen",
            parameters=[safety_params],
        ),
    ])

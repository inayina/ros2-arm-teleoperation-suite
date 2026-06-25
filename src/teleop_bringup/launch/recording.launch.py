"""L7: multi-modal LeRobot recorder."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    output_dir = LaunchConfiguration("output_dir")
    task = LaunchConfiguration("task")
    sync_slop = LaunchConfiguration("sync_slop")
    return LaunchDescription([
        DeclareLaunchArgument("output_dir", default_value="data/episodes"),
        DeclareLaunchArgument("task", default_value="teleop"),
        DeclareLaunchArgument("sync_slop", default_value="0.05"),
        Node(
            package="lerobot_recorder",
            executable="lerobot_recorder_node",
            name="lerobot_recorder",
            output="screen",
            parameters=[{
                "output_dir": output_dir,
                "task": task,
                "sync_slop": sync_slop,
            }],
        ),
    ])

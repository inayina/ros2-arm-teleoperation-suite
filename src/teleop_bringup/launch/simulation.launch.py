"""L5/L6: MuJoCo physics server + camera bridge."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_path = LaunchConfiguration("model_path")
    return LaunchDescription([
        DeclareLaunchArgument(
            "model_path",
            default_value="config/models/franka_panda.xml",
            description="MuJoCo XML path. Relative paths resolve from launch cwd.",
        ),
        Node(
            package="mujoco_sim",
            executable="mujoco_sim_node",
            name="mujoco_sim",
            output="screen",
            parameters=[{"model_path": model_path}],
        ),
        Node(
            package="camera_bridge",
            executable="camera_bridge_node",
            name="camera_bridge",
            output="screen",
        ),
    ])

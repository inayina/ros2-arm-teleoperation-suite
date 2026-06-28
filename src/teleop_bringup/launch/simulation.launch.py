"""L5/L6: MuJoCo physics server + camera bridge."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_path = LaunchConfiguration("model_path")
    randomize = LaunchConfiguration("randomize")
    camera_name = LaunchConfiguration("camera_name")
    camera_width = LaunchConfiguration("camera_width")
    camera_height = LaunchConfiguration("camera_height")
    camera_rate = LaunchConfiguration("camera_rate")
    return LaunchDescription([
        DeclareLaunchArgument(
            "model_path",
            default_value="config/models/franka_panda.xml",
            description="MuJoCo XML path. Relative paths resolve from launch cwd.",
        ),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("randomize", default_value="false"),
        DeclareLaunchArgument("camera_name", default_value="scene_camera"),
        DeclareLaunchArgument("camera_width", default_value="640"),
        DeclareLaunchArgument("camera_height", default_value="480"),
        DeclareLaunchArgument("camera_rate", default_value="30.0"),
        Node(
            package="mujoco_sim",
            executable="mujoco_sim_node",
            name="mujoco_sim",
            output="screen",
            parameters=[{
                "model_path": model_path,
                "headless": LaunchConfiguration("headless"),
                "randomize": randomize,
            }],
        ),
        Node(
            package="camera_bridge",
            executable="camera_bridge_node",
            name="camera_bridge",
            output="screen",
            parameters=[{
                "model_path": model_path,
                "camera_name": camera_name,
                "width": camera_width,
                "height": camera_height,
                "rate": camera_rate,
            }],
        ),
    ])

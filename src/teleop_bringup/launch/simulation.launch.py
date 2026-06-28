"""L5/L6: MuJoCo physics server + camera bridge."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_path = LaunchConfiguration("model_path")
    randomize = LaunchConfiguration("randomize")
    camera_name = LaunchConfiguration("camera_name")
    camera_width = LaunchConfiguration("camera_width")
    camera_height = LaunchConfiguration("camera_height")
    camera_rate = LaunchConfiguration("camera_rate")
    enable_wrist_camera = LaunchConfiguration("enable_wrist_camera")
    wrist_camera_width = LaunchConfiguration("wrist_camera_width")
    wrist_camera_height = LaunchConfiguration("wrist_camera_height")
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
        DeclareLaunchArgument("enable_wrist_camera", default_value="true"),
        DeclareLaunchArgument("wrist_camera_width", default_value="320"),
        DeclareLaunchArgument("wrist_camera_height", default_value="240"),
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
                "fovy_deg": 45.0,
                "frame_id": "scene_camera_optical_frame",
                "color_topic": "/camera/color/image_raw",
                "depth_topic": "/camera/depth/image_raw",
                "camera_info_topic": "/camera/color/camera_info",
            }],
        ),
        Node(
            package="camera_bridge",
            executable="camera_bridge_node",
            name="wrist_camera_bridge",
            output="screen",
            parameters=[{
                "model_path": model_path,
                "camera_name": "wrist_camera",
                "width": wrist_camera_width,
                "height": wrist_camera_height,
                "rate": camera_rate,
                "fovy_deg": 70.0,
                "frame_id": "wrist_camera_optical_frame",
                "color_topic": "/camera/wrist/color/image_raw",
                "depth_topic": "/camera/wrist/depth/image_raw",
                "camera_info_topic": "/camera/wrist/color/camera_info",
            }],
            condition=IfCondition(enable_wrist_camera),
        ),
    ])

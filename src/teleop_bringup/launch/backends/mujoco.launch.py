"""MuJoCo implementation of the simulator backend contract."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    model_path = LaunchConfiguration("model_path")
    randomize = LaunchConfiguration("randomize")
    camera_name = LaunchConfiguration("camera_name")
    scene_use_mujoco_renderer = LaunchConfiguration("scene_use_mujoco_renderer")
    camera_width = LaunchConfiguration("camera_width")
    camera_height = LaunchConfiguration("camera_height")
    camera_rate = LaunchConfiguration("camera_rate")
    publish_depth = LaunchConfiguration("publish_depth")
    enable_wrist_camera = LaunchConfiguration("enable_wrist_camera")
    wrist_use_mujoco_renderer = LaunchConfiguration("wrist_use_mujoco_renderer")
    wrist_camera_width = LaunchConfiguration("wrist_camera_width")
    wrist_camera_height = LaunchConfiguration("wrist_camera_height")
    contact_debug_enabled = LaunchConfiguration("contact_debug_enabled")
    contact_debug_period_s = LaunchConfiguration("contact_debug_period_s")
    grasp_assist_enabled = LaunchConfiguration("grasp_assist_enabled")
    gripper_force_max_n = LaunchConfiguration("gripper_force_max_n")
    gripper_contact_hold_margin = LaunchConfiguration(
        "gripper_contact_hold_margin"
    )
    gripper_force_squeeze_margin_max = LaunchConfiguration(
        "gripper_force_squeeze_margin_max"
    )

    return LaunchDescription([
        DeclareLaunchArgument("capture_mode", default_value="portfolio"),
        DeclareLaunchArgument("model_path", default_value="config/models/franka_panda.xml"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("randomize", default_value="false"),
        DeclareLaunchArgument("camera_name", default_value="scene_camera"),
        DeclareLaunchArgument("scene_use_mujoco_renderer", default_value="true"),
        DeclareLaunchArgument("camera_width", default_value="320"),
        DeclareLaunchArgument("camera_height", default_value="240"),
        DeclareLaunchArgument("camera_rate", default_value="10.0"),
        DeclareLaunchArgument("publish_depth", default_value="false"),
        DeclareLaunchArgument("enable_wrist_camera", default_value="false"),
        DeclareLaunchArgument("wrist_use_mujoco_renderer", default_value="true"),
        DeclareLaunchArgument("wrist_camera_width", default_value="320"),
        DeclareLaunchArgument("wrist_camera_height", default_value="240"),
        DeclareLaunchArgument("enable_tactile", default_value="false"),
        DeclareLaunchArgument("contact_debug_enabled", default_value="false"),
        DeclareLaunchArgument("contact_debug_period_s", default_value="1.0"),
        DeclareLaunchArgument("grasp_assist_enabled", default_value="false"),
        DeclareLaunchArgument("gripper_force_max_n", default_value="45.0"),
        DeclareLaunchArgument(
            "gripper_contact_hold_margin", default_value="0.008"
        ),
        DeclareLaunchArgument(
            "gripper_force_squeeze_margin_max", default_value="0.010"
        ),
        Node(
            package="mujoco_sim",
            executable="mujoco_sim_node",
            name="mujoco_sim",
            output="screen",
            parameters=[{
                "model_path": model_path,
                "headless": LaunchConfiguration("headless"),
                "randomize": randomize,
                "contact_debug_enabled": contact_debug_enabled,
                "contact_debug_period_s": contact_debug_period_s,
                "grasp_assist_enabled": grasp_assist_enabled,
                "gripper_force_max_n": gripper_force_max_n,
                "gripper_contact_hold_margin": gripper_contact_hold_margin,
                "gripper_force_squeeze_margin_max": gripper_force_squeeze_margin_max,
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
                "use_mujoco_renderer": scene_use_mujoco_renderer,
                "publish_depth": publish_depth,
            }],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("capture_mode"), "' == 'portfolio'"
            ])),
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
                "use_mujoco_renderer": wrist_use_mujoco_renderer,
                "publish_depth": publish_depth,
            }],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("capture_mode"), "' == 'portfolio' and '",
                enable_wrist_camera, "' == 'true'"
            ])),
        ),
        Node(
            package="camera_bridge",
            executable="camera_bridge_node",
            name="left_tactile_bridge",
            output="screen",
            parameters=[{
                "model_path": model_path,
                "camera_name": "left_tactile_camera",
                "width": 320,
                "height": 240,
                "rate": camera_rate,
                "fovy_deg": 90.0,
                "frame_id": "left_tactile_optical_frame",
                "color_topic": "/camera/tactile_left/image_raw",
                "depth_topic": "/camera/tactile_left/depth/image_raw",
                "camera_info_topic": "/camera/tactile_left/camera_info",
                "tactile_mode": True,
                "gel_depth_baseline": 0.0155,
                "gel_scale": 300.0,
            }],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("capture_mode"), "' == 'portfolio' and '",
                LaunchConfiguration("enable_tactile"), "' == 'true'"
            ])),
        ),
        Node(
            package="camera_bridge",
            executable="camera_bridge_node",
            name="right_tactile_bridge",
            output="screen",
            parameters=[{
                "model_path": model_path,
                "camera_name": "right_tactile_camera",
                "width": 320,
                "height": 240,
                "rate": camera_rate,
                "fovy_deg": 90.0,
                "frame_id": "right_tactile_optical_frame",
                "color_topic": "/camera/tactile_right/image_raw",
                "depth_topic": "/camera/tactile_right/depth/image_raw",
                "camera_info_topic": "/camera/tactile_right/camera_info",
                "tactile_mode": True,
                "gel_depth_baseline": 0.0155,
                "gel_scale": 300.0,
            }],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("capture_mode"), "' == 'portfolio' and '",
                LaunchConfiguration("enable_tactile"), "' == 'true'"
            ])),
        ),
    ])

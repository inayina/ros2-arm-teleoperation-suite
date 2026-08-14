"""L7: multi-modal LeRobot recorder."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    output_dir = LaunchConfiguration("output_dir")
    task = LaunchConfiguration("task")
    sync_slop = LaunchConfiguration("sync_slop")
    sync_queue_size = LaunchConfiguration("sync_queue_size")
    auto_record_seconds = LaunchConfiguration("auto_record_seconds")
    auto_record_delay_s = LaunchConfiguration("auto_record_delay_s")
    capture_mode = LaunchConfiguration("capture_mode")
    enable_wrist_camera = LaunchConfiguration("enable_wrist_camera")
    expected_frame_rate_hz = LaunchConfiguration("expected_frame_rate_hz")
    enable_system_telemetry = LaunchConfiguration("enable_system_telemetry")
    simulator_backend = LaunchConfiguration("simulator_backend")
    simulator_version = LaunchConfiguration("simulator_version")
    scene_id = LaunchConfiguration("scene_id")
    return LaunchDescription([
        DeclareLaunchArgument("output_dir", default_value="data/episodes"),
        DeclareLaunchArgument("task", default_value="teleop"),
        DeclareLaunchArgument("sync_slop", default_value="0.05"),
        DeclareLaunchArgument("sync_queue_size", default_value="30"),
        DeclareLaunchArgument("auto_record_seconds", default_value="0.0"),
        DeclareLaunchArgument("auto_record_delay_s", default_value="0.0"),
        DeclareLaunchArgument("capture_mode", default_value="portfolio"),
        DeclareLaunchArgument("enable_wrist_camera", default_value="true"),
        DeclareLaunchArgument("expected_frame_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("enable_system_telemetry", default_value="true"),
        DeclareLaunchArgument("simulator_backend", default_value=""),
        DeclareLaunchArgument("simulator_version", default_value=""),
        DeclareLaunchArgument("scene_id", default_value=""),
        Node(
            package="lerobot_recorder",
            executable="lerobot_recorder_node",
            name="lerobot_recorder",
            output="screen",
            parameters=[{
                "output_dir": output_dir,
                "task": task,
                "sync_slop": ParameterValue(sync_slop, value_type=float),
                "sync_queue_size": ParameterValue(sync_queue_size, value_type=int),
                "auto_record_seconds": ParameterValue(auto_record_seconds, value_type=float),
                "auto_record_delay_s": ParameterValue(auto_record_delay_s, value_type=float),
                "capture_mode": capture_mode,
                "enable_wrist_camera": ParameterValue(
                    enable_wrist_camera, value_type=bool),
                "expected_frame_rate_hz": ParameterValue(
                    expected_frame_rate_hz, value_type=float),
                "simulator_backend": simulator_backend,
                "simulator_version": simulator_version,
                "scene_id": scene_id,
            }],
        ),
        Node(
            package="lerobot_recorder",
            executable="system_telemetry_node",
            name="system_telemetry",
            output="screen",
            condition=IfCondition(enable_system_telemetry),
        ),
    ])

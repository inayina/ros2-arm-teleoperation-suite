"""Top-level orchestration for the V2 teleoperation stack.

Layer bring-up order (description -> simulation -> fieldbus -> ros2_control ->
safety -> motion -> recording). Use TimerAction to stagger dependent layers.

Examples:
  ros2 launch teleop_bringup full_system.launch.py
  ros2 launch teleop_bringup full_system.launch.py sim_backend:=mujoco
  ros2 launch teleop_bringup full_system.launch.py sim_backend:=isaac
  ros2 launch teleop_bringup full_system.launch.py use_sim:=false can_interface:=can0
  ros2 launch teleop_bringup full_system.launch.py controller:=forward record:=true
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(pkg, rel, args=None, condition=None):
    src = PathJoinSubstitution([FindPackageShare(pkg), "launch", rel])
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([src]),
        launch_arguments=(args or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    sim_backend = LaunchConfiguration("sim_backend")
    use_sim = LaunchConfiguration("use_sim")
    can_interface = LaunchConfiguration("can_interface")
    controller = LaunchConfiguration("controller")
    record = LaunchConfiguration("record")
    output_dir = LaunchConfiguration("output_dir")
    task = LaunchConfiguration("task")
    sync_slop = LaunchConfiguration("sync_slop")
    sync_queue_size = LaunchConfiguration("sync_queue_size")
    auto_record_seconds = LaunchConfiguration("auto_record_seconds")
    auto_record_delay_s = LaunchConfiguration("auto_record_delay_s")
    model_path = LaunchConfiguration("model_path")
    randomize = LaunchConfiguration("randomize")
    headless = LaunchConfiguration("headless")
    scene_use_mujoco_renderer = LaunchConfiguration("scene_use_mujoco_renderer")
    camera_width = LaunchConfiguration("camera_width")
    camera_height = LaunchConfiguration("camera_height")
    camera_rate = LaunchConfiguration("camera_rate")
    publish_depth = LaunchConfiguration("publish_depth")
    enable_wrist_camera = LaunchConfiguration("enable_wrist_camera")
    wrist_use_mujoco_renderer = LaunchConfiguration("wrist_use_mujoco_renderer")
    enable_tactile = LaunchConfiguration("enable_tactile")
    wrist_camera_width = LaunchConfiguration("wrist_camera_width")
    wrist_camera_height = LaunchConfiguration("wrist_camera_height")
    contact_debug_enabled = LaunchConfiguration("contact_debug_enabled")
    contact_debug_period_s = LaunchConfiguration("contact_debug_period_s")
    grasp_assist_enabled = LaunchConfiguration("grasp_assist_enabled")
    gripper_force_max_n = LaunchConfiguration("gripper_force_max_n")
    gripper_contact_hold_margin = LaunchConfiguration("gripper_contact_hold_margin")
    gripper_force_squeeze_margin_max = LaunchConfiguration("gripper_force_squeeze_margin_max")
    watchdog_timeout = LaunchConfiguration("watchdog_timeout")
    enable_grasp_monitor = LaunchConfiguration("enable_grasp_monitor")
    start_teleop = LaunchConfiguration("start_teleop")
    teleop_driver = LaunchConfiguration("teleop_driver")
    servo_mode = LaunchConfiguration("servo_mode")
    capture_mode = LaunchConfiguration("capture_mode")
    target_object_name = LaunchConfiguration("target_object_name")
    isaac_source_namespace = LaunchConfiguration("isaac_source_namespace")
    isaac_startup_timeout_s = LaunchConfiguration("isaac_startup_timeout_s")
    isaac_reset_timeout_s = LaunchConfiguration("isaac_reset_timeout_s")
    isaac_command_timeout_s = LaunchConfiguration("isaac_command_timeout_s")
    isaac_state_timeout_s = LaunchConfiguration("isaac_state_timeout_s")
    isaac_command_forward_rate_hz = LaunchConfiguration(
        "isaac_command_forward_rate_hz")
    simulator_version = LaunchConfiguration("simulator_version")
    scene_id = LaunchConfiguration("scene_id")

    common = {"use_sim": use_sim, "can_interface": can_interface}

    description = _include("teleop_description", "description.launch.py", common)
    simulation = _include("teleop_bringup", "simulation.launch.py",
                          {
                              "sim_backend": sim_backend,
                              "target_object_name": target_object_name,
                              "isaac_source_namespace": isaac_source_namespace,
                              "isaac_startup_timeout_s": isaac_startup_timeout_s,
                              "isaac_reset_timeout_s": isaac_reset_timeout_s,
                              "isaac_command_timeout_s": isaac_command_timeout_s,
                              "isaac_state_timeout_s": isaac_state_timeout_s,
                              "isaac_command_forward_rate_hz": isaac_command_forward_rate_hz,
                              "model_path": model_path,
                              "randomize": randomize,
                              "headless": headless,
                              "scene_use_mujoco_renderer": scene_use_mujoco_renderer,
                              "camera_width": camera_width,
                              "camera_height": camera_height,
                              "camera_rate": camera_rate,
                              "publish_depth": publish_depth,
                              "enable_wrist_camera": enable_wrist_camera,
                              "wrist_use_mujoco_renderer": wrist_use_mujoco_renderer,
                              "enable_tactile": enable_tactile,
                              "wrist_camera_width": wrist_camera_width,
                              "wrist_camera_height": wrist_camera_height,
                              "contact_debug_enabled": contact_debug_enabled,
                              "contact_debug_period_s": contact_debug_period_s,
                              "grasp_assist_enabled": grasp_assist_enabled,
                              "capture_mode": capture_mode,
                              "gripper_force_max_n": gripper_force_max_n,
                              "gripper_contact_hold_margin": gripper_contact_hold_margin,
                              "gripper_force_squeeze_margin_max": gripper_force_squeeze_margin_max,
                          })
    fieldbus = _include("teleop_bringup", "fieldbus.launch.py", common)
    ros2_control = _include(
        "teleop_bringup", "ros2_control.launch.py",
        {**common, "controller": controller, "sim_backend": sim_backend})
    safety = _include(
        "safety_monitor", "safety.launch.py",
        {"watchdog_timeout": watchdog_timeout})
    motion = _include(
        "teleop_bringup", "motion.launch.py",
        {
            "use_sim": use_sim,
            "can_interface": can_interface,
            "start_teleop": start_teleop,
            "teleop_driver": teleop_driver,
            "servo_mode": servo_mode,
        })
    recording = _include(
        "teleop_bringup", "recording.launch.py",
        {
            "output_dir": output_dir,
            "task": task,
            "sync_slop": sync_slop,
            "sync_queue_size": sync_queue_size,
            "auto_record_seconds": auto_record_seconds,
            "auto_record_delay_s": auto_record_delay_s,
            "capture_mode": capture_mode,
            "enable_wrist_camera": enable_wrist_camera,
            "expected_frame_rate_hz": camera_rate,
            "simulator_backend": sim_backend,
            "simulator_version": simulator_version,
            "scene_id": scene_id,
        },
        condition=IfCondition(record))
    grasp_monitor = _include(
        "teleop_bringup", "grasp_monitor.launch.py",
        condition=IfCondition(enable_grasp_monitor))

    return LaunchDescription([
        DeclareLaunchArgument(
            "sim_backend",
            default_value="mujoco",
            choices=["mujoco", "isaac"],
            description=(
                "Simulation backend. MuJoCo remains the default; Isaac uses an "
                "isolated external runtime and the isaac_sim_adapter package."
            ),
        ),
        DeclareLaunchArgument("use_sim", default_value="true"),
        DeclareLaunchArgument("can_interface", default_value="vcan0"),
        DeclareLaunchArgument("controller", default_value="impedance",
                              description="impedance | forward"),
        DeclareLaunchArgument("record", default_value="false"),
        DeclareLaunchArgument("capture_mode", default_value="portfolio",
                              description="training (low-dimensional only) | portfolio (scene/wrist video)"),
        DeclareLaunchArgument("target_object_name", default_value="object_red_box"),
        DeclareLaunchArgument("isaac_source_namespace", default_value="/isaac"),
        DeclareLaunchArgument("isaac_startup_timeout_s", default_value="45.0"),
        DeclareLaunchArgument("isaac_reset_timeout_s", default_value="5.0"),
        DeclareLaunchArgument("isaac_command_timeout_s", default_value="0.1"),
        DeclareLaunchArgument("isaac_state_timeout_s", default_value="0.1"),
        DeclareLaunchArgument("isaac_command_forward_rate_hz", default_value="250.0"),
        DeclareLaunchArgument("simulator_version", default_value=""),
        DeclareLaunchArgument("scene_id", default_value=""),
        DeclareLaunchArgument("output_dir", default_value="data/episodes"),
        DeclareLaunchArgument("task", default_value="teleop"),
        DeclareLaunchArgument("sync_slop", default_value="0.05"),
        DeclareLaunchArgument("sync_queue_size", default_value="30"),
        DeclareLaunchArgument("auto_record_seconds", default_value="0.0"),
        DeclareLaunchArgument("auto_record_delay_s", default_value="0.0"),
        DeclareLaunchArgument("model_path", default_value="config/models/franka_panda.xml"),
        DeclareLaunchArgument("randomize", default_value="false"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="MuJoCo renderer mode; Isaac is started externally"),
        DeclareLaunchArgument("scene_use_mujoco_renderer", default_value="true"),
        DeclareLaunchArgument("camera_width", default_value="320"),
        DeclareLaunchArgument("camera_height", default_value="240"),
        DeclareLaunchArgument("camera_rate", default_value="10.0"),
        DeclareLaunchArgument("publish_depth", default_value="false"),
        DeclareLaunchArgument("enable_wrist_camera", default_value="false"),
        DeclareLaunchArgument("wrist_use_mujoco_renderer", default_value="true"),
        DeclareLaunchArgument("enable_tactile", default_value="false"),
        DeclareLaunchArgument("wrist_camera_width", default_value="320"),
        DeclareLaunchArgument("wrist_camera_height", default_value="240"),
        DeclareLaunchArgument("contact_debug_enabled", default_value="false"),
        DeclareLaunchArgument("contact_debug_period_s", default_value="1.0"),
        DeclareLaunchArgument(
            "grasp_assist_enabled",
            default_value="false",
            description="Enable synthetic grasp assist in MuJoCo. Must stay false for training-grade batch collection.",
        ),
        DeclareLaunchArgument("gripper_force_max_n", default_value="45.0"),
        DeclareLaunchArgument("gripper_contact_hold_margin", default_value="0.008"),
        DeclareLaunchArgument("gripper_force_squeeze_margin_max", default_value="0.010"),
        DeclareLaunchArgument(
            "watchdog_timeout",
            default_value="0.5",
            description="Teleop heartbeat timeout passed to safety_monitor.",
        ),
        DeclareLaunchArgument(
            "enable_grasp_monitor",
            default_value="true",
            description="Enable grasp_monitor for physics-only grasp/slip evaluation (/grasp/status).",
        ),
        DeclareLaunchArgument("start_teleop", default_value="true"),
        DeclareLaunchArgument("teleop_driver", default_value="keyboard"),
        DeclareLaunchArgument(
            "servo_mode",
            default_value="pose",
            description="MoveIt Servo input: pose (teleop) | twist (batch incremental).",
        ),

        description,
        simulation,
        TimerAction(period=2.0, actions=[fieldbus]),
        TimerAction(period=2.0, actions=[recording]),
        TimerAction(period=2.0, actions=[grasp_monitor]),
        TimerAction(period=4.0, actions=[safety]),
        TimerAction(period=6.0, actions=[motion]),
        TimerAction(period=12.0, actions=[ros2_control]),
    ])

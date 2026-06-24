"""Top-level orchestration for the V2 teleoperation stack.

Layer bring-up order (description -> simulation -> fieldbus -> ros2_control ->
safety -> motion -> recording). Use TimerAction to stagger dependent layers.

Examples:
  ros2 launch teleop_bringup full_system.launch.py
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
    use_sim = LaunchConfiguration("use_sim")
    can_interface = LaunchConfiguration("can_interface")
    controller = LaunchConfiguration("controller")
    record = LaunchConfiguration("record")
    model_path = LaunchConfiguration("model_path")
    headless = LaunchConfiguration("headless")

    common = {"use_sim": use_sim, "can_interface": can_interface}

    description = _include("teleop_description", "description.launch.py", common)
    simulation = _include("teleop_bringup", "simulation.launch.py",
                          {"model_path": model_path, "headless": headless})
    fieldbus = _include("teleop_bringup", "fieldbus.launch.py", common)
    ros2_control = _include(
        "teleop_bringup", "ros2_control.launch.py",
        {**common, "controller": controller})
    safety = _include("safety_monitor", "safety.launch.py")
    motion = _include("teleop_bringup", "motion.launch.py",
                      {"use_sim": use_sim, "can_interface": can_interface})
    recording = _include(
        "teleop_bringup", "recording.launch.py", condition=IfCondition(record))

    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="true"),
        DeclareLaunchArgument("can_interface", default_value="vcan0"),
        DeclareLaunchArgument("controller", default_value="impedance",
                              description="impedance | forward"),
        DeclareLaunchArgument("record", default_value="false"),
        DeclareLaunchArgument("model_path", default_value="config/models/franka_panda.xml"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="true → MuJoCo offscreen renderer (no viewer window)"),

        description,
        simulation,
        TimerAction(period=2.0, actions=[fieldbus]),
        TimerAction(period=4.0, actions=[ros2_control]),
        TimerAction(period=6.0, actions=[safety]),
        TimerAction(period=9.0, actions=[motion]),
        TimerAction(period=10.0, actions=[recording]),
    ])

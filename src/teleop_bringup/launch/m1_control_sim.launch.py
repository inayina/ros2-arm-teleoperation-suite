"""M1 minimal closed-loop launch: ros2_control sim backend + MuJoCo Panda XML.

This launch intentionally excludes safety_monitor, MoveIt Servo and recorder.
It verifies the M1 acceptance path:

  forward_effort_controller -> canopen_hw_interface(use_sim=true)
    -> /sim/joint_effort_cmd -> mujoco_sim(config/models/franka_panda.xml)
    -> /sim/encoder_state -> canopen_hw_interface -> joint_state_broadcaster
    -> /joint_states

Example:
  ros2 launch teleop_bringup m1_control_sim.launch.py
  ros2 topic pub /forward_effort_controller/commands std_msgs/msg/Float64MultiArray \
    "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(pkg, rel, args=None):
    src = PathJoinSubstitution([FindPackageShare(pkg), "launch", rel])
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([src]),
        launch_arguments=(args or {}).items(),
    )


def generate_launch_description():
    model_path = LaunchConfiguration("model_path")
    can_interface = LaunchConfiguration("can_interface")

    description = _include(
        "teleop_description",
        "description.launch.py",
        {
            "use_sim": "true",
            "can_interface": can_interface,
        },
    )
    simulation = _include(
        "teleop_bringup",
        "simulation.launch.py",
        {"model_path": model_path},
    )
    ros2_control = _include(
        "teleop_bringup",
        "ros2_control.launch.py",
        {
            "use_sim": "true",
            "can_interface": can_interface,
            "controller": "forward",
        },
    )

    return LaunchDescription([
        DeclareLaunchArgument("model_path", default_value="config/models/franka_panda.xml"),
        DeclareLaunchArgument("can_interface", default_value="vcan0"),
        description,
        simulation,
        TimerAction(period=2.0, actions=[ros2_control]),
    ])

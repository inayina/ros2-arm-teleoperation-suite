"""M2 fieldbus closed-loop launch: CANopen DS402 + virtual servo + MuJoCo.

  forward_effort_controller -> canopen_hw_interface(use_sim=false)
    -> vcan0 RPDO/SYNC -> virtual_servo_driver -> /sim/joint_effort_cmd
    -> mujoco_sim -> /sim/encoder_state -> virtual_servo_driver -> vcan0 TPDO
    -> canopen_hw_interface -> joint_state_broadcaster -> /joint_states

Prerequisites:
  bash scripts/setup_vcan.sh

Example:
  ros2 launch teleop_bringup m2_fieldbus.launch.py
  ros2 service call /virtual_servo_driver/inject_fault_joint1 std_srvs/srv/Trigger
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
            "use_sim": "false",
            "can_interface": can_interface,
        },
    )
    fieldbus = _include(
        "teleop_bringup",
        "fieldbus.launch.py",
        {
            "use_sim": "false",
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
            "use_sim": "false",
            "can_interface": can_interface,
            "controller": "forward",
        },
    )

    return LaunchDescription([
        DeclareLaunchArgument("model_path", default_value="config/models/franka_panda.xml"),
        DeclareLaunchArgument("can_interface", default_value="vcan0"),
        description,
        fieldbus,
        simulation,
        TimerAction(period=2.0, actions=[ros2_control]),
    ])

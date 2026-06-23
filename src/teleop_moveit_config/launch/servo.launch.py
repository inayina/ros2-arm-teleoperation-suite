"""L2 motion layer: MoveIt Servo (pose-tracking) consuming /safe_master_pose."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml


def _load_yaml(pkg, rel):
    path = os.path.join(get_package_share_directory(pkg), rel)
    with open(path, "r") as fh:
        return yaml.safe_load(fh)


def generate_launch_description():
    servo_params = {"moveit_servo": _load_yaml("teleop_moveit_config", "config/servo.yaml")
                    ["moveit_servo"]["ros__parameters"]}
    kinematics = _load_yaml("teleop_moveit_config", "config/kinematics.yaml")

    # NOTE: robot_description / robot_description_semantic are expected on the
    # parameter server (set by description.launch + this package's SRDF).
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[servo_params, {"robot_description_kinematics": kinematics}],
        remappings=[("~/pose_target_cmds", "/safe_master_pose")],
    )

    return LaunchDescription([servo_node])

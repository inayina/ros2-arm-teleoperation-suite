"""L4 fieldbus: virtual servo drives (CAN mode only) + gripper driver.

In sim mode (use_sim:=true) the canopen_hw_interface talks to MuJoCo directly,
so the virtual_servo_driver is not started (it would contend for /sim cmd).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_sim = LaunchConfiguration("use_sim")
    can_interface = LaunchConfiguration("can_interface")
    not_sim = PythonExpression(["'", use_sim, "' == 'false'"])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="true"),
        DeclareLaunchArgument("can_interface", default_value="vcan0"),
        Node(
            package="virtual_servo_driver",
            executable="virtual_servo_driver",
            name="virtual_servo_driver",
            output="screen",
            parameters=[{"can_interface": can_interface}],
            condition=IfCondition(not_sim),
        ),
        Node(
            package="gripper_driver",
            executable="gripper_driver_node",
            name="gripper_driver",
            output="screen",
            condition=IfCondition(not_sim),
        ),
    ])

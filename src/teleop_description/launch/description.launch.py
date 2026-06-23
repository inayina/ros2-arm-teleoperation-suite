"""Publish robot_description (URDF from xacro) and TF via robot_state_publisher."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim = LaunchConfiguration("use_sim")
    can_interface = LaunchConfiguration("can_interface")

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("teleop_description"), "urdf", "panda.urdf.xacro"]
    )

    robot_description = {
        "robot_description": ParameterValue(Command(
            [
                FindExecutable(name="xacro"), " ", xacro_file,
                " use_sim:=", use_sim,
                " can_interface:=", can_interface,
            ]
        ), value_type=str)
    }

    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="true"),
        DeclareLaunchArgument("can_interface", default_value="vcan0"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
    ])

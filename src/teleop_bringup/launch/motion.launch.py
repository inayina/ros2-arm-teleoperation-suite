"""L0/L2: teleop input + MoveIt Servo motion layer."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    servo_launch = PathJoinSubstitution(
        [FindPackageShare("teleop_moveit_config"), "launch", "servo.launch.py"])

    return LaunchDescription([
        Node(
            package="teleop_input",
            executable="teleop_input_node",
            name="teleop_input",
            output="screen",
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource([servo_launch])),
    ])

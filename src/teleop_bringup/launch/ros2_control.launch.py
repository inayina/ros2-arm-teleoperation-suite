"""L3 control layer: controller_manager + broadcaster + chosen controller."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim = LaunchConfiguration("use_sim")
    can_interface = LaunchConfiguration("can_interface")
    controller = LaunchConfiguration("controller")  # impedance | forward

    is_impedance = PythonExpression(["'", controller, "' == 'impedance'"])

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("teleop_description"), "urdf", "panda.urdf.xacro"])
    robot_description = {
        "robot_description": ParameterValue(Command([
            FindExecutable(name="xacro"), " ", xacro_file,
            " use_sim:=", use_sim, " can_interface:=", can_interface,
        ]), value_type=str)
    }
    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("teleop_bringup"), "config", "controllers.yaml"])

    cm = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, controllers_yaml],
    )
    impedance_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "cartesian_impedance_controller",
            "-c", "/controller_manager",
            "--activate-as-group",
        ],
        condition=IfCondition(is_impedance),
    )
    forward_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "forward_effort_controller",
            "-c", "/controller_manager",
            "--activate-as-group",
        ],
        condition=UnlessCondition(is_impedance),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="true"),
        DeclareLaunchArgument("can_interface", default_value="vcan0"),
        DeclareLaunchArgument("controller", default_value="impedance"),
        cm, impedance_spawner, forward_spawner,
    ])

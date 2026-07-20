"""L3 control layer: controller_manager + broadcaster + chosen controller."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
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
    sim_backend = LaunchConfiguration("sim_backend")
    controller_thread_priority = LaunchConfiguration("controller_thread_priority")

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
        [
            FindPackageShare("teleop_bringup"),
            "config",
            PythonExpression([
                "'controllers_isaac.yaml' if '", sim_backend,
                "' == 'isaac' else 'controllers.yaml'",
            ]),
        ])
    control_rate_profile = PathJoinSubstitution(
        [
            FindPackageShare("teleop_bringup"),
            "config",
            PythonExpression([
                "'control_rate_sim.yaml' if '", use_sim,
                "' == 'true' else 'control_rate_real.yaml'",
            ]),
        ])

    cm = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_description,
            controllers_yaml,
            {
                # The simulated CanopenSystem write() publishes over DDS. Running
                # that path as FIFO can block behind non-RT middleware workers and
                # cause priority-inversion stalls. Keep simulation best-effort;
                # retain FIFO 50 for the direct CAN hardware path.
                "thread_priority": ParameterValue(
                    controller_thread_priority, value_type=int),
            },
            # Use an exact /controller_manager YAML key. A launch-generated
            # parameter dict is emitted under /** and loses precedence to the
            # exact key in controllers.yaml.
            control_rate_profile,
        ],
    )
    impedance_spawner = Node(
        package="controller_manager", executable="spawner",
        prefix="nice -n 19 ionice -c 3",
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
        prefix="nice -n 19 ionice -c 3",
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
        DeclareLaunchArgument("sim_backend", default_value="mujoco"),
        DeclareLaunchArgument(
            "controller_thread_priority",
            default_value=PythonExpression([
                "'0' if '", use_sim, "' == 'true' else '50'",
            ]),
            description="controller_manager FIFO priority; simulation defaults to 0",
        ),
        cm,
        TimerAction(period=3.0, actions=[impedance_spawner, forward_spawner]),
    ])

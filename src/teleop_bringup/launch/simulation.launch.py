"""Select the backend-neutral L5/L6 simulation implementation."""

# MuJoCo remains the default in P2/P3.  Isaac starts only the ROS-side adapter;
# its heavyweight runtime is intentionally managed by an isolated environment.

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare


SUPPORTED_SIM_BACKENDS = ("mujoco", "isaac")
IMPLEMENTED_SIM_BACKENDS = SUPPORTED_SIM_BACKENDS


def validate_backend_name(backend: str) -> str:
    """Return a normalized implemented backend or raise a diagnostic error."""
    normalized = str(backend).strip().lower()
    if normalized not in SUPPORTED_SIM_BACKENDS:
        choices = ", ".join(SUPPORTED_SIM_BACKENDS)
        raise ValueError(f"sim_backend must be one of: {choices}; got {backend!r}")
    if normalized not in IMPLEMENTED_SIM_BACKENDS:
        raise RuntimeError(f"sim_backend={normalized} has no installed adapter")
    return normalized


def _validate_backend(context):
    validate_backend_name(LaunchConfiguration("sim_backend").perform(context))
    return []


def generate_launch_description():
    sim_backend = LaunchConfiguration("sim_backend")
    mujoco_backend_launch = PathJoinSubstitution([
        FindPackageShare("teleop_bringup"),
        "launch",
        "backends",
        "mujoco.launch.py",
    ])
    isaac_backend_launch = PathJoinSubstitution([
        FindPackageShare("teleop_bringup"),
        "launch",
        "backends",
        "isaac.launch.py",
    ])

    backend_arguments = {
        "capture_mode": LaunchConfiguration("capture_mode"),
        "model_path": LaunchConfiguration("model_path"),
        "headless": LaunchConfiguration("headless"),
        "randomize": LaunchConfiguration("randomize"),
        "randomization_path": LaunchConfiguration("randomization_path"),
        "camera_name": LaunchConfiguration("camera_name"),
        "scene_use_mujoco_renderer": LaunchConfiguration("scene_use_mujoco_renderer"),
        "camera_width": LaunchConfiguration("camera_width"),
        "camera_height": LaunchConfiguration("camera_height"),
        "camera_rate": LaunchConfiguration("camera_rate"),
        "publish_depth": LaunchConfiguration("publish_depth"),
        "enable_wrist_camera": LaunchConfiguration("enable_wrist_camera"),
        "wrist_use_mujoco_renderer": LaunchConfiguration("wrist_use_mujoco_renderer"),
        "wrist_camera_width": LaunchConfiguration("wrist_camera_width"),
        "wrist_camera_height": LaunchConfiguration("wrist_camera_height"),
        "enable_tactile": LaunchConfiguration("enable_tactile"),
        "contact_debug_enabled": LaunchConfiguration("contact_debug_enabled"),
        "contact_debug_period_s": LaunchConfiguration("contact_debug_period_s"),
        "grasp_assist_enabled": LaunchConfiguration("grasp_assist_enabled"),
        "gripper_force_max_n": LaunchConfiguration("gripper_force_max_n"),
        "gripper_contact_hold_margin": LaunchConfiguration("gripper_contact_hold_margin"),
        "gripper_force_squeeze_margin_max": LaunchConfiguration(
            "gripper_force_squeeze_margin_max"
        ),
    }
    isaac_backend_arguments = {
        "target_object_name": LaunchConfiguration("target_object_name"),
        "isaac_source_namespace": LaunchConfiguration("isaac_source_namespace"),
        "isaac_startup_timeout_s": LaunchConfiguration("isaac_startup_timeout_s"),
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            "sim_backend",
            default_value="mujoco",
            choices=list(SUPPORTED_SIM_BACKENDS),
            description=(
                "Simulation backend. MuJoCo remains the default; Isaac uses "
                "an externally managed runtime plus the ROS adapter."
            ),
        ),
        DeclareLaunchArgument("capture_mode", default_value="portfolio"),
        DeclareLaunchArgument("target_object_name", default_value="object_red_box"),
        DeclareLaunchArgument("isaac_source_namespace", default_value="/isaac"),
        DeclareLaunchArgument("isaac_startup_timeout_s", default_value="45.0"),
        DeclareLaunchArgument(
            "model_path",
            default_value="config/models/franka_panda.xml",
            description="Legacy MuJoCo XML path retained for P1 compatibility.",
        ),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("randomize", default_value="false"),
        DeclareLaunchArgument(
            "randomization_path",
            default_value="config/randomization.yaml",
        ),
        DeclareLaunchArgument("camera_name", default_value="scene_camera"),
        DeclareLaunchArgument("scene_use_mujoco_renderer", default_value="true"),
        DeclareLaunchArgument("camera_width", default_value="320"),
        DeclareLaunchArgument("camera_height", default_value="240"),
        DeclareLaunchArgument("camera_rate", default_value="10.0"),
        DeclareLaunchArgument("publish_depth", default_value="false"),
        DeclareLaunchArgument("enable_wrist_camera", default_value="false"),
        DeclareLaunchArgument("wrist_use_mujoco_renderer", default_value="true"),
        DeclareLaunchArgument("wrist_camera_width", default_value="320"),
        DeclareLaunchArgument("wrist_camera_height", default_value="240"),
        DeclareLaunchArgument("enable_tactile", default_value="false"),
        DeclareLaunchArgument("contact_debug_enabled", default_value="false"),
        DeclareLaunchArgument("contact_debug_period_s", default_value="1.0"),
        DeclareLaunchArgument(
            "grasp_assist_enabled",
            default_value="false",
            description=(
                "Legacy MuJoCo grasp assist. Must stay false for training-grade "
                "data collection."
            ),
        ),
        DeclareLaunchArgument("gripper_force_max_n", default_value="45.0"),
        DeclareLaunchArgument("gripper_contact_hold_margin", default_value="0.008"),
        DeclareLaunchArgument("gripper_force_squeeze_margin_max", default_value="0.010"),
        OpaqueFunction(function=_validate_backend),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([mujoco_backend_launch]),
            launch_arguments=backend_arguments.items(),
            condition=IfCondition(PythonExpression([
                "'", sim_backend, "' == 'mujoco'"
            ])),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([isaac_backend_launch]),
            launch_arguments=isaac_backend_arguments.items(),
            condition=IfCondition(PythonExpression([
                "'", sim_backend, "' == 'isaac'"
            ])),
        ),
    ])

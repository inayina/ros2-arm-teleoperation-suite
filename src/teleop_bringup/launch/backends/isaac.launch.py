"""Start the ROS-side adapter for an externally managed Isaac Sim backend."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    target_object_name = LaunchConfiguration("target_object_name")
    source_namespace = LaunchConfiguration("isaac_source_namespace")
    startup_timeout_s = LaunchConfiguration("isaac_startup_timeout_s")

    adapter = Node(
        package="isaac_sim_adapter",
        executable="isaac_sim_adapter",
        name="isaac_sim_adapter",
        output="screen",
        parameters=[{
            "target_object_name": target_object_name,
            "source_namespace": source_namespace,
            "startup_timeout_s": startup_timeout_s,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "target_object_name",
            default_value="object_red_box",
            description="P3 Isaac scene object exposed on /sim/object_pose.",
        ),
        DeclareLaunchArgument(
            "isaac_source_namespace",
            default_value="/isaac",
            description="Raw ROS namespace published by the isolated Isaac process.",
        ),
        DeclareLaunchArgument(
            "isaac_startup_timeout_s",
            default_value="45.0",
            description="Fail if no raw Isaac joint state arrives in this interval.",
        ),
        LogInfo(msg=(
            "Isaac backend selected. Start scripts/isaac_panda_backend.py with "
            "the isolated Isaac Sim Python environment; this ROS launch does "
            "not import or embed Isaac Sim."
        )),
        adapter,
        RegisterEventHandler(
            OnProcessExit(
                target_action=adapter,
                on_exit=[EmitEvent(
                    event=Shutdown(reason="Isaac ROS adapter exited")
                )],
            )
        ),
    ])

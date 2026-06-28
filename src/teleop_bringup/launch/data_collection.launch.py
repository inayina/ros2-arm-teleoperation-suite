import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    randomize_arg = DeclareLaunchArgument('randomize', default_value='true')
    record_arg = DeclareLaunchArgument('record', default_value='true')
    headless_arg = DeclareLaunchArgument('headless', default_value='false')

    teleop_bringup_dir = get_package_share_directory('teleop_bringup')

    # We reuse the full_system.launch.py but pass arguments
    full_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(teleop_bringup_dir, 'launch', 'full_system.launch.py')
        ),
        launch_arguments={
            'randomize': LaunchConfiguration('randomize'),
            'record': LaunchConfiguration('record'),
            'headless': LaunchConfiguration('headless'),
        }.items()
    )

    return LaunchDescription([
        randomize_arg,
        record_arg,
        headless_arg,
        full_system
    ])

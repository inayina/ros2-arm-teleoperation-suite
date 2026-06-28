# Copyright 2026 ros2-arm-teleoperation-suite contributors
# SPDX-License-Identifier: MIT
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""L0/L2: teleop input + MoveIt Servo motion layer."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim')
    can_interface = LaunchConfiguration('can_interface')
    start_teleop = LaunchConfiguration('start_teleop')
    teleop_driver = LaunchConfiguration('teleop_driver')

    servo_launch = PathJoinSubstitution(
        [FindPackageShare('teleop_moveit_config'), 'launch', 'servo.launch.py'])
    teleop_config = PathJoinSubstitution(
        [FindPackageShare('teleop_input'), 'config', 'teleop_config.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true'),
        DeclareLaunchArgument('can_interface', default_value='vcan0'),
        DeclareLaunchArgument('start_teleop', default_value='true'),
        DeclareLaunchArgument('teleop_driver', default_value='keyboard'),

        Node(
            package='teleop_input',
            executable='teleop_input_node',
            name='teleop_input',
            output='screen',
            parameters=[teleop_config, {'driver_type': teleop_driver}],
            condition=IfCondition(start_teleop),
        ),
        # Pass use_sim and can_interface to servo.launch.py so that the xacro
        # command generates the correct robot_description (sim vs. real CAN).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([servo_launch]),
            launch_arguments={
                'use_sim': use_sim,
                'can_interface': can_interface,
            }.items(),
        ),
    ])

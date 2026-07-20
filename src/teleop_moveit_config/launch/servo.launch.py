# Copyright 2026 ros2-arm-teleoperation-suite contributors
# SPDX-License-Identifier: MIT
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""L2 motion layer: MoveIt Servo (pose-tracking) consuming /safe_master_pose."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def _load_yaml(pkg, rel):
    path = os.path.join(get_package_share_directory(pkg), rel)
    with open(path, 'r') as fh:
        return yaml.safe_load(fh)


def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim')
    can_interface = LaunchConfiguration('can_interface')
    servo_mode = LaunchConfiguration('servo_mode')
    servo_thread_priority = LaunchConfiguration('servo_thread_priority')

    xacro_file = PathJoinSubstitution(
        [FindPackageShare('teleop_description'), 'urdf', 'panda.urdf.xacro']
    )
    srdf_file = os.path.join(
        get_package_share_directory('teleop_moveit_config'), 'srdf', 'panda.srdf')

    robot_description = {
        'robot_description': ParameterValue(Command(
            [
                FindExecutable(name='xacro'), ' ', xacro_file,
                ' use_sim:=', use_sim,
                ' can_interface:=', can_interface,
            ]
        ), value_type=str)
    }
    with open(srdf_file, 'r') as fh:
        robot_description_semantic = {'robot_description_semantic': fh.read()}

    # Load servo params under the moveit_servo namespace (Jazzy generate_parameter_library).
    _servo_yaml = _load_yaml('teleop_moveit_config', 'config/servo.yaml')
    servo_params = {
        'moveit_servo': _servo_yaml['moveit_servo']['ros__parameters'],
    }
    # The simulation data path crosses DDS and a separate MuJoCo process. A
    # FIFO Servo loop can starve those best-effort middleware workers and cause
    # priority-inversion stalls. Keep simulation best-effort while retaining
    # MoveIt Servo's priority 40 on the direct hardware control path.
    servo_params['moveit_servo']['thread_priority'] = ParameterValue(
        servo_thread_priority, value_type=int)
    kinematics = _load_yaml('teleop_moveit_config', 'config/kinematics.yaml')
    joint_limits = _load_yaml('teleop_moveit_config', 'config/joint_limits.yaml')

    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        output='screen',
        # MoveIt 2.12.4 attempts FIFO before loading the configured
        # thread_priority. Constrain the process rlimit at exec time for the
        # DDS-backed simulator; do not constrain the real-hardware process.
        prefix=PythonExpression([
            "'prlimit --rtprio=0:0 --' if '", use_sim,
            "' == 'true' else ''",
        ]),
        parameters=[
            robot_description,
            robot_description_semantic,
            servo_params,
            {'robot_description_kinematics': kinematics},
            {'robot_description_planning': joint_limits},
        ],
        remappings=[
            ('~/pose_target_cmds', '/safe_master_pose'),
            ('~/delta_twist_cmds', '/safe_master_twist'),
        ],
    )

    # MoveIt Servo starts paused; initialize it with service waits/retries so
    # /joint_target is created reliably even under heavy simulator startup load.
    servo_init = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'teleop_moveit_config', 'initialize_servo',
            '--servo-node', '/servo_node',
            '--joint-target-topic', '/joint_target',
            '--pose-input-topic', '/safe_master_pose',
            '--twist-input-topic', '/safe_master_twist',
            '--command-type', servo_mode,
            '--timeout', '30.0',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true'),
        DeclareLaunchArgument('can_interface', default_value='vcan0'),
        DeclareLaunchArgument('servo_mode', default_value='pose',
                              description='pose | twist (MoveIt Servo command type)'),
        DeclareLaunchArgument(
            'servo_thread_priority',
            default_value=PythonExpression([
                "'0' if '", use_sim, "' == 'true' else '40'",
            ]),
            description='MoveIt Servo FIFO priority; simulation defaults to 0',
        ),
        servo_node,
        servo_init,
    ])

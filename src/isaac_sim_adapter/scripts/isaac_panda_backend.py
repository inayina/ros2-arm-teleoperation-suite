#!/usr/bin/env python3
"""Run the isolated P3 Isaac Panda scene and publish raw ROS 2 topics.

This script must be executed with the Isaac Sim Python environment, not with
the regular ROS workspace interpreter.  Source ROS 2 Jazzy first so Isaac's
ROS bridge and rclpy use the same middleware as the workspace.
"""

from __future__ import annotations

import argparse
import json
import time

from isaacsim import SimulationApp


def parse_args():
    """Parse the deliberately small P3 scene surface."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--gui', action='store_true', help='Show the Isaac UI')
    parser.add_argument('--duration-sec', type=float, default=120.0)
    parser.add_argument('--width', type=int, default=320)
    parser.add_argument('--height', type=int, default=240)
    parser.add_argument('--camera-rate', type=float, default=10.0)
    parser.add_argument('--command-timeout-s', type=float, default=0.1)
    parser.add_argument(
        '--arm-command-mode', choices=('effort', 'position'), default='effort',
        help='Arm execution boundary; position uses Isaac-local drive control',
    )
    parser.add_argument(
        '--observe-effort-only', action='store_true',
        help='Validate effort traffic without switching or driving arm DOFs',
    )
    return parser.parse_args()


ARGS = parse_args()
SIMULATION_APP = SimulationApp({
    'headless': not ARGS.gui,
    'hide_ui': not ARGS.gui,
    'width': ARGS.width,
    'height': ARGS.height,
    'multi_gpu': False,
    'max_gpu_count': 1,
})


def create_joint_graph(og, sdf_path) -> None:
    """Publish Panda joint state through the non-deprecated Isaac 6.0 path."""
    og.Controller.edit(
        {'graph_path': '/World/JointStateGraph', 'evaluator_name': 'execution'},
        {
            og.Controller.Keys.CREATE_NODES: [
                ('OnPlaybackTick', 'omni.graph.action.OnPlaybackTick'),
                ('ReadJointState', 'isaacsim.sensors.physics.IsaacReadJointState'),
                ('PublishJointState', 'isaacsim.ros2.bridge.ROS2PublishJointState'),
                ('ReadSimTime', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
            ],
            og.Controller.Keys.SET_VALUES: [
                ('ReadJointState.inputs:prim', [sdf_path('/World/Franka')]),
                ('PublishJointState.inputs:topicName', '/isaac/joint_states'),
            ],
            og.Controller.Keys.CONNECT: [
                ('OnPlaybackTick.outputs:tick', 'ReadJointState.inputs:execIn'),
                ('ReadJointState.outputs:execOut', 'PublishJointState.inputs:execIn'),
                (
                    'ReadJointState.outputs:jointNames',
                    'PublishJointState.inputs:jointNames',
                ),
                (
                    'ReadJointState.outputs:jointPositions',
                    'PublishJointState.inputs:jointPositions',
                ),
                (
                    'ReadJointState.outputs:jointVelocities',
                    'PublishJointState.inputs:jointVelocities',
                ),
                (
                    'ReadJointState.outputs:jointEfforts',
                    'PublishJointState.inputs:jointEfforts',
                ),
                (
                    'ReadJointState.outputs:jointDofTypes',
                    'PublishJointState.inputs:jointDofTypes',
                ),
                (
                    'ReadJointState.outputs:stageMetersPerUnit',
                    'PublishJointState.inputs:stageMetersPerUnit',
                ),
                (
                    'ReadJointState.outputs:sensorTime',
                    'PublishJointState.inputs:sensorTime',
                ),
                (
                    'ReadSimTime.outputs:simulationTime',
                    'PublishJointState.inputs:timeStamp',
                ),
            ],
        },
    )


def create_camera_graph(og, sdf_path) -> None:
    """Publish one low-resolution scene RGB stream for the P3 subset."""
    og.Controller.edit(
        {'graph_path': '/World/CameraGraph', 'evaluator_name': 'execution'},
        {
            og.Controller.Keys.CREATE_NODES: [
                ('OnPlaybackTick', 'omni.graph.action.OnPlaybackTick'),
                ('CreateRenderProduct', 'isaacsim.core.nodes.IsaacCreateRenderProduct'),
                ('PublishRgb', 'isaacsim.ros2.bridge.ROS2CameraHelper'),
            ],
            og.Controller.Keys.SET_VALUES: [
                (
                    'CreateRenderProduct.inputs:cameraPrim',
                    [sdf_path('/World/SceneCamera')],
                ),
                ('CreateRenderProduct.inputs:height', ARGS.height),
                ('CreateRenderProduct.inputs:width', ARGS.width),
                (
                    'PublishRgb.inputs:topicName',
                    '/isaac/camera/color/image_raw',
                ),
                ('PublishRgb.inputs:type', 'rgb'),
            ],
            og.Controller.Keys.CONNECT: [
                (
                    'OnPlaybackTick.outputs:tick',
                    'CreateRenderProduct.inputs:execIn',
                ),
                (
                    'CreateRenderProduct.outputs:execOut',
                    'PublishRgb.inputs:execIn',
                ),
                (
                    'CreateRenderProduct.outputs:renderProductPath',
                    'PublishRgb.inputs:renderProductPath',
                ),
            ],
        },
    )


def main() -> None:
    """Build the scene, bridge the P3 topics, and step for a bounded time."""
    try:
        import numpy as np
        import omni.graph.core as og
        import omni.replicator.core as rep
        import rclpy
        import usdrt.Sdf
        from geometry_msgs.msg import PoseStamped, WrenchStamped
        from isaac_sim_adapter.effort_control import LatestEffortCommand
        from isaac_sim_adapter.effort_control import PANDA_ARM_JOINTS
        from isaac_sim_adapter.effort_control import PANDA_TORQUE_LIMITS_NM
        from isaac_sim_adapter.policy_control import offset_pose_in_local_frame
        from isaac_sim_adapter.policy_control import validate_panda_joint_positions
        from isaacsim.core.api import World
        from isaacsim.core.api.objects import DynamicCuboid
        from isaacsim.core.utils.extensions import enable_extension
        from isaacsim.core.utils.types import ArticulationAction
        from isaacsim.robot.manipulators.examples.franka import Franka
        from rclpy.qos import qos_profile_sensor_data
        from std_msgs.msg import Bool, Empty, Float64, Float64MultiArray, String

        if not enable_extension('isaacsim.ros2.bridge'):
            raise RuntimeError('Failed to enable isaacsim.ros2.bridge')
        for _ in range(10):
            SIMULATION_APP.update()

        world = World(stage_units_in_meters=1.0, backend='numpy', device='cpu')
        world.scene.add_default_ground_plane()
        franka = world.scene.add(Franka(
            prim_path='/World/Franka',
            name='franka',
            end_effector_prim_name='panda_hand',
        ))
        red_box = world.scene.add(DynamicCuboid(
            prim_path='/World/object_red_box',
            name='object_red_box',
            position=np.array([0.45, 0.0, 0.04]),
            scale=np.array([0.06, 0.06, 0.08]),
            color=np.array([0.8, 0.05, 0.05]),
            mass=0.08,
        ))
        rep.functional.create.camera(
            position=(1.25, 1.0, 1.05),
            look_at=(0.35, 0.0, 0.25),
            parent='/World',
            name='SceneCamera',
        )
        world.reset()
        # Match the upstream impedance controller contract: MuJoCo applies
        # robot gravity compensation, so Isaac must do the same while scene
        # objects remain fully dynamic under gravity.
        franka.disable_gravity()

        create_joint_graph(og, usdrt.Sdf.Path)
        create_camera_graph(og, usdrt.Sdf.Path)

        rclpy.init()
        node = rclpy.create_node('isaac_panda_backend')
        object_pub = node.create_publisher(
            PoseStamped, '/isaac/object_pose', 10
        )
        ee_pub = node.create_publisher(PoseStamped, '/isaac/ee_pose', 10)
        ft_pub = node.create_publisher(WrenchStamped, '/isaac/ft_sensor', 10)
        reset_pub = node.create_publisher(
            Bool, '/isaac/reset_scene_done', 1
        )
        heartbeat_pub = node.create_publisher(
            String, '/isaac/backend_heartbeat', 1
        )
        reset_requested = False
        effort_control_enabled = False
        gripper_target = 1.0
        gripper_command_pending = False
        gripper_command_count = 0
        position_target = None
        position_target_pending = False
        position_command_count = 0
        command_count = 0
        invalid_command_count = 0
        reset_history_drop_count = 0
        active_effort_steps = 0
        zero_fail_safe_steps = 0
        last_effort_status = 'no_command'
        command_enable_after = 0.0
        effort_buffer = LatestEffortCommand(
            command_timeout_s=ARGS.command_timeout_s,
            state_timeout_s=max(ARGS.command_timeout_s * 2.0, 0.1),
        )
        arm_joint_indices = np.array([
            int(franka._articulation_view._metadata.joint_indices[name])
            for name in PANDA_ARM_JOINTS
        ], dtype=np.int32)
        hand_joint_row = 1 + int(
            franka._articulation_view._metadata.joint_indices['panda_hand_joint']
        )

        def configure_effort_control() -> None:
            nonlocal effort_control_enabled
            if effort_control_enabled:
                return
            controller = franka.get_articulation_controller()
            for joint_index in arm_joint_indices:
                controller.switch_dof_control_mode(int(joint_index), 'effort')
            controller.set_max_efforts(
                np.asarray(PANDA_TORQUE_LIMITS_NM, dtype=np.float32),
                joint_indices=arm_joint_indices,
            )
            effort_control_enabled = True
            print('ISAAC_E1_EVENT=' + json.dumps({
                'event': 'effort_control_enabled',
                'joint_indices': arm_joint_indices.tolist(),
                'monotonic_s': round(time.monotonic(), 9),
            }, sort_keys=True), flush=True)

        def request_reset(_message: Empty) -> None:
            nonlocal reset_requested
            effort_buffer.begin_reset()
            reset_requested = True

        def receive_effort(message: Float64MultiArray) -> None:
            nonlocal command_count, invalid_command_count
            nonlocal reset_history_drop_count
            if time.monotonic() < command_enable_after:
                reset_history_drop_count += 1
                return
            effort_buffer.update_state()
            try:
                decision = effort_buffer.accept(message.data)
            except (TypeError, ValueError, OverflowError) as error:
                invalid_command_count += 1
                print('ISAAC_E1_EVENT=' + json.dumps({
                    'event': 'invalid_effort_command',
                    'details': str(error),
                    'monotonic_s': round(time.monotonic(), 9),
                }, sort_keys=True), flush=True)
                return
            if decision.status != 'active':
                return
            command_count += 1
            if not ARGS.observe_effort_only:
                configure_effort_control()

        def receive_gripper(message: Float64) -> None:
            nonlocal gripper_target, gripper_command_count
            nonlocal gripper_command_pending
            value = float(message.data)
            if not np.isfinite(value):
                print('ISAAC_E2_EVENT=' + json.dumps({
                    'event': 'invalid_gripper_command',
                    'monotonic_s': round(time.monotonic(), 9),
                }, sort_keys=True), flush=True)
                return
            gripper_target = max(0.0, min(1.0, value))
            gripper_command_pending = True
            gripper_command_count += 1

        def receive_position(message: Float64MultiArray) -> None:
            nonlocal position_target, position_target_pending
            nonlocal position_command_count, invalid_command_count
            try:
                position_target = np.asarray(
                    validate_panda_joint_positions(message.data),
                    dtype=np.float32,
                )
            except (TypeError, ValueError, OverflowError) as error:
                invalid_command_count += 1
                print('ISAAC_E2_EVENT=' + json.dumps({
                    'event': 'invalid_position_command',
                    'details': str(error),
                    'monotonic_s': round(time.monotonic(), 9),
                }, sort_keys=True), flush=True)
                return
            position_target_pending = True
            position_command_count += 1

        node.create_subscription(
            Empty, '/isaac/reset_scene_cmd', request_reset, 1
        )
        if ARGS.arm_command_mode == 'effort':
            node.create_subscription(
                Float64MultiArray,
                '/isaac/joint_effort_cmd',
                receive_effort,
                qos_profile_sensor_data,
            )
        else:
            node.create_subscription(
                Float64MultiArray,
                '/isaac/joint_position_cmd',
                receive_position,
                10,
            )
        node.create_subscription(
            Float64, '/isaac/gripper_cmd', receive_gripper, 10
        )

        world.play()
        started_at = time.monotonic()
        deadline = started_at + max(0.0, ARGS.duration_sec)
        camera_period = 1.0 / max(0.1, ARGS.camera_rate)
        last_render = 0.0
        last_aux_publish = 0.0
        frames = 0
        print('ISAAC_E1_READY=' + json.dumps({
            'status': 'READY',
            'joint_topic': '/isaac/joint_states',
            'object_topic': '/isaac/object_pose',
            'ee_topic': '/isaac/ee_pose',
            'ft_topic': '/isaac/ft_sensor',
            'camera_topic': '/isaac/camera/color/image_raw',
            'effort_command_topic': '/isaac/joint_effort_cmd',
            'position_command_topic': '/isaac/joint_position_cmd',
            'gripper_command_topic': '/isaac/gripper_cmd',
            'command_timeout_s': ARGS.command_timeout_s,
            'command_qos': 'sensor_data',
            'torque_limits_nm': list(PANDA_TORQUE_LIMITS_NM),
            'resolution': [ARGS.width, ARGS.height],
            'robot_gravity_compensation': 'disable_gravity',
            'observe_effort_only': ARGS.observe_effort_only,
            'arm_command_mode': ARGS.arm_command_mode,
        }, sort_keys=True), flush=True)

        while SIMULATION_APP.is_running():
            now = time.monotonic()
            if ARGS.duration_sec > 0.0 and now >= deadline:
                break
            render = now - last_render >= camera_period
            world.step(render=render)
            if render:
                last_render = now
            rclpy.spin_once(node, timeout_sec=0.0)

            if reset_requested:
                try:
                    world.reset()
                    franka.disable_gravity()
                    effort_control_enabled = False
                    position_target = None
                    position_target_pending = False
                    red_box.set_world_pose(
                        position=np.array([0.45, 0.0, 0.04])
                    )
                    effort_buffer.complete_reset()
                    effort_buffer.update_state()
                    command_enable_after = time.monotonic() + 0.05
                    reset_pub.publish(Bool(data=True))
                except Exception as error:  # Isaac owns the runtime details.
                    print(f'ISAAC_P3_RESET_ERROR={error!r}', flush=True)
                    reset_pub.publish(Bool(data=False))
                reset_requested = False

            effort_buffer.update_state(now)
            effort_decision = effort_buffer.output(now)
            if ARGS.arm_command_mode == 'effort':
                if effort_decision.should_publish:
                    if not effort_control_enabled and not ARGS.observe_effort_only:
                        configure_effort_control()
                    if not ARGS.observe_effort_only:
                        franka.apply_action(ArticulationAction(
                            joint_efforts=np.asarray(
                                effort_decision.efforts, dtype=np.float32
                            ),
                            joint_indices=arm_joint_indices,
                        ))
                    if effort_decision.status == 'active':
                        active_effort_steps += 1
                    else:
                        zero_fail_safe_steps += 1
            elif position_target_pending:
                franka.apply_action(ArticulationAction(
                    joint_positions=position_target,
                    joint_indices=arm_joint_indices,
                ))
                position_target_pending = False
            if gripper_command_pending:
                # Use the initialized ParallelGripper interface. A partial
                # articulation action with explicit finger indices stops the
                # Isaac 6.0 timeline after arm DOFs switch to effort mode.
                franka.gripper.set_joint_positions(np.asarray(
                    [0.04 * gripper_target, 0.04 * gripper_target],
                    dtype=np.float32,
                ))
                gripper_command_pending = False
            if effort_decision.status != last_effort_status:
                if effort_decision.status in {
                    'command_stale', 'state_stale', 'reset_in_progress'
                }:
                    print('ISAAC_E1_EVENT=' + json.dumps({
                        'event': effort_decision.status,
                        'command_age_s': effort_decision.command_age_s,
                        'state_age_s': effort_decision.state_age_s,
                        'response': 'zero_effort',
                        'monotonic_s': round(now, 9),
                    }, sort_keys=True), flush=True)
                last_effort_status = effort_decision.status

            if now - last_aux_publish >= 0.05:
                position, orientation = red_box.get_world_pose()
                pose = PoseStamped()
                pose.header.stamp = node.get_clock().now().to_msg()
                pose.header.frame_id = 'world'
                pose.pose.position.x = float(position[0])
                pose.pose.position.y = float(position[1])
                pose.pose.position.z = float(position[2])
                pose.pose.orientation.w = float(orientation[0])
                pose.pose.orientation.x = float(orientation[1])
                pose.pose.orientation.y = float(orientation[2])
                pose.pose.orientation.z = float(orientation[3])
                object_pub.publish(pose)
                hand_position, hand_orientation_wxyz = (
                    franka.end_effector.get_world_pose()
                )
                # Isaac's Franka helper exposes panda_hand, while the dataset
                # and MoveIt Servo contract use panda_ee. Match the URDF fixed
                # panda_hand -> panda_ee transform (local +Z, 0.10 m).
                hand_orientation_xyzw = (
                    float(hand_orientation_wxyz[1]),
                    float(hand_orientation_wxyz[2]),
                    float(hand_orientation_wxyz[3]),
                    float(hand_orientation_wxyz[0]),
                )
                ee_position, ee_orientation_xyzw = offset_pose_in_local_frame(
                    hand_position, hand_orientation_xyzw, (0.0, 0.0, 0.10)
                )
                ee_pose = PoseStamped()
                ee_pose.header.stamp = pose.header.stamp
                ee_pose.header.frame_id = 'world'
                ee_pose.pose.position.x = float(ee_position[0])
                ee_pose.pose.position.y = float(ee_position[1])
                ee_pose.pose.position.z = float(ee_position[2])
                ee_pose.pose.orientation.x = ee_orientation_xyzw[0]
                ee_pose.pose.orientation.y = ee_orientation_xyzw[1]
                ee_pose.pose.orientation.z = ee_orientation_xyzw[2]
                ee_pose.pose.orientation.w = ee_orientation_xyzw[3]
                ee_pub.publish(ee_pose)

                reaction = franka.get_measured_joint_forces(
                    np.array([hand_joint_row])
                )[0]
                wrench = WrenchStamped()
                wrench.header.stamp = pose.header.stamp
                wrench.header.frame_id = 'panda_hand'
                wrench.wrench.force.x = float(reaction[0])
                wrench.wrench.force.y = float(reaction[1])
                wrench.wrench.force.z = float(reaction[2])
                wrench.wrench.torque.x = float(reaction[3])
                wrench.wrench.torque.y = float(reaction[4])
                wrench.wrench.torque.z = float(reaction[5])
                ft_pub.publish(wrench)
                heartbeat_pub.publish(String(data=json.dumps({
                    'status': 'e1_running',
                    'effort_status': effort_decision.status,
                    'command_count': command_count,
                }, sort_keys=True)))
                last_aux_publish = now
            frames += 1

        elapsed = time.monotonic() - started_at
        print('ISAAC_E1_DONE=' + json.dumps({
            'status': 'PASS',
            'frames': frames,
            'elapsed_sec': round(elapsed, 3),
            'command_count': command_count,
            'invalid_command_count': invalid_command_count,
            'reset_history_drop_count': reset_history_drop_count,
            'active_effort_steps': active_effort_steps,
            'zero_fail_safe_steps': zero_fail_safe_steps,
            'gripper_command_count': gripper_command_count,
            'position_command_count': position_command_count,
            'final_gripper_target': gripper_target,
        }, sort_keys=True), flush=True)
        world.stop()
        node.destroy_node()
        rclpy.shutdown()
    finally:
        SIMULATION_APP.close()


if __name__ == '__main__':
    main()

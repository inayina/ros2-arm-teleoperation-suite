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
        from isaacsim.core.api import World
        from isaacsim.core.api.objects import DynamicCuboid
        from isaacsim.core.utils.extensions import enable_extension
        from isaacsim.robot.manipulators.examples.franka import Franka
        from std_msgs.msg import Bool, Empty, String

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
        hand_joint_row = 1 + int(
            franka._articulation_view._metadata.joint_indices['panda_hand_joint']
        )

        def request_reset(_message: Empty) -> None:
            nonlocal reset_requested
            reset_requested = True

        node.create_subscription(
            Empty, '/isaac/reset_scene_cmd', request_reset, 1
        )

        world.play()
        started_at = time.monotonic()
        deadline = started_at + max(0.0, ARGS.duration_sec)
        camera_period = 1.0 / max(0.1, ARGS.camera_rate)
        last_render = 0.0
        last_aux_publish = 0.0
        frames = 0
        print('ISAAC_P3_READY=' + json.dumps({
            'status': 'READY',
            'joint_topic': '/isaac/joint_states',
            'object_topic': '/isaac/object_pose',
            'ee_topic': '/isaac/ee_pose',
            'ft_topic': '/isaac/ft_sensor',
            'camera_topic': '/isaac/camera/color/image_raw',
            'resolution': [ARGS.width, ARGS.height],
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
                    red_box.set_world_pose(
                        position=np.array([0.45, 0.0, 0.04])
                    )
                    reset_pub.publish(Bool(data=True))
                except Exception as error:  # Isaac owns the runtime details.
                    print(f'ISAAC_P3_RESET_ERROR={error!r}', flush=True)
                    reset_pub.publish(Bool(data=False))
                reset_requested = False

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
                ee_position, ee_orientation = franka.end_effector.get_world_pose()
                ee_pose = PoseStamped()
                ee_pose.header.stamp = pose.header.stamp
                ee_pose.header.frame_id = 'world'
                ee_pose.pose.position.x = float(ee_position[0])
                ee_pose.pose.position.y = float(ee_position[1])
                ee_pose.pose.position.z = float(ee_position[2])
                ee_pose.pose.orientation.w = float(ee_orientation[0])
                ee_pose.pose.orientation.x = float(ee_orientation[1])
                ee_pose.pose.orientation.y = float(ee_orientation[2])
                ee_pose.pose.orientation.z = float(ee_orientation[3])
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
                heartbeat_pub.publish(String(data='p3_running'))
                last_aux_publish = now
            frames += 1

        elapsed = time.monotonic() - started_at
        print('ISAAC_P3_DONE=' + json.dumps({
            'status': 'PASS',
            'frames': frames,
            'elapsed_sec': round(elapsed, 3),
        }, sort_keys=True), flush=True)
        world.stop()
        node.destroy_node()
        rclpy.shutdown()
    finally:
        SIMULATION_APP.close()


if __name__ == '__main__':
    main()

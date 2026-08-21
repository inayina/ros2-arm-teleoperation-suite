#!/usr/bin/env python3
"""Fail-closed MuJoCo policy-input freshness probe for scene+wrist SmolVLA.

This program never publishes an execution command.  It proves that all five
runtime observations required by the dual-camera policy are present and fresh
before a closed-loop rollout is allowed to start.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
import time
from typing import Any


@dataclass(frozen=True)
class Requirement:
    key: str
    topic: str
    kind: str


REQUIREMENTS = (
    Requirement('encoder_state', '/sim/encoder_state', 'joint_state'),
    Requirement('ee_pose', '/ee_pose', 'pose'),
    Requirement('gripper_state', '/gripper/state', 'float64'),
    Requirement('scene_rgb', '/camera/color/image_raw', 'image'),
    Requirement('wrist_rgb', '/camera/wrist/color/image_raw', 'image'),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--timeout-s', type=float, default=15.0)
    return parser.parse_args()


def summarize(requirement: Requirement, message: Any) -> dict[str, Any]:
    if requirement.kind == 'joint_state':
        values = [float(value) for value in message.position]
        return {
            'position_count': len(values),
            'name_count': len(message.name),
            'finite': all(math.isfinite(value) for value in values),
            'valid': len(values) >= 7 and all(math.isfinite(value) for value in values),
        }
    if requirement.kind == 'pose':
        p = message.pose.position
        q = message.pose.orientation
        values = [float(p.x), float(p.y), float(p.z), float(q.x), float(q.y), float(q.z), float(q.w)]
        return {
            'frame_id': str(message.header.frame_id),
            'xyz_xyzw': values,
            'finite': all(math.isfinite(value) for value in values),
            'valid': all(math.isfinite(value) for value in values),
        }
    if requirement.kind == 'float64':
        value = float(message.data)
        return {'value': value, 'finite': math.isfinite(value), 'valid': math.isfinite(value)}
    if requirement.kind == 'image':
        width, height = int(message.width), int(message.height)
        return {
            'width': width,
            'height': height,
            'encoding': str(message.encoding),
            'valid': width > 0 and height > 0 and len(message.data) > 0,
        }
    raise ValueError(f'unsupported kind {requirement.kind!r}')


def main() -> int:
    args = parse_args()
    if args.timeout_s <= 0.0:
        raise SystemExit('--timeout-s must be positive')

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, JointState
    from std_msgs.msg import Float64

    type_by_kind = {
        'joint_state': JointState,
        'pose': PoseStamped,
        'float64': Float64,
        'image': Image,
    }
    observed: dict[str, dict[str, Any]] = {}
    started = time.monotonic()
    rclpy.init(args=None)
    node = Node('mujoco_dualcam_runtime_preflight')
    subscriptions = []

    def make_callback(requirement: Requirement):
        def callback(message: Any) -> None:
            summary = summarize(requirement, message)
            if summary['valid']:
                observed[requirement.key] = {
                    'topic': requirement.topic,
                    'received_monotonic_s': time.monotonic(),
                    **summary,
                }
        return callback

    for requirement in REQUIREMENTS:
        subscriptions.append(node.create_subscription(
            type_by_kind[requirement.kind], requirement.topic,
            make_callback(requirement), qos_profile_sensor_data,
        ))

    deadline = started + args.timeout_s
    try:
        while time.monotonic() < deadline and len(observed) < len(REQUIREMENTS):
            rclpy.spin_once(node, timeout_sec=min(0.2, deadline - time.monotonic()))
    finally:
        finished = time.monotonic()
        missing = [item.key for item in REQUIREMENTS if item.key not in observed]
        payload = {
            'contract_version': 'mujoco_dualcam_policy_input_v1',
            'status': 'PASS' if not missing else 'FAIL',
            'backend': 'mujoco',
            'camera_variant': 'scene_wrist',
            'policy_input_contract': {
                'state': 'observation.state[15]',
                'state_layout': ['joint_position[7]', 'ee_pose_xyzw[7]', 'measured_gripper[1]'],
                'images': ['scene', 'wrist'],
                'object_pose_is_policy_input': False,
            },
            'timeout_s': args.timeout_s,
            'elapsed_ms': round((finished - started) * 1000.0, 3),
            'required_topics': [item.__dict__ for item in REQUIREMENTS],
            'observed': observed,
            'missing': missing,
            'claims_task_success': False,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(payload, indent=2, sort_keys=True) + '\n', encoding='utf-8')
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(payload, sort_keys=True))
    return 0 if payload['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())

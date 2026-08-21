#!/usr/bin/env python3
"""Fail-closed freshness gate for the isolated Isaac-to-ROS bridge.

This probe has no command publisher.  It proves that the raw Isaac topics are
being transformed into the canonical topics required by the policy runtime
before a learned-policy process may start.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
import time
from typing import Any


@dataclass(frozen=True)
class TopicRequirement:
    key: str
    topic: str
    message_kind: str


BASE_REQUIREMENTS = (
    TopicRequirement('raw_joint_state', '/isaac/joint_states', 'joint_state'),
    TopicRequirement('encoder_state', '/sim/encoder_state', 'joint_state'),
    TopicRequirement('ee_pose', '/ee_pose', 'pose'),
    TopicRequirement('gripper_state', '/gripper/state', 'float64'),
    TopicRequirement('scene_rgb', '/camera/color/image_raw', 'image'),
    TopicRequirement('wrist_rgb', '/camera/wrist/color/image_raw', 'image'),
)
CONTROL_JOINT_STATES = TopicRequirement(
    'control_joint_state', '/joint_states', 'joint_state'
)


def required_topics(require_control_joint_states: bool) -> tuple[TopicRequirement, ...]:
    """Return the policy-input topic contract, plus the control-state proof."""
    if require_control_joint_states:
        return BASE_REQUIREMENTS + (CONTROL_JOINT_STATES,)
    return BASE_REQUIREMENTS


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--timeout-s', type=float, default=12.0)
    parser.add_argument('--require-control-joint-states', action='store_true')
    return parser.parse_args(argv)


def _message_summary(requirement: TopicRequirement, message: Any) -> dict[str, Any]:
    if requirement.message_kind == 'joint_state':
        return {
            'position_count': len(message.position),
            'name_count': len(message.name),
            'valid': len(message.position) >= 7,
        }
    if requirement.message_kind == 'image':
        return {
            'width': int(message.width),
            'height': int(message.height),
            'encoding': str(message.encoding),
            'valid': int(message.width) > 0 and int(message.height) > 0,
        }
    if requirement.message_kind == 'pose':
        return {
            'frame_id': str(message.header.frame_id),
            'valid': True,
        }
    if requirement.message_kind == 'float64':
        return {'value': float(message.data), 'valid': True}
    raise ValueError(f'unsupported message kind: {requirement.message_kind}')


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.timeout_s <= 0.0:
        raise SystemExit('--timeout-s must be positive')

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, JointState
    from std_msgs.msg import Float64

    message_types = {
        'joint_state': JointState,
        'pose': PoseStamped,
        'float64': Float64,
        'image': Image,
    }
    requirements = required_topics(args.require_control_joint_states)
    seen: dict[str, dict[str, Any]] = {}

    rclpy.init(args=None)
    node = Node('isaac_ros_topic_gate')
    subscriptions = []
    started = time.monotonic()

    def make_callback(requirement: TopicRequirement):
        def callback(message: Any) -> None:
            summary = _message_summary(requirement, message)
            if summary['valid']:
                seen[requirement.key] = {
                    'topic': requirement.topic,
                    'received_monotonic_s': time.monotonic(),
                    **summary,
                }

        return callback

    for requirement in requirements:
        subscriptions.append(node.create_subscription(
            message_types[requirement.message_kind],
            requirement.topic,
            make_callback(requirement),
            qos_profile_sensor_data,
        ))

    deadline = started + args.timeout_s
    try:
        while time.monotonic() < deadline and len(seen) != len(requirements):
            rclpy.spin_once(node, timeout_sec=min(0.2, deadline - time.monotonic()))
    finally:
        finished = time.monotonic()
        missing = [item.key for item in requirements if item.key not in seen]
        payload = {
            'contract_version': 'isaac_ros_topic_gate_v1',
            'status': 'PASS' if not missing else 'FAIL',
            'require_control_joint_states': args.require_control_joint_states,
            'timeout_s': args.timeout_s,
            'elapsed_ms': round((finished - started) * 1000.0, 3),
            'required_topics': [
                {'key': item.key, 'topic': item.topic, 'kind': item.message_kind}
                for item in requirements
            ],
            'observed': seen,
            'missing': missing,
            'claims_task_success': False,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(payload, indent=2, sort_keys=True) + '\n')
        node.destroy_node()
        rclpy.shutdown()

    print(json.dumps(payload, sort_keys=True))
    return 0 if payload['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())

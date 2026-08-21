#!/usr/bin/env python3
"""Replay a learned-policy prefix, then capture a scripted MuJoCo recovery.

The policy is never loaded.  A validated prefix from an authoritative
``actions.jsonl`` trace is replayed with its recorded timing.  Recording starts
only after the prefix, so the training episode contains expert corrective
commands rather than the failed learned-policy commands.  Privileged object
pose is used only by the scripted expert and upstream GT, never as policy input.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time
from typing import Any, Sequence

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64, Header, String

from isaac_sim_adapter.policy_trace_replay import load_executed_prefix
from isaac_sim_adapter.scripted_oracle import (
    compute_oracle_targets,
    gate_xy,
    interpolate_gripper,
    interpolate_pose,
    xy_distance,
)
from teleop_interfaces.msg import SafetyStatus, TaskEvaluationStatus
from teleop_interfaces.srv import EndEpisode


CONTRACT_VERSION = 'policy_visited_recovery_execution_v1'
TASK_TEXT = 'pick up the red box and place it in the left bin'


def _xyz(message: PoseStamped) -> tuple[float, float, float]:
    point = message.pose.position
    return (float(point.x), float(point.y), float(point.z))


def _quat(message: PoseStamped) -> tuple[float, float, float, float]:
    value = message.pose.orientation
    return (float(value.x), float(value.y), float(value.z), float(value.w))


class RecoveryCoordinator(Node):
    def __init__(self) -> None:
        super().__init__('mujoco_policy_visited_recovery')
        self.ee: PoseStamped | None = None
        self.obj: PoseStamped | None = None
        self.safety: SafetyStatus | None = None
        self.gt: TaskEvaluationStatus | None = None
        self.create_subscription(
            PoseStamped, '/ee_pose', self._on_ee, qos_profile_sensor_data
        )
        self.create_subscription(
            PoseStamped, '/sim/object_pose', self._on_object, qos_profile_sensor_data
        )
        self.create_subscription(SafetyStatus, '/safety/status', self._on_safety, 10)
        self.create_subscription(
            TaskEvaluationStatus, '/task/evaluation_status', self._on_gt, 10
        )
        self.pose_pub = self.create_publisher(PoseStamped, '/teleop/cmd_pose', 10)
        self.gripper_pub = self.create_publisher(Float64, '/teleop/gripper_cmd', 10)
        self.heartbeat_pub = self.create_publisher(Header, '/teleop/heartbeat', 10)
        self.record_pub = self.create_publisher(String, '/teleop/record_trigger', 10)
        self.end_episode = self.create_client(
            EndEpisode, '/lerobot_recorder/end_episode'
        )
        self.last_target: tuple[float, ...] | None = None

    def _on_ee(self, message: PoseStamped) -> None:
        self.ee = message

    def _on_object(self, message: PoseStamped) -> None:
        self.obj = message

    def _on_safety(self, message: SafetyStatus) -> None:
        self.safety = message

    def _on_gt(self, message: TaskEvaluationStatus) -> None:
        if message.gt_source == 'upstream_continuous_task_evaluator':
            self.gt = message

    def spin(self, timeout_s: float = 0.0) -> None:
        rclpy.spin_once(self, timeout_sec=max(0.0, float(timeout_s)))

    def heartbeat(self) -> None:
        message = Header()
        message.stamp = self.get_clock().now().to_msg()
        message.frame_id = 'mujoco_policy_visited_recovery'
        self.heartbeat_pub.publish(message)

    def publish_action(self, values: Sequence[float]) -> None:
        if len(values) != 8 or not all(math.isfinite(float(v)) for v in values):
            raise ValueError('recovery command must be finite absolute EEF8')
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'panda_link0'
        message.pose.position.x = float(values[0])
        message.pose.position.y = float(values[1])
        message.pose.position.z = float(values[2])
        message.pose.orientation.x = float(values[3])
        message.pose.orientation.y = float(values[4])
        message.pose.orientation.z = float(values[5])
        message.pose.orientation.w = float(values[6])
        self.pose_pub.publish(message)
        self.gripper_pub.publish(Float64(data=float(values[7])))
        self.heartbeat()
        self.last_target = tuple(float(v) for v in values)

    def wait_ready(self, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        single_authority_count = 0
        while time.monotonic() < deadline:
            self.heartbeat()
            self.spin(0.05)
            authority_ok = (
                self.count_publishers('/teleop/cmd_pose') == 1
                and self.count_publishers('/teleop/gripper_cmd') == 1
            )
            single_authority_count = single_authority_count + 1 if authority_ok else 0
            if (
                self.ee is not None
                and self.obj is not None
                and self.safety is not None
                and bool(self.safety.ok)
                and not bool(self.safety.estop_active)
                and single_authority_count >= 3
                and self.end_episode.wait_for_service(timeout_sec=0.0)
            ):
                return
        pose_count = self.count_publishers('/teleop/cmd_pose')
        gripper_count = self.count_publishers('/teleop/gripper_cmd')
        raise TimeoutError(
            'runtime not ready or command authority not exclusive: '
            f'pose_publishers={pose_count} gripper_publishers={gripper_count}'
        )

    def replay_prefix(self, actions: Sequence[Any], *, settle_s: float) -> None:
        if not actions:
            raise ValueError('empty replay prefix')
        started = time.monotonic()
        current = 0
        final_s = float(actions[-1].relative_s)
        period = 1.0 / 50.0
        while True:
            elapsed = time.monotonic() - started
            while current + 1 < len(actions) and float(actions[current + 1].relative_s) <= elapsed:
                current += 1
            self.publish_action(actions[current].bounded_action)
            self.spin(0.0)
            if elapsed >= final_s + max(0.0, settle_s):
                return
            time.sleep(period)

    def start_recording(self) -> None:
        # Repeated event is idempotent and avoids a one-shot discovery race.
        for _ in range(3):
            self.record_pub.publish(String(data='start'))
            self.heartbeat()
            self.spin(0.05)
        time.sleep(0.35)

    def finish_recording(self, *, commit: bool) -> dict[str, Any]:
        request = EndEpisode.Request()
        request.discard = not bool(commit)
        future = self.end_episode.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        response = future.result()
        if response is None:
            raise TimeoutError('recorder end_episode service timed out')
        return {
            'requested_commit': bool(commit),
            'service_success': bool(response.success),
            'message': str(response.message),
            'dataset_path': str(response.dataset_path),
            'frame_count': int(response.frame_count),
        }

    def move(
        self,
        target_xyz: tuple[float, float, float],
        orientation: tuple[float, float, float, float],
        *,
        gripper_start: float,
        gripper_target: float,
        duration_s: float,
        rate_hz: float,
    ) -> tuple[float, float, float]:
        if self.ee is None:
            raise RuntimeError('EE feedback unavailable')
        start = _xyz(self.ee)
        steps = max(1, int(duration_s * rate_hz))
        for index in range(1, steps + 1):
            alpha = index / steps
            xyz = interpolate_pose(start, target_xyz, alpha)
            gripper = interpolate_gripper(gripper_start, gripper_target, alpha)
            self.publish_action((*xyz, *orientation, gripper))
            self.spin(0.0)
            time.sleep(1.0 / rate_hz)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            self.spin(0.05)
            if self.ee is not None:
                return _xyz(self.ee)
        raise TimeoutError('EE feedback unavailable after move')

    def hold(
        self,
        target_xyz: tuple[float, float, float],
        orientation: tuple[float, float, float, float],
        *,
        gripper: float,
        duration_s: float,
        rate_hz: float,
    ) -> tuple[float, float, float]:
        for _ in range(max(1, int(duration_s * rate_hz))):
            self.publish_action((*target_xyz, *orientation, gripper))
            self.spin(0.0)
            time.sleep(1.0 / rate_hz)
        if self.ee is None:
            raise RuntimeError('EE feedback unavailable after hold')
        return _xyz(self.ee)

    def wait_gt_lift(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.last_target is not None:
                self.publish_action(self.last_target)
            self.spin(0.05)
            if (
                self.gt is not None
                and self.gt.validity == 'VALID'
                and int(self.gt.lift) == int(TaskEvaluationStatus.OUTCOME_TRUE)
            ):
                return True
        return False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--actions', type=Path, required=True)
    parser.add_argument('--prefix-count', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--case-id', required=True)
    parser.add_argument('--ready-timeout-s', type=float, default=30.0)
    parser.add_argument('--replay-settle-s', type=float, default=0.5)
    parser.add_argument('--rate-hz', type=float, default=20.0)
    parser.add_argument('--approach-s', type=float, default=3.0)
    parser.add_argument('--hover-s', type=float, default=0.4)
    parser.add_argument('--descend-s', type=float, default=6.0)
    parser.add_argument('--close-s', type=float, default=0.8)
    parser.add_argument('--grasp-pause-s', type=float, default=0.5)
    parser.add_argument('--lift-s', type=float, default=8.0)
    parser.add_argument('--hold-s', type=float, default=1.0)
    parser.add_argument('--approach-xy-tol', type=float, default=0.02)
    parser.add_argument('--descend-xy-tol', type=float, default=0.02)
    parser.add_argument('--descend-z-tol', type=float, default=0.015)
    parser.add_argument('--gt-lift-timeout-s', type=float, default=5.0)
    return parser.parse_args()


def run(args: argparse.Namespace) -> dict[str, Any]:
    trace = load_executed_prefix(args.actions, args.prefix_count)
    report: dict[str, Any] = {
        'contract_version': CONTRACT_VERSION,
        'artifact_type': 'policy_visited_recovery_execution_report',
        'simulation_backend': 'mujoco',
        'case_id': args.case_id,
        'task': TASK_TEXT,
        'policy_input_excludes_object_pose': True,
        'claims_task_success': False,
        'expert_source': 'scripted_oracle_privileged_gt',
        'source_trace': {
            'path': trace.source_path,
            'sha256': trace.source_sha256,
            'prefix_count': len(trace.actions),
            'recorded_duration_s': trace.duration_s,
            'clipped_count': trace.clipped_count,
        },
        'status': 'RUNNING',
        'phases': [],
        'recorder': None,
        'errors': [],
    }
    node = RecoveryCoordinator()
    recording = False
    try:
        node.wait_ready(args.ready_timeout_s)
        node.replay_prefix(trace.actions, settle_s=args.replay_settle_s)
        if node.ee is None or node.obj is None:
            raise RuntimeError('replay completed without EE/object feedback')
        visited = _xyz(node.ee)
        obj = _xyz(node.obj)
        orientation = _quat(node.ee)
        targets = compute_oracle_targets(
            obj,
            visited,
            hover_z=0.12,
            pick_z_offset=0.010,
            lift_z=0.12,
            gripper_close_target=0.40,
        )
        report['policy_visited_state'] = {
            'ee_xyz': visited,
            'object_xyz_gt_only': obj,
            'ee_object_xy_m': xy_distance(visited, obj),
        }
        report['expert_targets_gt_only'] = targets.as_dict()
        node.start_recording()
        recording = True
        gripper = float(trace.actions[-1].bounded_action[7])
        phase_plan = (
            ('ALIGN', targets.approach_xy, 1.0, args.approach_s),
            ('HOVER', targets.hover, 1.0, args.hover_s),
            ('DESCEND', targets.pick, 1.0, args.descend_s),
            ('CLOSE', targets.pick, targets.gripper_close_target, args.close_s),
            ('GRASP_SETTLE', targets.pick, targets.gripper_close_target, args.grasp_pause_s),
            ('LIFT', targets.lift, targets.gripper_close_target, args.lift_s),
            ('HOLD', targets.lift, targets.gripper_close_target, args.hold_s),
        )
        for name, target, gripper_target, duration in phase_plan:
            if name in {'HOVER', 'GRASP_SETTLE', 'HOLD'}:
                final = node.hold(
                    target,
                    orientation,
                    gripper=gripper_target,
                    duration_s=duration,
                    rate_hz=args.rate_hz,
                )
            else:
                final = node.move(
                    target,
                    orientation,
                    gripper_start=gripper,
                    gripper_target=gripper_target,
                    duration_s=duration,
                    rate_hz=args.rate_hz,
                )
            gripper = gripper_target
            node.spin(0.05)
            obj_now = _xyz(node.obj) if node.obj is not None else obj
            xy = xy_distance(final, obj_now)
            phase = {
                'name': name,
                'target_xyz': target,
                'final_ee_xyz': final,
                'object_xyz_gt_only': obj_now,
                'ee_object_xy_m': xy,
                'gripper_cmd': gripper_target,
            }
            report['phases'].append(phase)
            if name == 'ALIGN':
                ok, _ = gate_xy(final, target, tolerance_m=args.approach_xy_tol)
                if not ok:
                    raise RuntimeError(f'ALIGN gate failed: ee_object_xy_m={xy:.4f}')
            if name == 'DESCEND':
                ok, _ = gate_xy(final, target, tolerance_m=args.descend_xy_tol)
                z_error = abs(final[2] - target[2])
                if not ok or z_error > args.descend_z_tol:
                    raise RuntimeError(
                        f'DESCEND gate failed: xy={xy:.4f} z_error={z_error:.4f}'
                    )

        gt_lift = node.wait_gt_lift(args.gt_lift_timeout_s)
        report['gt_snapshot'] = {
            'validity': None if node.gt is None else str(node.gt.validity),
            'phase': None if node.gt is None else str(node.gt.phase),
            'reach': None if node.gt is None else int(node.gt.reach),
            'grasp': None if node.gt is None else int(node.gt.grasp),
            'lift': None if node.gt is None else int(node.gt.lift),
            'gt_source': None if node.gt is None else str(node.gt.gt_source),
        }
        report['recorder'] = node.finish_recording(commit=gt_lift)
        recording = False
        if not gt_lift:
            raise RuntimeError('upstream continuous GT did not observe Lift')
        if not report['recorder']['service_success']:
            raise RuntimeError(
                f'recorder commit failed: {report["recorder"]["message"]}'
            )
        report['status'] = 'CAPTURE_ACCEPTED_PENDING_MIDSTREAM_QA'
        return report
    except Exception as exc:  # noqa: BLE001 - report exact bounded-run failure
        report['errors'].append(str(exc))
        report['status'] = 'CAPTURE_REJECTED'
        if recording:
            try:
                report['recorder'] = node.finish_recording(commit=False)
            except Exception as discard_exc:  # noqa: BLE001
                report['errors'].append(f'recorder discard failed: {discard_exc}')
        return report
    finally:
        node.destroy_node()


def main() -> int:
    args = parse_args()
    rclpy.init()
    try:
        report = run(args)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps({'status': report['status'], 'output': str(args.output)}))
    return 0 if report['status'] == 'CAPTURE_ACCEPTED_PENDING_MIDSTREAM_QA' else 2


if __name__ == '__main__':
    raise SystemExit(main())

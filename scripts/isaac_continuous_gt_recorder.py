#!/usr/bin/env python3
"""Stream ContinuousTaskEvaluator during Isaac ACT rollouts → episode_results.jsonl.

v1 fixes vs invalid_evaluator_v0:
- gripper_command (/teleop/gripper_cmd) and gripper_state (/gripper/state) are separate
- closed detection uses gripper_state (fallback to command only if state never arrived)
- MultiThreadedExecutor + drain spin to avoid gripper callback starvation
- /ft_sensor → peak_force_n
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import signal
import sys
import time

from geometry_msgs.msg import PoseStamped, WrenchStamped
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64
from synth_data_gen.continuous_evaluator import (
    append_episode_result,
    ContinuousTaskEvaluator,
    EvaluatorSample,
)
from synth_data_gen.task_gt_live import (
    build_task_gt_snapshot,
    populate_task_evaluation_status,
    TaskGtSnapshot,
)
from teleop_interfaces.msg import TaskEvaluationStatus


class IsaacContinuousGtNode(Node):
    def __init__(
        self,
        *,
        evaluator: ContinuousTaskEvaluator,
        bin_xy: tuple[float, float],
        trace_run_id: str,
        episode_id: str,
        parent_event_id: str = '',
    ) -> None:
        super().__init__('isaac_continuous_gt')
        self._ev = evaluator
        self._bin_xy = bin_xy
        self._trace_run_id = trace_run_id
        self._episode_id = episode_id
        self._parent_event_id = parent_event_id
        self._task_event_sequence = 0
        self._object: tuple[float, float, float] | None = None
        self._ee: tuple[float, float, float] | None = None
        self._gripper_command: float | None = None
        self._gripper_state: float | None = None
        self._contact_force_n: float | None = None
        self._min_gripper_command: float | None = None
        self._min_gripper_state: float | None = None
        self._gripper_cmd_count = 0
        self._gripper_state_count = 0
        self._ft_count = 0
        self._initialized = False
        self._group = ReentrantCallbackGroup()
        self._task_gt_pub = self.create_publisher(
            TaskEvaluationStatus,
            '/task/evaluation_status',
            10,
        )
        self.create_subscription(
            PoseStamped,
            '/sim/object_pose',
            self._on_object,
            qos_profile_sensor_data,
            callback_group=self._group,
        )
        self.create_subscription(
            PoseStamped,
            '/ee_pose',
            self._on_ee,
            qos_profile_sensor_data,
            callback_group=self._group,
        )
        self.create_subscription(
            Float64,
            '/teleop/gripper_cmd',
            self._on_gripper_command,
            10,
            callback_group=self._group,
        )
        self.create_subscription(
            Float64,
            '/gripper/state',
            self._on_gripper_state,
            qos_profile_sensor_data,
            callback_group=self._group,
        )
        self.create_subscription(
            WrenchStamped,
            '/ft_sensor',
            self._on_ft,
            qos_profile_sensor_data,
            callback_group=self._group,
        )
        self.create_timer(0.05, self._tick, callback_group=self._group)

    def _on_object(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        if all(math.isfinite(float(v)) for v in (p.x, p.y, p.z)):
            self._object = (float(p.x), float(p.y), float(p.z))

    def _on_ee(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        if all(math.isfinite(float(v)) for v in (p.x, p.y, p.z)):
            self._ee = (float(p.x), float(p.y), float(p.z))

    def _on_gripper_command(self, msg: Float64) -> None:
        if not math.isfinite(float(msg.data)):
            return
        value = max(0.0, min(1.0, float(msg.data)))
        self._gripper_command = value
        self._gripper_cmd_count += 1
        self._min_gripper_command = (
            value
            if self._min_gripper_command is None
            else min(self._min_gripper_command, value)
        )

    def _on_gripper_state(self, msg: Float64) -> None:
        if not math.isfinite(float(msg.data)):
            return
        value = max(0.0, min(1.0, float(msg.data)))
        self._gripper_state = value
        self._gripper_state_count += 1
        self._min_gripper_state = (
            value
            if self._min_gripper_state is None
            else min(self._min_gripper_state, value)
        )

    def _on_ft(self, msg: WrenchStamped) -> None:
        force = msg.wrench.force
        values = (float(force.x), float(force.y), float(force.z))
        if not all(math.isfinite(v) for v in values):
            return
        magnitude = math.sqrt(sum(v * v for v in values))
        self._contact_force_n = magnitude
        self._ft_count += 1

    def _gripper_for_evaluator(self) -> float:
        # Prefer measured state; fall back to command only if state never arrived.
        if self._gripper_state is not None:
            return float(self._gripper_state)
        if self._gripper_command is not None:
            return float(self._gripper_command)
        return 1.0

    def _tick(self) -> None:
        if self._object is None:
            self._publish_task_gt(
                build_task_gt_snapshot(
                    self._ev,
                    initialized=False,
                    current_object_xyz=None,
                )
            )
            return
        if not self._initialized:
            self._ev.reset(
                initial_object_xyz=self._object,
                bin_xy=self._bin_xy,
                reset_monotonic_s=time.monotonic(),
            )
            self._initialized = True
        self._ev.observe(
            EvaluatorSample(
                t_monotonic=time.monotonic(),
                object_xyz=self._object,
                ee_xyz=self._ee,
                gripper=self._gripper_for_evaluator(),
                contact_force_n=self._contact_force_n,
            )
        )
        self._publish_task_gt(
            build_task_gt_snapshot(
                self._ev,
                initialized=True,
                current_object_xyz=self._object,
            )
        )

    def _publish_task_gt(self, snapshot: TaskGtSnapshot) -> None:
        now = self.get_clock().now().to_msg()
        msg = populate_task_evaluation_status(
            TaskEvaluationStatus(),
            snapshot,
            stamp=now,
            trace_run_id=self._trace_run_id,
            episode_id=self._episode_id,
            event_sequence=self._task_event_sequence,
            parent_event_id=self._parent_event_id,
        )
        self._task_gt_pub.publish(msg)
        self._task_event_sequence += 1

    def publish_final(self, success: bool) -> None:
        self._publish_task_gt(
            build_task_gt_snapshot(
                self._ev,
                initialized=self._initialized,
                current_object_xyz=self._object,
                final_success=bool(success) if self._initialized else None,
            )
        )


def _drain_executor(executor: MultiThreadedExecutor, rounds: int = 32) -> None:
    for _ in range(max(1, rounds)):
        executor.spin_once(timeout_sec=0.0)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episode-results-path', type=Path, required=True)
    parser.add_argument('--evaluation-run-id', required=True)
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--episode-index', type=int, required=True)
    parser.add_argument('--model-id', required=True)
    parser.add_argument('--scene-id', default='panda_pick_place_v1')
    parser.add_argument('--suite-id', default='nominal')
    parser.add_argument('--backend', default='isaac')
    parser.add_argument('--bin-xy', default='0.40,-0.35')
    parser.add_argument('--validation-mode', default='place')
    parser.add_argument('--lift-success-delta', type=float, default=0.03)
    parser.add_argument(
        '--gripper-close-max',
        type=float,
        default=0.12,
        help=(
            'Normalized gripper <= this counts as closed. Default 0.12 matches '
            'thin-object / empty-close gates; cube side-grasp needs ~0.70.'
        ),
    )
    parser.add_argument('--raw-episode-path', type=str, default='')
    parser.add_argument('--video-path', type=str, default='')
    parser.add_argument('--runtime-log-path', type=str, required=True)
    parser.add_argument('--event-log-path', type=str, required=True)
    parser.add_argument('--nfr-sample-path', type=str, required=True)
    parser.add_argument('--wait-for-report', type=Path, default=None)
    parser.add_argument(
        '--exit-on-report',
        action='store_true',
        help=(
            'Exit soon after report.json appears. Disabled by default because '
            'policy_inference may flush a mid-episode FAIL report before gripper closes.'
        ),
    )
    parser.add_argument('--max-duration-s', type=float, default=320.0)
    parser.add_argument('--evaluator-version-tag', default='panda_continuous_gt_v1')
    parser.add_argument('--parent-event-id', default='')
    args = parser.parse_args()

    bin_parts = [float(p) for p in args.bin_xy.split(',')]
    bin_xy = (bin_parts[0], bin_parts[1])
    evaluator = ContinuousTaskEvaluator(
        lift_success_delta=args.lift_success_delta,
        gripper_close_max=args.gripper_close_max,
        validation_mode=args.validation_mode,
    )

    stop = {'flag': False}

    def _stop(_s=None, _f=None) -> None:
        stop['flag'] = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    for path in (args.runtime_log_path, args.event_log_path, args.nfr_sample_path):
        Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(args.event_log_path).write_text('', encoding='utf-8')
    Path(args.nfr_sample_path).write_text('{}\n', encoding='utf-8')

    rclpy.init()
    episode_id = f'episode_{args.episode_index:04d}_seed_{args.seed}'
    node = IsaacContinuousGtNode(
        evaluator=evaluator,
        bin_xy=bin_xy,
        trace_run_id=args.evaluation_run_id,
        episode_id=episode_id,
        parent_event_id=args.parent_event_id,
    )
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    deadline = time.monotonic() + max(1.0, float(args.max_duration_s))
    report_seen = False
    try:
        while rclpy.ok() and not stop['flag']:
            if time.monotonic() >= deadline:
                break
            if (
                args.wait_for_report is not None
                and args.wait_for_report.is_file()
                and not report_seen
            ):
                report_seen = True
                node.get_logger().info(
                    f'report observed at {args.wait_for_report}; '
                    'continuing until SIGTERM unless --exit-on-report'
                )
                if args.exit_on_report:
                    settle_until = time.monotonic() + 2.0
                    while time.monotonic() < settle_until and rclpy.ok():
                        _drain_executor(executor, rounds=64)
                        time.sleep(0.01)
                    break
            _drain_executor(executor, rounds=32)
            time.sleep(0.01)
    finally:
        execution_status = 'completed'
        if not node._initialized:
            execution_status = 'infrastructure_failure'
            evaluator._abort_reason = 'no object pose observed'
        video = args.video_path.strip() or None
        if args.raw_episode_path.strip():
            raw = args.raw_episode_path.strip()
        elif args.wait_for_report is not None:
            raw = str(args.wait_for_report.parent)
        else:
            raw = str(Path(args.runtime_log_path).parent)
        Path(args.runtime_log_path).write_text(
            (
                f'seed={args.seed}\n'
                f'initialized={node._initialized}\n'
                f'report_seen={report_seen}\n'
                f'evaluator_tag={args.evaluator_version_tag}\n'
                f'gripper_cmd_count={node._gripper_cmd_count}\n'
                f'gripper_state_count={node._gripper_state_count}\n'
                f'ft_count={node._ft_count}\n'
                f'min_gripper_command={node._min_gripper_command}\n'
                f'min_gripper_state={node._min_gripper_state}\n'
                f'peak_force_n={evaluator._peak_force_n}\n'
            ),
            encoding='utf-8',
        )
        row = evaluator.finalize(
            evaluation_run_id=args.evaluation_run_id,
            identity={
                'model_id': args.model_id,
                'backend': args.backend,
                'scene_id': args.scene_id,
                'suite_id': args.suite_id,
                'seed': int(args.seed),
                'episode_index': int(args.episode_index),
            },
            evidence={
                'raw_episode_path': raw or str(Path(args.runtime_log_path).parent),
                'video_path': video,
                'runtime_log_path': str(args.runtime_log_path),
                'event_log_path': str(args.event_log_path),
                'nfr_sample_path': str(args.nfr_sample_path),
            },
            execution_status=execution_status,
        )
        if node._initialized:
            node.publish_final(bool(row['outcome'].get('success')))
            _drain_executor(executor, rounds=64)
        # Diagnostic sidecar fields (non-schema; stored in runtime log + print).
        print(
            'ISAAC_GT_DIAG '
            f'seed={args.seed} '
            f'min_gripper_command={node._min_gripper_command} '
            f'min_gripper_state={node._min_gripper_state} '
            f'gripper_state_count={node._gripper_state_count} '
            f'ft_count={node._ft_count} '
            f'peak_force_n={row["contact_safety"].get("peak_force_n")}',
            flush=True,
        )
        append_episode_result(str(args.episode_results_path), row)
        print(
            f'ISAAC_EPISODE_RESULT seed={args.seed} '
            f"success={row['outcome'].get('success')} "
            f'status={execution_status} '
            f"reason={row['outcome'].get('failure_reason')}",
            flush=True,
        )
        try:
            executor.remove_node(node)
        except Exception:  # noqa: BLE001
            pass
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        try:
            executor.shutdown()
        except Exception:  # noqa: BLE001
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())

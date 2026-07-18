"""Run a bounded five-repeat Isaac effort sequence and save E1 evidence."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from isaac_sim_adapter.effort_control import PANDA_ARM_JOINTS
from isaac_sim_adapter.effort_control import ZERO_EFFORT


DEFAULT_SEQUENCE = (
    (0.20, ZERO_EFFORT),
    (0.25, (1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
    (0.25, (-1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
    (0.20, ZERO_EFFORT),
)


def percentile(values: list[float], fraction: float) -> float | None:
    """Return a deterministic nearest-rank percentile for a small sample."""
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1))
    return float(ordered[index])


def resample_trajectory(
    samples: list[list[float]], points: int = 50
) -> list[list[float]]:
    """Nearest-index resampling used only for repeatability comparison."""
    if not samples:
        return []
    if len(samples) == 1:
        return [list(samples[0]) for _ in range(points)]
    return [
        list(samples[round(index * (len(samples) - 1) / (points - 1))])
        for index in range(points)
    ]


def trajectory_rmse(
    reference: list[list[float]], candidate: list[list[float]]
) -> float | None:
    """Compute joint-space RMSE between equally resampled trajectories."""
    left = resample_trajectory(reference)
    right = resample_trajectory(candidate)
    if not left or not right:
        return None
    squared = [
        (a - b) ** 2
        for left_row, right_row in zip(left, right)
        for a, b in zip(left_row, right_row)
    ]
    return math.sqrt(sum(squared) / len(squared))


def vector_l2(left: list[float], right: list[float]) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right)))


class E1ActionSequenceRunner(Node):
    """Small ROS client that exercises the canonical effort boundary."""

    def __init__(self, *, command_rate_hz: float) -> None:
        super().__init__('isaac_e1_action_sequence')
        self.command_rate_hz = float(command_rate_hz)
        self._latest_state: list[float] | None = None
        self._latest_state_at = 0.0
        self._samples: list[tuple[float, list[float]]] = []
        self._command_pub = self.create_publisher(
            Float64MultiArray,
            '/sim/joint_effort_cmd',
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            '/sim/encoder_state',
            self._on_state,
            qos_profile_sensor_data,
        )
        self._reset_client = self.create_client(Trigger, '/sim/reset_scene')

    def _on_state(self, message: JointState) -> None:
        lookup = {name: index for index, name in enumerate(message.name)}
        if any(name not in lookup for name in PANDA_ARM_JOINTS):
            return
        if len(message.position) < len(message.name):
            return
        now = time.monotonic()
        positions = [
            float(message.position[lookup[name]]) for name in PANDA_ARM_JOINTS
        ]
        self._latest_state = positions
        self._latest_state_at = now
        self._samples.append((now, positions))

    def wait_for_graph(self, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            reset_ready = self._reset_client.service_is_ready()
            command_ready = self._command_pub.get_subscription_count() > 0
            state_ready = self._latest_state is not None
            if reset_ready and command_ready and state_ready:
                return
        raise RuntimeError(
            'E1 graph preflight timed out: reset service, effort subscriber '
            'and fresh encoder state are required'
        )

    def reset(self, timeout_s: float) -> float:
        request_started = time.monotonic()
        future = self._reset_client.call_async(Trigger.Request())
        deadline = request_started + timeout_s
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
        if not future.done():
            raise RuntimeError('reset service call timed out')
        response = future.result()
        if response is None or not response.success:
            message = 'no response' if response is None else response.message
            raise RuntimeError(f'reset failed: {message}')
        reset_completed = time.monotonic()
        prior_state_at = self._latest_state_at
        state_deadline = reset_completed + timeout_s
        while time.monotonic() < state_deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            if self._latest_state_at > max(prior_state_at, reset_completed):
                return time.monotonic() - request_started
        raise RuntimeError('no post-reset encoder state arrived')

    def run_once(self, *, settle_s: float) -> dict[str, object]:
        self._samples = []
        publish_times: list[float] = []
        period = 1.0 / self.command_rate_hz
        started = time.monotonic()
        segment_ends: list[tuple[float, tuple[float, ...]]] = []
        accumulated = 0.0
        for duration_s, efforts in DEFAULT_SEQUENCE:
            accumulated += duration_s
            segment_ends.append((accumulated, efforts))

        next_publish = started
        while time.monotonic() - started < accumulated:
            now = time.monotonic()
            elapsed = now - started
            efforts = segment_ends[-1][1]
            for segment_end, candidate in segment_ends:
                if elapsed < segment_end:
                    efforts = candidate
                    break
            if now >= next_publish:
                self._command_pub.publish(
                    Float64MultiArray(data=list(efforts))
                )
                publish_times.append(now)
                next_publish += period
                if now - next_publish > period:
                    next_publish = now + period
            rclpy.spin_once(self, timeout_sec=min(0.002, period / 2.0))

        self._command_pub.publish(Float64MultiArray(data=list(ZERO_EFFORT)))
        settle_deadline = time.monotonic() + settle_s
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(self, timeout_sec=0.005)

        states = [positions for _, positions in self._samples]
        state_times = [timestamp for timestamp, _ in self._samples]
        if not states:
            raise RuntimeError('no encoder samples captured during action sequence')
        command_periods_ms = [
            (right - left) * 1000.0
            for left, right in zip(publish_times, publish_times[1:])
        ]
        state_periods = [
            right - left for left, right in zip(state_times, state_times[1:])
        ]
        expected_state_period = statistics.median(state_periods) if state_periods else 0.0
        gap_count = sum(
            period_s > expected_state_period * 2.0
            for period_s in state_periods
        ) if expected_state_period > 0.0 else 0
        return {
            'sample_count': len(states),
            'trajectory': states,
            'final_joint_positions': states[-1],
            'command_publish_period_ms': {
                'p50': percentile(command_periods_ms, 0.50),
                'p95': percentile(command_periods_ms, 0.95),
                'max': max(command_periods_ms) if command_periods_ms else None,
            },
            'state_frequency_hz': (
                1.0 / statistics.mean(state_periods) if state_periods else None
            ),
            'state_gap_count': gap_count,
        }


def parse_args(args=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repeats', type=int, default=5)
    parser.add_argument('--command-rate-hz', type=float, default=100.0)
    parser.add_argument('--reset-timeout-s', type=float, default=8.0)
    parser.add_argument('--graph-timeout-s', type=float, default=20.0)
    parser.add_argument('--settle-s', type=float, default=0.2)
    parser.add_argument(
        '--renderer-pressure-source',
        default='not_requested',
        help='Observed renderer load, for example scene_camera_30hz.',
    )
    parser.add_argument(
        '--output',
        type=Path,
        default=Path('/tmp/isaac_e1_action_sequence.json'),
    )
    return parser.parse_args(args)


def main(args=None) -> int:
    parsed = parse_args(args)
    if parsed.repeats != 5:
        raise ValueError('E1 acceptance requires exactly 5 repeats')
    if parsed.command_rate_hz <= 0.0:
        raise ValueError('command-rate-hz must be positive')

    rclpy.init()
    runner = E1ActionSequenceRunner(command_rate_hz=parsed.command_rate_hz)
    started = time.monotonic()
    try:
        runner.wait_for_graph(parsed.graph_timeout_s)
        results: list[dict[str, object]] = []
        for repeat_index in range(parsed.repeats):
            reset_recovery_s = runner.reset(parsed.reset_timeout_s)
            result = runner.run_once(settle_s=parsed.settle_s)
            result['repeat_index'] = repeat_index
            result['reset_recovery_ms'] = reset_recovery_s * 1000.0
            results.append(result)

        reference = results[0]['trajectory']
        reference_final = results[0]['final_joint_positions']
        for result in results:
            result['trajectory_rmse_vs_repeat_0_rad'] = trajectory_rmse(
                reference, result['trajectory']
            )
            result['final_l2_vs_repeat_0_rad'] = vector_l2(
                reference_final, result['final_joint_positions']
            )
            del result['trajectory']

        report = {
            'artifact_type': 'isaac_e1_action_sequence_evidence',
            'evidence_level': 'runtime_observed',
            'physical_task_success_evaluated': False,
            'action_boundary': '/sim/joint_effort_cmd',
            'action_semantics': 'Panda joint effort Nm, latest-value with watchdog',
            'sequence': [
                {'duration_s': duration, 'efforts_nm': list(efforts)}
                for duration, efforts in DEFAULT_SEQUENCE
            ],
            'repeat_count': parsed.repeats,
            'command_rate_hz': parsed.command_rate_hz,
            'renderer_pressure_source': parsed.renderer_pressure_source,
            'command_subscription_count': runner._command_pub.get_subscription_count(),
            'elapsed_s': time.monotonic() - started,
            'results': results,
            'limitations': [
                'Deterministic effort-sequence infrastructure test; not a learned policy.',
                'No grasp or physical task-success claim is produced by E1.',
            ],
        }
        parsed.output.parent.mkdir(parents=True, exist_ok=True)
        parsed.output.write_text(
            json.dumps(report, indent=2, sort_keys=True), encoding='utf-8'
        )
        print(json.dumps(report, sort_keys=True), flush=True)
        return 0
    finally:
        runner._command_pub.publish(Float64MultiArray(data=list(ZERO_EFFORT)))
        runner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())

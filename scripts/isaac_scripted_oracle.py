#!/usr/bin/env python3
"""Isaac scripted-oracle grasp: approach → hover → descend → close → lift.

Uses the same teleop command surface as pregrasp warmstart
(/teleop/cmd_pose, /teleop/gripper_cmd, /teleop/heartbeat). Physical lift
success is owned by ContinuousTaskEvaluator — this node only reports whether
the scripted phases completed.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64, Header

from isaac_sim_adapter.scripted_oracle import (
    DEFAULT_APPROACH_XY_TOL,
    DEFAULT_DESCEND_XY_TOL,
    DEFAULT_GRIPPER_CLOSE_TARGET,
    DEFAULT_HOVER_Z,
    DEFAULT_LIFT_Z,
    DEFAULT_PICK_Z_OFFSET,
    ORACLE_PHASES,
    OracleReport,
    build_phase_record,
    compute_oracle_targets,
    default_phase_durations_s,
    gate_xy,
    interpolate_gripper,
    interpolate_pose,
    phase_plan,
    xy_distance,
)


class ScriptedOracle(Node):
    def __init__(self) -> None:
        super().__init__('isaac_scripted_oracle')
        self._ee: PoseStamped | None = None
        self._obj: PoseStamped | None = None
        self.create_subscription(
            PoseStamped, '/ee_pose', self._on_ee, qos_profile_sensor_data
        )
        self.create_subscription(
            PoseStamped, '/sim/object_pose', self._on_obj, qos_profile_sensor_data
        )
        self._pose_pub = self.create_publisher(PoseStamped, '/teleop/cmd_pose', 10)
        self._hb_pub = self.create_publisher(Header, '/teleop/heartbeat', 10)
        self._grip_pub = self.create_publisher(Float64, '/teleop/gripper_cmd', 10)
        self._min_gripper_cmd: float | None = None

    def _on_ee(self, message: PoseStamped) -> None:
        self._ee = message

    def _on_obj(self, message: PoseStamped) -> None:
        self._obj = message

    def wait_poses(self, timeout_s: float) -> tuple[PoseStamped, PoseStamped]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._ee is not None and self._obj is not None:
                return self._ee, self._obj
        raise TimeoutError('timed out waiting for /ee_pose and /sim/object_pose')

    def _publish_heartbeat(self) -> None:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'isaac_scripted_oracle'
        self._hb_pub.publish(header)

    def _track_gripper(self, value: float) -> None:
        self._min_gripper_cmd = (
            value if self._min_gripper_cmd is None else min(self._min_gripper_cmd, value)
        )

    def _xyz_from(self, pose: PoseStamped) -> tuple[float, float, float]:
        p = pose.pose.position
        return (float(p.x), float(p.y), float(p.z))

    def _orientation_from(
        self, pose: PoseStamped
    ) -> tuple[float, float, float, float]:
        q = pose.pose.orientation
        return (float(q.x), float(q.y), float(q.z), float(q.w))

    def move_to(
        self,
        target_xyz: tuple[float, float, float],
        orientation_xyzw: tuple[float, float, float, float],
        *,
        gripper_start: float,
        gripper_target: float,
        duration_s: float,
        rate_hz: float,
    ) -> tuple[float, float, float]:
        ee, _ = self.wait_poses(5.0)
        start = self._xyz_from(ee)
        steps = max(1, int(duration_s * rate_hz))
        period = 1.0 / max(1e-3, rate_hz)
        for index in range(1, steps + 1):
            alpha = index / steps
            xyz = interpolate_pose(start, target_xyz, alpha)
            grip = interpolate_gripper(gripper_start, gripper_target, alpha)
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'panda_link0'
            pose.pose.position.x = xyz[0]
            pose.pose.position.y = xyz[1]
            pose.pose.position.z = xyz[2]
            (
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ) = orientation_xyzw
            self._pose_pub.publish(pose)
            self._grip_pub.publish(Float64(data=grip))
            self._track_gripper(grip)
            self._publish_heartbeat()
            time.sleep(period)
            rclpy.spin_once(self, timeout_sec=0.0)
        final_ee, _ = self.wait_poses(3.0)
        return self._xyz_from(final_ee)

    def hold_pose(
        self,
        target_xyz: tuple[float, float, float],
        orientation_xyzw: tuple[float, float, float, float],
        *,
        gripper: float,
        duration_s: float,
        rate_hz: float,
    ) -> tuple[float, float, float]:
        steps = max(1, int(duration_s * rate_hz))
        period = 1.0 / max(1e-3, rate_hz)
        for _ in range(steps):
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'panda_link0'
            pose.pose.position.x = target_xyz[0]
            pose.pose.position.y = target_xyz[1]
            pose.pose.position.z = target_xyz[2]
            (
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ) = orientation_xyzw
            self._pose_pub.publish(pose)
            self._grip_pub.publish(Float64(data=gripper))
            self._track_gripper(gripper)
            self._publish_heartbeat()
            time.sleep(period)
            rclpy.spin_once(self, timeout_sec=0.0)
        final_ee, _ = self.wait_poses(3.0)
        return self._xyz_from(final_ee)


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hover-z', type=float, default=DEFAULT_HOVER_Z)
    parser.add_argument('--pick-z-offset', type=float, default=DEFAULT_PICK_Z_OFFSET)
    parser.add_argument('--lift-z', type=float, default=DEFAULT_LIFT_Z)
    parser.add_argument(
        '--gripper-close-target', type=float, default=DEFAULT_GRIPPER_CLOSE_TARGET
    )
    parser.add_argument('--rate-hz', type=float, default=20.0)
    parser.add_argument('--timeout-s', type=float, default=20.0)
    parser.add_argument('--approach-xy-tol', type=float, default=DEFAULT_APPROACH_XY_TOL)
    parser.add_argument('--descend-xy-tol', type=float, default=DEFAULT_DESCEND_XY_TOL)
    parser.add_argument('--descend-z-tol', type=float, default=0.015)
    parser.add_argument('--approach-s', type=float, default=4.0)
    parser.add_argument('--hover-s', type=float, default=2.5)
    parser.add_argument('--descend-s', type=float, default=4.0)
    parser.add_argument('--close-s', type=float, default=3.0)
    parser.add_argument('--grasp-pause-s', type=float, default=2.0)
    parser.add_argument('--lift-s', type=float, default=3.5)
    parser.add_argument('--hold-s', type=float, default=2.0)
    parser.add_argument(
        '--output',
        type=Path,
        default=Path('oracle_report.json'),
        help='Path for oracle_report.json (phase completion only)',
    )
    return parser.parse_args(argv)


def run_oracle(args: argparse.Namespace) -> OracleReport:
    durations = default_phase_durations_s(
        approach_s=args.approach_s,
        hover_s=args.hover_s,
        descend_s=args.descend_s,
        close_s=args.close_s,
        grasp_pause_s=args.grasp_pause_s,
        lift_s=args.lift_s,
        hold_s=args.hold_s,
    )
    notes: list[str] = []
    phase_records: list[dict] = []
    completed: list[str] = []

    node = ScriptedOracle()
    try:
        ee, obj = node.wait_poses(args.timeout_s)
        orientation = node._orientation_from(ee)
        ee_xyz = node._xyz_from(ee)
        obj_xyz = node._xyz_from(obj)
        targets = compute_oracle_targets(
            obj_xyz,
            ee_xyz,
            hover_z=args.hover_z,
            pick_z_offset=args.pick_z_offset,
            lift_z=args.lift_z,
            gripper_close_target=args.gripper_close_target,
        )
        node.get_logger().info(
            f'Oracle targets object={targets.object_xyz} '
            f'approach={targets.approach_xy} hover={targets.hover} '
            f'pick={targets.pick} lift={targets.lift}'
        )

        plan = phase_plan(targets)
        assert [p[0] for p in plan] == list(ORACLE_PHASES)

        gripper_cmd = targets.gripper_open
        abort = False
        for phase_name, target_xyz, grip_target in plan:
            if abort:
                break
            duration = float(durations[phase_name])
            node.get_logger().info(
                f'Oracle phase={phase_name} target={target_xyz} '
                f'gripper→{grip_target:.3f} duration={duration:.1f}s'
            )
            if phase_name in ('hold', 'grasp_pause'):
                assert target_xyz is not None
                final = node.hold_pose(
                    target_xyz,
                    orientation,
                    gripper=grip_target,
                    duration_s=duration,
                    rate_hz=args.rate_hz,
                )
            elif phase_name == 'close':
                assert target_xyz is not None
                # Hold pick pose while closing fingers.
                node.hold_pose(
                    target_xyz,
                    orientation,
                    gripper=gripper_cmd,
                    duration_s=0.2,
                    rate_hz=args.rate_hz,
                )
                final = node.move_to(
                    target_xyz,
                    orientation,
                    gripper_start=gripper_cmd,
                    gripper_target=grip_target,
                    duration_s=duration,
                    rate_hz=args.rate_hz,
                )
            else:
                assert target_xyz is not None
                final = node.move_to(
                    target_xyz,
                    orientation,
                    gripper_start=gripper_cmd,
                    gripper_target=grip_target,
                    duration_s=duration,
                    rate_hz=args.rate_hz,
                )

            gripper_cmd = grip_target
            _, cur_obj = node.wait_poses(3.0)
            obj_now = node._xyz_from(cur_obj)
            ok = True
            detail = ''
            if phase_name == 'approach_xy':
                ok, dist = gate_xy(final, targets.approach_xy, tolerance_m=args.approach_xy_tol)
                detail = f'ee_target_xy={dist:.4f}'
                if not ok:
                    notes.append(f'approach_xy gate failed ({detail})')
                    abort = True
            elif phase_name == 'descend':
                ok, dist = gate_xy(final, targets.pick, tolerance_m=args.descend_xy_tol)
                z_err = abs(final[2] - targets.pick[2])
                detail = f'ee_pick_xy={dist:.4f} ee_pick_z_err={z_err:.4f}'
                if ok and z_err > float(args.descend_z_tol):
                    # Trim Z before close — high approach was kicking the cube.
                    node.get_logger().info(
                        f'descend Z trim: err={z_err:.4f} → {targets.pick}'
                    )
                    final = node.move_to(
                        targets.pick,
                        orientation,
                        gripper_start=gripper_cmd,
                        gripper_target=grip_target,
                        duration_s=max(1.5, duration * 0.4),
                        rate_hz=args.rate_hz,
                    )
                    z_err = abs(final[2] - targets.pick[2])
                    ok, dist = gate_xy(
                        final, targets.pick, tolerance_m=args.descend_xy_tol
                    )
                    detail = f'ee_pick_xy={dist:.4f} ee_pick_z_err={z_err:.4f}'
                if ok and z_err > float(args.descend_z_tol):
                    notes.append(f'descend Z gate failed ({detail})')
                    ok = False
                    abort = True
                elif not ok:
                    notes.append(f'descend XY gate failed ({detail})')
                    abort = True
            elif phase_name in ('close', 'grasp_pause'):
                # Abort if close ejected the object far from the pick XY.
                obj_xy = xy_distance(obj_now, targets.object_xyz)
                detail = f'object_drift_xy={obj_xy:.4f}'
                if obj_xy > 0.08:
                    notes.append(f'{phase_name} ejected object ({detail})')
                    ok = False
                    abort = True

            record = build_phase_record(
                phase_name,
                target_xyz=target_xyz,
                ee_xyz=final,
                object_xyz=obj_now,
                gripper_cmd=grip_target,
                ok=ok,
                detail=detail,
            )
            phase_records.append(
                {
                    'name': record.name,
                    'target_xyz': record.target_xyz,
                    'ee_xyz': record.ee_xyz,
                    'object_xyz': record.object_xyz,
                    'ee_object_xy_m': record.ee_object_xy_m,
                    'gripper_cmd': record.gripper_cmd,
                    'ok': record.ok,
                    'detail': record.detail,
                }
            )
            if ok:
                completed.append(phase_name)
            else:
                break

        final_ee, final_obj = node.wait_poses(3.0)
        all_done = completed == list(ORACLE_PHASES)
        status = 'PASS' if all_done else 'FAIL'
        return OracleReport(
            status=status,
            phases_completed=completed,
            phases=phase_records,
            targets=targets.as_dict(),
            initial_object_xyz=obj_xyz,
            final_ee_xyz=node._xyz_from(final_ee),
            final_object_xyz=node._xyz_from(final_obj),
            min_gripper_cmd=node._min_gripper_cmd,
            all_phases_completed=all_done,
            notes=notes,
        )
    finally:
        node.destroy_node()


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    rclpy.init()
    try:
        report = run_oracle(args)
    except Exception as exc:  # noqa: BLE001 — surface any bringup/timeout failure
        report = OracleReport(
            status='FAIL',
            notes=[f'oracle aborted: {exc}'],
        )
    finally:
        if rclpy.ok():
            rclpy.shutdown()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    payload = report.to_dict()
    args.output.write_text(json.dumps(payload, indent=2) + '\n', encoding='utf-8')
    # Also write report.json so suite GT recorder wait-for-report works unchanged.
    if args.output.name != 'report.json':
        sibling = args.output.with_name('report.json')
        sibling.write_text(json.dumps(payload, indent=2) + '\n', encoding='utf-8')

    print(json.dumps(payload, indent=2))
    if report.status != 'PASS':
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

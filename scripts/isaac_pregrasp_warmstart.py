#!/usr/bin/env python3
"""Warm-start the EE toward a hover pose above the current object.

Used by Isaac ACT diagnostics to enter the visual state where offline ACT
chunks already plan a close, without claiming task success from the warm-start
itself.
"""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64, Header


class PregraspWarmstart(Node):
    def __init__(self) -> None:
        super().__init__('isaac_pregrasp_warmstart')
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
        header.frame_id = 'isaac_pregrasp_warmstart'
        self._hb_pub.publish(header)

    def move_to(
        self,
        target_xyz: tuple[float, float, float],
        orientation_xyzw: tuple[float, float, float, float],
        *,
        duration_s: float,
        rate_hz: float,
    ) -> None:
        ee, _ = self.wait_poses(5.0)
        start = (
            ee.pose.position.x, ee.pose.position.y, ee.pose.position.z,
        )
        steps = max(1, int(duration_s * rate_hz))
        period = 1.0 / rate_hz
        for index in range(1, steps + 1):
            alpha = index / steps
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'panda_link0'
            pose.pose.position.x = start[0] + (target_xyz[0] - start[0]) * alpha
            pose.pose.position.y = start[1] + (target_xyz[1] - start[1]) * alpha
            pose.pose.position.z = start[2] + (target_xyz[2] - start[2]) * alpha
            (
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ) = orientation_xyzw
            self._pose_pub.publish(pose)
            self._grip_pub.publish(Float64(data=1.0))
            self._publish_heartbeat()
            time.sleep(period)
            rclpy.spin_once(self, timeout_sec=0.0)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hover-z', type=float, default=0.12)
    parser.add_argument('--pick-z-offset', type=float, default=0.04)
    parser.add_argument('--duration-s', type=float, default=6.0)
    parser.add_argument('--rate-hz', type=float, default=20.0)
    parser.add_argument('--timeout-s', type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init()
    node = PregraspWarmstart()
    try:
        ee, obj = node.wait_poses(args.timeout_s)
        orientation = (
            ee.pose.orientation.x,
            ee.pose.orientation.y,
            ee.pose.orientation.z,
            ee.pose.orientation.w,
        )
        obj_xyz = (obj.pose.position.x, obj.pose.position.y, obj.pose.position.z)
        hover = (obj_xyz[0], obj_xyz[1], args.hover_z)
        pregrasp = (
            obj_xyz[0],
            obj_xyz[1],
            max(0.03, obj_xyz[2] + args.pick_z_offset),
        )
        node.get_logger().info(
            f'Warmstart hover {hover} then pregrasp {pregrasp} '
            f'(object={obj_xyz})'
        )
        half = max(1.0, args.duration_s * 0.55)
        node.move_to(hover, orientation, duration_s=half, rate_hz=args.rate_hz)
        node.move_to(
            pregrasp, orientation,
            duration_s=max(1.0, args.duration_s - half),
            rate_hz=args.rate_hz,
        )
        final_ee, _ = node.wait_poses(3.0)
        dist = math.hypot(
            final_ee.pose.position.x - obj_xyz[0],
            final_ee.pose.position.y - obj_xyz[1],
        )
        node.get_logger().info(
            f'Warmstart done: ee_obj_xy={dist:.4f} m z={final_ee.pose.position.z:.4f}'
        )
        if dist > 0.08:
            raise SystemExit(f'warmstart missed object XY (ee_obj_xy={dist:.4f})')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

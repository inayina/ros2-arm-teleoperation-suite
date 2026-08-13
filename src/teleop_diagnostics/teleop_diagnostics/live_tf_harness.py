# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Live robot_state_publisher TF harness (observer-only, timed, fail-closed).

Spins an isolated ROS domain with RSP + /joint_states publisher, then performs
tf2 lookups for panda_link0 → panda_ee. Does not publish control commands.
"""

from __future__ import annotations

import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path
from typing import Optional, Sequence

import numpy as np
import yaml

from teleop_diagnostics import FRAME_BASE, FRAME_EE, PANDA_ARM_JOINTS
from teleop_diagnostics.tf_source import (
    RobotStatePublisherTfSource,
    TfLookupRequest,
    transform_msg_to_matrix,
)
from teleop_diagnostics.types import (
    EvidenceClass,
    GeometryDiagnosticsError,
    InputStatus,
    PoseSample,
)
from teleop_diagnostics.urdf_fk import expand_xacro_to_urdf


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


class LiveTfHarness:
    """Context manager: robot_state_publisher + joint_state source + TF buffer."""

    def __init__(
        self,
        *,
        xacro_path: Optional[Path] = None,
        domain_id: int = 91,
        max_age_sec: float = 0.5,
        lookup_timeout_sec: float = 2.0,
    ):
        self.xacro_path = xacro_path or (
            _repo_root() / "src/teleop_description/urdf/panda.urdf.xacro"
        )
        self.domain_id = int(domain_id)
        self.max_age_sec = float(max_age_sec)
        self.lookup_timeout_sec = float(lookup_timeout_sec)
        self._tmpdir: Optional[tempfile.TemporaryDirectory] = None
        self._rsp_proc: Optional[subprocess.Popen] = None
        self._node = None
        self._js_pub = None
        self._buffer = None
        self._listener = None
        self._rclpy = None
        self._prev_domain: Optional[str] = None
        self.available = False
        self.detail = ""
        self.tf_source: Optional[RobotStatePublisherTfSource] = None

    def __enter__(self) -> "LiveTfHarness":
        self._prev_domain = os.environ.get("ROS_DOMAIN_ID")
        os.environ["ROS_DOMAIN_ID"] = str(self.domain_id)
        # Keep ROS logs inside a writable workspace temp (sandbox-safe).
        self._tmpdir = tempfile.TemporaryDirectory(prefix="teleop_diag_rsp_")
        log_dir = Path(self._tmpdir.name) / "ros_log"
        log_dir.mkdir(parents=True, exist_ok=True)
        os.environ["ROS_LOG_DIR"] = str(log_dir)
        os.environ["RCUTILS_LOGGING_USE_STDOUT"] = "1"
        try:
            import rclpy
            from rclpy.duration import Duration
            from rclpy.node import Node
            from sensor_msgs.msg import JointState
            from tf2_ros import Buffer, TransformListener

            self._rclpy = rclpy
            urdf_xml = expand_xacro_to_urdf(self.xacro_path)
            params_path = Path(self._tmpdir.name) / "rsp_params.yaml"
            # YAML literal block for robot_description
            params_path.write_text(
                yaml.safe_dump(
                    {"robot_state_publisher": {"ros__parameters": {"robot_description": urdf_xml}}},
                    sort_keys=False,
                ),
                encoding="utf-8",
            )
            self._rsp_proc = subprocess.Popen(
                [
                    "ros2",
                    "run",
                    "robot_state_publisher",
                    "robot_state_publisher",
                    "--ros-args",
                    "--params-file",
                    str(params_path),
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
                env=os.environ.copy(),
            )
            if not rclpy.ok():
                rclpy.init()
            self._node = Node("teleop_diagnostics_live_tf")
            self._js_pub = self._node.create_publisher(JointState, "/joint_states", 10)
            self._buffer = Buffer(cache_time=Duration(seconds=10.0))
            self._listener = TransformListener(self._buffer, self._node)

            def _lookup(req: TfLookupRequest) -> PoseSample:
                return self._lookup_impl(req)

            self.tf_source = RobotStatePublisherTfSource(
                lookup_fn=_lookup,
                max_age_sec=self.max_age_sec,
                backend_provenance=(
                    f"tf2_buffer_lookup:robot_state_publisher;"
                    f"domain_id={self.domain_id}"
                ),
            )
            # Warm-up: RSP needs joint_states before the moving chain connects
            # panda_link0 → panda_ee (otherwise TF reports unconnected trees).
            time.sleep(0.5)
            warm = None
            for _ in range(40):
                warm = self.lookup_ee([0.0] * 7)
                if warm.input_status == InputStatus.AVAILABLE:
                    break
                time.sleep(0.05)
            if warm is None or warm.input_status != InputStatus.AVAILABLE:
                self.available = False
                self.detail = (warm.detail if warm else "") or "live TF warm-up failed"
            else:
                self.available = True
                self.detail = "live TF AVAILABLE"
            return self
        except Exception as exc:  # noqa: BLE001
            self.available = False
            self.detail = f"live TF harness init failed: {exc}"
            self.__exit__(None, None, None)
            # Re-enter as unavailable shell (no raise): caller checks available.
            self.tf_source = RobotStatePublisherTfSource()
            return self

    def _publish_joints(self, q: Sequence[float]) -> float:
        from sensor_msgs.msg import JointState

        assert self._node is not None and self._js_pub is not None
        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name = list(PANDA_ARM_JOINTS)
        msg.position = [float(v) for v in q]
        if len(msg.position) != 7:
            raise GeometryDiagnosticsError(
                f"invalid joint count for live TF: {len(msg.position)}"
            )
        if not all(np.isfinite(v) for v in msg.position):
            raise GeometryDiagnosticsError("NaN/Inf joint for live TF publish")
        self._js_pub.publish(msg)
        return float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)

    def _lookup_impl(self, req: TfLookupRequest) -> PoseSample:
        from rclpy.duration import Duration
        from rclpy.time import Time

        assert self._node is not None and self._buffer is not None and self._rclpy is not None
        if req.frame_from != FRAME_BASE or req.frame_to != FRAME_EE:
            # Still allow, but wrong names will surface as MISSING.
            pass
        deadline = time.monotonic() + self.lookup_timeout_sec
        last_err = "MISSING TF"
        while time.monotonic() < deadline:
            self._rclpy.spin_once(self._node, timeout_sec=0.02)
            try:
                # Use Time() = latest available in buffer.
                when = Time()
                tf = self._buffer.lookup_transform(
                    req.frame_from,
                    req.frame_to,
                    when,
                    timeout=Duration(seconds=0.05),
                )
                stamp = float(tf.header.stamp.sec) + 1e-9 * float(tf.header.stamp.nanosec)
                now = self._node.get_clock().now().nanoseconds * 1e-9
                age = now - stamp
                if age > req.max_age_sec:
                    raise GeometryDiagnosticsError(
                        f"STALE TF age={age:.3f}s > max_age={req.max_age_sec:.3f}s"
                    )
                T = transform_msg_to_matrix(tf.transform)
                return PoseSample(
                    source="robot_state_publisher_tf",
                    frame_from=req.frame_from,
                    frame_to=req.frame_to,
                    reference_point=req.frame_to,
                    evidence_class=EvidenceClass.MODEL,
                    backend_provenance=(
                        f"tf2_buffer_lookup:robot_state_publisher;"
                        f"domain_id={self.domain_id}"
                    ),
                    input_status=InputStatus.AVAILABLE,
                    matrix=T,
                    stamp_sec=stamp,
                    detail=f"age_sec={age:.4f}",
                )
            except GeometryDiagnosticsError:
                raise
            except Exception as exc:  # noqa: BLE001
                last_err = str(exc)
                time.sleep(0.02)
        raise GeometryDiagnosticsError(f"MISSING TF after timeout: {last_err}")

    def lookup_ee(self, q: Sequence[float]) -> PoseSample:
        if self.tf_source is None or self._node is None:
            return PoseSample(
                source="robot_state_publisher_tf",
                frame_from=FRAME_BASE,
                frame_to=FRAME_EE,
                reference_point=FRAME_EE,
                evidence_class=EvidenceClass.INSUFFICIENT_DATA,
                backend_provenance="tf_backend=unavailable",
                input_status=InputStatus.UNAVAILABLE,
                matrix=None,
                detail=self.detail or "harness not started",
            )
        # Publish repeatedly while looking up so RSP sees current q.
        stamp = self._publish_joints(q)
        for _ in range(10):
            self._publish_joints(q)
            if self._rclpy is not None:
                self._rclpy.spin_once(self._node, timeout_sec=0.02)
        sample = self.tf_source.forward(q, stamp_sec=stamp, max_age_sec=self.max_age_sec)
        return sample

    def __exit__(self, exc_type, exc, tb) -> None:
        try:
            if self._node is not None:
                self._node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        self._node = None
        self._js_pub = None
        self._buffer = None
        self._listener = None
        try:
            if self._rclpy is not None and self._rclpy.ok():
                # Do not global-shutdown if other nodes exist; best-effort.
                self._rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
        if self._rsp_proc is not None:
            try:
                os.killpg(self._rsp_proc.pid, signal.SIGTERM)
            except Exception:  # noqa: BLE001
                try:
                    self._rsp_proc.terminate()
                except Exception:  # noqa: BLE001
                    pass
            try:
                self._rsp_proc.wait(timeout=2.0)
            except Exception:  # noqa: BLE001
                try:
                    os.killpg(self._rsp_proc.pid, signal.SIGKILL)
                except Exception:  # noqa: BLE001
                    pass
            self._rsp_proc = None
        # Nuclear cleanup for this harness only.
        subprocess.run(
            ["pkill", "-9", "-f", "teleop_diagnostics_live_tf"],
            check=False,
            capture_output=True,
        )
        # Also reap any RSP started from our temp params directory.
        if self._tmpdir is not None:
            marker = Path(self._tmpdir.name).name
            subprocess.run(
                ["pkill", "-9", "-f", marker],
                check=False,
                capture_output=True,
            )
        if self._tmpdir is not None:
            try:
                self._tmpdir.cleanup()
            except Exception:  # noqa: BLE001
                pass
            self._tmpdir = None
        if self._prev_domain is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = self._prev_domain

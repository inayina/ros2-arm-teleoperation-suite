#!/usr/bin/env python3
"""Fuse grasp observations into passive state/advice JSON topics."""

from dataclasses import dataclass
import json
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64, String


IDLE = "IDLE"
APPROACHING = "APPROACHING"
READY_TO_CLOSE = "READY_TO_CLOSE"
CLOSING = "CLOSING"
CONTACT_DETECTED = "CONTACT_DETECTED"
LIFTING = "LIFTING"
GRASP_SUCCESS = "GRASP_SUCCESS"
GRASP_FAILED = "GRASP_FAILED"
SLIP_AFTER_LIFT = "SLIP_AFTER_LIFT"
RELEASED_BY_COMMAND = "RELEASED_BY_COMMAND"
ASSISTED_GRASP = "ASSISTED_GRASP"

SUCCESS = "SUCCESS"
MISS_OBJECT = "MISS_OBJECT"
WEAK_CONTACT = "WEAK_CONTACT"


@dataclass
class GraspMonitorParams:
    ready_distance_threshold: float = 0.08
    success_distance_threshold: float = 0.10
    slip_distance_threshold: float = 0.12
    object_drop_threshold: float = 0.03
    lift_start_threshold: float = 0.03
    success_lift_delta: float = 0.06
    success_hold_time: float = 1.0
    force_threshold: float = 2.0
    open_threshold: float = 0.6
    closed_threshold: float = 0.25


@dataclass
class GraspObservation:
    timestamp: float
    ee_z: Optional[float] = None
    object_z: Optional[float] = None
    ee_object_dist: Optional[float] = None
    gripper_opening: Optional[float] = None
    gripper_cmd: Optional[float] = None
    object_contacts: int = 0
    finger_object_contacts: int = 0
    force_magnitude: Optional[float] = None
    force_delta: Optional[float] = None
    grasp_assist_attached: bool = False


def _finite(value) -> bool:
    if value is None:
        return False
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _json_number(value):
    if not _finite(value):
        return None
    return float(value)


class GraspStateEstimator:
    """Small deterministic state machine for grasp phase/failure diagnosis."""

    def __init__(self, params: GraspMonitorParams):
        self.params = params
        self.state = IDLE
        self._reset_cycle()

    def _reset_cycle(self):
        self.had_contact = False
        self.had_finger_contact = False
        self.closing_seen = False
        self.closed_seen = False
        self.was_closed = False
        self.contact_ee_z = None
        self.contact_object_z = None
        self.max_object_z = None
        self.object_rose_seen = False
        self.success_candidate_since = None
        self.prev_gripper_opening = None

    def update(self, obs: GraspObservation) -> dict:
        p = self.params
        opening = obs.gripper_opening
        cmd = obs.gripper_cmd
        dist = obs.ee_object_dist
        contact_now = self._contact_detected(obs)
        opening_decreasing = (
            _finite(opening)
            and _finite(self.prev_gripper_opening)
            and float(opening) < float(self.prev_gripper_opening) - 0.01
        )
        closing_commanded = _finite(cmd) and float(cmd) <= p.closed_threshold
        closing = opening_decreasing or closing_commanded
        closed_by_opening = _finite(opening) and float(opening) <= p.closed_threshold
        closed = closed_by_opening or (closing_commanded and not _finite(opening))
        open_gripper = _finite(opening) and float(opening) >= p.open_threshold
        ready = (
            _finite(dist)
            and float(dist) < p.ready_distance_threshold
            and open_gripper
        )
        reopened_by_command = self.had_contact and (
            (_finite(cmd) and float(cmd) >= p.open_threshold)
            or (open_gripper and self.was_closed)
        )
        lost_finger_contact = (
            self.had_finger_contact and int(obs.finger_object_contacts) <= 0
        )

        if closing:
            self.closing_seen = True
        if closed:
            self.closed_seen = True
            self.was_closed = True

        if open_gripper and not contact_now and self.state in (
            GRASP_FAILED,
            SLIP_AFTER_LIFT,
            RELEASED_BY_COMMAND,
            GRASP_SUCCESS,
            ASSISTED_GRASP,
        ):
            self._reset_cycle()

        if contact_now and not self.had_contact:
            self.contact_ee_z = obs.ee_z if _finite(obs.ee_z) else None
            self.contact_object_z = obs.object_z if _finite(obs.object_z) else None
            self.success_candidate_since = None
        if contact_now:
            self.had_contact = True
            if int(obs.finger_object_contacts) > 0:
                self.had_finger_contact = True

        self._update_lift_tracking(obs)

        if reopened_by_command:
            state, classification, reason, advice = (
                RELEASED_BY_COMMAND,
                RELEASED_BY_COMMAND,
                "Gripper reopened after contact.",
                "Gripper was reopened by command. Check teleop/gripper command sequence.",
            )
        elif self._slip_after_lift(obs, lost_finger_contact, closed):
            state, classification, reason, advice = (
                SLIP_AFTER_LIFT,
                SLIP_AFTER_LIFT,
                "Object separated after initially rising with the gripper.",
                "Object slipped after lift. Check friction, grip force, lift speed, and contact parameters.",
            )
        elif self._assisted_success_condition(obs):
            state, classification, reason, advice = (
                ASSISTED_GRASP,
                ASSISTED_GRASP,
                "Synthetic grasp assist is attached during lift.",
                "Assist is active; rerun with grasp_assist_enabled:=false for physical grasp validation.",
            )
        elif self._success_condition(obs):
            if self.success_candidate_since is None:
                self.success_candidate_since = obs.timestamp
            if obs.timestamp - self.success_candidate_since >= p.success_hold_time:
                state, classification, reason, advice = (
                    GRASP_SUCCESS,
                    SUCCESS,
                    "Object followed end-effector during lift.",
                    "Grasp is stable.",
                )
            else:
                state, classification, reason, advice = (
                    LIFTING,
                    None,
                    "",
                    "Object is expected to follow end-effector during lift.",
                )
        else:
            self.success_candidate_since = None
            state, classification, reason, advice = self._nonterminal_state(
                obs, ready, closing, closed, contact_now)

        self.state = state
        if _finite(opening):
            self.prev_gripper_opening = float(opening)
        return self._status(obs, state, classification, reason, advice)

    def _contact_detected(self, obs: GraspObservation) -> bool:
        return (
            int(obs.finger_object_contacts) > 0
            or int(obs.object_contacts) > 0
            or (_finite(obs.force_magnitude)
                and abs(float(obs.force_magnitude)) > self.params.force_threshold)
            or (_finite(obs.force_delta)
                and abs(float(obs.force_delta)) > self.params.force_threshold)
        )

    def _update_lift_tracking(self, obs: GraspObservation):
        if not self.had_contact or not _finite(obs.object_z):
            return
        obj_z = float(obs.object_z)
        if self.max_object_z is None or obj_z > self.max_object_z:
            self.max_object_z = obj_z
        if (
            _finite(self.contact_object_z)
            and obj_z - float(self.contact_object_z) >= self.params.lift_start_threshold
        ):
            self.object_rose_seen = True

    def _ee_lifted(self, obs: GraspObservation) -> bool:
        return (
            self.had_contact
            and _finite(obs.ee_z)
            and _finite(self.contact_ee_z)
            and float(obs.ee_z) - float(self.contact_ee_z)
            >= self.params.lift_start_threshold
        )

    def _slip_after_lift(
        self, obs: GraspObservation, lost_finger_contact: bool, closed: bool
    ) -> bool:
        if not self.had_contact or not self.object_rose_seen:
            return False
        dropped = (
            _finite(obs.object_z)
            and _finite(self.max_object_z)
            and float(self.max_object_z) - float(obs.object_z)
            >= self.params.object_drop_threshold
        )
        separated = (
            _finite(obs.ee_object_dist)
            and float(obs.ee_object_dist) > self.params.slip_distance_threshold
        )
        contact_lost = lost_finger_contact and closed
        return dropped or separated or contact_lost

    def _success_condition(self, obs: GraspObservation) -> bool:
        return (
            self.had_contact
            and not obs.grasp_assist_attached
            and self._ee_lifted(obs)
            and _finite(obs.object_z)
            and _finite(self.contact_object_z)
            and float(obs.object_z) - float(self.contact_object_z)
            >= self.params.success_lift_delta
            and _finite(obs.ee_object_dist)
            and float(obs.ee_object_dist) < self.params.success_distance_threshold
        )

    def _assisted_success_condition(self, obs: GraspObservation) -> bool:
        return (
            bool(obs.grasp_assist_attached)
            and self.had_contact
            and self._ee_lifted(obs)
            and _finite(obs.object_z)
            and _finite(self.contact_object_z)
            and float(obs.object_z) - float(self.contact_object_z)
            >= self.params.success_lift_delta
            and _finite(obs.ee_object_dist)
            and float(obs.ee_object_dist) < self.params.success_distance_threshold
        )

    def _nonterminal_state(
        self,
        obs: GraspObservation,
        ready: bool,
        closing: bool,
        closed: bool,
        contact_now: bool,
    ):
        command_open = (
            _finite(obs.gripper_cmd)
            and float(obs.gripper_cmd) >= self.params.open_threshold
        )
        weak_contact = (
            self.closing_seen
            and closed
            and not command_open
            and contact_now
            and int(obs.finger_object_contacts) <= 0
            and not self.object_rose_seen
            and not obs.grasp_assist_attached
        )
        if (
            self.closing_seen
            and closed
            and not command_open
            and not self.had_contact
            and not contact_now
        ):
            return (
                GRASP_FAILED,
                MISS_OBJECT,
                "Gripper closed without object or finger-object contact.",
                "No contact detected. Adjust approach pose or capture radius.",
            )
        if weak_contact:
            return (
                GRASP_FAILED,
                WEAK_CONTACT,
                "Only weak/non-finger contact was detected before lift.",
                "Contact is weak. Check approach alignment, friction, and grip force.",
            )
        if self._ee_lifted(obs):
            return (
                LIFTING,
                None,
                "",
                "Object is expected to follow end-effector during lift.",
            )
        if contact_now or self.had_contact:
            return (
                CONTACT_DETECTED,
                None,
                "",
                "Contact detected between gripper and object.",
            )
        if closing:
            return CLOSING, None, "", "Gripper is closing."
        if ready:
            return (
                READY_TO_CLOSE,
                None,
                "",
                "End-effector is close to object. Ready to close gripper.",
            )
        if _finite(obs.ee_object_dist):
            return APPROACHING, None, "", "Move end-effector toward the object."
        return IDLE, None, "", "Waiting for grasp-relevant signals."

    def _status(
        self,
        obs: GraspObservation,
        state: str,
        classification: Optional[str],
        reason: str,
        advice: str,
    ) -> dict:
        failure_type = (
            classification
            if classification not in (None, SUCCESS, ASSISTED_GRASP)
            else None
        )
        return {
            "timestamp": _json_number(obs.timestamp),
            "state": state,
            "classification": classification,
            "failure_type": failure_type,
            "reason": reason,
            "advice": advice,
            "ee_object_dist": _json_number(obs.ee_object_dist),
            "gripper_opening": _json_number(obs.gripper_opening),
            "gripper_cmd": _json_number(obs.gripper_cmd),
            "object_contacts": int(obs.object_contacts),
            "finger_object_contacts": int(obs.finger_object_contacts),
            "force_magnitude": _json_number(obs.force_magnitude),
            "force_delta": _json_number(obs.force_delta),
            "grasp_assist_attached": bool(obs.grasp_assist_attached),
        }

    def debug_flags(self) -> dict:
        return {
            "had_contact": self.had_contact,
            "had_finger_contact": self.had_finger_contact,
            "closing_seen": self.closing_seen,
            "closed_seen": self.closed_seen,
            "object_rose_seen": self.object_rose_seen,
            "contact_ee_z": _json_number(self.contact_ee_z),
            "contact_object_z": _json_number(self.contact_object_z),
            "max_object_z": _json_number(self.max_object_z),
            "success_candidate_since": _json_number(self.success_candidate_since),
        }


class GraspMonitorNode(Node):
    def __init__(self):
        super().__init__("grasp_monitor")
        self._declare_monitor_parameters()
        self.params = self._load_params()
        self.estimator = GraspStateEstimator(self.params)
        self.debug_publish_rate = max(
            1.0, float(self.get_parameter("debug_publish_rate").value))

        self.latest_ee_xyz = None
        self.latest_object_xyz = None
        self.latest_gripper_opening = None
        self.latest_contact_debug = {}
        self.latest_contact_debug_time = None
        self.latest_force_magnitude = None
        self.latest_force_delta = None
        self.prev_force_magnitude = None
        self.contact_parse_errors = 0

        self.status_pub = self.create_publisher(String, "/grasp/status", 10)
        self.advice_pub = self.create_publisher(String, "/grasp/advice", 10)
        self.debug_pub = self.create_publisher(String, "/grasp/debug", 10)

        self.create_subscription(
            PoseStamped, "/ee_pose", self._on_ee_pose, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, "/sim/object_pose", self._on_object_pose,
            qos_profile_sensor_data)
        self.create_subscription(
            WrenchStamped, "/ft_sensor", self._on_ft_sensor,
            qos_profile_sensor_data)
        self.create_subscription(Float64, "/gripper/state", self._on_gripper_state, 10)
        self.create_subscription(String, "/grasp/contact_debug", self._on_contact_debug, 10)

        self.create_timer(1.0 / self.debug_publish_rate, self._publish)
        self.get_logger().info("grasp_monitor up.")

    def _declare_monitor_parameters(self):
        self.declare_parameter("ready_distance_threshold", 0.08)
        self.declare_parameter("success_distance_threshold", 0.10)
        self.declare_parameter("slip_distance_threshold", 0.12)
        self.declare_parameter("object_drop_threshold", 0.03)
        self.declare_parameter("lift_start_threshold", 0.03)
        self.declare_parameter("success_lift_delta", 0.06)
        self.declare_parameter("success_hold_time", 1.0)
        self.declare_parameter("force_threshold", 2.0)
        self.declare_parameter("open_threshold", 0.6)
        self.declare_parameter("closed_threshold", 0.25)
        self.declare_parameter("debug_publish_rate", 10.0)

    def _load_params(self) -> GraspMonitorParams:
        return GraspMonitorParams(
            ready_distance_threshold=self._float_param("ready_distance_threshold"),
            success_distance_threshold=self._float_param("success_distance_threshold"),
            slip_distance_threshold=self._float_param("slip_distance_threshold"),
            object_drop_threshold=self._float_param("object_drop_threshold"),
            lift_start_threshold=self._float_param("lift_start_threshold"),
            success_lift_delta=self._float_param("success_lift_delta"),
            success_hold_time=self._float_param("success_hold_time"),
            force_threshold=self._float_param("force_threshold"),
            open_threshold=self._float_param("open_threshold"),
            closed_threshold=self._float_param("closed_threshold"),
        )

    def _float_param(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _on_ee_pose(self, msg: PoseStamped):
        self.latest_ee_xyz = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        )

    def _on_object_pose(self, msg: PoseStamped):
        self.latest_object_xyz = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        )

    def _on_ft_sensor(self, msg: WrenchStamped):
        fx = float(msg.wrench.force.x)
        fy = float(msg.wrench.force.y)
        fz = float(msg.wrench.force.z)
        magnitude = math.sqrt(fx * fx + fy * fy + fz * fz)
        if self.prev_force_magnitude is None:
            self.latest_force_delta = 0.0
        else:
            self.latest_force_delta = abs(magnitude - self.prev_force_magnitude)
        self.prev_force_magnitude = magnitude
        self.latest_force_magnitude = magnitude

    def _on_gripper_state(self, msg: Float64):
        self.latest_gripper_opening = self._as_float(msg.data)

    def _on_contact_debug(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.contact_parse_errors += 1
            if self.contact_parse_errors <= 3:
                self.get_logger().warn("Failed to parse /grasp/contact_debug JSON.")
            return
        if not isinstance(payload, dict):
            return
        self.latest_contact_debug = payload
        self.latest_contact_debug_time = self._now_seconds()

    def _publish(self):
        now = self._now_seconds()
        obs = self._make_observation(now)
        status = self.estimator.update(obs)
        advice = {
            "timestamp": status["timestamp"],
            "state": status["state"],
            "classification": status["classification"],
            "failure_type": status["failure_type"],
            "reason": status["reason"],
            "advice": status["advice"],
        }
        debug = {
            "timestamp": status["timestamp"],
            "state": status["state"],
            "inputs": {
                "ee_pose_received": self.latest_ee_xyz is not None,
                "object_pose_received": self.latest_object_xyz is not None,
                "ft_sensor_received": self.latest_force_magnitude is not None,
                "gripper_state_received": self.latest_gripper_opening is not None,
                "contact_debug_received": self.latest_contact_debug_time is not None,
                "contact_debug_age_s": self._contact_debug_age(now),
            },
            "signals": {
                "ee_object_dist": status["ee_object_dist"],
                "gripper_opening": status["gripper_opening"],
                "gripper_cmd": status["gripper_cmd"],
                "object_contacts": status["object_contacts"],
                "finger_object_contacts": status["finger_object_contacts"],
                "force_magnitude": status["force_magnitude"],
                "force_delta": status["force_delta"],
                "grasp_assist_attached": status["grasp_assist_attached"],
            },
            "state_machine": self.estimator.debug_flags(),
        }
        self._publish_json(self.status_pub, status)
        self._publish_json(self.advice_pub, advice)
        self._publish_json(self.debug_pub, debug)

    def _make_observation(self, now: float) -> GraspObservation:
        contact = self._fresh_contact_debug(now)
        ee_z = self.latest_ee_xyz[2] if self.latest_ee_xyz is not None else None
        object_z = (
            self.latest_object_xyz[2] if self.latest_object_xyz is not None else None
        )
        dist = self._as_float(contact.get("ee_object_dist"))
        if dist is None and self.latest_ee_xyz is not None and self.latest_object_xyz is not None:
            dist = math.dist(self.latest_ee_xyz, self.latest_object_xyz)

        gripper_opening = self.latest_gripper_opening
        if gripper_opening is None:
            gripper_opening = self._as_float(contact.get("gripper_opening"))
        gripper_cmd = self._as_float(contact.get("gripper_cmd"))
        if gripper_cmd is None:
            gripper_cmd = gripper_opening

        return GraspObservation(
            timestamp=now,
            ee_z=ee_z,
            object_z=object_z,
            ee_object_dist=dist,
            gripper_opening=gripper_opening,
            gripper_cmd=gripper_cmd,
            object_contacts=self._as_int(contact.get("object_contacts"), default=0),
            finger_object_contacts=self._as_int(
                contact.get("finger_object_contacts"), default=0),
            force_magnitude=self.latest_force_magnitude,
            force_delta=self.latest_force_delta,
            grasp_assist_attached=bool(contact.get("grasp_assist_attached", False)),
        )

    def _fresh_contact_debug(self, now: float) -> dict:
        if self.latest_contact_debug_time is None:
            return {}
        if now - self.latest_contact_debug_time > 1.0:
            return {}
        return self.latest_contact_debug

    def _contact_debug_age(self, now: float):
        if self.latest_contact_debug_time is None:
            return None
        return _json_number(now - self.latest_contact_debug_time)

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_json(self, publisher, payload: dict):
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True, allow_nan=False)
        publisher.publish(msg)

    def _as_float(self, value):
        return _json_number(value)

    def _as_int(self, value, default=0) -> int:
        try:
            return int(value)
        except (TypeError, ValueError):
            return default


def main(args=None):
    rclpy.init(args=args)
    node = GraspMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

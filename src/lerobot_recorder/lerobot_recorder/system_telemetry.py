#!/usr/bin/env python3
"""Low-rate, out-of-band host and ROS process telemetry."""

from __future__ import annotations

import json

import psutil
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node


PROCESS_PATTERNS = {
    "mujoco_sim": ("mujoco_sim_node", "__node:=mujoco_sim"),
    "scene_camera": ("camera_bridge_node", "__node:=camera_bridge"),
    "recorder": ("lerobot_recorder_node", "__node:=lerobot_recorder"),
    "ros2_control": ("ros2_control_node",),
    "servo": ("servo_node",),
    "safety_monitor": ("safety_monitor_node", "__node:=safety_monitor"),
}


def match_processes(processes, patterns=PROCESS_PATTERNS):
    """Match by full command line so Python console scripts remain identifiable."""
    matched = {name: [] for name in patterns}
    for proc in processes:
        try:
            cmdline = " ".join(proc.cmdline())
        except (psutil.AccessDenied, psutil.NoSuchProcess, psutil.ZombieProcess):
            continue
        for name, candidates in patterns.items():
            if any(candidate in cmdline for candidate in candidates):
                matched[name].append(proc)
    return matched


class SystemTelemetryNode(Node):
    def __init__(self):
        super().__init__("system_telemetry")
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("enable_affinity", False)
        self.declare_parameter("affinity_rules_json", "{}")
        self.declare_parameter("cpu_pressure_threshold_percent", 85.0)
        self.declare_parameter("recorder_effective_hz_min", 8.0)
        self.declare_parameter("evidence_samples_required", 3)
        self._publisher = self.create_publisher(
            DiagnosticArray, "/system/telemetry", 10)
        self._recorder_effective_hz = 0.0
        self._recorder_recording = False
        self._pressure_streak = 0
        self._affinity_evidence_ready = False
        self.create_subscription(
            DiagnosticArray,
            "/recorder/diagnostics",
            self._on_recorder_diagnostics,
            10,
        )
        self._prime_process_cpu()
        rate = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._publish)

    def _prime_process_cpu(self):
        for proc in psutil.process_iter():
            try:
                proc.cpu_percent(None)
            except (psutil.AccessDenied, psutil.NoSuchProcess, psutil.ZombieProcess):
                pass

    @staticmethod
    def _kv(payload):
        return [
            KeyValue(
                key=str(key),
                value=(
                    json.dumps(value)
                    if isinstance(value, (bool, list, dict)) else str(value)
                ),
            )
            for key, value in payload.items()
        ]

    def _on_recorder_diagnostics(self, msg: DiagnosticArray):
        for status in msg.status:
            if status.name != "lerobot_recorder/health":
                continue
            values = {item.key: item.value for item in status.values}
            try:
                self._recorder_effective_hz = float(
                    values.get("effective_hz", 0.0))
                self._recorder_recording = (
                    values.get("recording", "False").lower() == "true")
            except ValueError:
                return

    def _apply_affinity(self, name, processes):
        if (
            not bool(self.get_parameter("enable_affinity").value)
            or not self._affinity_evidence_ready
        ):
            return
        try:
            rules = json.loads(
                str(self.get_parameter("affinity_rules_json").value))
            cpus = rules.get(name)
            if not isinstance(cpus, list) or not cpus:
                return
            for proc in processes:
                proc.cpu_affinity([int(cpu) for cpu in cpus])
        except (ValueError, TypeError, json.JSONDecodeError, psutil.Error) as exc:
            self.get_logger().warn(f"affinity rule failed for {name}: {exc}")

    def _publish(self):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        host = DiagnosticStatus()
        host.name = "system_telemetry/host"
        host.hardware_id = "linux_host"
        host.level = DiagnosticStatus.OK
        host.message = "ok"
        memory = psutil.virtual_memory()
        cpu_total = psutil.cpu_percent(None)
        cpu_per_core = psutil.cpu_percent(None, percpu=True)
        high_cpu = cpu_total >= float(
            self.get_parameter("cpu_pressure_threshold_percent").value)
        low_recorder_hz = (
            self._recorder_recording
            and self._recorder_effective_hz < float(
                self.get_parameter("recorder_effective_hz_min").value)
        )
        self._pressure_streak = (
            self._pressure_streak + 1 if high_cpu and low_recorder_hz else 0
        )
        self._affinity_evidence_ready = self._pressure_streak >= int(
            self.get_parameter("evidence_samples_required").value)
        host.values = self._kv({
            "cpu_total_percent": cpu_total,
            "cpu_per_core_percent": json.dumps(cpu_per_core),
            "memory_percent": memory.percent,
            "memory_used_bytes": memory.used,
            "memory_total_bytes": memory.total,
            "recorder_effective_hz": self._recorder_effective_hz,
            "cpu_capture_pressure_streak": self._pressure_streak,
            "affinity_evidence_ready": self._affinity_evidence_ready,
            "affinity_enabled": bool(
                self.get_parameter("enable_affinity").value),
        })
        statuses = [host]
        matches = match_processes(psutil.process_iter())
        for name, processes in matches.items():
            self._apply_affinity(name, processes)
            status = DiagnosticStatus()
            status.name = f"system_telemetry/process/{name}"
            status.hardware_id = "linux_process"
            status.level = DiagnosticStatus.OK
            status.message = "running" if processes else "disabled_or_not_running"
            cpu = 0.0
            rss = 0
            affinities = []
            pids = []
            for proc in processes:
                try:
                    pids.append(proc.pid)
                    cpu += proc.cpu_percent(None)
                    rss += proc.memory_info().rss
                    affinities.append(proc.cpu_affinity())
                except (psutil.AccessDenied, psutil.NoSuchProcess, psutil.ZombieProcess):
                    continue
            status.values = self._kv({
                "pids": json.dumps(pids),
                "cpu_percent": cpu,
                "rss_bytes": rss,
                "affinity": json.dumps(affinities),
            })
            statuses.append(status)
        msg.status = statuses
        self._publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SystemTelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

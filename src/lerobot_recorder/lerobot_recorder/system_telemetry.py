#!/usr/bin/env python3
"""Low-rate, out-of-band host and ROS process telemetry."""

from __future__ import annotations

from collections import defaultdict
import json
import subprocess
import threading
import time

import psutil
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node


PROCESS_PATTERNS = {
    "mujoco_sim": ("mujoco_sim_node", "__node:=mujoco_sim"),
    "isaac_sim_adapter": ("isaac_sim_adapter", "__node:=isaac_sim_adapter"),
    "scene_camera": ("camera_bridge_node", "__node:=camera_bridge"),
    "recorder": ("lerobot_recorder_node", "__node:=lerobot_recorder"),
    "ros2_control": ("ros2_control_node",),
    "servo": ("servo_node",),
    "safety_monitor": ("safety_monitor_node", "__node:=safety_monitor"),
    "policy_inference": ("smolvla_policy_inference_node",),
    "isaac_backend": ("isaac_panda_backend.py", "isaac_panda_backend"),
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


def _metric_value(token: str):
    """Return an integer nvidia-smi metric, or None for unsupported values."""
    token = token.strip()
    if token in {"", "-", "N/A", "[Not Supported]"}:
        return None
    try:
        return int(float(token))
    except ValueError:
        return None


def parse_nvidia_compute_apps(output: str) -> dict[int, dict]:
    """Parse per-process framebuffer memory from nvidia-smi CSV output."""
    metrics: dict[int, dict] = {}
    for line in output.splitlines():
        fields = [field.strip() for field in line.split(",")]
        if len(fields) < 3:
            continue
        try:
            pid = int(fields[0])
        except ValueError:
            continue
        memory_mib = _metric_value(fields[2])
        entry = metrics.setdefault(pid, {
            "gpu_memory_used_bytes": 0,
            "gpu_uuids": [],
        })
        if memory_mib is not None:
            entry["gpu_memory_used_bytes"] += memory_mib * 1024 * 1024
        if fields[1] and fields[1] not in entry["gpu_uuids"]:
            entry["gpu_uuids"].append(fields[1])
    return metrics


def parse_nvidia_pmon(output: str) -> dict[int, dict]:
    """Parse per-PID SM/memory/encoder/decoder percentages from nvidia-smi pmon."""
    metrics: dict[int, dict] = {}
    for line in output.splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        fields = line.split()
        if len(fields) < 7:
            continue
        try:
            gpu_index = int(fields[0])
            pid = int(fields[1])
        except ValueError:
            continue
        entry = metrics.setdefault(pid, {
            "gpu_indices": [],
            "gpu_sm_util_percent_sum": 0,
            "gpu_memory_util_percent_sum": 0,
            "gpu_encoder_util_percent_sum": 0,
            "gpu_decoder_util_percent_sum": 0,
        })
        if gpu_index not in entry["gpu_indices"]:
            entry["gpu_indices"].append(gpu_index)
        for key, token in (
            ("gpu_sm_util_percent_sum", fields[3]),
            ("gpu_memory_util_percent_sum", fields[4]),
            ("gpu_encoder_util_percent_sum", fields[5]),
            ("gpu_decoder_util_percent_sum", fields[6]),
        ):
            value = _metric_value(token)
            if value is not None:
                entry[key] += value
    return metrics


def merge_gpu_process_metrics(*sources: dict[int, dict]) -> dict[int, dict]:
    """Merge independent nvidia-smi views into one PID-keyed schema."""
    merged: dict[int, dict] = {}
    for source in sources:
        for pid, values in source.items():
            entry = merged.setdefault(pid, {})
            for key, value in values.items():
                if isinstance(value, list):
                    current = entry.setdefault(key, [])
                    current.extend(item for item in value if item not in current)
                elif key.endswith("_sum") or key.endswith("_bytes"):
                    entry[key] = entry.get(key, 0) + value
                else:
                    entry[key] = value
    return merged


def collect_nvidia_gpu_processes(command_timeout_s=2.5, runner=subprocess.run) -> dict:
    """Collect one NVIDIA per-process sample without making NVML a hard dependency."""
    commands = (
        [
            "nvidia-smi",
            "--query-compute-apps=pid,gpu_uuid,used_gpu_memory",
            "--format=csv,noheader,nounits",
        ],
        ["nvidia-smi", "pmon", "-c", "1", "-s", "um"],
    )
    results = []
    errors = []
    for command in commands:
        try:
            result = runner(
                command,
                capture_output=True,
                text=True,
                timeout=max(0.1, float(command_timeout_s)),
                check=False,
            )
            results.append(result.stdout if result.returncode == 0 else "")
            if result.returncode != 0:
                errors.append(
                    (result.stderr or result.stdout or "nvidia-smi failed").strip())
        except (FileNotFoundError, subprocess.SubprocessError, OSError) as exc:
            results.append("")
            errors.append(str(exc))
    available = any(bool(output) for output in results) or (
        len(errors) < len(commands)
    )
    return {
        "provider": "nvidia-smi",
        "available": available,
        "error": "; ".join(errors)[:512],
        "processes": merge_gpu_process_metrics(
            parse_nvidia_compute_apps(results[0]),
            parse_nvidia_pmon(results[1]),
        ),
        "sample_monotonic_s": time.monotonic(),
    }


def aggregate_gpu_metrics(pids, process_metrics: dict[int, dict]) -> dict:
    """Aggregate GPU metrics for all OS PIDs belonging to one logical component."""
    records = [process_metrics[pid] for pid in pids if pid in process_metrics]
    lists = defaultdict(list)
    sums = defaultdict(int)
    for record in records:
        for key, value in record.items():
            if isinstance(value, list):
                lists[key].extend(item for item in value if item not in lists[key])
            elif key.endswith("_sum") or key.endswith("_bytes"):
                sums[key] += value
    return {
        "gpu_metrics_available": bool(records),
        "gpu_memory_used_bytes": sums["gpu_memory_used_bytes"],
        "gpu_sm_util_percent_sum": sums["gpu_sm_util_percent_sum"],
        "gpu_memory_util_percent_sum": sums["gpu_memory_util_percent_sum"],
        "gpu_encoder_util_percent_sum": sums["gpu_encoder_util_percent_sum"],
        "gpu_decoder_util_percent_sum": sums["gpu_decoder_util_percent_sum"],
        "gpu_indices": lists["gpu_indices"],
        "gpu_uuids": lists["gpu_uuids"],
    }


class GpuTelemetrySampler:
    """Continuously sample nvidia-smi off the ROS timer thread and cache the result."""

    def __init__(self, period_s=1.0, command_timeout_s=2.5):
        self._period_s = max(0.2, float(period_s))
        self._command_timeout_s = max(0.1, float(command_timeout_s))
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._latest = {
            "provider": "nvidia-smi",
            "available": False,
            "error": "first sample pending",
            "processes": {},
            "sample_monotonic_s": 0.0,
        }
        self._thread = threading.Thread(
            target=self._run, name="gpu-telemetry", daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            sample = collect_nvidia_gpu_processes(self._command_timeout_s)
            with self._lock:
                self._latest = sample
            self._stop.wait(self._period_s)

    def snapshot(self):
        with self._lock:
            result = dict(self._latest)
            result["processes"] = {
                pid: dict(metrics)
                for pid, metrics in self._latest["processes"].items()
            }
            return result

    def close(self):
        self._stop.set()
        self._thread.join(timeout=0.5)


class SystemTelemetryNode(Node):
    def __init__(self):
        super().__init__("system_telemetry")
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("enable_affinity", False)
        self.declare_parameter("affinity_rules_json", "{}")
        self.declare_parameter("cpu_pressure_threshold_percent", 85.0)
        self.declare_parameter("recorder_effective_hz_min", 8.0)
        self.declare_parameter("evidence_samples_required", 3)
        self.declare_parameter("enable_gpu_telemetry", True)
        self.declare_parameter("gpu_sample_period_s", 1.0)
        self.declare_parameter("gpu_command_timeout_s", 2.5)
        self._publisher = self.create_publisher(
            DiagnosticArray, "/system/telemetry", 10)
        self._recorder_effective_hz = 0.0
        self._recorder_recording = False
        self._pressure_streak = 0
        self._affinity_evidence_ready = False
        self._gpu_sampler = None
        if bool(self.get_parameter("enable_gpu_telemetry").value):
            self._gpu_sampler = GpuTelemetrySampler(
                period_s=float(self.get_parameter("gpu_sample_period_s").value),
                command_timeout_s=float(
                    self.get_parameter("gpu_command_timeout_s").value),
            )
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
        gpu_sample = self._gpu_sampler.snapshot() if self._gpu_sampler else {
            "provider": "disabled",
            "available": False,
            "error": "GPU telemetry disabled by parameter",
            "processes": {},
            "sample_monotonic_s": 0.0,
        }
        gpu_sample_age_s = (
            max(0.0, time.monotonic() - gpu_sample["sample_monotonic_s"])
            if gpu_sample["sample_monotonic_s"] else -1.0
        )
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
            "gpu_telemetry_enabled": self._gpu_sampler is not None,
            "gpu_provider_available": gpu_sample["available"],
            "gpu_sample_age_s": gpu_sample_age_s,
        })
        gpu_status = DiagnosticStatus()
        gpu_status.name = "system_telemetry/gpu_provider"
        gpu_status.hardware_id = "nvidia_gpu"
        gpu_status.level = (
            DiagnosticStatus.OK
            if self._gpu_sampler is None or gpu_sample["available"]
            else DiagnosticStatus.WARN
        )
        gpu_status.message = (
            "disabled" if self._gpu_sampler is None
            else ("ok" if gpu_sample["available"] else "unavailable")
        )
        gpu_status.values = self._kv({
            "provider": gpu_sample["provider"],
            "available": gpu_sample["available"],
            "sample_age_s": gpu_sample_age_s,
            "error": gpu_sample["error"],
            "observed_gpu_process_count": len(gpu_sample["processes"]),
        })
        statuses = [host, gpu_status]
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
            gpu_metrics = aggregate_gpu_metrics(pids, gpu_sample["processes"])
            status.values = self._kv({
                "pids": json.dumps(pids),
                "cpu_percent": cpu,
                "rss_bytes": rss,
                "affinity": json.dumps(affinities),
                **gpu_metrics,
                "gpu_sample_age_s": gpu_sample_age_s,
            })
            statuses.append(status)
        msg.status = statuses
        self._publisher.publish(msg)

    def destroy_node(self):
        if self._gpu_sampler is not None:
            self._gpu_sampler.close()
            self._gpu_sampler = None
        return super().destroy_node()


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

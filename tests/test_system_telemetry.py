from types import SimpleNamespace

from lerobot_recorder.system_telemetry import (
    aggregate_gpu_metrics,
    collect_nvidia_gpu_processes,
    match_processes,
    parse_nvidia_compute_apps,
    parse_nvidia_pmon,
)


class FakeProcess:
    def __init__(self, command):
        self.command = command

    def cmdline(self):
        return self.command


def test_process_matching_uses_full_python_cmdline_and_ros_remap():
    processes = [
        FakeProcess([
            "/usr/bin/python3",
            "/opt/ros/lib/camera_bridge/camera_bridge_node",
            "--ros-args",
            "-r",
            "__node:=camera_bridge",
        ]),
        FakeProcess([
            "/usr/bin/python3",
            "/opt/ros/lib/lerobot_recorder/lerobot_recorder_node",
            "--ros-args",
            "-r",
            "__node:=lerobot_recorder",
        ]),
    ]
    matched = match_processes(processes)
    assert matched["scene_camera"] == [processes[0]]
    assert matched["recorder"] == [processes[1]]


def test_compute_app_parser_aggregates_memory_by_pid_and_gpu():
    parsed = parse_nvidia_compute_apps(
        "42, GPU-a, 128\n42, GPU-b, 64\n77, GPU-a, N/A\n")
    assert parsed[42]["gpu_memory_used_bytes"] == 192 * 1024 * 1024
    assert parsed[42]["gpu_uuids"] == ["GPU-a", "GPU-b"]
    assert parsed[77]["gpu_memory_used_bytes"] == 0


def test_pmon_parser_exposes_per_process_utilization():
    parsed = parse_nvidia_pmon(
        "# gpu pid type sm mem enc dec command\n"
        "0 42 C 65 12 0 0 python3\n"
        "1 42 C 20 5 - - python3\n")
    assert parsed[42]["gpu_indices"] == [0, 1]
    assert parsed[42]["gpu_sm_util_percent_sum"] == 85
    assert parsed[42]["gpu_memory_util_percent_sum"] == 17


def test_gpu_collector_and_logical_process_aggregation_share_pid_schema():
    outputs = iter([
        SimpleNamespace(returncode=0, stdout="42, GPU-a, 128\n", stderr=""),
        SimpleNamespace(returncode=0, stdout="0 42 C 65 12 0 0 python3\n", stderr=""),
    ])

    def runner(*_args, **_kwargs):
        return next(outputs)

    sample = collect_nvidia_gpu_processes(runner=runner)
    aggregate = aggregate_gpu_metrics([42, 99], sample["processes"])
    assert sample["available"] is True
    assert aggregate == {
        "gpu_metrics_available": True,
        "gpu_memory_used_bytes": 128 * 1024 * 1024,
        "gpu_sm_util_percent_sum": 65,
        "gpu_memory_util_percent_sum": 12,
        "gpu_encoder_util_percent_sum": 0,
        "gpu_decoder_util_percent_sum": 0,
        "gpu_indices": [0],
        "gpu_uuids": ["GPU-a"],
    }


def test_gpu_collector_reports_provider_failure_instead_of_zero_metrics():
    def runner(*_args, **_kwargs):
        return SimpleNamespace(
            returncode=9,
            stdout="",
            stderr="NVIDIA driver unavailable",
        )

    sample = collect_nvidia_gpu_processes(runner=runner)
    assert sample["available"] is False
    assert sample["processes"] == {}
    assert "NVIDIA driver unavailable" in sample["error"]

"""Static teardown guarantees for the bounded collection scripts."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_stop_stack_covers_renderer_recorder_and_telemetry():
    source = (ROOT / "scripts/stop_stack.sh").read_text(encoding="utf-8")

    assert "camera_bridge_node" in source
    assert "lerobot_recorder_node" in source
    assert "system_telemetry_node" in source
    assert "robot_state_publisher" in source
    assert "aggregator_node" in source
    assert "kill -KILL" in source
    assert "collect_stack_pids" in source
    assert '(( ${#seen[@]} > 0 ))' in source


def test_batch_preflight_interrupts_complete_launch_process_group():
    source = (ROOT / "scripts/run_batch_preflight_smoke.sh").read_text(encoding="utf-8")

    assert source.count('kill -INT -- "-${LAUNCH_PID}"') == 2

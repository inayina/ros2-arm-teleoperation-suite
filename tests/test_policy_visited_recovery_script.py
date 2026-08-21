from pathlib import Path


SCRIPT = Path('scripts/mujoco_policy_visited_recovery.py').read_text(encoding='utf-8')


def test_recovery_script_keeps_single_command_authority_and_recorder_gate() -> None:
    assert "count_publishers('/teleop/cmd_pose') == 1" in SCRIPT
    assert "count_publishers('/teleop/gripper_cmd') == 1" in SCRIPT
    assert "String(data='start')" in SCRIPT
    assert 'request.discard = not bool(commit)' in SCRIPT
    assert "gt_source == 'upstream_continuous_task_evaluator'" in SCRIPT
    assert "'claims_task_success': False" in SCRIPT


def test_recovery_records_only_after_policy_prefix() -> None:
    replay = SCRIPT.index('node.replay_prefix')
    record = SCRIPT.index('node.start_recording()')
    assert replay < record

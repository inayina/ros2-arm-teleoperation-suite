from pathlib import Path


SOURCE = Path(
    "src/safety_monitor/src/safety_monitor_node.cpp"
).read_text(encoding="utf-8")


def _function_body(signature: str, next_signature: str) -> str:
    return SOURCE.rsplit(signature, 1)[1].split(next_signature, 1)[0]


def test_locked_helpers_never_publish_to_dds():
    trip_body = _function_body(
        "bool trip_estop_locked(", "bool velocity_estop_allowed_locked(")
    status_body = _function_body(
        "make_status_messages_locked()", "JointLimitMonitor joint_limit_")
    assert "publish(" not in trip_body
    assert "publish(" not in status_body
    assert "publish_estop_locked" not in SOURCE
    assert "publish_status_locked" not in SOURCE


def test_timer_snapshots_under_lock_then_publishes_after_scope():
    timer_body = _function_body("void on_timer()", "void publish_estop(")
    lock_end = timer_body.index("    }\n    pub_status_->publish(status);")
    assert "std::tie(status, diagnostics)" in timer_body[:lock_end]
    assert "->publish(" not in timer_body[:lock_end]
    assert "pub_status_->publish(status);" in timer_body[lock_end:]
    assert "pub_diag_->publish(diagnostics);" in timer_body[lock_end:]


def test_command_callbacks_publish_only_after_mutex_scope():
    pose_body = _function_body("void on_cmd_pose(", "void on_cmd_twist(")
    twist_body = _function_body("void on_cmd_twist(", "void on_trigger_estop(")
    for body, publisher in (
        (pose_body, "pub_safe_pose_->publish"),
        (twist_body, "pub_safe_twist_->publish"),
    ):
        lock_end = body.index("    if (estop_changed)")
        assert "->publish(" not in body[:lock_end]
        assert publisher in body[lock_end:]

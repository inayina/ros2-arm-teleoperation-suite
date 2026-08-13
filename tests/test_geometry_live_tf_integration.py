# Copyright 2026 ros2-arm-teleoperation-suite contributors
"""Integration: real robot_state_publisher TF vs independent URDF FK.

Starts an isolated RSP + joint_state publisher via LiveTfHarness, looks up
panda_link0 → panda_ee, compares to IndependentUrdfFk, then tears down with
timeout + process-group / pkill cleanup.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import time
from pathlib import Path

import numpy as np
import pytest

from teleop_diagnostics import FRAME_EE
from teleop_diagnostics.live_tf_harness import LiveTfHarness
from teleop_diagnostics.poses import READY_Q
from teleop_diagnostics.types import EvidenceClass, InputStatus
from teleop_diagnostics.urdf_fk import IndependentUrdfFk, expand_xacro_to_urdf, load_urdf_model

REPO = Path(__file__).resolve().parents[1]
XACRO = REPO / "src/teleop_description/urdf/panda.urdf.xacro"

# Isolated DDS domain for this test only.
_TEST_DOMAIN_ID = 96
# Hard wall-clock budget for the whole integration (includes RSP warm-up).
_WALL_TIMEOUT_SEC = 60.0


def _ros_stack_available() -> bool:
    if shutil.which("ros2") is None:
        return False
    try:
        import rclpy  # noqa: F401
        from tf2_ros import Buffer  # noqa: F401
    except ImportError:
        return False
    proc = subprocess.run(
        ["ros2", "pkg", "prefix", "robot_state_publisher"],
        check=False,
        capture_output=True,
        text=True,
    )
    return proc.returncode == 0


def _nuke_diag_processes() -> None:
    """Best-effort cleanup; must not leave RSP / diag nodes behind."""
    patterns = (
        "teleop_diagnostics_live_tf",
        # Harness params file path is unique enough for our RSP child.
        "teleop_diag_rsp_",
    )
    for pat in patterns:
        subprocess.run(
            ["pkill", "-9", "-f", pat],
            check=False,
            capture_output=True,
        )


@pytest.mark.launch_test
@pytest.mark.skipif(not _ros_stack_available(), reason="ROS 2 / RSP / tf2 not available")
def test_live_rsp_tf_panda_link0_to_panda_ee_matches_urdf_fk() -> None:
    """Bring up real RSP, lookup TF, compare to URDF FK, then hard-clean."""
    log_dir = REPO / ".ros_diag_log" / "pytest_live_tf"
    log_dir.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_LOG_DIR"] = str(log_dir)
    os.environ["RCUTILS_LOGGING_USE_STDOUT"] = "1"

    robot = load_urdf_model(expand_xacro_to_urdf(XACRO))
    urdf_fk = IndependentUrdfFk(robot)
    deadline = time.monotonic() + _WALL_TIMEOUT_SEC
    harness: LiveTfHarness | None = None

    def _check_budget(where: str) -> None:
        if time.monotonic() > deadline:
            raise TimeoutError(
                f"live TF integration exceeded {_WALL_TIMEOUT_SEC:.0f}s at {where}"
            )

    try:
        _check_budget("before_harness")
        harness = LiveTfHarness(
            xacro_path=XACRO,
            domain_id=_TEST_DOMAIN_ID,
            max_age_sec=1.0,
            lookup_timeout_sec=3.0,
        )
        harness.__enter__()
        _check_budget("after_harness_enter")

        assert harness.available, (
            f"live RSP TF harness not available: {harness.detail}"
        )
        assert harness._rsp_proc is not None and harness._rsp_proc.poll() is None, (
            "robot_state_publisher process exited early"
        )

        for name, q in (("zero", (0.0,) * 7), ("ready", READY_Q)):
            _check_budget(f"pose_{name}")
            live = harness.lookup_ee(q)
            model = urdf_fk.forward(q, target_link=FRAME_EE)

            assert live.input_status == InputStatus.AVAILABLE, (
                f"{name}: TF not AVAILABLE ({live.input_status}): {live.detail}"
            )
            assert live.matrix is not None
            assert live.frame_from == "panda_link0"
            assert live.frame_to == "panda_ee"
            assert live.evidence_class == EvidenceClass.MODEL
            assert "robot_state_publisher" in live.backend_provenance
            assert "/ee_pose" not in live.backend_provenance

            assert model.matrix is not None
            dp = float(np.linalg.norm(live.translation() - model.translation()))
            # Live RSP and independent URDF FK should agree tightly on nominal model.
            assert dp < 1e-6, f"{name}: URDF vs live TF translation residual {dp} m"

    finally:
        if harness is not None:
            try:
                harness.__exit__(None, None, None)
            except Exception:  # noqa: BLE001
                pass
        _nuke_diag_processes()
        # Confirm our harness node is gone; do not require global RSP absence
        # (other user stacks may exist on other domains).
        leftover = subprocess.run(
            ["pgrep", "-af", "teleop_diagnostics_live_tf"],
            check=False,
            capture_output=True,
            text=True,
        )
        assert leftover.returncode != 0, (
            f"leftover diagnostics node after cleanup:\n{leftover.stdout}"
        )


@pytest.mark.launch_test
@pytest.mark.skipif(not _ros_stack_available(), reason="ROS 2 / RSP / tf2 not available")
def test_live_rsp_tf_wrong_frame_is_missing_not_identity() -> None:
    """Wrong tip frame must fail-closed (MISSING/INVALID), never identity."""
    log_dir = REPO / ".ros_diag_log" / "pytest_live_tf"
    log_dir.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_LOG_DIR"] = str(log_dir)

    harness = LiveTfHarness(
        xacro_path=XACRO,
        domain_id=_TEST_DOMAIN_ID + 1,
        lookup_timeout_sec=1.5,
    )
    try:
        harness.__enter__()
        assert harness.available, harness.detail
        # Force a bad frame lookup through the attached source.
        assert harness.tf_source is not None
        harness.tf_source.frame_to = "not_a_real_frame"
        sample = harness.tf_source.forward([0.0] * 7)
        assert sample.matrix is None
        assert sample.input_status in (
            InputStatus.MISSING,
            InputStatus.INVALID,
            InputStatus.UNAVAILABLE,
        )
        assert sample.evidence_class != EvidenceClass.SIM_GT
        # Must not look like identity success.
        assert sample.input_status != InputStatus.AVAILABLE
    finally:
        try:
            harness.__exit__(None, None, None)
        except Exception:  # noqa: BLE001
            pass
        _nuke_diag_processes()

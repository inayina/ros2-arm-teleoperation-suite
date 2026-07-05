from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from grasp_monitor.grasp_monitor_node import (
    ASSISTED_GRASP,
    GRASP_FAILED,
    GRASP_SUCCESS,
    MISS_OBJECT,
    RELEASED_BY_COMMAND,
    SLIP_AFTER_LIFT,
    GraspMonitorParams,
    GraspObservation,
    GraspStateEstimator,
)


def obs(t, **kwargs):
    defaults = {
        "ee_z": 0.20,
        "object_z": 0.00,
        "ee_object_dist": 0.05,
        "gripper_opening": 0.8,
        "gripper_cmd": 0.8,
        "object_contacts": 0,
        "finger_object_contacts": 0,
        "force_magnitude": 0.0,
        "force_delta": 0.0,
    }
    defaults.update(kwargs)
    return GraspObservation(timestamp=t, **defaults)


def test_success_after_object_follows_lift_for_hold_time():
    estimator = GraspStateEstimator(GraspMonitorParams(success_hold_time=0.2))

    estimator.update(obs(0.0))
    estimator.update(obs(
        0.1,
        gripper_opening=0.2,
        gripper_cmd=0.0,
        object_contacts=1,
        finger_object_contacts=1,
    ))
    estimator.update(obs(
        0.3,
        ee_z=0.25,
        object_z=0.04,
        gripper_opening=0.2,
        gripper_cmd=0.0,
        object_contacts=1,
        finger_object_contacts=1,
    ))
    estimator.update(obs(
        0.5,
        ee_z=0.31,
        object_z=0.08,
        gripper_opening=0.2,
        gripper_cmd=0.0,
        object_contacts=1,
        finger_object_contacts=1,
    ))
    status = estimator.update(obs(
        0.8,
        ee_z=0.31,
        object_z=0.08,
        gripper_opening=0.2,
        gripper_cmd=0.0,
        object_contacts=1,
        finger_object_contacts=1,
    ))

    assert status["state"] == GRASP_SUCCESS
    assert status["classification"] == "SUCCESS"


def test_assisted_grasp_is_not_physical_success():
    estimator = GraspStateEstimator(GraspMonitorParams(success_hold_time=0.2))

    estimator.update(obs(0.0))
    estimator.update(obs(
        0.1,
        gripper_opening=0.2,
        gripper_cmd=0.0,
        object_contacts=1,
        finger_object_contacts=1,
        grasp_assist_attached=True,
    ))
    status = estimator.update(obs(
        0.5,
        ee_z=0.31,
        object_z=0.08,
        gripper_opening=0.2,
        gripper_cmd=0.0,
        object_contacts=1,
        finger_object_contacts=1,
        grasp_assist_attached=True,
    ))

    assert status["state"] == ASSISTED_GRASP
    assert status["classification"] == ASSISTED_GRASP
    assert status["failure_type"] is None


def test_closed_gripper_without_contact_is_miss_object():
    estimator = GraspStateEstimator(GraspMonitorParams())

    estimator.update(obs(0.0))
    status = estimator.update(obs(
        0.2,
        gripper_opening=0.1,
        gripper_cmd=0.0,
        object_contacts=0,
        finger_object_contacts=0,
    ))

    assert status["state"] == GRASP_FAILED
    assert status["failure_type"] == MISS_OBJECT


def test_object_drop_after_lift_is_slip_after_lift():
    estimator = GraspStateEstimator(GraspMonitorParams())

    estimator.update(obs(
        0.0,
        gripper_opening=0.2,
        gripper_cmd=0.0,
        object_contacts=1,
        finger_object_contacts=1,
    ))
    estimator.update(obs(
        0.4,
        ee_z=0.30,
        object_z=0.08,
        gripper_opening=0.2,
        gripper_cmd=0.0,
        object_contacts=1,
        finger_object_contacts=1,
    ))
    status = estimator.update(obs(
        0.8,
        ee_z=0.32,
        object_z=0.03,
        ee_object_dist=0.13,
        gripper_opening=0.2,
        gripper_cmd=0.0,
        object_contacts=0,
        finger_object_contacts=0,
    ))

    assert status["state"] == SLIP_AFTER_LIFT
    assert status["failure_type"] == SLIP_AFTER_LIFT


def test_reopen_after_contact_is_released_by_command():
    estimator = GraspStateEstimator(GraspMonitorParams())

    estimator.update(obs(
        0.0,
        gripper_opening=0.2,
        gripper_cmd=0.0,
        object_contacts=1,
        finger_object_contacts=1,
    ))
    status = estimator.update(obs(
        0.3,
        gripper_opening=0.8,
        gripper_cmd=0.8,
        object_contacts=0,
        finger_object_contacts=0,
    ))

    assert status["state"] == RELEASED_BY_COMMAND
    assert status["failure_type"] == RELEASED_BY_COMMAND

from synth_data_gen.motion_primitives import (
    compute_twist_linear,
    position_error,
    update_max_tracking_error,
)


def test_compute_twist_linear_moves_toward_target():
    vx, vy, vz, reached = compute_twist_linear([0.0, 0.0, 0.5], [0.1, 0.0, 0.5], 0.05)
    assert not reached
    assert vx > 0.0
    assert abs(vy) < 1e-9
    assert abs(vz) < 1e-9


def test_compute_twist_linear_reached_at_target():
    _, _, _, reached = compute_twist_linear([0.3, 0.1, 0.4], [0.3, 0.1, 0.4], 0.05)
    assert reached


def test_update_max_tracking_error():
    assert update_max_tracking_error([0.0, 0.0, 0.0], [0.03, 0.0, 0.0], 0.01) == 0.03

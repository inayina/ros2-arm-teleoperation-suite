import numpy as np

from mujoco_sim.mujoco_sim_node import fallback_ee_transform, _quat_wxyz_from_matrix


def test_fallback_ready_pose_publishes_finite_non_origin_ee_pose():
    q_ready = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])

    t_ee = fallback_ee_transform(q_ready)
    position = t_ee[:3, 3]
    quat = _quat_wxyz_from_matrix(t_ee[:3, :3])

    assert np.all(np.isfinite(position))
    assert np.all(np.isfinite(quat))
    assert np.linalg.norm(position) > 0.2
    assert abs(np.linalg.norm(quat) - 1.0) < 1e-6

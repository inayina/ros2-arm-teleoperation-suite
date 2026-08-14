import unittest
from unittest.mock import MagicMock
import numpy as np
from mujoco_sim.domain_randomizer import DomainRandomizer
from mujoco_sim.camera_extrinsics import make_transform, matrix_to_quat_wxyz


class MockMujoco:
    class mjtObj:
        mjOBJ_BODY = 1
        mjOBJ_GEOM = 2
        mjOBJ_JOINT = 3
        mjOBJ_CAMERA = 4
        mjOBJ_LIGHT = 5

    # 多目标物体名称到 ID 的映射
    _NAME_TO_ID = {
        "object_red_box": 0,
        "red_box_geom": 0,
        "red_box_joint": 0,
        "object_blue_cylinder": 1,
        "blue_cylinder_geom": 1,
        "blue_cylinder_joint": 1,
        "object_green_sphere": 2,
        "green_sphere_geom": 2,
        "green_sphere_joint": 2,
        "scene_camera": 0,
        "key": 0,
        "world": 0,
    }

    _ID_TO_NAME = {v: k for k, v in _NAME_TO_ID.items()}

    def mj_name2id(self, model, obj_type, name):
        return self._NAME_TO_ID.get(name, -1)

    def mj_id2name(self, model, obj_type, obj_id):
        if obj_type == self.mjtObj.mjOBJ_BODY and obj_id == 0:
            return "world"
        return self._ID_TO_NAME.get(obj_id)


class MockModel:
    def __init__(self):
        self.body_mass = [1.0, 1.0, 1.0]          # 3 个物体
        self.geom_friction = [
            [1.0, 0.005, 0.0001],
            [1.0, 0.005, 0.0001],
            [1.0, 0.005, 0.0001],
        ]
        self.jnt_qposadr = [0, 7, 14]             # 每个自由关节 7 个 qpos
        self.jnt_dofadr = [0, 6, 12]
        self.cam_pos = [np.array([0.55, -0.75, 0.55], dtype=float)]
        self.cam_quat = [np.array([1.0, 0.0, 0.0, 0.0], dtype=float)]
        self.cam_bodyid = [0]
        self.cam_fovy = [45.0]
        self.light_diffuse = [[0.8, 0.8, 0.8]]

class MockData:
    def __init__(self):
        # 三个自由关节共 7*3=21 个 qpos，6*3=18 个 qvel
        self.qpos = np.zeros(21)
        self.qpos[0] = 0.4   # red_box 默认 X
        self.qpos[2] = 0.025  # Z
        self.qpos[3] = 1.0   # qw
        self.qvel = np.zeros(18)

class TestDomainRandomizer(unittest.TestCase):
    def test_randomizer_disabled(self):
        config = {
            "domain_randomization": {
                "enabled": False,
                "object": {
                    "mass_range": [0.1, 0.5]
                }
            }
        }
        randomizer = DomainRandomizer(config)
        assert randomizer.enabled is False

        model = MockModel()
        data = MockData()
        # Simulate sunk / ejected free joints left from a prior grasp attempt.
        data.qpos[14] = 1.5
        data.qpos[15] = 0.9
        data.qpos[16] = -0.039
        data.qvel[12:18] = 0.5
        mujoco = MockMujoco()
        mujoco.mj_forward = MagicMock()

        randomizer.apply(model, data, mujoco)
        # Mass randomization stays off, but objects must still be restored.
        assert model.body_mass[0] == 1.0
        assert abs(data.qpos[0] - 0.35) < 1e-9
        assert abs(data.qpos[1] - (-0.07)) < 1e-9
        assert abs(data.qpos[2] - 0.025) < 1e-9
        assert abs(data.qpos[14] - 0.45) < 1e-9
        assert abs(data.qpos[16] - 0.03) < 1e-9
        assert all(v == 0.0 for v in data.qvel[12:18])
        mujoco.mj_forward.assert_called()

    def test_randomizer_enabled(self):
        config = {
            "domain_randomization": {
                "enabled": True,
                "seed": 42,
                "camera": {
                    "scene_camera": {
                        "pos_noise": [-0.05, 0.05],
                        "rot_noise": [-5.0, 5.0]
                    }
                },
                "object": {
                    "mass_range": [0.1, 0.5],
                    "friction_range": [0.5, 1.5],
                    "initial_pos_range": {
                        "x": [0.35, 0.55],
                        "y": [-0.2, 0.2]
                    },
                    "yaw_range_deg_by_object": {
                        "object_red_box": [-10.0, 10.0],
                    },
                },
                "lighting": {
                    "key": {
                        "diffuse_noise": [-0.2, 0.2]
                    }
                }
            }
        }

        randomizer = DomainRandomizer(config)
        assert randomizer.enabled is True

        model = MockModel()
        data = MockData()
        mujoco = MockMujoco()
        mujoco.mj_forward = MagicMock()

        # Apply first time
        randomizer.apply(model, data, mujoco)

        # 1. Verify object randomization for 3 objects
        # Mass of ALL three objects should be within [0.1, 0.5]
        for idx in range(3):
            assert 0.1 <= model.body_mass[idx] <= 0.5, (
                f"object[{idx}] mass {model.body_mass[idx]} not in [0.1, 0.5]")

        # Box/cylinder keep friction_range; sphere gets a raised floor so
        # fingertip contact does not eject it on sticky-reset retries.
        assert 0.5 <= model.geom_friction[0][0] <= 1.5
        assert 0.5 <= model.geom_friction[1][0] <= 1.5
        assert 3.5 <= model.geom_friction[2][0] <= 5.0

        # X/Y/Z of each object should be in valid range (Z rewritten every reset)
        expected_z = (0.025, 0.03, 0.03)  # box / cylinder / sphere defaults
        for idx in range(3):
            adr = idx * 7
            assert 0.35 <= data.qpos[adr] <= 0.55, (
                f"object[{idx}] X {data.qpos[adr]} out of range")
            assert -0.2 <= data.qpos[adr + 1] <= 0.2, (
                f"object[{idx}] Y {data.qpos[adr + 1]} out of range")
            assert abs(data.qpos[adr + 2] - expected_z[idx]) < 1e-9, (
                f"object[{idx}] Z {data.qpos[adr + 2]} != {expected_z[idx]}")

        # The xyz-only scripted expert uses a fixed gripper yaw, so the box
        # randomization is intentionally bounded to face-on grasps.
        red_yaw = 2.0 * np.arctan2(data.qpos[6], data.qpos[3])
        assert np.deg2rad(-10.0) <= red_yaw <= np.deg2rad(10.0)

        # Velocity should all be zero
        assert all(v == 0.0 for v in data.qvel)

        # 2. Verify camera caching and camera-local randomization
        assert 0 in randomizer.orig_cam_pos
        assert 0 in randomizer.orig_cam_quat
        assert np.allclose(randomizer.orig_cam_pos[0], [0.55, -0.75, 0.55])

        first_state = randomizer.last_camera_states["scene_camera"]
        # Identity nominal quat ⇒ local Δt appears directly in parent/world cam_pos.
        dx = model.cam_pos[0][0] - randomizer.orig_cam_pos[0][0]
        dy = model.cam_pos[0][1] - randomizer.orig_cam_pos[0][1]
        dz = model.cam_pos[0][2] - randomizer.orig_cam_pos[0][2]
        assert -0.05 <= dx <= 0.05
        assert -0.05 <= dy <= 0.05
        assert -0.05 <= dz <= 0.05
        assert np.allclose(first_state.perturbation_translation, [dx, dy, dz], atol=1e-9)

        q = np.asarray(model.cam_quat[0], dtype=float)
        assert abs(np.linalg.norm(q) - 1.0) < 1e-5
        T_eff = make_transform(model.cam_pos[0], quat_wxyz=q)
        T_nom = make_transform(
            randomizer.orig_cam_pos[0], quat_wxyz=randomizer.orig_cam_quat[0]
        )
        T_delta = np.linalg.inv(T_nom) @ T_eff
        assert np.allclose(T_delta[:3, 3], first_state.perturbation_translation, atol=1e-9)

        # Independent authority with same seed/draw must match (single random stream).
        from mujoco_sim.camera_extrinsics import (
            CameraExtrinsicAuthority,
            apply_config_randomization,
        )
        twin = apply_config_randomization(
            CameraExtrinsicAuthority(MockModel(), MockMujoco()).nominal("scene_camera"),
            config["domain_randomization"]["camera"],
            seed=42,
            draw_index=0,
        )
        assert np.allclose(
            twin.perturbation_translation, first_state.perturbation_translation, atol=1e-12
        )
        assert np.allclose(
            twin.perturbation_quat_wxyz, first_state.perturbation_quat_wxyz, atol=1e-12
        )

        # 3. Verify lighting randomization
        assert 0 in randomizer.orig_light_diffuse
        assert np.allclose(randomizer.orig_light_diffuse[0], [0.8, 0.8, 0.8])
        for val in model.light_diffuse[0]:
            assert 0.6 <= val <= 1.0

        # Apply second time: no drift vs cached nominal; new draw still in range.
        data.qpos[14] = 1.5
        data.qpos[15] = 0.8
        data.qpos[16] = 0.4
        data.qvel[12:18] = 1.0
        randomizer.apply(model, data, mujoco)
        dx2 = model.cam_pos[0][0] - randomizer.orig_cam_pos[0][0]
        assert -0.05 <= dx2 <= 0.05
        assert 0.35 <= data.qpos[14] <= 0.55
        assert -0.2 <= data.qpos[15] <= 0.2
        assert abs(data.qpos[16] - 0.03) < 1e-9
        assert all(v == 0.0 for v in data.qvel[12:18])

    def test_initial_pos_by_object_overrides_range(self):
        """Per-object XY must win over a shared degenerate range (Phase-1)."""
        config = {
            "domain_randomization": {
                "enabled": True,
                "seed": 59,
                "object": {
                    "mass_range": [0.04, 0.04],
                    "friction_range": [2.2, 2.2],
                    "initial_pos_range": {
                        "x": [0.42, 0.42],
                        "y": [0.10, 0.10],
                    },
                    "initial_pos_by_object": {
                        "object_red_box": [0.42, 0.10],
                        "object_blue_cylinder": [0.52, -0.14],
                        "object_green_sphere": [0.52, 0.14],
                    },
                    "yaw_range_deg_by_object": {
                        "object_red_box": [0.0, 0.0],
                        "object_blue_cylinder": [0.0, 0.0],
                        "object_green_sphere": [0.0, 0.0],
                    },
                },
            }
        }
        randomizer = DomainRandomizer(config)
        model = MockModel()
        data = MockData()
        mujoco = MockMujoco()
        mujoco.mj_forward = MagicMock()
        randomizer.apply(model, data, mujoco)

        assert abs(data.qpos[0] - 0.42) < 1e-9
        assert abs(data.qpos[1] - 0.10) < 1e-9
        assert abs(data.qpos[7] - 0.52) < 1e-9
        assert abs(data.qpos[8] - (-0.14)) < 1e-9
        assert abs(data.qpos[14] - 0.52) < 1e-9
        assert abs(data.qpos[15] - 0.14) < 1e-9

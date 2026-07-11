import unittest
from unittest.mock import MagicMock
import numpy as np
from mujoco_sim.domain_randomizer import DomainRandomizer

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
    }

    def mj_name2id(self, model, obj_type, name):
        return self._NAME_TO_ID.get(name, -1)

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
        self.cam_pos = [[0.55, -0.75, 0.55]]
        self.cam_quat = [[1.0, 0.0, 0.0, 0.0]]
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
                    }
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

        # Velocity should all be zero
        assert all(v == 0.0 for v in data.qvel)

        # 2. Verify camera caching and randomization
        # Cached values
        assert 0 in randomizer.orig_cam_pos
        assert 0 in randomizer.orig_cam_quat
        assert np.allclose(randomizer.orig_cam_pos[0], [0.55, -0.75, 0.55])

        # Pos is shifted by pos_noise
        dx = model.cam_pos[0][0] - randomizer.orig_cam_pos[0][0]
        dy = model.cam_pos[0][1] - randomizer.orig_cam_pos[0][1]
        dz = model.cam_pos[0][2] - randomizer.orig_cam_pos[0][2]
        assert -0.05 <= dx <= 0.05
        assert -0.05 <= dy <= 0.05
        assert -0.05 <= dz <= 0.05

        # Quat is shifted by rot_noise
        q = model.cam_quat[0]
        # Magnitude should be close to 1.0 (normalized)
        mag = np.linalg.norm(q)
        assert abs(mag - 1.0) < 1e-5

        # 3. Verify lighting randomization
        assert 0 in randomizer.orig_light_diffuse
        assert np.allclose(randomizer.orig_light_diffuse[0], [0.8, 0.8, 0.8])
        for val in model.light_diffuse[0]:
            assert 0.6 <= val <= 1.0

        # Apply second time to verify no drift accumulation
        # Model cam_pos should remain within pos_noise range relative to the ORIGIN cached value
        # Simulate an ejected sphere that wandered off the table before reset.
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

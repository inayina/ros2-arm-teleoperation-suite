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

    def mj_name2id(self, model, obj_type, name):
        if name in ("target_object", "target_object_geom", "target_object_joint", "scene_camera", "key"):
            return 0
        return -1

class MockModel:
    def __init__(self):
        self.body_mass = [1.0]
        self.geom_friction = [[1.0, 0.005, 0.0001]]
        self.jnt_qposadr = [0]
        self.jnt_dofadr = [0]
        self.cam_pos = [[0.55, -0.75, 0.55]]
        self.cam_quat = [[1.0, 0.0, 0.0, 0.0]]
        self.light_diffuse = [[0.8, 0.8, 0.8]]

class MockData:
    def __init__(self):
        self.qpos = np.array([0.4, 0.0, 0.05, 1.0, 0.0, 0.0, 0.0])
        self.qvel = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

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
        mujoco = MockMujoco()
        
        randomizer.apply(model, data, mujoco)
        # Should not have changed anything since it's disabled
        assert model.body_mass[0] == 1.0

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
        
        # Apply first time
        randomizer.apply(model, data, mujoco)
        
        # 1. Verify object randomization
        # Mass should be randomized within [0.1, 0.5]
        assert 0.1 <= model.body_mass[0] <= 0.5
        # Friction should be randomized within [0.5, 1.5]
        assert 0.5 <= model.geom_friction[0][0] <= 1.5
        # Initial positions
        assert 0.35 <= data.qpos[0] <= 0.55
        assert -0.2 <= data.qpos[1] <= 0.2
        # Velocity for joint reset to 0
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
        randomizer.apply(model, data, mujoco)
        dx2 = model.cam_pos[0][0] - randomizer.orig_cam_pos[0][0]
        assert -0.05 <= dx2 <= 0.05

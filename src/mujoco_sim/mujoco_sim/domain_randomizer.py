import math
import random
from typing import Dict, Any

import numpy as np


class DomainRandomizer:
    """Applies domain randomization to a MuJoCo model and data."""

    def __init__(self, config: Dict[str, Any]):
        self.config = config.get("domain_randomization", {})
        self.enabled = self.config.get("enabled", False)
        seed = self.config.get("seed", None)
        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)
        self.orig_cam_pos = {}
        self.orig_cam_quat = {}
        self.orig_light_diffuse = {}

    def apply(self, model, data, mujoco):
        """Apply randomizations to the scene."""
        if not self.enabled:
            return

        self._randomize_object(model, data, mujoco)
        self._randomize_cameras(model, mujoco)
        self._randomize_lighting(model, mujoco)

    def _randomize_object(self, model, data, mujoco):
        obj_cfg = self.config.get("object", {})
        if not obj_cfg:
            return

        # We assume the object is named "target_object" with a "target_object_joint" free joint
        # and "target_object_geom" geom
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target_object")
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "target_object_geom")
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "target_object_joint")

        if body_id < 0 or geom_id < 0 or joint_id < 0:
            return

        # Randomize mass
        mass_range = obj_cfg.get("mass_range")
        if mass_range:
            new_mass = random.uniform(mass_range[0], mass_range[1])
            model.body_mass[body_id] = new_mass

        # Randomize friction (sliding)
        friction_range = obj_cfg.get("friction_range")
        if friction_range:
            new_fric = random.uniform(friction_range[0], friction_range[1])
            model.geom_friction[geom_id][0] = new_fric

        # Randomize initial pose (we set the qpos for the free joint)
        pos_range = obj_cfg.get("initial_pos_range", {})
        x_range = pos_range.get("x")
        y_range = pos_range.get("y")

        if x_range and y_range:
            qpos_adr = model.jnt_qposadr[joint_id]
            # free joint has 7 qpos values: x, y, z, qw, qx, qy, qz
            data.qpos[qpos_adr] = random.uniform(x_range[0], x_range[1])
            data.qpos[qpos_adr + 1] = random.uniform(y_range[0], y_range[1])
            # Reset velocity for the free joint (6 DOFs)
            dof_adr = model.jnt_dofadr[joint_id]
            data.qvel[dof_adr : dof_adr + 6] = 0.0

    def _randomize_cameras(self, model, mujoco):
        cam_cfg = self.config.get("camera", {})
        for cam_name, params in cam_cfg.items():
            cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, cam_name)
            if cam_id < 0:
                continue

            # Cache original pose to prevent drift over multiple resets
            if cam_id not in self.orig_cam_pos:
                self.orig_cam_pos[cam_id] = np.array(model.cam_pos[cam_id])
            if cam_id not in self.orig_cam_quat:
                self.orig_cam_quat[cam_id] = np.array(model.cam_quat[cam_id])

            pos_noise = params.get("pos_noise")
            if pos_noise:
                dx = random.uniform(pos_noise[0], pos_noise[1])
                dy = random.uniform(pos_noise[0], pos_noise[1])
                dz = random.uniform(pos_noise[0], pos_noise[1])
                model.cam_pos[cam_id][0] = self.orig_cam_pos[cam_id][0] + dx
                model.cam_pos[cam_id][1] = self.orig_cam_pos[cam_id][1] + dy
                model.cam_pos[cam_id][2] = self.orig_cam_pos[cam_id][2] + dz

            rot_noise = params.get("rot_noise")
            if rot_noise:
                # rot_noise in degrees, convert to radians
                r = math.radians(random.uniform(rot_noise[0], rot_noise[1]))
                p = math.radians(random.uniform(rot_noise[0], rot_noise[1]))
                y = math.radians(random.uniform(rot_noise[0], rot_noise[1]))

                # Convert RPY angles to noise quaternion (WXYZ)
                cy = math.cos(y * 0.5)
                sy = math.sin(y * 0.5)
                cp = math.cos(p * 0.5)
                sp = math.sin(p * 0.5)
                cr = math.cos(r * 0.5)
                sr = math.sin(r * 0.5)

                qw = cr * cp * cy + sr * sp * sy
                qx = sr * cp * cy - cr * sp * sy
                qy = cr * sp * cy + sr * cp * sy
                qz = cr * cp * sy - sr * sp * cy
                noise_q = np.array([qw, qx, qy, qz])

                # Multiply original quaternion with noise quaternion
                orig_q = self.orig_cam_quat[cam_id]
                w1, x1, y1, z1 = orig_q
                w2, x2, y2, z2 = noise_q

                new_w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
                new_x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
                new_y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
                new_z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

                # Normalize quaternion to prevent numerical issues
                mag = math.sqrt(new_w**2 + new_x**2 + new_y**2 + new_z**2)
                if mag > 1e-6:
                    new_w /= mag
                    new_x /= mag
                    new_y /= mag
                    new_z /= mag

                model.cam_quat[cam_id] = [new_w, new_x, new_y, new_z]

    def _randomize_lighting(self, model, mujoco):
        light_cfg = self.config.get("lighting", {})
        for light_name, params in light_cfg.items():
            light_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_LIGHT, light_name)
            if light_id < 0:
                continue

            # Cache original diffuse to prevent drift over multiple resets
            if light_id not in self.orig_light_diffuse:
                self.orig_light_diffuse[light_id] = np.array(model.light_diffuse[light_id])

            diffuse_noise = params.get("diffuse_noise")
            if diffuse_noise:
                dn = random.uniform(diffuse_noise[0], diffuse_noise[1])
                for i in range(3):
                    val = self.orig_light_diffuse[light_id][i] + dn
                    model.light_diffuse[light_id][i] = max(0.0, min(1.0, val))

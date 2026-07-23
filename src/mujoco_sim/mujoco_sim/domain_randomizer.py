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
        """Apply randomizations to the scene.

        Even when domain randomization is disabled, free-joint object poses are
        restored to a known table-top rest. Otherwise /sim/reset_scene only
        resets the arm and leaves ejected / sunk objects (e.g. sphere at z<0).
        """
        self.restore_object_poses(model, data, mujoco, randomize=self.enabled)
        if not self.enabled:
            return
        self._randomize_cameras(model, mujoco)
        self._randomize_lighting(model, mujoco)

    def restore_object_poses(self, model, data, mujoco, randomize: bool = False):
        """Place all teaching objects on the table with zero free-joint velocity."""
        obj_cfg = self.config.get("object", {}) if self.config else {}
        objects_to_place = [
            ("object_red_box", "red_box_joint", "red_box_geom"),
            ("object_blue_cylinder", "blue_cylinder_joint", "blue_cylinder_geom"),
            ("object_green_sphere", "green_sphere_joint", "green_sphere_geom"),
        ]
        placed_positions = []
        pos_range = obj_cfg.get("initial_pos_range", {}) if randomize else {}
        x_range = pos_range.get("x")
        y_range = pos_range.get("y")
        yaw_range_deg_by_object = (
            obj_cfg.get("yaw_range_deg_by_object", {}) if randomize else {}
        )
        default_yaw_range_deg = obj_cfg.get("yaw_range_deg", [-180.0, 180.0])
        mass_range = obj_cfg.get("mass_range") if randomize else None
        friction_range = obj_cfg.get("friction_range") if randomize else None

        # Match XML body origins so randomize:=false restores the proven
        # table-top layout instead of arbitrary fallback slots.
        fallback_xy = {
            "object_red_box": (0.35, -0.07),
            "object_blue_cylinder": (0.40, 0.10),
            "object_green_sphere": (0.45, 0.00),
        }
        for obj_name, joint_name, geom_name in objects_to_place:
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, obj_name)
            geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if body_id < 0 or geom_id < 0 or joint_id < 0:
                continue

            if mass_range:
                model.body_mass[body_id] = random.uniform(mass_range[0], mass_range[1])
            if friction_range:
                lo, hi = friction_range[0], friction_range[1]
                if obj_name == "object_green_sphere":
                    lo = max(lo, float(obj_cfg.get("sphere_friction_min", 3.5)))
                    hi = max(hi, float(obj_cfg.get("sphere_friction_max", 5.0)))
                model.geom_friction[geom_id][0] = random.uniform(lo, hi)

            qpos_adr = int(model.jnt_qposadr[joint_id])
            dof_adr = int(model.jnt_dofadr[joint_id])
            data.qvel[dof_adr : dof_adr + 6] = 0.0

            z_rest = float(obj_cfg.get("initial_z", 0.025))
            if obj_name == "object_green_sphere":
                # Spawn above the table then settle; soft-contact planes otherwise
                # tunnel the sphere to z≈-0.04 across prior red/blue episodes.
                z_rest = float(obj_cfg.get("sphere_initial_z", 0.03))
            elif obj_name == "object_blue_cylinder":
                z_rest = float(obj_cfg.get("cylinder_initial_z", 0.03))
            elif obj_name == "object_red_box":
                z_rest = float(obj_cfg.get("box_initial_z", 0.025))

            x, y = fallback_xy.get(obj_name, (0.43, 0.0))
            pos_by_object = obj_cfg.get("initial_pos_by_object", {})
            if isinstance(pos_by_object, dict) and obj_name in pos_by_object:
                fixed = pos_by_object[obj_name]
                if not isinstance(fixed, (list, tuple)) or len(fixed) != 2:
                    raise ValueError(
                        f"initial_pos_by_object[{obj_name}] must be [x, y]"
                    )
                x, y = float(fixed[0]), float(fixed[1])
            elif x_range and y_range:
                placed = False
                for _ in range(100):
                    cand_x = random.uniform(x_range[0], x_range[1])
                    cand_y = random.uniform(y_range[0], y_range[1])
                    too_close = any(
                        math.sqrt((cand_x - px) ** 2 + (cand_y - py) ** 2) < 0.10
                        for px, py in placed_positions
                    )
                    if not too_close:
                        x, y = cand_x, cand_y
                        placed = True
                        break
                if not placed:
                    for px, py in placed_positions:
                        if math.sqrt((x - px) ** 2 + (y - py) ** 2) < 0.10:
                            x = min(x_range[1], max(x_range[0], x + 0.12))
                            y = min(y_range[1], max(y_range[0], y - 0.08))
                            break


            data.qpos[qpos_adr] = x
            data.qpos[qpos_adr + 1] = y
            data.qpos[qpos_adr + 2] = z_rest
            yaw = 0.0
            if randomize:
                yaw_range_deg = yaw_range_deg_by_object.get(
                    obj_name, default_yaw_range_deg)
                if not isinstance(yaw_range_deg, (list, tuple)) or len(yaw_range_deg) != 2:
                    raise ValueError(
                        f"yaw range for {obj_name} must be [min_deg, max_deg]"
                    )
                yaw_min_deg, yaw_max_deg = map(float, yaw_range_deg)
                if yaw_min_deg > yaw_max_deg:
                    raise ValueError(
                        f"yaw range for {obj_name} has min > max: {yaw_range_deg}"
                    )
                yaw = math.radians(random.uniform(yaw_min_deg, yaw_max_deg))
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            data.qpos[qpos_adr + 3] = cy
            data.qpos[qpos_adr + 4] = 0.0
            data.qpos[qpos_adr + 5] = 0.0
            data.qpos[qpos_adr + 6] = sy
            placed_positions.append((x, y))

        try:
            mujoco.mj_forward(model, data)
        except Exception:
            pass

    def _randomize_object(self, model, data, mujoco):
        # Kept for tests / callers; pose restore + optional DR live in restore_object_poses.
        self.restore_object_poses(model, data, mujoco, randomize=True)

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

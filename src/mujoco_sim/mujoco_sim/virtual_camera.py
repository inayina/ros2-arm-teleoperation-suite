"""MuJoCo offscreen virtual camera helpers for M6 perception."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CameraModel:
    """Camera calibration derived from MuJoCo fovy and image size."""

    name: str
    width: int
    height: int
    fovy_deg: float
    frame_id: str

    @property
    def intrinsic_matrix(self) -> list[float]:
        focal = (self.height / 2.0) / math.tan(math.radians(self.fovy_deg / 2.0))
        cx = self.width / 2.0
        cy = self.height / 2.0
        return [focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0]

    @property
    def projection_matrix(self) -> list[float]:
        k = self.intrinsic_matrix
        return [k[0], 0.0, k[2], 0.0, 0.0, k[4], k[5], 0.0, 0.0, 0.0, 1.0, 0.0]


class VirtualCamera:
    """Render RGB and metric depth from a MuJoCo camera."""

    def __init__(self, mujoco_module, model, camera: CameraModel):
        self._mujoco = mujoco_module
        self._model = model
        self.camera = camera
        self._renderer = mujoco_module.Renderer(
            model, height=camera.height, width=camera.width
        )
        self._camera_id = mujoco_module.mj_name2id(
            model, mujoco_module.mjtObj.mjOBJ_CAMERA, camera.name
        )
        if self._camera_id < 0:
            raise RuntimeError(f"MuJoCo camera '{camera.name}' not found")

    def render_rgb(self, data) -> np.ndarray:
        """Return one RGB frame without paying for an unused depth pass."""
        self._renderer.disable_depth_rendering()
        self._renderer.update_scene(data, camera=self._camera_id)
        return np.asarray(self._renderer.render(), dtype=np.uint8).copy()

    def render_depth(self, data) -> np.ndarray:
        """Return one metric-depth frame."""
        self._renderer.enable_depth_rendering()
        self._renderer.update_scene(data, camera=self._camera_id)
        depth_raw = np.asarray(self._renderer.render(), dtype=np.float32).copy()
        self._renderer.disable_depth_rendering()

        extent = float(data.model.stat.extent)
        near = float(data.model.vis.map.znear) * extent
        far = float(data.model.vis.map.zfar) * extent
        with np.errstate(divide="ignore", invalid="ignore"):
            depth_m = near / (1.0 - depth_raw * (1.0 - near / far))
        return depth_m.astype(np.float32, copy=False)

    def render(self, data) -> tuple[np.ndarray, np.ndarray]:
        """Return `(rgb8, depth_m)` arrays for depth-enabled consumers."""
        return self.render_rgb(data), self.render_depth(data)

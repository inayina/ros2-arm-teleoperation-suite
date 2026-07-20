"""Tests for camera_bridge target-object joint mapping."""

from camera_bridge.object_sync import MANIPULABLE_OBJECTS, object_joint_name
from pathlib import Path
import xml.etree.ElementTree as ET


def test_object_joint_name_matches_mujoco_sim_convention() -> None:
    assert object_joint_name("object_red_box") == "red_box_joint"
    assert object_joint_name("object_blue_cylinder") == "blue_cylinder_joint"
    assert object_joint_name("object_green_sphere") == "green_sphere_joint"


def test_manipulable_objects_cover_sorting_targets() -> None:
    assert MANIPULABLE_OBJECTS == (
        "object_red_box",
        "object_blue_cylinder",
        "object_green_sphere",
    )


def test_scene_camera_is_fixed_in_worldbody() -> None:
    root = ET.parse(Path("config/models/franka_panda.xml")).getroot()
    worldbody = root.find("worldbody")
    assert worldbody is not None
    assert worldbody.find("./camera[@name='scene_camera']") is not None
    nested_scene = worldbody.findall(".//body//camera[@name='scene_camera']")
    assert nested_scene == []


def test_camera_parameter_poll_is_async() -> None:
    source = Path(
        "src/camera_bridge/camera_bridge/camera_bridge_node.py"
    ).read_text(encoding="utf-8")
    assert "call_async(request)" in source
    assert "_get_params_client.call(request)" not in source


def test_scene_rgb_only_path_skips_depth_render() -> None:
    source = Path(
        "src/camera_bridge/camera_bridge/camera_bridge_node.py"
    ).read_text(encoding="utf-8")
    assert "needs_depth = self.publish_depth or self.tactile_mode" in source
    assert "rgb = self._camera.render_rgb(self._data)" in source


def test_camera_publish_path_has_monotonic_burst_gate() -> None:
    source = Path(
        "src/camera_bridge/camera_bridge/camera_bridge_node.py"
    ).read_text(encoding="utf-8")
    assert "now_wall_s = time.monotonic()" in source
    assert "self._min_publish_period_s * 0.9" in source


def test_mujoco_publish_path_preserves_renderer_orientation() -> None:
    source = Path(
        "src/camera_bridge/camera_bridge/camera_bridge_node.py"
    ).read_text(encoding="utf-8")

    assert "np.ascontiguousarray(rgb)" in source
    assert "np.flipud(rgb)" not in source
    assert "np.flipud(depth_arr)" not in source


def test_production_camera_can_fail_closed_instead_of_drawing_frames() -> None:
    source = Path(
        "src/camera_bridge/camera_bridge/camera_bridge_node.py"
    ).read_text(encoding="utf-8")
    assert "self._camera is None and not self.synthetic_fallback" in source
    assert "refusing synthetic camera data" in source


def test_batch_preflight_requires_real_mujoco_scene_renderer() -> None:
    source = Path("scripts/run_batch_preflight_smoke.sh").read_text(encoding="utf-8")
    assert 'BATCH_PREFLIGHT_SCENE_USE_MUJOCO_RENDERER:-true' in source
    assert "wait_for_scene_renderer" in source
    assert "refusing synthetic training video" in source

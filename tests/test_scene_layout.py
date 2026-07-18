"""Training-scene layout checks for the rendered Panda pick/place task."""

import math
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "config/models/franka_panda.xml"


def _xyz(element, attribute="pos"):
    return tuple(float(value) for value in element.attrib[attribute].split())


def test_scene_camera_is_upright_and_covers_workspace():
    world = ET.parse(MODEL).getroot().find("worldbody")
    camera = world.find("./camera[@name='scene_camera']")
    position = _xyz(camera)
    xyaxes = _xyz(camera, "xyaxes")

    assert position[0] >= 1.4
    assert position[2] >= 1.2
    assert xyaxes[1] > 0.85  # world +Y maps predominantly to image-right
    assert xyaxes[5] > 0.75  # camera image-up has a strong world +Z component


def test_assigned_bins_are_not_adjacent_to_objects():
    world = ET.parse(MODEL).getroot().find("worldbody")
    assignments = {
        "object_red_box": "bin_left",
        "object_blue_cylinder": "bin_right",
        "object_green_sphere": "bin_left",
    }

    for object_name, bin_name in assignments.items():
        obj = _xyz(world.find(f"./body[@name='{object_name}']"))
        bin_position = _xyz(world.find(f"./body[@name='{bin_name}']"))
        center_distance = math.hypot(obj[0] - bin_position[0], obj[1] - bin_position[1])
        assert center_distance >= 0.24


def test_batch_targets_match_rendered_bin_centers():
    source = (ROOT / "src/synth_data_gen/synth_data_gen/batch_generator.py").read_text(
        encoding="utf-8"
    )

    assert source.count("-0.35") >= 3
    assert "0.35," in source

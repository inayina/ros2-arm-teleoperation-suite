import json
from pathlib import Path
import runpy

import pytest

_ARCHIVE = runpy.run_path(
    str(Path(__file__).resolve().parents[1] / "scripts" / "episode_archive.py")
)
_copy_v21_media = _ARCHIVE["_copy_v21_media"]
from lerobot_recorder.lerobot_v21_dataset import video_episode_path


def _write_info(root: Path, features: dict) -> None:
    meta = root / "meta"
    meta.mkdir(parents=True)
    (meta / "info.json").write_text(
        json.dumps({"codebase_version": "v2.1", "features": features}),
        encoding="utf-8",
    )


def test_copy_v21_media_accepts_declared_scene_only_dataset(tmp_path):
    source = tmp_path / "source"
    dest = tmp_path / "dest"
    _write_info(
        source,
        {"observation.images.scene": {"dtype": "video", "shape": [240, 320, 3]}},
    )
    scene = video_episode_path(source, "observation.images.scene", 0)
    scene.parent.mkdir(parents=True)
    scene.write_bytes(b"scene-video")

    _copy_v21_media(source, 0, dest, 4)

    copied = video_episode_path(dest, "observation.images.scene", 4)
    assert copied.read_bytes() == b"scene-video"
    assert not video_episode_path(dest, "observation.images.wrist", 4).exists()


def test_copy_v21_media_rejects_missing_declared_video(tmp_path):
    source = tmp_path / "source"
    _write_info(
        source,
        {"observation.images.scene": {"dtype": "video", "shape": [240, 320, 3]}},
    )

    with pytest.raises(FileNotFoundError, match="observation.images.scene"):
        _copy_v21_media(source, 0, tmp_path / "dest", 0)

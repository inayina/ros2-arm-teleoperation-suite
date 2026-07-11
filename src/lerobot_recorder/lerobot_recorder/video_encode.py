"""Encode RGB frame stacks to H.264 MP4 via ffmpeg."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import numpy as np

RGB_IMAGE_KEYS = (
    "observation.images.scene",
    "observation.images.wrist",
)
DEFAULT_FPS = 30.0


def ffmpeg_available() -> bool:
    return shutil.which("ffmpeg") is not None


def ffprobe_frame_count(video_path: Path) -> int:
    if not shutil.which("ffprobe"):
        raise RuntimeError("ffprobe not found on PATH")
    output = subprocess.check_output(
        [
            "ffprobe",
            "-v",
            "error",
            "-select_streams",
            "v:0",
            "-count_packets",
            "-show_entries",
            "stream=nb_read_packets",
            "-of",
            "csv=p=0",
            str(video_path),
        ],
        text=True,
    ).strip()
    return int(output)


def encode_rgb_frames_to_mp4(
    frames: list[np.ndarray],
    output_path: Path,
    *,
    fps: float = DEFAULT_FPS,
) -> None:
    if not frames:
        raise ValueError("No RGB frames provided for video encoding")
    if not ffmpeg_available():
        raise RuntimeError("ffmpeg not found on PATH; install ffmpeg to write MP4 episodes")

    first = np.asarray(frames[0], dtype=np.uint8)
    if first.ndim != 3 or first.shape[-1] != 3:
        raise ValueError(f"Expected HxWx3 uint8 frames, got shape {first.shape}")

    height, width = int(first.shape[0]), int(first.shape[1])
    output_path.parent.mkdir(parents=True, exist_ok=True)
    command = [
        "ffmpeg",
        "-y",
        "-hide_banner",
        "-loglevel",
        "error",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "rgb24",
        "-s",
        f"{width}x{height}",
        "-r",
        str(fps),
        "-i",
        "-",
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        str(output_path),
    ]
    with subprocess.Popen(command, stdin=subprocess.PIPE) as process:
        assert process.stdin is not None
        for frame in frames:
            array = np.asarray(frame, dtype=np.uint8)
            if array.shape[:2] != (height, width):
                raise ValueError("All frames must share the same HxW shape")
            process.stdin.write(array.tobytes())
        process.stdin.close()
        return_code = process.wait()
    if return_code != 0:
        raise RuntimeError(f"ffmpeg failed with exit code {return_code}")

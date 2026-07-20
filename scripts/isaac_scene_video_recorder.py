#!/usr/bin/env python3
"""Record /camera/color/image_raw into an MP4 for Isaac ACT evaluation evidence.

Runs alongside policy inference; exits cleanly on SIGINT/SIGTERM or max duration.
Does not block the policy path if ffmpeg is unavailable (writes a note file).
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def image_to_rgb(message: Image) -> np.ndarray:
    channels = {
        'rgb8': (3, False, False),
        'bgr8': (3, True, False),
        'rgba8': (4, False, True),
        'bgra8': (4, True, True),
        'mono8': (1, False, False),
    }
    if message.encoding not in channels:
        raise ValueError(f'unsupported encoding {message.encoding!r}')
    channel_count, reverse, drop_alpha = channels[message.encoding]
    row_bytes = int(message.width) * channel_count
    raw = np.frombuffer(bytes(message.data), dtype=np.uint8)
    required = int(message.height) * int(message.step)
    rows = raw[:required].reshape(int(message.height), int(message.step))
    image = rows[:, :row_bytes].reshape(
        int(message.height), int(message.width), channel_count
    )
    if channel_count == 1:
        image = np.repeat(image, 3, axis=2)
    elif drop_alpha:
        image = image[:, :, :3]
    if reverse:
        image = image[:, :, ::-1]
    return np.ascontiguousarray(image)


class SceneVideoRecorder(Node):
    def __init__(self, topic: str, max_frames: int) -> None:
        super().__init__('isaac_scene_video_recorder')
        self._frames: list[np.ndarray] = []
        self._max_frames = max(1, int(max_frames))
        self._done = False
        self.create_subscription(
            Image, topic, self._on_image, qos_profile_sensor_data
        )

    def _on_image(self, message: Image) -> None:
        if self._done or len(self._frames) >= self._max_frames:
            self._done = True
            return
        try:
            self._frames.append(image_to_rgb(message))
        except ValueError as exc:
            self.get_logger().warn(f'skip frame: {exc}')


def encode_mp4(frames: list[np.ndarray], output: Path, fps: float) -> None:
    import shutil
    import subprocess

    if not frames:
        raise RuntimeError('no frames captured')
    if shutil.which('ffmpeg') is None:
        raise RuntimeError('ffmpeg not on PATH')
    height, width = frames[0].shape[:2]
    cmd = [
        'ffmpeg', '-y', '-f', 'rawvideo', '-vcodec', 'rawvideo',
        '-pix_fmt', 'rgb24', '-s', f'{width}x{height}', '-r', str(fps),
        '-i', '-', '-an', '-vcodec', 'libx264', '-pix_fmt', 'yuv420p',
        str(output),
    ]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    assert proc.stdin is not None
    for frame in frames:
        proc.stdin.write(frame.tobytes())
    proc.stdin.close()
    _, err = proc.communicate(timeout=120)
    if proc.returncode != 0:
        raise RuntimeError(err.decode('utf-8', errors='replace')[-500:])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--topic', default='/camera/color/image_raw')
    parser.add_argument('--fps', type=float, default=10.0)
    parser.add_argument('--max-frames', type=int, default=400)
    parser.add_argument('--max-duration-s', type=float, default=200.0)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    note = args.output.with_suffix('.note.txt')

    stop = {'flag': False}

    def _stop(_signum=None, _frame=None) -> None:
        stop['flag'] = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    rclpy.init()
    node = SceneVideoRecorder(args.topic, args.max_frames)
    deadline = time.monotonic() + max(1.0, float(args.max_duration_s))
    frames: list[np.ndarray] = []
    try:
        while rclpy.ok() and not stop['flag'] and not node._done:
            if time.monotonic() >= deadline:
                break
            try:
                rclpy.spin_once(node, timeout_sec=0.05)
            except Exception as exc:  # noqa: BLE001 — SIGTERM/shutdown mid-spin
                # ExternalShutdownException and siblings must not skip encode.
                print(f'ISAAC_SCENE_VIDEO_SPIN_STOP={type(exc).__name__}', flush=True)
                break
    finally:
        frames = list(node._frames)
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass

    try:
        encode_mp4(frames, args.output, args.fps)
    except Exception as exc:  # noqa: BLE001 — evidence path must not crash suite
        try:
            note.write_text(f'video_failed: {exc}\nframes={len(frames)}\n', encoding='utf-8')
        except Exception:  # noqa: BLE001
            pass
        try:
            print(f'ISAAC_SCENE_VIDEO_FAILED={exc}', flush=True)
        except Exception:  # noqa: BLE001 — parent may have closed stdout after SIGTERM
            pass
        return 0

    # Encode succeeded; never let note/stdout failures rewrite this as video_failed.
    try:
        note.write_text(f'frames={len(frames)}\n', encoding='utf-8')
    except Exception:  # noqa: BLE001
        pass
    try:
        print(f'ISAAC_SCENE_VIDEO={args.output} frames={len(frames)}', flush=True)
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""Capture M6 perception evidence frames and dataset schema screenshots.

The script is intended to run while the M6 stack is active. It saves one real
frame from each camera topic, then optionally falls back to a LeRobot dataset
for missing frames and for the dataset features PNG.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
REQUIRED_FEATURES = {
    "observation.images.scene",
    "observation.images.wrist",
    "observation.images.tactile_left",
    "observation.images.tactile_right",
    "observation.depth.scene",
}
SYNC_FIELDS = [
    ("scene RGB", "observation.images.scene"),
    ("scene depth", "observation.depth.scene"),
    ("wrist RGB", "observation.images.wrist"),
    ("tactile left", "observation.images.tactile_left"),
    ("tactile right", "observation.images.tactile_right"),
    ("joint state", "observation.state"),
    ("ee pose", "observation.ee_pose"),
    ("object pose", "observation.object_pose"),
    ("force torque", "observation.ft"),
    ("action", "action"),
]
IMAGE_FIELDS = {
    "observation.images.scene": "camera_rgb_view.png",
    "observation.images.wrist": "wrist_camera_view.png",
    "observation.images.tactile_left": "tactile_left_view.png",
    "observation.images.tactile_right": "tactile_right_view.png",
}
TOPICS = {
    "/camera/color/image_raw": "camera_rgb_view.png",
    "/camera/wrist/color/image_raw": "wrist_camera_view.png",
    "/camera/tactile_left/image_raw": "tactile_left_view.png",
    "/camera/tactile_right/image_raw": "tactile_right_view.png",
}


def _font(size: int, bold: bool = False) -> ImageFont.ImageFont:
    candidates = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
        if bold
        else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf"
        if bold
        else "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
    ]
    for path in candidates:
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


F_TITLE = _font(28, True)
F_BODY = _font(18)
F_MONO = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 17)
F_MONO_SMALL = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 14)


def _image_msg_to_array(msg) -> np.ndarray:
    encoding = msg.encoding.lower()
    if encoding in ("rgb8", "bgr8"):
        channels = 3
        row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        arr = row[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
        if encoding == "bgr8":
            arr = arr[:, :, ::-1]
        return arr.copy()
    if encoding in ("rgba8", "bgra8"):
        channels = 4
        row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        arr = row[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
        if encoding == "bgra8":
            arr = arr[:, :, [2, 1, 0, 3]]
        return arr[:, :, :3].copy()
    if encoding in ("mono8", "8uc1"):
        row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        gray = row[:, : msg.width]
        return np.repeat(gray[:, :, None], 3, axis=2).copy()
    if encoding == "16uc1":
        stride = msg.step // np.dtype(np.uint16).itemsize
        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, stride)
        return _depth_to_rgb(depth[:, : msg.width].astype(np.float32))
    if encoding == "32fc1":
        stride = msg.step // np.dtype(np.float32).itemsize
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, stride)
        return _depth_to_rgb(depth[:, : msg.width])
    raise ValueError(f"unsupported image encoding: {msg.encoding}")


def _depth_to_rgb(depth: np.ndarray) -> np.ndarray:
    finite = np.isfinite(depth)
    if not np.any(finite):
        gray = np.zeros(depth.shape, dtype=np.uint8)
    else:
        lo, hi = np.percentile(depth[finite], [2, 98])
        scale = max(float(hi - lo), 1e-6)
        gray = np.clip((depth - lo) / scale, 0.0, 1.0)
        gray = np.nan_to_num(gray, nan=0.0, posinf=1.0, neginf=0.0)
        gray = (gray * 255.0).astype(np.uint8)
    return np.repeat(gray[:, :, None], 3, axis=2)


def _save_array(arr: np.ndarray, path: Path) -> None:
    arr = np.asarray(arr)
    if arr.ndim == 2:
        arr = _depth_to_rgb(arr.astype(np.float32))
    if arr.ndim == 3 and arr.shape[2] == 4:
        arr = arr[:, :, :3]
    if arr.dtype != np.uint8:
        arr = np.clip(arr, 0, 255).astype(np.uint8)
    if arr.ndim != 3 or arr.shape[2] != 3:
        raise ValueError(f"cannot save image with shape {arr.shape}")
    path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(arr, mode="RGB").save(path)


def _discover_dataset(explicit: str | None) -> Path | None:
    if explicit:
        path = Path(explicit)
        return path if path.exists() else None
    candidates = [
        ROOT / ".m6_validation/episodes/episode_000000/train",
        ROOT / "data/episodes/episode_000000/train",
    ]
    return next((p for p in candidates if p.exists()), None)


class _CaptureResult:
    def __init__(self) -> None:
        self.saved: set[str] = set()
        self.errors: list[str] = []


def _capture_live(output_dir: Path, timeout_s: float) -> _CaptureResult:
    result = _CaptureResult()
    default_log_dir = ROOT / ".m6_validation/ros_logs"
    default_log_dir.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("ROS_LOG_DIR", str(default_log_dir))
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image as ImageMsg
    except Exception as exc:
        result.errors.append(f"ROS image capture unavailable: {exc}")
        return result

    class FrameCapture(Node):
        def __init__(self) -> None:
            super().__init__("m6_media_frame_capture")
            self.pending = dict(TOPICS)
            for topic in TOPICS:
                self.create_subscription(
                    ImageMsg,
                    topic,
                    lambda msg, topic=topic: self._on_image(topic, msg),
                    qos_profile_sensor_data,
                )

        def _on_image(self, topic: str, msg) -> None:
            filename = self.pending.pop(topic, None)
            if filename is None:
                return
            try:
                _save_array(_image_msg_to_array(msg), output_dir / filename)
                result.saved.add(filename)
            except Exception as exc:
                result.errors.append(f"{topic}: {exc}")

    rclpy.init(args=None)
    node = FrameCapture()
    deadline = time.monotonic() + timeout_s
    try:
        while rclpy.ok() and node.pending and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        missing = sorted(node.pending)
        node.destroy_node()
        rclpy.shutdown()
    if missing:
        result.errors.append("missing live frames: " + ", ".join(missing))
    return result


def _load_dataset(path: Path):
    try:
        from datasets import load_from_disk
    except Exception as exc:
        raise RuntimeError(f"datasets package unavailable: {exc}") from exc
    return load_from_disk(str(path))


def _save_dataset_frames(ds, output_dir: Path, already_saved: set[str], overwrite: bool) -> list[str]:
    saved: list[str] = []
    if len(ds) == 0:
        return saved
    frame = ds[0]
    for field, filename in IMAGE_FIELDS.items():
        if not overwrite and filename in already_saved:
            continue
        if field not in ds.features:
            continue
        _save_array(np.asarray(frame[field]), output_dir / filename)
        saved.append(filename)
    return saved


def _render_features_png(ds, dataset_path: Path, output_path: Path) -> tuple[bool, list[str]]:
    keys = list(ds.features.keys())
    missing = sorted(REQUIRED_FEATURES.difference(keys))
    if missing:
        return False, missing

    width, height = 1280, 820
    bg = (9, 14, 20)
    panel = (18, 26, 36)
    text = (232, 238, 244)
    muted = (139, 154, 174)
    green = (91, 214, 138)
    yellow = (245, 196, 66)
    cyan = (82, 190, 255)

    img = Image.new("RGB", (width, height), bg)
    d = ImageDraw.Draw(img)
    d.rounded_rectangle((42, 36, width - 42, height - 38), radius=18, fill=panel, outline=(62, 82, 104), width=2)
    d.text((74, 64), "M6 LeRobot Dataset Features", fill=text, font=F_TITLE)
    d.text((76, 106), f"source: {dataset_path}", fill=muted, font=F_MONO_SMALL)
    d.text((76, 134), f"frames: {len(ds)}", fill=green, font=F_BODY)

    y = 178
    for key in keys:
        value = str(ds.features[key])
        color = yellow if "tactile" in key else (cyan if "images" in key or "depth" in key else text)
        d.text((84, y), key, fill=color, font=F_MONO)
        clipped = value if len(value) < 82 else value[:79] + "..."
        d.text((510, y), clipped, fill=muted, font=F_MONO_SMALL)
        y += 31
        if y > height - 96:
            d.text((84, y), "...", fill=muted, font=F_MONO)
            break

    d.rounded_rectangle((74, height - 82, width - 74, height - 56), radius=8, fill=(12, 20, 29))
    d.text(
        (92, height - 78),
        "required M6 fields present: scene, wrist, tactile_left, tactile_right, depth.scene",
        fill=green,
        font=F_MONO_SMALL,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    img.save(output_path)
    return True, []


def _render_sync_png(ds, dataset_path: Path, output_path: Path) -> tuple[bool, list[str]]:
    keys = set(ds.features.keys())
    missing = sorted(field for _, field in SYNC_FIELDS if field not in keys)
    if "timestamp" not in keys:
        missing.append("timestamp")
    if missing:
        return False, missing

    stamps = np.asarray(ds["timestamp"], dtype=np.float64)
    if stamps.size == 0:
        return False, ["non-empty dataset"]
    rel = stamps - stamps[0]
    duration = float(rel[-1]) if rel.size > 1 else 0.0
    mean_dt = float(np.mean(np.diff(stamps))) if stamps.size > 1 else 0.0
    mean_hz = (1.0 / mean_dt) if mean_dt > 0 else 0.0

    width, height = 1280, 820
    bg = (9, 14, 20)
    panel = (18, 26, 36)
    row_bg = (24, 34, 46)
    text = (232, 238, 244)
    muted = (139, 154, 174)
    green = (91, 214, 138)
    cyan = (82, 190, 255)
    yellow = (245, 196, 66)

    img = Image.new("RGB", (width, height), bg)
    d = ImageDraw.Draw(img)
    d.rounded_rectangle((42, 36, width - 42, height - 38), radius=18, fill=panel, outline=(62, 82, 104), width=2)
    d.text((74, 64), "M6 Multimodal Sync Evidence", fill=text, font=F_TITLE)
    d.text((76, 106), f"source: {dataset_path}", fill=muted, font=F_MONO_SMALL)
    d.text(
        (76, 136),
        f"frames: {len(ds)}   duration: {duration:.3f}s   mean dataset cadence: {mean_hz:.2f} Hz",
        fill=green,
        font=F_BODY,
    )

    x0, x1 = 570, width - 88
    y0 = 206
    row_h = 47
    span = max(duration, 1e-6)
    tick_xs = [x0 + int(float(t) / span * (x1 - x0)) if duration > 0 else x0 for t in rel]

    d.text((84, 174), "stream field", fill=muted, font=F_MONO_SMALL)
    d.text((x0, 174), "recorded frame ticks from synchronized LeRobot rows", fill=muted, font=F_MONO_SMALL)
    for idx, (label, field) in enumerate(SYNC_FIELDS):
        y = y0 + idx * row_h
        color = yellow if "tactile" in field else (cyan if "images" in field or "depth" in field else text)
        d.rounded_rectangle((74, y - 10, width - 74, y + 28), radius=8, fill=row_bg)
        d.text((90, y - 1), label, fill=color, font=F_MONO_SMALL)
        d.text((238, y - 1), field, fill=muted, font=F_MONO_SMALL)
        d.line((x0, y + 9, x1, y + 9), fill=(67, 82, 100), width=1)
        for x in tick_xs:
            d.rectangle((x - 2, y + 3, x + 2, y + 15), fill=color)

    footer_y = height - 118
    d.rounded_rectangle((74, footer_y, width - 74, footer_y + 60), radius=10, fill=(12, 20, 29))
    d.text(
        (92, footer_y + 12),
        "This plot is generated from the saved LeRobotDataset rows; each tick is one synchronized recorder frame.",
        fill=green,
        font=F_MONO_SMALL,
    )
    d.text(
        (92, footer_y + 34),
        "It proves schema-level multimodal alignment, not raw per-topic transport jitter.",
        fill=muted,
        font=F_MONO_SMALL,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    img.save(output_path)
    return True, []


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default=str(ROOT / "media/m6"), help="directory for PNG outputs")
    parser.add_argument("--dataset", default=None, help="LeRobot dataset train directory")
    parser.add_argument("--timeout", type=float, default=8.0, help="seconds to wait for live image topics")
    parser.add_argument("--no-live", action="store_true", help="skip ROS topic capture")
    parser.add_argument(
        "--overwrite-from-dataset",
        action="store_true",
        help="overwrite image PNGs from dataset even if live capture already saved them",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    saved: set[str] = set()
    warnings: list[str] = []

    if not args.no_live:
        live = _capture_live(output_dir, args.timeout)
        saved.update(live.saved)
        warnings.extend(live.errors)

    dataset_path = _discover_dataset(args.dataset)
    if dataset_path is None:
        warnings.append("no LeRobot dataset found for schema/frame fallback")
    else:
        try:
            ds = _load_dataset(dataset_path)
            dataset_saved = _save_dataset_frames(
                ds,
                output_dir,
                already_saved=saved,
                overwrite=args.overwrite_from_dataset,
            )
            saved.update(dataset_saved)
            ok, missing = _render_features_png(
                ds,
                dataset_path,
                output_dir / "lerobot_dataset_features.png",
            )
            if ok:
                saved.add("lerobot_dataset_features.png")
            else:
                warnings.append(
                    "dataset is missing required M6 fields; not overwriting features PNG: "
                    + ", ".join(missing)
                )
            ok, missing = _render_sync_png(
                ds,
                dataset_path,
                output_dir / "multimodal_sync.png",
            )
            if ok:
                saved.add("multimodal_sync.png")
            else:
                warnings.append(
                    "dataset is missing required sync fields; not overwriting multimodal_sync.png: "
                    + ", ".join(missing)
                )
        except Exception as exc:
            warnings.append(f"dataset media fallback failed: {exc}")

    for filename in sorted(saved):
        print(f"[capture_m6_media] wrote {output_dir / filename}")
    for warning in warnings:
        print(f"[capture_m6_media] WARN: {warning}", file=sys.stderr)

    manifest = {
        "generated_at_unix": time.time(),
        "output_dir": str(output_dir),
        "dataset_path": str(dataset_path) if dataset_path else None,
        "fresh_files": sorted(saved),
        "warnings": warnings,
        "note": "Only fresh_files are valid for this capture run; pre-existing PNGs may be stale.",
    }
    (output_dir / "capture_manifest.json").write_text(
        json.dumps(manifest, indent=2),
        encoding="utf-8",
    )

    core = {
        "camera_rgb_view.png",
        "wrist_camera_view.png",
        "tactile_left_view.png",
        "tactile_right_view.png",
        "lerobot_dataset_features.png",
    }
    missing_core = sorted(core.difference(saved))
    if missing_core:
        print(
            "[capture_m6_media] missing core media: " + ", ".join(missing_core),
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

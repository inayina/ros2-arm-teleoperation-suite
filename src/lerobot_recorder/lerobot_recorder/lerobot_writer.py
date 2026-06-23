"""Write a buffered episode to LeRobot dataset format.

Falls back to a .npz dump when `datasets` is unavailable so the recorder is
always functional. M6 task: emit a proper LeRobotDataset v2 directory layout.
"""
import os
import time

import numpy as np

try:
    from datasets import Dataset, Features, Sequence, Value, Image as HFImage  # noqa: F401
    _HAS_DATASETS = True
except Exception:
    _HAS_DATASETS = False


def write_episode(out_dir: str, episode_index: int, frames: list, task: str = "teleop") -> str:
    """frames: list of dicts with the per-step modalities. Returns output path."""
    os.makedirs(out_dir, exist_ok=True)
    ep_dir = os.path.join(out_dir, f"episode_{episode_index:06d}")
    os.makedirs(ep_dir, exist_ok=True)

    if _HAS_DATASETS:
        # Keep image arrays as lists; datasets can store them as arrays.
        ds = Dataset.from_list(frames)
        ds.save_to_disk(ep_dir)
    else:
        flat = {}
        for key in frames[0].keys():
            try:
                flat[key] = np.asarray([f[key] for f in frames], dtype=object)
            except Exception:
                flat[key] = np.asarray([f[key] for f in frames])
        np.savez_compressed(os.path.join(ep_dir, "frames.npz"), **flat)
        with open(os.path.join(ep_dir, "meta.txt"), "w") as fh:
            fh.write(f"task={task}\nframes={len(frames)}\nsaved={time.time()}\n")
    return ep_dir

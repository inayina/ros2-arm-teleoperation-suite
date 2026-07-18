"""LeRobot ACT checkpoint loading and image/state preprocessing for deployment."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np


SCENE_KEY = 'observation.images.scene'
IMAGE_SIZE = 224
IMAGE_MEAN = np.asarray([0.485, 0.456, 0.406], dtype=np.float32)
IMAGE_STD = np.asarray([0.229, 0.224, 0.225], dtype=np.float32)


def _build_policy(metadata: dict[str, Any], device: str):
    from lerobot.configs import FeatureType, PolicyFeature
    from lerobot.policies.act.configuration_act import ACTConfig
    from lerobot.policies.act.modeling_act import ACTPolicy

    config = ACTConfig(
        input_features={
            'observation.state': PolicyFeature(
                type=FeatureType.STATE, shape=(int(metadata['state_dim']),)
            ),
            SCENE_KEY: PolicyFeature(
                type=FeatureType.VISUAL, shape=(3, IMAGE_SIZE, IMAGE_SIZE)
            ),
        },
        output_features={
            'action': PolicyFeature(
                type=FeatureType.ACTION, shape=(int(metadata['action_dim']),)
            ),
        },
        device=device,
        chunk_size=int(metadata['chunk_size']),
        n_action_steps=int(metadata['chunk_size']),
        n_obs_steps=int(metadata['n_obs_steps']),
        vision_backbone='resnet18',
        pretrained_backbone_weights=None,
    )
    policy = ACTPolicy(config)
    policy.to(device)
    return policy


def load_scene_act_checkpoint(path: Path, device: str):
    import torch

    payload = torch.load(path, map_location=device, weights_only=False)
    metadata = payload.get('metadata', {})
    required = {
        'policy_type': 'scene_act_lerobot',
        'action_type': 'ee_delta_gripper',
        'state_dim': 8,
        'action_dim': 7,
        'n_obs_steps': 1,
    }
    for key, expected in required.items():
        if metadata.get(key) != expected:
            raise ValueError(
                f'incompatible checkpoint metadata {key}={metadata.get(key)!r}; '
                f'expected {expected!r}'
            )
    normalization = metadata.get('normalization', {})
    for key, expected_dim in (
        ('state_mean', 8), ('state_std', 8),
        ('action_mean', 7), ('action_std', 7),
    ):
        if len(normalization.get(key, [])) != expected_dim:
            raise ValueError(f'incompatible checkpoint normalization: {key}')
    policy = _build_policy(metadata, device)
    policy.load_state_dict(payload['state_dict'])
    policy.eval()
    return policy, metadata


def preprocess_rgb(rgb: np.ndarray, torch):
    """Match the training short-side resize and center-crop exactly."""
    from PIL import Image

    if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
        raise ValueError(f'expected uint8 RGB [H,W,3], got {rgb.dtype} {rgb.shape}')
    image = Image.fromarray(rgb, mode='RGB')
    width, height = image.size
    scale = max(IMAGE_SIZE / width, IMAGE_SIZE / height)
    resized = image.resize(
        (round(width * scale), round(height * scale)),
        resample=Image.Resampling.BILINEAR,
    )
    left = (resized.width - IMAGE_SIZE) // 2
    top = (resized.height - IMAGE_SIZE) // 2
    cropped = resized.crop((left, top, left + IMAGE_SIZE, top + IMAGE_SIZE))
    array = np.asarray(cropped, dtype=np.float32) / 255.0
    array = (array - IMAGE_MEAN) / IMAGE_STD
    return torch.from_numpy(np.transpose(array, (2, 0, 1)).copy())


class SceneACTRuntime:
    """Small stateful inference wrapper returning one denormalized action."""

    def __init__(self, checkpoint: Path, device: str) -> None:
        import torch

        self.torch = torch
        self.device = device
        self.policy, self.metadata = load_scene_act_checkpoint(checkpoint, device)
        normalization = self.metadata['normalization']
        self.state_mean = torch.tensor(
            normalization['state_mean'], dtype=torch.float32, device=device
        )
        self.state_std = torch.tensor(
            normalization['state_std'], dtype=torch.float32, device=device
        )
        self.action_mean = torch.tensor(
            normalization['action_mean'], dtype=torch.float32, device=device
        )
        self.action_std = torch.tensor(
            normalization['action_std'], dtype=torch.float32, device=device
        )

    def infer(self, state: list[float], rgb: np.ndarray) -> list[float]:
        if len(state) != 8:
            raise ValueError(f'expected observation.state[8], got [{len(state)}]')
        state_tensor = self.torch.tensor(
            state, dtype=self.torch.float32, device=self.device
        )
        state_tensor = (state_tensor - self.state_mean) / self.state_std
        image_tensor = preprocess_rgb(rgb, self.torch).to(self.device)
        batch = {
            'observation.state': state_tensor.unsqueeze(0),
            SCENE_KEY: image_tensor.unsqueeze(0),
        }
        with self.torch.no_grad():
            normalized = self.policy.predict_action_chunk(batch)[0, 0]
            action = normalized * self.action_std + self.action_mean
        return action.detach().cpu().to(self.torch.float64).tolist()

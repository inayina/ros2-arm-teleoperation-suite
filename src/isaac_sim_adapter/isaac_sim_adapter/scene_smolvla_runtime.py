"""Recovery-v3 SmolVLA runtime for bounded Isaac S4 (absolute EEF + gripper).

Loads base SmolVLA weights + LoRA adapter with local VLM paths, then exposes
``select_action`` at deploy ``n_action_steps`` from the checked-in S4 contract
(chunk 10 / K 5 by default).

Does not launch Isaac, does not claim task success, and does not retrain.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import tempfile
from typing import Any, Sequence

from isaac_sim_adapter.s4_runtime_contract import assert_runtime_matches_contract
from isaac_sim_adapter.s4_runtime_contract import load_s4_runtime_contract
import numpy as np

DEFAULT_TASK = 'pick up the red box and place it in the left bin\n'
_S4_CONTRACT = load_s4_runtime_contract()
SCENE_IMAGE_KEY = _S4_CONTRACT.camera_key
WRIST_IMAGE_KEY = 'observation.images.wrist'
CAMERA_KEYS = tuple(_S4_CONTRACT.camera_keys)
IMAGE_HW = (_S4_CONTRACT.image_height, _S4_CONTRACT.image_width)


def _patch_vlm_tokenizer(pre: dict[str, Any], vlm_dir: Path) -> dict[str, Any]:
    for step in pre.get('steps', []):
        step_cfg = step.get('config') or {}
        if step_cfg.get('tokenizer_name'):
            step_cfg['tokenizer_name'] = str(vlm_dir.resolve())
    return pre


def prepare_lora_workdir(
    *,
    base_dir: Path,
    lora_dir: Path,
    vlm_dir: Path,
) -> Path:
    """Materialize a writable workdir: base weights + LoRA config/processors."""
    base_dir = Path(base_dir).expanduser().resolve()
    lora_dir = Path(lora_dir).expanduser().resolve()
    vlm_dir = Path(vlm_dir).expanduser().resolve()
    if not (base_dir / 'model.safetensors').is_file():
        raise FileNotFoundError(f'missing base weights: {base_dir / "model.safetensors"}')
    if not (lora_dir / 'adapter_model.safetensors').is_file():
        raise FileNotFoundError(
            f'missing LoRA adapter: {lora_dir / "adapter_model.safetensors"}'
        )
    if not vlm_dir.is_dir():
        raise FileNotFoundError(f'missing VLM dir: {vlm_dir}')

    tmp = Path(tempfile.mkdtemp(prefix='smolvla_s4_workdir_'))
    os.symlink((base_dir / 'model.safetensors').resolve(), tmp / 'model.safetensors')
    for name in (
        'policy_preprocessor.json',
        'policy_postprocessor.json',
        'policy_preprocessor_step_5_normalizer_processor.safetensors',
        'policy_postprocessor_step_0_unnormalizer_processor.safetensors',
    ):
        src = lora_dir / name
        if not src.is_file():
            raise FileNotFoundError(f'LoRA missing processor asset: {src}')
        os.symlink(src.resolve(), tmp / name)

    cfg = json.loads((lora_dir / 'config.json').read_text(encoding='utf-8'))
    cfg['vlm_model_name'] = str(vlm_dir)
    (tmp / 'config.json').write_text(json.dumps(cfg, indent=2) + '\n', encoding='utf-8')

    pre = json.loads((tmp / 'policy_preprocessor.json').read_text(encoding='utf-8'))
    (tmp / 'policy_preprocessor.json').unlink()
    pre = _patch_vlm_tokenizer(pre, vlm_dir)
    (tmp / 'policy_preprocessor.json').write_text(
        json.dumps(pre, indent=2) + '\n', encoding='utf-8'
    )
    return tmp


def rgb_uint8_to_chw01(rgb: np.ndarray, height: int, width: int) -> np.ndarray:
    """Resize RGB uint8 HxWx3 → float32 CHW in [0, 1]."""
    if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
        raise ValueError(f'expected uint8 RGB [H,W,3], got {rgb.dtype} {rgb.shape}')
    try:
        import cv2
    except ImportError as exc:  # pragma: no cover - deploy env always has cv2/PIL
        raise RuntimeError('OpenCV required to resize SmolVLA camera frames') from exc
    resized = cv2.resize(rgb, (width, height), interpolation=cv2.INTER_AREA)
    return np.transpose(resized.astype(np.float32) / 255.0, (2, 0, 1))


def compose_state15(
    joints7: Sequence[float],
    ee_pose7: Sequence[float],
    gripper: float,
) -> np.ndarray:
    joints = np.asarray(joints7, dtype=np.float32).reshape(-1)
    ee = np.asarray(ee_pose7, dtype=np.float32).reshape(-1)
    if joints.shape[0] != 7:
        raise ValueError(f'joints must be [7], got {joints.shape}')
    if ee.shape[0] != 7:
        raise ValueError(f'ee_pose must be [7], got {ee.shape}')
    grip = np.asarray([float(gripper)], dtype=np.float32)
    return np.concatenate([joints, ee, grip], axis=0)


class SceneSmolVLARuntime:
    """Stateful SmolVLA wrapper: Recovery state[15] + scene/wrist RGB → action[8]."""

    def __init__(
        self,
        *,
        base_dir: Path,
        lora_dir: Path,
        vlm_dir: Path,
        device: str = 'cuda',
        n_action_steps: int | None = None,
        task: str = DEFAULT_TASK,
    ) -> None:
        # Offline before any transformers / hub import.
        os.environ.setdefault('HF_HUB_OFFLINE', '1')
        os.environ.setdefault('TRANSFORMERS_OFFLINE', '1')
        os.environ.setdefault('HF_DATASETS_OFFLINE', '1')
        os.environ.setdefault('HF_HUB_DISABLE_TELEMETRY', '1')

        import torch
        from lerobot.policies.factory import make_pre_post_processors
        from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
        from peft import PeftModel

        if device not in {'cpu', 'cuda'}:
            raise ValueError('device must be cpu or cuda')
        if device == 'cuda' and not torch.cuda.is_available():
            raise RuntimeError('device=cuda requested but CUDA is unavailable')

        self.torch = torch
        self.device = device
        self.task = task if task.endswith('\n') else f'{task}\n'
        self.workdir = prepare_lora_workdir(
            base_dir=base_dir, lora_dir=lora_dir, vlm_dir=vlm_dir
        )

        policy = SmolVLAPolicy.from_pretrained(
            str(self.workdir), local_files_only=True
        )
        policy = policy.to(device).eval()
        policy = PeftModel.from_pretrained(policy, str(Path(lora_dir).resolve()))
        policy.eval()
        cfg_obj = policy.config if hasattr(policy, 'config') else policy.base_model.config
        contract = load_s4_runtime_contract()
        chunk_size = int(getattr(cfg_obj, 'chunk_size', contract.chunk_size))
        if n_action_steps is None:
            steps = int(contract.n_action_steps)
        else:
            steps = int(n_action_steps)
            if steps < 1 or steps > chunk_size:
                raise ValueError(
                    f'n_action_steps must be in [1, {chunk_size}], got {steps}'
                )
        # Keep deploy K aligned with Recovery §8 / eval_gate_v3 execution.
        if hasattr(policy, 'config'):
            policy.config.n_action_steps = steps
        elif hasattr(policy, 'base_model') and hasattr(policy.base_model, 'config'):
            policy.base_model.config.n_action_steps = steps

        preprocess, postprocess = make_pre_post_processors(
            cfg_obj,
            str(self.workdir),
            preprocessor_overrides={'device_processor': {'device': str(device)}},
        )
        self.policy = policy
        self.preprocess = preprocess
        self.postprocess = postprocess
        self.metadata = {
            'policy_type': 'smolvla_recovery_v3',
            'action_type': 'absolute_eef_gripper',
            'policy_action_semantics': contract.policy_action_semantics,
            'state_dim': contract.state_dim,
            'action_dim': contract.action_dim,
            'chunk_size': chunk_size,
            'deploy_n_action_steps': steps,
            'camera_key': SCENE_IMAGE_KEY,
            'camera_keys': list(CAMERA_KEYS),
            's4_contract_version': contract.contract_version,
            'lora_dir': str(Path(lora_dir).resolve()),
            'base_dir': str(Path(base_dir).resolve()),
            'vlm_dir': str(Path(vlm_dir).resolve()),
            'workdir': str(self.workdir),
            'claims_task_success': False,
        }
        # Default deploy path must match the checked-in midstream contract.
        # Explicit n_action_steps overrides are allowed for diagnostics only.
        if n_action_steps is None:
            assert_runtime_matches_contract(
                chunk_size=chunk_size,
                n_action_steps=steps,
                state_dim=contract.state_dim,
                action_dim=contract.action_dim,
                policy_action_semantics=contract.policy_action_semantics,
                contract=contract,
            )
        elif chunk_size != contract.chunk_size:
            raise ValueError(
                f'checkpoint chunk_size={chunk_size} != contract '
                f'{contract.chunk_size}'
            )

    def reset(self) -> None:
        if hasattr(self.policy, 'reset'):
            self.policy.reset()
        elif hasattr(self.policy, 'base_model') and hasattr(
            self.policy.base_model, 'reset'
        ):
            self.policy.base_model.reset()

    def infer(
        self,
        state15: Sequence[float],
        rgb_uint8: np.ndarray,
        wrist_rgb_uint8: np.ndarray | None = None,
        *,
        task: str | None = None,
    ) -> list[float]:
        state = np.asarray(state15, dtype=np.float32).reshape(-1)
        want_state = int(self.metadata['state_dim'])
        want_action = int(self.metadata['action_dim'])
        if state.shape[0] != want_state:
            raise ValueError(f'expected state[{want_state}], got {state.shape}')
        img = rgb_uint8_to_chw01(rgb_uint8, IMAGE_HW[0], IMAGE_HW[1])
        instruction = self.task if task is None else (
            task if task.endswith('\n') else f'{task}\n'
        )
        if WRIST_IMAGE_KEY in CAMERA_KEYS and wrist_rgb_uint8 is None:
            raise ValueError('dual-camera S4 runtime requires wrist RGB input')
        batch_in = {
            'observation.state': self.torch.from_numpy(state).unsqueeze(0),
            SCENE_IMAGE_KEY: self.torch.from_numpy(img).unsqueeze(0),
            'task': [instruction],
        }
        if WRIST_IMAGE_KEY in CAMERA_KEYS:
            wrist_img = rgb_uint8_to_chw01(
                wrist_rgb_uint8, IMAGE_HW[0], IMAGE_HW[1]
            )
            batch_in[WRIST_IMAGE_KEY] = self.torch.from_numpy(
                wrist_img
            ).unsqueeze(0)
        batch = self.preprocess(batch_in)
        with self.torch.inference_mode():
            pred = self.policy.select_action(batch)
            if self.postprocess is not None:
                pred = self.postprocess(pred)
        values = pred.detach().float().cpu().numpy().reshape(-1)
        if values.shape[0] < want_action:
            raise ValueError(
                f'expected action[{want_action}], got {values.shape[0]}'
            )
        return [float(v) for v in values[:want_action]]

    def predict_chunk(
        self,
        state15: Sequence[float],
        rgb_uint8: np.ndarray,
        wrist_rgb_uint8: np.ndarray | None = None,
        *,
        task: str | None = None,
    ) -> list[list[float]]:
        """Return the native postprocessed SmolVLA action chunk."""
        state = np.asarray(state15, dtype=np.float32).reshape(-1)
        want_state = int(self.metadata['state_dim'])
        want_action = int(self.metadata['action_dim'])
        if state.shape[0] != want_state:
            raise ValueError(f'expected state[{want_state}], got {state.shape}')
        img = rgb_uint8_to_chw01(rgb_uint8, IMAGE_HW[0], IMAGE_HW[1])
        instruction = self.task if task is None else (
            task if task.endswith('\n') else f'{task}\n'
        )
        if WRIST_IMAGE_KEY in CAMERA_KEYS and wrist_rgb_uint8 is None:
            raise ValueError('dual-camera S4 runtime requires wrist RGB input')
        batch_in = {
            'observation.state': self.torch.from_numpy(state).unsqueeze(0),
            SCENE_IMAGE_KEY: self.torch.from_numpy(img).unsqueeze(0),
            'task': [instruction],
        }
        if WRIST_IMAGE_KEY in CAMERA_KEYS:
            wrist_img = rgb_uint8_to_chw01(
                wrist_rgb_uint8, IMAGE_HW[0], IMAGE_HW[1]
            )
            batch_in[WRIST_IMAGE_KEY] = self.torch.from_numpy(
                wrist_img
            ).unsqueeze(0)
        batch = self.preprocess(batch_in)
        with self.torch.inference_mode():
            if hasattr(self.policy, 'predict_action_chunk'):
                chunk = self.policy.predict_action_chunk(batch)
            elif hasattr(self.policy, 'base_model') and hasattr(
                self.policy.base_model, 'predict_action_chunk'
            ):
                chunk = self.policy.base_model.predict_action_chunk(batch)
            else:
                raise RuntimeError('SmolVLA policy lacks predict_action_chunk')
            if self.postprocess is not None:
                try:
                    chunk = self.postprocess(chunk)
                except Exception:
                    steps = [
                        self.postprocess(chunk[:, index, :])
                        for index in range(chunk.shape[1])
                    ]
                    chunk = self.torch.stack(steps, dim=1)
        values = chunk.detach().float().cpu().numpy()
        if values.ndim == 3:
            values = values[0]
        expected_chunk = int(self.metadata['chunk_size'])
        if values.ndim != 2 or values.shape[0] < expected_chunk:
            raise ValueError(
                f'expected chunk[{expected_chunk}, {want_action}], got '
                f'{values.shape}'
            )
        if values.shape[1] < want_action:
            raise ValueError(f'expected action[{want_action}], got {values.shape}')
        return [
            [float(value) for value in values[index, :want_action]]
            for index in range(expected_chunk)
        ]

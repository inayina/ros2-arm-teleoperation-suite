"""Remote SmolVLA PolicyBackend client over a loopback/SSH HTTP tunnel."""

from __future__ import annotations

import base64
import http.client
import json
import math
from typing import Any
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen
from urllib.parse import urlsplit

import numpy as np

from isaac_sim_adapter.policy_runtime import ABSOLUTE_ACTION_SCHEMA
from isaac_sim_adapter.policy_runtime import ActionChunkEnvelope
from isaac_sim_adapter.policy_runtime import EpisodeContext
from isaac_sim_adapter.policy_runtime import ModelObservation
from isaac_sim_adapter.policy_runtime import PolicyArtifact
from isaac_sim_adapter.policy_runtime import RawObservation
from isaac_sim_adapter.policy_runtime import RuntimeHealth


PROTOCOL_VERSION = 'smolvla_remote_inference_v1'
CHUNK_SIZE = 10
EXECUTE_K = 5
STATE_DIM = 15
IMAGE_SHAPE = (240, 320, 3)


class RemoteSmolVlaPolicyBackend:
    """Fail-closed remote backend; safety remains owned by the local node."""

    def __init__(
        self,
        endpoint: str,
        *,
        timeout_s: float = 0.45,
        health: RuntimeHealth | None = None,
    ) -> None:
        endpoint = str(endpoint).rstrip('/')
        if not endpoint.startswith(('http://', 'https://')):
            raise ValueError('remote endpoint must start with http:// or https://')
        if timeout_s <= 0.0:
            raise ValueError('remote timeout must be positive')
        self.endpoint = endpoint
        self.timeout_s = float(timeout_s)
        self._health = health or RuntimeHealth()
        self._artifact: PolicyArtifact | None = None
        self._context: EpisodeContext | None = None
        parsed = urlsplit(endpoint)
        self._http_connection = (
            http.client.HTTPConnection(
                parsed.hostname,
                parsed.port or 80,
                timeout=self.timeout_s,
            )
            if parsed.scheme == 'http' and parsed.hostname
            else None
        )
        self.metadata = {
            'policy_type': 'smolvla_recovery_v3_remote',
            'action_type': 'absolute_eef_gripper',
            'policy_action_semantics': 'absolute_eef_gripper_v0',
            'state_dim': STATE_DIM,
            'action_dim': 8,
            'chunk_size': CHUNK_SIZE,
            'deploy_n_action_steps': EXECUTE_K,
            'claims_task_success': False,
            'remote_endpoint': self.endpoint,
        }

    def _request(self, path: str, payload: dict[str, Any]) -> dict[str, Any]:
        body = json.dumps(payload, separators=(',', ':')).encode('utf-8')
        if self._http_connection is not None:
            try:
                self._http_connection.request(
                    'POST', path, body=body,
                    headers={
                        'Content-Type': 'application/json',
                        'Connection': 'keep-alive',
                    },
                )
                response = self._http_connection.getresponse()
                raw = response.read()
                if response.status >= 400:
                    raise RuntimeError(
                        f'remote HTTP {response.status}: '
                        f'{raw.decode("utf-8", errors="replace")[:500]}'
                    )
                value = json.loads(raw.decode('utf-8'))
            except Exception as error:
                self._http_connection.close()
                raise RuntimeError(f'remote inference unavailable: {error}') from error
            if not isinstance(value, dict):
                raise RuntimeError('remote response must be a JSON object')
            if value.get('protocol_version') != PROTOCOL_VERSION:
                raise RuntimeError('remote protocol version mismatch')
            if 'error' in value:
                raise RuntimeError(f"remote inference error: {value['error']}")
            return value
        request = Request(
            f'{self.endpoint}{path}',
            data=body,
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        try:
            with urlopen(request, timeout=self.timeout_s) as response:
                value = json.loads(response.read().decode('utf-8'))
        except HTTPError as error:
            detail = error.read().decode('utf-8', errors='replace')
            raise RuntimeError(f'remote HTTP {error.code}: {detail[:500]}') from error
        except (URLError, TimeoutError, OSError) as error:
            raise RuntimeError(f'remote inference unavailable: {error}') from error
        if not isinstance(value, dict):
            raise RuntimeError('remote response must be a JSON object')
        if value.get('protocol_version') != PROTOCOL_VERSION:
            raise RuntimeError('remote protocol version mismatch')
        if 'error' in value:
            raise RuntimeError(f"remote inference error: {value['error']}")
        return value

    @staticmethod
    def _encode_jpeg(image: Any, name: str) -> str:
        array = np.asarray(image)
        if array.shape != IMAGE_SHAPE or array.dtype != np.uint8:
            raise ValueError(f'{name} must be uint8 RGB {IMAGE_SHAPE}, got {array.dtype} {array.shape}')
        import cv2

        ok, encoded = cv2.imencode(
            '.jpg', cv2.cvtColor(array, cv2.COLOR_RGB2BGR),
            [cv2.IMWRITE_JPEG_QUALITY, 95],
        )
        if not ok:
            raise RuntimeError(f'failed to encode {name} as JPEG')
        return base64.b64encode(encoded.tobytes()).decode('ascii')

    def load(self, artifact: PolicyArtifact) -> None:
        self._artifact = artifact
        self._health.policy_loaded = True
        self._health.validity = 'WARMING_UP'
        self._health.reason_code = 'observation_warming_up'

    def reset(self, context: EpisodeContext) -> None:
        if self._artifact is None:
            raise RuntimeError('load must precede reset')
        response = self._request('/reset', {'protocol_version': PROTOCOL_VERSION})
        if response.get('reset') is not True:
            raise RuntimeError('remote reset was not acknowledged')
        self._context = context

    def build_observation(self, raw: RawObservation) -> ModelObservation:
        state = tuple(float(value) for value in raw.state)
        if len(state) != STATE_DIM or not all(math.isfinite(value) for value in state):
            raise ValueError('SmolVLA observation state must be finite state[15]')
        self._encode_jpeg(raw.image, 'scene image')
        if raw.wrist_image is None:
            raise ValueError('remote dual-camera backend requires wrist image')
        self._encode_jpeg(raw.wrist_image, 'wrist image')
        return ModelObservation(
            observation_sequence=raw.observation_sequence,
            captured_monotonic_ns=raw.captured_monotonic_ns,
            state=state,
            image=raw.image,
            task=raw.task,
            wrist_image=raw.wrist_image,
        )

    def predict_chunk(self, observation: ModelObservation) -> ActionChunkEnvelope:
        if self._artifact is None or self._context is None:
            raise RuntimeError('remote backend must be loaded and reset before prediction')
        started = time.monotonic_ns()
        response = self._request(
            '/predict',
            {
                'protocol_version': PROTOCOL_VERSION,
                'observation_sequence': observation.observation_sequence,
                'captured_monotonic_ns': observation.captured_monotonic_ns,
                'state': list(observation.state),
                'image_encoding': 'jpeg',
                'scene_jpeg_b64': self._encode_jpeg(observation.image, 'scene image'),
                'wrist_jpeg_b64': self._encode_jpeg(observation.wrist_image, 'wrist image'),
                'task': observation.task,
            },
        )
        finished = time.monotonic_ns()
        if int(response.get('observation_sequence', -1)) != observation.observation_sequence:
            raise RuntimeError('remote observation_sequence mismatch')
        if response.get('action_schema_version') != ABSOLUTE_ACTION_SCHEMA:
            raise ValueError('remote action schema mismatch')
        actions = response.get('actions')
        if not isinstance(actions, list) or len(actions) != CHUNK_SIZE:
            raise ValueError(f'remote action chunk must contain {CHUNK_SIZE} actions')
        execute_k = int(response.get('execute_k', -1))
        if execute_k != EXECUTE_K:
            raise ValueError(f'remote execute_k must be {EXECUTE_K}')
        return ActionChunkEnvelope(
            observation_sequence=observation.observation_sequence,
            observation_captured_monotonic_ns=observation.captured_monotonic_ns,
            action_schema_version=ABSOLUTE_ACTION_SCHEMA,
            actions=tuple(tuple(float(value) for value in action) for action in actions),
            execute_k=execute_k,
            inference_started_monotonic_ns=started,
            inference_finished_monotonic_ns=finished,
            from_native_chunk=True,
        )

    def health(self) -> RuntimeHealth:
        return self._health

    def close(self) -> None:
        if self._http_connection is not None:
            self._http_connection.close()

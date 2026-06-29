# Inter-Repo Interface Contracts

This project is the upstream ROS 2 simulation, control, and episode recording
system. It should produce loadable episodes and simulation evidence, but it
should not own dataset curation or policy training.

## Repository Roles

| Repository | Role | Owns | Does not own |
|---|---|---|---|
| `ros2-arm-teleoperation-suite` | System runtime and data capture | ROS 2/MuJoCo stack, recorder schema, raw episodes, sim validation | Dataset curation, large training runs, production policy quality claims |
| `robot-arm-episode-data-lab` | Dataset processing and release | Raw import, schema validation, filtering, splits, dataset manifests | ROS 2 runtime, policy optimization |
| `robot-arm-policy-training-lab` | Policy training and export | ACT/Diffusion Policy configs, training runs, checkpoints, evaluation reports | Episode recording, raw data cleaning |

## Contract A: Runtime -> Dataset Lab

Producer: `ros2-arm-teleoperation-suite`

Consumer: `robot-arm-episode-data-lab`

The recorder writes one HuggingFace-loadable dataset per episode:

```text
data/episodes/
└── episode_{episode_index:06d}/
    └── train/
        ├── dataset_info.json
        ├── state.json
        └── data-*.arrow
```

The canonical local config is:

```text
config/lerobot/act_m6_dataset.yaml
```

Required frame fields:

| Field | Type/shape | Meaning |
|---|---|---|
| `observation.state` | float32 `[7]` | Panda measured joint state |
| `action` | float32 `[8]` | 7 arm action values plus gripper command |
| `observation.ee_pose` | float32 `[7]` | `[x, y, z, qx, qy, qz, qw]` |
| `observation.ft` | float32 `[6]` | force/torque vector |
| `observation.gripper` | float32 `[1]` | gripper opening/state |
| `observation.images.scene` | uint8 `[H, W, 3]` | scene RGB camera |
| `observation.images.wrist` | uint8 `[H, W, 3]` | wrist RGB camera |
| `observation.images.tactile_left` | uint8 `[H, W, 3]` | left fingertip GelSight-like tactile RGB |
| `observation.images.tactile_right` | uint8 `[H, W, 3]` | right fingertip GelSight-like tactile RGB |
| `observation.depth.scene` | float32 `[H, W]` | scene depth camera |
| `timestamp` | float64 | synchronized frame timestamp |
| `frame_index` | int64 | frame index within episode |
| `episode_index` | int64 | episode index |
| `done` | bool | terminal frame flag |
| `task` | string | task label/instruction |

Optional filter fields may be added when available:

```text
safety_estop
drive_fault
success
failure_reason
observation.object_pose
```

Compatibility rules:

- The dataset lab may reject episodes missing required fields.
- Older episodes without `observation.images.wrist` or tactile image fields must be marked as legacy
  and kept out of default ACT/Diffusion Policy training splits.
- Any episode containing `safety_estop=true` or `drive_fault=true` should be
  excluded from default imitation-learning splits unless explicitly requested.
- Data files and generated raw episodes stay out of Git.

Validation entry point in this repo:

```bash
bash scripts/validate_m6_perception_recorder.sh --launch
```

## Contract B: Training Export -> Runtime

Producer: `robot-arm-policy-training-lab`

Consumer: `ros2-arm-teleoperation-suite`

The training repository exports model artifacts under a versioned run
directory, for example:

```text
checkpoints/
└── act_panda_pick_m6_v0_1/
    ├── checkpoint.pt
    ├── policy_config.yaml
    ├── normalization.json
    ├── metrics.json
    └── manifest.yaml
```

`manifest.yaml` is the runtime-facing contract. It must declare:

| Key | Meaning |
|---|---|
| `policy.type` | `act` or `diffusion_policy` |
| `policy.framework` | training/inference framework |
| `policy.checkpoint` | checkpoint path relative to manifest |
| `policy.config` | policy config path relative to manifest |
| `policy.normalization` | normalization stats path relative to manifest |
| `dataset.schema` | dataset schema name, e.g. `lerobot_act_m6` |
| `dataset.source_manifest` | dataset release manifest used for training |
| `io.observation_keys` | observation fields required at inference |
| `io.action_key` | model action output name |
| `io.action_dim` | expected action dimension |
| `runtime.publish` | ROS 2 command topics expected by runtime adapter |

The runtime adapter in this repo should treat exported policies as immutable
artifacts. It may load a checkpoint for sim2sim validation, but training
quality and benchmark claims belong to the training repository.

# Inter-Repo Interface Contracts

This project is the upstream ROS 2 simulation, control, and episode recording
system. It should produce loadable episodes and simulation evidence, but it
should not own dataset curation, policy training, or downstream bridge runtime.

## Repository Roles

| Repository | Role | Owns | Does not own |
|---|---|---|---|
| `ros2-arm-teleoperation-suite` | System runtime and data capture | ROS 2/MuJoCo stack, recorder schema, raw episodes, sim validation | Dataset curation, large training runs, production policy quality claims |
| `robot-arm-episode-data-lab` | Dataset processing, training, and export | Raw import, schema validation, filtering, splits, dataset manifests, ACT/Diffusion training, checkpoints, evaluation reports | ROS 2 runtime, MuJoCo control, raw episode collection |
| `/home/ina/ros2_ws/src/ros2-moveit-pybullet-bridge` | Downstream bridge/runtime | MoveIt/PyBullet bridge, policy replay/deployment validation, runtime monitoring, ROS topic adaptation | Raw episode recording, dataset cleaning/release, batch collection, policy training |

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
| `language_instruction` | string | natural-language task instruction |
| `success` | bool | true for accepted training demonstrations |
| `observation.object_pose` | float32 `[7]` | privileged target object pose for filtering/debug |

Optional filter fields may be added when available:

```text
safety_estop
drive_fault
failure_reason
```

Each accepted episode should also ship `episode_*/meta.json` with:

```text
upstream_gate   # teleop | batch_generator | ...
success
```

Known physical-validation gates:

| `upstream_gate` | Meaning |
|---|---|
| `batch_generator` | FSM + lift/place validation passed before `stop_success`; failed attempts discarded |
| `teleop` | Manual or teleop_input stop trigger; success follows recorder stop command |

Midstream datasets adapted from upstream should copy `upstream_gate` into
`manifest.json` and set `filter_scope=training_split_only` when physical
validation already happened upstream.

Compatibility rules:

- The dataset lab may reject episodes missing required fields.
- Older episodes without `observation.images.wrist` or tactile image fields must be marked as legacy
  and kept out of default ACT/Diffusion Policy training splits.
- Any episode containing `safety_estop=true` or `drive_fault=true` should be
  excluded from default imitation-learning splits unless explicitly requested.
- Default ACT/Diffusion training imports should require `success=true` on all
  frames and non-empty `language_instruction`.
- Data files and generated raw episodes stay out of Git.

Validation entry point in this repo:

```bash
python3 scripts/validate_dataset.py data/episodes --min-frames 5
```

## Contract B: Dataset/Training Lab -> Downstream Bridge

Producer: `robot-arm-episode-data-lab`

Consumer: `/home/ina/ros2_ws/src/ros2-moveit-pybullet-bridge`

The middle repository owns explicit schema adaptation, dataset release
manifests, ACT/Diffusion training, evaluation, and checkpoint export. It may
combine upstream `observation.state[7]` plus `observation.gripper[1]` into
`state[8]`, and it may derive action schemas such as `ee_delta_gripper[7]`.

Release manifests should record raw source, schema name, splits, filter rules,
language/action/state dimensions, and whether `success` labels are present.
Default releases for ACT/Diffusion workflows should require `success=true` and
exclude `safety_estop=true` or `drive_fault=true`.

Training manifests should record dataset release, policy type, training config,
normalization, metrics, and checkpoint paths.

No upstream runtime node or downstream bridge component should silently truncate
`action[8]` to `action[7]` or reinterpret `state[7]` as `state[8]`; those
changes must be visible in the dataset lab adapter/release manifest.

## Contract C: Middle Repo Policy Artifact -> Downstream Bridge

Producer: `robot-arm-episode-data-lab`

Consumer: `/home/ina/ros2_ws/src/ros2-moveit-pybullet-bridge`

The middle repository exports runtime-facing model artifacts under a versioned
run directory. The downstream bridge consumes those artifacts, for example:

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

The downstream bridge should treat exported policies as immutable artifacts. It
may load a checkpoint for MoveIt/PyBullet replay, deployment validation, and
runtime monitoring, but training quality, benchmark claims, and checkpoint
selection belong to the middle data/training repository.

## Contract D: Feedback Loop -> Upstream

The three repositories should form a closed optimization loop. Middle-repo data
quality results and downstream replay/deployment results should flow back into
this upstream repository, but only as lightweight, auditable feedback artifacts,
not as bulk datasets or training outputs.

Allowed upstream feedback artifacts:

| Artifact | Purpose |
|---|---|
| Quality reports | Failure classes, per-task success rates, filter reasons, missing fields, action/state dimension issues, language-template issues. Use `docs/templates/feedback_report.yaml`. |
| Collection config suggestions | Object distributions, domain randomization ranges, camera/tactile viewpoints, episode length, language templates, validation thresholds. Use `docs/templates/collection_tuning_suggestion.yaml`. |
| Downstream replay summaries | MoveIt planning failure types, action-limit violations, control-frequency mismatch, frame/quaternion issues, gripper-command compatibility. Use `docs/templates/downstream_replay_summary.yaml`. |
| Interface contracts | Schema versions, policy IO requirements, runtime topic mappings, normalization-key conventions |
| Tiny fixtures | Minimal episodes, mock manifests, or failure cases used only for tests/regression checks |

Artifacts that should not flow back into this repo:

- Full cleaned dataset releases, splits, parquet/arrow files, or large
  statistics caches.
- ACT/Diffusion checkpoint binaries, training logs, full wandb/tensorboard
  directories, or large metrics dumps.
- Large downstream replay logs, videos, or rollout datasets.
- Full derived training datasets such as adapted `state[8]` or
  `ee_delta_gripper[7]` copies.

Recommended upstream landing zones:

- Collection and success-rate feedback should update `docs/sorting_dev_guide.md`,
  `batch_generator` parameters, or validation thresholds.
- Schema and interface feedback should update this document,
  `.agents/skills/ros2-teleop-dev/references/three_repo_handoff.md`, and tiny
  schema fixtures.
- Runtime/deployment feedback should update controller, safety, recorder, topic
  contracts, or downstream bridge adaptation notes.

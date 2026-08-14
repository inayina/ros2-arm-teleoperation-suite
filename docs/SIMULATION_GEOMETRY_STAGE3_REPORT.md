# Simulation Geometry Stage 3 Report — Camera Extrinsic Contract

| Field | Value |
|---|---|
| Status | Stage 3A/3B/3C implemented (REPORT_ONLY) |
| Date | 2026-08-14 |
| Audit baseline commit | `f3a760774d02aabf6a6bdd2993a53e1738b867b5` |
| Stage 1/2 implementation commit | `a131e180a77709f60d8b3a2bfb1a8cb0762b64e0` |
| Evidence generation commit | stamped in each `run_manifest.json` as `evidence_generation_commit` |
| Physical | `NOT_RUN/UNAVAILABLE` |
| Hand-eye solver | **not implemented** |
| Control law / Servo tip modified | **false** |

## A. HEAD / baseline

Working tree continues Stage 1/2 (`teleop_diagnostics` TF/FK/fault injection). Stage 3 adds
camera extrinsic authority on top without changing impedance or MoveIt Servo tips.

Regression (this run):

- `pytest` geometry Stage 1/2 + Stage 3 + camera_bridge + domain_randomizer + recorder + fallback FK: **83 passed**
- `test_impedance_math`: **7/7 passed**

## B. Existing camera problem (code-confirmed)

| Fact | Evidence |
|---|---|
| Image / CameraInfo `frame_id = scene_camera_optical_frame` | `camera_bridge_node.py` + `mujoco.launch.py` |
| TF tree had no camera frames | no `TransformBroadcaster` before Stage 3; audit B.1 |
| `camera_bridge` loads an **independent** `MjModel.from_xml_path` | `camera_bridge_node.py` `_try_init_mujoco` |
| Synced into camera model | joints (`/joint_states`), gripper (`/gripper/state`), **target** object (`/sim/object_pose`) |
| **Not** synced | non-target object poses (reset to camera-local initial), **camera pose**, lighting, mass/friction DR |
| DomainRandomizer mutated **main** `model.cam_pos/quat` only | `domain_randomizer.py`; launch passed `randomize` only to `mujoco_sim` |

Therefore ROS Header claimed an optical frame that did not exist in TF, and camera pose
randomization did not reach the rendered RGB stream.

## C. Scene camera authority (Scheme B)

**Nominal authority = MuJoCo XML** (`config/models/franka_panda.xml`). No duplicate
`camera_extrinsics.yaml` with copied numbers.

Shared module: `mujoco_sim/camera_extrinsics.py`

```text
T_parent_camera_effective = T_parent_camera_nominal @ ΔT_camera_local
```

v1 composition is **camera-local only** (right-multiply). World-frame additive ΔT is not supported.

Frames:

```text
world → scene_camera_link (MuJoCo camera axes)
     → scene_camera_optical_frame (ROS REP-103: +x right, +y down, +z forward)
```

MuJoCo→optical fixed: `diag(1, -1, -1)`.

Live path: `mujoco_sim` publishes `/sim/camera_extrinsic` (TRANSIENT_LOCAL JSON) after reset;
`camera_bridge` applies that effective pose to its renderer model and publishes static TF.

## D. Scene TF / Renderer consistency

Offline Stage 3A (`evidence/camera_stage3_scene/`):

| Check | Result |
|---|---|
| renderer vs TF (link) max ‖Δp‖ | **0.0 m** |
| header optical name vs TF optical | match |
| zero perturbation repeatable | true |
| missing camera rejected | true |

## E. Randomization / injection

- Seeded local noise uses a shared `(seed, camera_name, draw_index)` stream so simulator and
  bridge do not draw independently.
- Injection matrix: `dx+10mm`, `dx+30mm`, `dz+30mm`, `yaw+1°`, `yaw+2°`.
- Expected: Nominal vs Effective tracks injection; **Renderer vs TF ≈ 0** (same effective).

## F. Wrist pose tuning (Stage 3B)

Artifacts: `evidence/wrist_camera_pose_tuning/`

| Candidate | visible (4 scen.) | mean border margin | depth∈[0.08,0.55] |
|---|---|---|---|
| A current XML (side look) | 0/4 | n/a | 4 |
| **B look fingers** | **4/4** | **0.405** | 0 |
| C higher + pitch | 1/4 | 0.008 | 4 |
| D centerline offset | 4/4 | 0.386 | 0 |

**Selected: `B_look_fingers`** — maximizes target visibility and border margin on GT
projection (pregrasp/approach/grasp/lift). Trade-off: near-field depths fall outside the
loose mid-range band (expected for eye-in-hand); C keeps mid-range depth but loses visibility.

Frozen XML:

```xml
<camera name="wrist_camera" pos="0.0 0.0 -0.02" xyaxes="1 0 0 0 -1 0" fovy="70.0" />
```

Pose class: **`DESIGN_NOMINAL`** (simulation sensor placement).  
**Not** `PHYSICAL_CALIBRATED`. Evidence class: `MODEL / SIM_GT supported design choice`.

## G. Wrist camera contract (Stage 3C)

`evidence/camera_stage3_wrist/`:

| Check | Result |
|---|---|
| `T_hand_camera` spread across poses | ~1e-16 m (stable) |
| `T_world_camera` spread | ~0.90 m (changes with motion) |
| eye-in-hand contract | **confirmed** |
| dx/dz/yaw injections | Nominal↔Effective tracks; Renderer↔TF ≈ 0 |

## H. Files changed (summary)

| Area | Files |
|---|---|
| Extrinsic authority | `src/mujoco_sim/mujoco_sim/camera_extrinsics.py` |
| DR + publish | `domain_randomizer.py`, `mujoco_sim_node.py` |
| Renderer + TF | `camera_bridge_node.py`, `camera_bridge/package.xml` |
| Launch | `mujoco.launch.py` (`camera_pose_class`) |
| XML freeze | `config/models/franka_panda.xml` (wrist DESIGN_NOMINAL) |
| Diagnostics | `stage3a_cli.py`, `stage3b_cli.py`, `stage3c_cli.py`, `camera_contract.py`, `wrist_pose_candidates.py`, `report.py` provenance |
| Tests | `tests/test_camera_extrinsic_stage3.py` (+ DR test updates) |
| Docs | this report; Stage 1/2 provenance clarification |

## I. Tests / commands

```bash
export PYTHONPATH=src/mujoco_sim:src/camera_bridge:src/teleop_diagnostics:$PYTHONPATH
python3 -m teleop_diagnostics.stage3a_cli --out-dir evidence/camera_stage3_scene
python3 -m teleop_diagnostics.stage3b_cli --out-dir evidence/wrist_camera_pose_tuning
python3 -m teleop_diagnostics.stage3c_cli --out-dir evidence/camera_stage3_wrist
pytest -q tests/test_camera_extrinsic_stage3.py \
  tests/test_geometry_diagnostics_stage1.py \
  tests/test_geometry_diagnostics_stage2.py \
  tests/test_domain_randomizer.py \
  tests/test_camera_bridge_object_mapping.py
```

## J. Evidence artifacts

```text
evidence/camera_stage3_scene/{run_manifest,camera_extrinsics,camera_samples,camera_fault_matrix}.*
evidence/wrist_camera_pose_tuning/{run_manifest,pose_candidates,pose_metrics,selected_wrist_camera.xml.snippet}
evidence/camera_stage3_wrist/{run_manifest,camera_extrinsics,camera_samples,camera_fault_matrix}.*
```

(`evidence/` remains gitignored; regenerate locally.)

## K. Remaining gaps

- Hand-eye / AX=XB / AprilTag / PnP solvers
- Timestamp skew diagnostics (Stage 4)
- Physical camera / physical calibration
- Lighting randomization still does not cross the independent camera model (only camera pose does)
- Non-target object poses still not synced into camera_bridge
- PASS thresholds still not set (REPORT_ONLY)

## L. Stage 4 recommendation (do not implement here)

Signed publication-time skew, controlled delay injection, and clear
`PUBLISH_TIME_SKEW` vs future `SOURCE_TIME_SKEW` once a shared acquisition clock exists.

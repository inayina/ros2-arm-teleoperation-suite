# Simulation Geometry Stage 1 Report

| Field | Value |
|---|---|
| Status | Stage 1 implemented (REPORT_ONLY) |
| Date | 2026-08-13 |
| Commit (evidence run) | regenerate; see `evidence_generation_commit` in manifest |
| Audit baseline commit | `f3a760774d02aabf6a6bdd2993a53e1738b867b5` |
| Implementation commit | `a131e180a77709f60d8b3a2bfb1a8cb0762b64e0` |
| Package | `teleop_diagnostics` |
| Physical | `NOT_RUN/UNAVAILABLE` |
| Control law modified | **false** |
| TCP 0.207 m compensation | **not applied** |

## Direct answers

### 1. Controller analytic FK reference point

**`panda_link7`** (Franka modified-DH frame after joint 7).

Not `panda_hand`, not `panda_ee`, not an unnamed internal TCP.

Evidence at q=0:

| Source | translation [m] | vs controller ‖Δp‖ |
|---|---|---|
| controller analytic FK | `[0.088, 0.000, 1.033]` | — |
| URDF `panda_link7` | `[0.088, 0.000, 1.033]` | `≈ 3.3e-13` |
| URDF / MuJoCo `panda_hand` | `[0.088, 0.000, 0.926]` | `≈ 0.107` |
| URDF / MuJoCo `panda_ee` | `[0.088, 0.000, 0.826]` | `≈ 0.207` |

Rotation geodesic: controller ≡ `panda_link7` (`0` rad); vs hand/ee ≈ `π/4` (fixed hand yaw).

Jacobian uses the same tip origin as FK (`p[kNumJoints]` after 7 DH joints) → also `panda_link7`.

### 2. Why z≈1.033 vs z≈0.826 at zero pose

- Analytic DH sums `d1+d3+d5 = 0.333+0.316+0.384 = 1.033` and stops after joint 7.
- URDF/MuJoCo `panda_ee` continues through fixed joints:
  - `panda_link7 → panda_hand`: `xyz=(0,0,0.107)`, yaw `-π/4`
  - `panda_hand → panda_ee`: `xyz=(0,0,0.10)`
- At q=0, `panda_link7` local +z points along base **−z**, so those fixed offsets move EE **down** by `0.107+0.10=0.207` m → `1.033−0.207=0.826`.

### 3. Is 0.207 m from the fixed transform contract?

**Yes.** It is exactly `0.107 + 0.10` from the URDF/MuJoCo fixed chain. No additional unexplained DH/model convention offset was required to account for the zero-pose translation gap. Orientation mismatch vs EE is the hand fixed yaw (`π/4`), not a second length bias.

Do **not** add a blind `+0.207 m` TCP unless redesigning the impedance tip to `panda_ee` with explicit approval.

## Cross-model (offline REPORT_ONLY)

Artifacts: `evidence/geometry_stage1/{run_manifest,geometry_diagnostics,geometry_samples}.*`

| Pair (ready/zero) | Notes |
|---|---|
| MODEL URDF FK vs KDL FK | residual ≈ 0 (URDF-order KDL) |
| MODEL URDF FK vs MuJoCo `panda_ee` | translation ~1e-12 m; rotation ~6e-8 rad → `SIM_GT` |
| MODEL vs live RSP TF | `INSUFFICIENT_DATA` offline (no TF buffer) |
| controller vs `panda_link7` | ~0 across nominal poses |
| controller vs `panda_ee` | translation residual stays `0.207 m` along link7 local z at tested poses |

Provenance policy: real MuJoCo → `SIM_GT`; fallback → `MODEL` (never used by this package); unknown → `INSUFFICIENT_DATA`. Topic name `/ee_pose` is never provenance.

## Contract freeze

See `src/teleop_diagnostics/config/controller_reference_frame_contract.yaml`.

Docs/symbol comments in `impedance_math.{hpp,cpp}` updated to say `panda_link7`. Control law unchanged. MoveIt Servo tip remains `panda_ee` → **known servo vs impedance frame mismatch** remains documented, not “fixed” by TCP hack.

## How to regenerate

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m teleop_diagnostics.geometry_cli --out-dir evidence/geometry_stage1
pytest -q tests/test_geometry_diagnostics_stage1.py
```

## Live TF closeout (2026-08-13)

Stage 1 exit gate **PASS** after isolated live `robot_state_publisher` TF:

- command: `python3 -m teleop_diagnostics.stage1_live_cli --out-dir evidence/geometry_stage1_live_tf`
- URDF/KDL vs live TF vs MuJoCo SIM_GT residuals ~1e-12 m on nominal poses
- see [`SIMULATION_GEOMETRY_STAGE2_REPORT.md`](./SIMULATION_GEOMETRY_STAGE2_REPORT.md)

## Out of scope (Stage 1 stop rules honored)

No joint-zero fault injection, camera extrinsic, hand-eye, timestamp skew, calibration solver, PASS thresholds, or control-command path in diagnostics.

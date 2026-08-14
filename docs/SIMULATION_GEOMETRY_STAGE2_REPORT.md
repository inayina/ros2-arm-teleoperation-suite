# Simulation Geometry Stage 2 Report

| Field | Value |
|---|---|
| Status | Stage 1 closeout **PASS** + Stage 2 fault injection implemented |
| Date | 2026-08-13 |
| Audit baseline commit | `f3a760774d02aabf6a6bdd2993a53e1738b867b5` |
| Implementation commit | `a131e180a77709f60d8b3a2bfb1a8cb0762b64e0` |
| Evidence generation commit | regenerate via `python3 -m teleop_diagnostics.stage2_cli` (stamped in `run_manifest.json`) |
| Package | `teleop_diagnostics` |
| Physical | `NOT_RUN/UNAVAILABLE` |
| Control law modified | **false** |
| Runtime topics mutated | **false** |

> **Provenance note:** An earlier draft used a single `Commit (evidence)` field equal to the
> audit baseline SHA (`f3a7607…`). That was ambiguous because Stage 2 *implementation* landed
> later as `a131e18…`, while some evidence CSVs were generated from an uncommitted working
> tree that still reported `HEAD=f3a7607`. Prefer the three fields above; do not treat
> `audit_baseline_commit` as the implementation SHA.

> Note: requested filename `docs/SIMULATION_GEOMETRY_TIMING_DIAGNOSTICS.md` does **not**
> exist in this repo. The audit baseline remains
> [`GEOMETRY_TIMING_DIAGNOSTICS_AUDIT.md`](./GEOMETRY_TIMING_DIAGNOSTICS_AUDIT.md).

---

## Current contract (not a silent bug)

```text
Impedance controller analytic FK/Jacobian reference → panda_link7
MoveIt Servo tip                                     → panda_ee
MuJoCo SIM_GT site / independent URDF·KDL FK tip     → panda_ee
```

The `0.207 m` gap between controller tip and `panda_ee` is the fixed chain
`panda_link7 → panda_hand (0.107) → panda_ee (0.10)`.
This is a **frame contract difference**. Residual diagnosis always
canonicalizes to `panda_ee` before comparing. Raw `link7` vs `ee` subtract is refused.

If product intent requires the impedance tip to be `panda_ee`, see
[`CONTROL_FRAME_ALIGNMENT_PROPOSAL.md`](./CONTROL_FRAME_ALIGNMENT_PROPOSAL.md)
(proposal only — not implemented).

---

## Stage 1 closeout — live TF

Command:

```bash
export ROS_DOMAIN_ID=95
export ROS_LOG_DIR=$PWD/.ros_diag_log
timeout 180s python3 -m teleop_diagnostics.stage1_live_cli \
  --out-dir evidence/geometry_stage1_live_tf --domain-id 95
```

Harness starts isolated `robot_state_publisher` + `/joint_states` publisher + tf2 buffer
(observer-only; does not command the arm).

| Check | Result |
|---|---|
| live TF queryable (`panda_link0`→`panda_ee`) | **true** |
| pose count | 8 (zero, ready, near-limit, 5 seeded random) |
| unexplained systematic mismatch | **false** |
| Stage 1 exit gate | **PASS** |

Sample residuals (`REPORT_ONLY`):

| Pair | pose | ‖Δp‖ [m] | geodesic [rad] |
|---|---|---|---|
| URDF vs live TF | zero | ~1e-16 | 0 |
| live TF vs MuJoCo SIM_GT | zero | ~1.7e-12 | ~6e-8 |
| URDF vs live TF | ready | ~2e-16 | 0 |
| live TF vs MuJoCo SIM_GT | ready | ~5.9e-12 | ~6e-8 |

Artifacts: `evidence/geometry_stage1_live_tf/{run_manifest,geometry_diagnostics,geometry_samples}.*`

Pytest integration (real RSP, timeout + cleanup):

```bash
pytest -q tests/test_geometry_live_tf_integration.py -m launch_test
```

---

## Stage 2 — diagnostic fault injection

Faults apply **only** to diagnostic FK copies. Never published to `/joint_states`,
controller, MuJoCo authority, safety, or recorder.

### Joint-zero (injected offset, not calibration)

Multi-pose mean translation residual vs nominal URDF EE:

| Injection | mean ‖Δp‖ [m] | std across poses [m] |
|---|---|---|
| 0 deg | 0 | 0 |
| joint3 ±0.5 deg | ~0.0025 | ~0.0015 |
| joint3 +2.0 deg | ~0.0101 | ~0.0058 |
| joint1 +0.5 & joint5 −1.0 deg | ~0.0051 | ~0.0035 |

**Pattern:** residual magnitude is **pose-dependent** (non-trivial std across postures).

### TCP offset (on diagnostic `T_link7_ee` copy)

At ready pose, tool-local residual matches injection:

| Injection | ‖Δp‖ base [m] | tool-local | rot [rad] |
|---|---|---|---|
| zero | ~0 | ~0 | 0 |
| dx +10 mm | 0.010 | local x ≈ 0.010 | 0 |
| dz +10 mm | 0.010 | local z ≈ 0.010 | 0 |
| dz +30 mm | 0.030 | local z ≈ 0.030 | 0 |
| yaw +1 deg | ~0 | ~0 | ≈ 0.01745 |

**Pattern:** pure TCP translation appears as a **stable tool-local bias**; rotation faults
show geodesic ≈ injected angle with near-zero translation.

### Frame normalization proof

- Raw `controller(link7)` vs `URDF(ee)` → **rejected** (`refuse raw residual`)
- `canonicalize_to_ee(controller)` vs `URDF(ee)` → ‖Δp‖ ≈ 0, rot ≈ 0
- Therefore the known 0.207 m is **never** labeled `TCP fault`

### Ambiguity

On a **single** posture, similar base-frame ‖Δp‖ can come from joint-zero or TCP.
Need multi-pose dependence + tool-local residual to reduce ambiguity → status may be
`AMBIGUOUS` / `SUSPECTED` (never `ROOT_CAUSE_CONFIRMED`).

---

## Semantics

Allowed: `REPORT_ONLY | SUSPECTED | AMBIGUOUS | INSUFFICIENT_DATA | ERROR_INPUT`  
Forbidden: `PASS | FAIL | CALIBRATED | ROOT_CAUSE_CONFIRMED`

Stage 2 is **simulation diagnostic fault injection**, not:

- real joint calibration
- real TCP calibration
- hand-eye
- physical troubleshooting evidence

`PHYSICAL = NOT_RUN/UNAVAILABLE`

---

## Regenerate

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 -m teleop_diagnostics.stage1_live_cli --out-dir evidence/geometry_stage1_live_tf
python3 -m teleop_diagnostics.stage2_cli --out-dir evidence/geometry_stage2
pytest -q tests/test_geometry_diagnostics_stage1.py tests/test_geometry_diagnostics_stage2.py
```

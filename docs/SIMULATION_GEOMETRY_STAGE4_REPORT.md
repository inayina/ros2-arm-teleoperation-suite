# Simulation Geometry Stage 4 Report — Timestamp Skew Diagnostics

| Field | Value |
|---|---|
| Status | Stage 4 implemented (REPORT_ONLY) |
| Date | 2026-08-14 |
| Audit baseline commit | `f3a760774d02aabf6a6bdd2993a53e1738b867b5` |
| Stage 1/2 implementation commit | `a131e180a77709f60d8b3a2bfb1a8cb0762b64e0` |
| Stage 3 implementation commit | `4f71a494988fbe59b280cc3b99c2e4502eb52556` |
| Evidence generation commit | stamped in `run_manifest.json` as `evidence_generation_commit`; this run was generated from a **dirty** working tree on Stage-3 HEAD, so that SHA is **not** the Stage-4 commit (`evidence_working_tree_dirty=true`) |
| Physical | `NOT_RUN/UNAVAILABLE` |
| Skew class v1 | `PUBLISH_TIME_SKEW` only |
| `SOURCE_TIME_SKEW` | **UNAVAILABLE** (no `/clock`, no physics sample ID) |
| Recorder slop gate / schema / control law modified | **false** |

## A. HEAD / baseline

Stage 4 adds observer-only signed Header skew on diagnostic copies. It does not
change `MultiModalSync` emit/reject, recorder frame schema, impedance, or Servo
tips. Delays are applied to **copies**; live topics are not mutated.

Regression (this run):

- `pytest` Stage 1/2/3 + camera_bridge + domain_randomizer + fallback FK + recorder + time_sync + Stage 4: **97 passed**
- `test_impedance_math`: **7/7 passed**

## B. Existing timestamp problem (code-confirmed)

| Fact | Evidence |
|---|---|
| Header stamp is publisher `now()`, not acquisition/physics time | publishers stamp at publish; no `/clock`; no sample ID |
| Recorder gate is **unsigned** | `MultiModalSync._stale_keys`: reject iff `abs(delta) > sync_slop` (default `0.05 s`) |
| Recorder frame `timestamp` | scene color stamp; fallback joint; source stamps not retained |
| Gripper `/gripper/state` and `/teleop/gripper_cmd` | `std_msgs/Float64` **without Header** |
| Latest-cache sync | camera tick + most recent sample per modality; ApproximateTimeSynchronizer is not used |

Therefore a signed “image later than joints” vs “joints later than image” cannot be
read from the existing emit gate, and gripper cannot enter a Header-skew table.

## C. Signed publication convention

```text
signed_delta_s = t_modality - t_anchor(scene color)
```

Positive: modality Header is later than the scene image. Negative: earlier.

v1 class is **only** `PUBLISH_TIME_SKEW`. `SOURCE_TIME_SKEW` is recorded as
`UNAVAILABLE` on every row; claiming `SOURCE_TIME_CONFIRMED` / `ACQUISITION_SKEW`
is forbidden.

Shared module: `teleop_diagnostics/timestamps.py`

Observer on the recorder (additive, gate unchanged):
`MultiModalSync.signed_publication_skews()` and
`diagnostics_snapshot()["publication_skew"]`.

## D. Controlled delay (diagnostic copies)

Injected delays: `±10/30/50/100 ms` on joint / EE / object copies.

| Check | Result |
|---|---|
| recovered `signed_delta_s` vs injected | **true** (max abs error `1e-12` s) |
| injected image–joint / image–EE / image–object p50 | **0.0 s** (symmetric ± set) |
| injected max | **±0.100 s** |
| live topics mutated | **false** |

## E. Recorder gate (unchanged, observed)

Default `sync_slop = 0.05 s`. Gate remains `abs(delta) > slop`.

| Delay | Emit? |
|---|---|
| 0 | emit |
| +30 ms / −30 ms | emit (inside slop) |
| +100 ms / −100 ms | stale reject (sign-symmetric) |

Observer signed skew at zero delay: all paired Header modalities `0.0 s`.

## F. Sequence / missing / no-header

| Scenario | `input_status` | `result_semantics` | PASS? |
|---|---|---|---|
| missing joint stamp | `MISSING` | `INSUFFICIENT_DATA` | no |
| NaN EE stamp | `INVALID` | `ERROR_INPUT` | no |
| gripper state Float64 | `UNAVAILABLE` | `INSUFFICIENT_DATA` | no |
| gripper cmd Float64 | `UNAVAILABLE` | `INSUFFICIENT_DATA` | no |
| color sequence demo | FIRST → IN_ORDER → DUPLICATE → OUT_OF_ORDER | REPORT_ONLY where stamps exist | no PASS |

Forbidden labels (`PASS`, `FAIL`, `CALIBRATED`, `ROOT_CAUSE_CONFIRMED`,
`SOURCE_TIME_CONFIRMED`, `ACQUISITION_SKEW`) are asserted absent.

## G. Motion sensitivity (MODEL, not live)

Latest-cache spatialization model: `Δx ≈ |v| · |Δt|`.

| Case | Result |
|---|---|
| static `v = 0`, 50 ms | **0.0 m** |
| fast `v = 0.5 m/s`, 50 ms | **0.025 m** |

Evidence class is `MODEL`. This is not a live high-speed bag measurement.

## H. Provenance (do not collapse)

| Field | Meaning |
|---|---|
| `audit_baseline_commit` | Pre-diagnostics snapshot |
| `stage12_implementation_commit` | Stage 1/2 code |
| `stage3_implementation_commit` | Stage 3 camera contract |
| `evidence_generation_commit` | `git rev-parse HEAD` at CLI run |
| `evidence_working_tree_dirty` | If true, HEAD SHA is **not** the uncommitted Stage-4 tree |

## I. What Stage 4 did **not** do

- No `/clock` production contract
- No physics sample ID / source-time measurement
- No change to `sync_slop`, emit/reject, or recorder schema
- No gripper Header retrofit
- No control-law or Servo tip change
- No PASS thresholds
- No live ROS bag / high-rate motion capture

## J. Evidence artifacts

```text
evidence/timestamp_stage4/run_manifest.json
evidence/timestamp_stage4/timestamp_skew.csv
evidence/timestamp_stage4/timestamp_diagnostics.json
```

(`evidence/` remains gitignored; regenerate with
`python3 -m teleop_diagnostics.stage4_cli --out-dir evidence/timestamp_stage4`.)

## K. Remaining gaps

- Shared acquisition clock / physics sample ID (future `SOURCE_TIME_SKEW`)
- Live bag signed p50/p95 under real publish jitter
- Gripper Header (currently UNAVAILABLE)
- PASS thresholds still not set (REPORT_ONLY)

Lighting / non-target object sync is implemented on `/sim/scene_visual`
(see [Stage 3 §M](./SIMULATION_GEOMETRY_STAGE3_REPORT.md)).

## L. Calibration / source-time (do not implement here)

Hand-eye and `/clock` remain out of scope. Only after a shared acquisition
contract exists can `SOURCE_TIME_SKEW` leave `UNAVAILABLE`.

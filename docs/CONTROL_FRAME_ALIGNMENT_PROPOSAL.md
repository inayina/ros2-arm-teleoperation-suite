# Control Frame Alignment Proposal (NOT IMPLEMENTED)

| Field | Value |
|---|---|
| Status | Proposal only |
| Date | 2026-08-13 |
| Control law changed | **No** |
| MoveIt Servo tip changed | **No** |

## Current state

| Component | Tip / reference |
|---|---|
| `impedance_math::forward_kinematics` / `jacobian` | `panda_link7` (modified DH after joint 7) |
| `CartesianImpedanceController` desired/current | same analytic FK → `panda_link7` |
| MoveIt Servo `ee_frame_name` / `robot_link_command_frame` | `panda_ee` |
| URDF / MuJoCo / independent diagnostics FK | `panda_ee` |

Fixed chain (URDF):

```text
T_link7_ee_nominal = T_link7_hand (z=0.107, yaw=-π/4) · T_hand_ee (z=0.10)
‖t‖ = 0.207 m
```

Because desired and measured poses both use the same analytic FK, the impedance loop
is **internally consistent** at `panda_link7`. The mismatch is versus Servo / task EE.

## Desired state (if product requires EE tracking)

Impedance tip should match Servo / task TCP:

```text
T_base_ee = T_base_link7_analytic(q) · T_link7_ee_nominal
```

Jacobian must use the **same** EE reference point (not link7 origin).

## Required transform

Apply URDF-derived `T_link7_ee_nominal` after analytic DH (do **not** invent a free
`+0.207 m` scalar without the hand yaw).

## Control / Jacobian implications

1. Update `forward_kinematics` return frame to `panda_ee` **or** add an explicit
   `forward_kinematics_ee` and switch the controller to it.
2. Recompute geometric Jacobian at the EE origin (linear columns change).
3. Re-tune / re-validate Cartesian gains; nullspace behavior may shift slightly.
4. Keep Stage-1/2 diagnostics as the regression oracle (URDF/KDL/TF/SIM_GT on `panda_ee`).

## Regression test requirements (before any merge)

- Existing `teleop_controllers` GTests must be rewritten for EE zero-pose
  (`z≈0.826`, not `1.033`) or split link7 vs EE suites.
- `tests/test_geometry_diagnostics_stage1.py` controller-vs-link7 contract test must
  be updated if the controller tip changes.
- Live TF closeout + Stage-2 zero-injection baseline must remain clean.
- Servo → impedance tracking integration smoke (sim) required before claiming alignment.

## Explicit non-goals of this proposal

- Do not implement in the same change as Stage 2 diagnostics.
- Do not claim physical calibration or Sim2Real readiness from this rename/transform.

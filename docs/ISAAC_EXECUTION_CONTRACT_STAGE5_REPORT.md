# Isaac Execution Contract: Stage 5 Report

Status: fixed-target Servo isolation passed; one bounded B learned-policy
retry reached the remote inference service but did not establish a valid
Isaac-to-ROS execution loop. This is not a learned-policy task result.

## Execution contract from current HEAD

The upstream policy node receives state from `/sim/encoder_state`,
`/gripper/state`, `/ee_pose`, scene and wrist image snapshots. It composes
state15 at 10 Hz, obtains remote asynchronous chunks, and delegates commands
to `PandaPolicyExecutionAdapter`. The policy action is action8 absolute EEF
pose in `xyzw`; `bound_absolute_eef_gripper()` clamps workspace and gripper and
normalizes the quaternion before target-pose construction.

The published target is a `PoseStamped` in `panda_link0`. MoveIt Servo current
configuration uses planning frame `panda_link0`, end-effector/tip `panda_ee`,
and a 125 Hz publish period. The Isaac backend publishes its virtual EE pose by
starting at `panda_hand` and applying the explicit local +Z 0.100 m
hand-to-EE transform. No change to TF, URDF, Servo solver, limits or geometry
was made.

## Command ownership audit

The S4 entrypoint launches with `start_teleop:=false`. Authoritative mode uses
the execution adapter and checks, before issuing its first target, that there
is exactly one publisher each for pose and gripper commands. The non-legacy
branch bypasses the legacy command path. This supports
`SINGLE_WRITER` as a static contract; a B runtime graph was not available, so
it is not live-observed.

## Historical evidence provenance check

The prior Isaac J4/J6-limit log names checkpoint
`recovery_v3_lora_20260723T125632Z/.../005705`, not B dual-camera checkpoint
`smolvla_wrist_ablation_v1_B/.../005460`. It cannot be used as evidence that B
caused J4/J6 drift. This is recorded as `CHECKPOINT_PROVENANCE_MISMATCH`.

## Bounded scripted-oracle attempt

A time-bounded Stage 5 oracle launch wrote its attempt material to
`/home/ina/robot-sim-lab/robot-arm-episode-data-lab/runs/smolvla_execution_contract_audit/isaac_oracle_baseline`.
The first bounded local run reached `ISAAC_E1_READY`, initialized MoveIt Servo,
and entered the oracle execution chain. Its observed GT timeline reached
`reach=true` and then `grasp=true`; no lift report was produced before the
outer bounded run ended. A second bounded retry was stopped during startup
before producing an oracle report. These are partial runtime observations, not
task success and not a policy result.

The local runtime facts are now confirmed: NVIDIA RTX PRO 500 is visible under
the Isaac environment, Isaac Sim starts, and the local Franka USD exists at
`/home/ina/isaac_assets/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd`.

The ordered fixed-target diagnostics completed against the same expert target:
full-pose, position-only and orientation-only Servo runs each converged from
the nominal q0 with final position residuals of approximately 16 micrometres
and final orientation absolute-dot products above 0.9999999999. The PyBullet
IK witness for that target also had sub-micrometre position residual,
sub-microradian rotation residual, and comfortable J4/J6 margins. These are
bounded target and cross-backend witnesses, not global task-success evidence.

The authorized single-seed B retry used the real remote RTX 3090 server, SSH
tunnel, dual cameras, state15/action8, async chunking, warmup and authoritative
execution. The remote service received 103 `/predict` requests, so model
forward was reached. Isaac itself ran its 120-second bounded window and
reported `ISAAC_E1_DONE status=PASS`, but the ROS adapter log remained at
`waiting for /isaac/joint_states`; MoveIt Servo repeatedly reported
`Waiting to receive robot state update`; and policy telemetry remained empty.
The final GT row was `DONE/FAIL` with
`gripper never closed below 0.700` and all four subgoals false. This is an
execution-chain failure or precondition gap, not evidence that B failed to
learn the task. The raw run report and video are under
`runs/smolvla_execution_contract_audit/learned_policy_seed1_local_isaac_remote_B_retry/`.

## Required next evidence

The next gate remains `ISAAC_ROS_BRIDGE_EVIDENCE_REQUIRED`: prove live receipt
of `/isaac/joint_states` and the two raw camera topics by the ROS adapter, with
a non-empty policy observation trace and a confirmed command path. Do not
retrain, recollect, expand seeds, or alter geometry/control parameters. The
authorized single B retry is now consumed; no further learned-policy attempt
is justified by this evidence.

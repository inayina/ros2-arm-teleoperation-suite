# Dual-camera MuJoCo smoke result

Status: **E — RUNTIME / INTERFACE FAILURE**. No learned-policy action was
executed, so this is not evidence that the policy failed to learn.

Scope: one deterministic, in-distribution MuJoCo Stage A only. No Isaac,
retraining, data collection, camera ablation, or K sweep was run.

## Required answers

1. **Checkpoint** — `smolvla_wrist_ablation_v1_B`, formal run
   `train_20260818_retry2`, final checkpoint `005460/pretrained_model`.
   Adapter SHA-256:
   `943633adfb0c8201e46a088507e4e9191843754617093024e12cdeb8c6be950a`.
2. **Dual-camera input** — yes at runtime preflight: fresh scene `320x240`
   and wrist `320x240` frames were observed. The contract declares state15
   plus scene and wrist only; object pose is explicitly excluded.
3. **Runtime normal** — no. Remote RTX 3090 inference loaded and completed
   async warmup, but the MuJoCo/ROS lifecycle supervisor ended the policy loop
   before its first ordinary observation/action cycle.
4. **Approach** — not evaluable; no predicted action was emitted.
5. **Minimum EE-object distance** — not evaluable; no closed-loop action
   sequence occurred.
6. **Descend** — not evaluable; no policy command was executed.
7. **Close** — not observed.
8. **Minimum gripper command** — not evaluable; no predicted command exists.
9. **Contact/grasp evidence** — not observed.
10. **Object lift delta** — not evaluable; no learned-policy rollout reached
    evaluator finalization.
11. **Classification** — E, runtime/interface failure, not A/B/C/D policy
    behavior.
12. **One next experiment** — run one lifecycle-certified MuJoCo smoke with
    the same checkpoint and fixed seed, after proving the launch supervisor
    keeps the live ROS graph and policy node alive through one non-command
    observation cycle. Do not change model, data, K, or training settings.

## Evidence

- `evidence/smolvla_dualcam_mujoco_smoke_20260821T100000Z/trial/runtime_preflight.json`
  records all five required MuJoCo observations as fresh and valid.
- `evidence/smolvla_dualcam_mujoco_smoke_20260821T100000Z/trial/policy.log`
  records remote dual-camera async warmup completion, but no policy action.
- `evidence/smolvla_dualcam_mujoco_smoke_20260821T100000Z/run_manifest.json`
  preserves MuJoCo, scene+wrist, state15, action8, chunk10/K5 provenance.

This is simulation-only evidence and makes no real-robot or Sim2Real claim.

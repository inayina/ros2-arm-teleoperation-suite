# Task Phase / Failure-Onset Telemetry Contract

**Contract**: `panda_task_timeline_v1` + `smolvla_observation_telemetry_v2`  
**Owner**: upstream Evaluation Agent (`ros2-arm-teleoperation-suite`)  
**Purpose**: align policy-input `state15` with continuous Task GT phase using a shared monotonic clock.  
**Boundary**: diagnostic only; not task success, causal proof, Sim2Real, or real robot.

## Timeline producer

`isaac_continuous_gt_recorder.py` writes one JSONL row for every published
`TaskEvaluationStatus`. The previous `gt_events.jsonl` placeholder is now the
authoritative Task GT timeline.

Required identity/time fields:

- `contract_version=panda_task_timeline_v1`;
- `trace_run_id`, `episode_id`, monotonically increasing `event_sequence`;
- `monotonic_ns` for cross-process alignment and `ros_time_ns` for audit;
- `phase_source=upstream_continuous_task_evaluator`;
- `phase_semantics=continuous_gt_achieved_subgoal_frontier`.

The phase is the evaluator's observed subgoal frontier, not a post-hoc label and
not normalized episode progress. Allowed analysis phases are `HOVER`, `DESCEND`,
`CLOSE`, `LIFT`, `TRANSPORT`, `PLACE`, and `RELEASE`. `IDLE`, `DONE`, stale, and
`UNAVAILABLE` rows cannot enter phase distance calculations.

## Failure onset

`failure_onset.kind` has fail-closed semantics:

| Kind | Meaning | Exact onset eligible? |
|---|---|---:|
| `observed_event` | first observed E-stop, timeout, drop, or slip transition | yes |
| `terminal_nonachievement` | required subgoal absent at the episode deadline | no |
| `none_observed` | no failure event observed yet | no |
| `unavailable` | Task GT source unavailable | no |

Terminal lift/reach/grasp failure must not be backdated to an earlier frame.
Consequently, failure-precedence analysis requires at least three episodes with
exact observable onset; otherwise it reports unavailable rather than inventing
causal ordering.

## Observation sidecar

Policy telemetry v2 adds:

- `episode_id` identical to the Task GT episode;
- `observation_monotonic_ns`, captured with the state used by the policy;
- `inference_completed_monotonic_ns`;
- existing `state15`, action/gripper and latency fields.

The runner injects the same evaluation run and episode identity into both
processes. Midstream joins the nearest Task GT sample and rejects alignment age
above 100 ms by default.

## Training reference requirement

True `P(state | phase)` comparison additionally requires immutable train-split
frame annotations using `panda_train_frame_phase_v1`, the same phase source and
the same phase semantics. Existing Recovery v3 train parquets do not contain
these annotations and must not be retroactively labeled by normalized progress
or midstream object-pose heuristics.

Until a separately approved expert-data capture produces those annotations, the
phase-conditioned analysis implementation is contract-tested but a real report
must fail closed.

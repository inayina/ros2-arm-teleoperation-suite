# Isaac E1 Action Execution

## 1. Scope and status

E1 closes the existing upstream control boundary:

```text
policy / MoveIt Servo
  -> ros2_control impedance controller
  -> /sim/joint_effort_cmd (Panda 7 joint efforts, N·m)
  -> isaac_sim_adapter (validation, latest-value, watchdog)
  -> /isaac/joint_effort_cmd
  -> Isaac Panda articulation effort control
```

This does not add a second IK implementation and does not move the downstream
`PandaActionAdapter(pybullet_ik)` into Isaac. It verifies deterministic action
execution infrastructure with one fixed effort sequence; it is not a learned-policy
rollout and does not produce a grasp-success label.

## 2. Implemented runtime behavior

- `effort_control.py::LatestEffortCommand` rejects wrong dimensions, NaN/Inf and
  commands without a fresh post-reset state; Panda torque limits are applied.
- `adapter_node.py::IsaacSimAdapter` subscribes to `/sim/joint_effort_cmd` with
  SensorDataQoS and forwards only the latest valid command.
- A monotonic command watchdog and state-age gate apply zero effort after timeout.
  The Isaac process has a second local watchdog, so remote adapter failure cannot
  replay an effort indefinitely.
- reset clears command/history/state epoch on both sides. New commands remain
  blocked until reset completes and a newer state arrives; late pre-reset raw
  commands are dropped during a bounded post-reset grace interval.
- control/state, reset service, sensor, camera and status callbacks use separate
  callback groups under a five-thread executor.
- `/sim/backend_events` records invalid command, clipping, watchdog and reset events;
  `/sim/backend_status` publishes command/state age, QoS, reset and watchdog health.

Fail-safe response is deliberately `zero_effort` for this simulator E1. It is not a
real-robot safety policy; hardware/HIL admission still needs the safety controller,
drive state machine and verified HOLD/Quick-Stop behavior.

## 3. Five-repeat acceptance run

Build and run the bounded validation:

```bash
colcon build --symlink-install --packages-select \
  isaac_sim_adapter teleop_bringup

timeout 100s scripts/run_isaac_e1_validation.sh \
  evidence/isaac_e1_action_execution
```

The script starts every long-running process behind `timeout`, exercises exactly five
reset/action cycles with the scene camera at 30 Hz, saves QoS/NFR/process evidence,
and kills the Isaac/adapter/runner processes in its exit trap.

Expected files:

```text
evidence/isaac_e1_action_execution/
├── action_sequence_report.json
├── backend.log
├── backend_e1_events.log
├── adapter.log
├── runner.log
├── effort_qos.txt
├── state_qos.txt
├── final_backend_status.txt
├── kernel.txt
├── scheduler_snapshot.txt
└── gpu.txt
```

`action_sequence_report.json` records each repeat's final joint state, trajectory RMSE
and terminal L2 difference relative to repeat 0, reset recovery, state frequency/gaps,
and command publish period P50/P95/max. Runtime numbers are valid only when these files
come from an actual run; sample or planned values must not be inserted.

### 2026-07-18 canonical runtime result

Evidence: `evidence/isaac_e1_action_execution_20260718_final/`.

| Check | Observed result |
|---|---|
| Runtime | Isaac Sim 6.0, RTX PRO 500 6113 MiB, scene camera 30 Hz |
| Repeats | 5/5 reset + fixed effort sequence completed |
| Backend | `PASS`; 1,122 physics frames / 25.012 s; 165 accepted commands; 0 invalid |
| Command QoS | publisher and subscriber both BEST_EFFORT + VOLATILE |
| Command period | requested 100 Hz; per-repeat P95 11.596–11.882 ms; max 12.369 ms |
| State stream | 23.959–26.733 Hz; one detected gap across five repeats |
| Reset recovery | 753.530–925.106 ms |
| Local watchdog | adapter disconnect produced `command_stale` at 104.080 ms and `zero_effort` |
| Reset history | backend dropped 4 late pre-reset commands |
| Repeatability | trajectory RMSE vs repeat 0 up to 0.360 rad; final-state L2 up to 1.337 rad |
| Health | runtime snapshot `OK`; joint/object/camera/EE/FT active; reset inactive |

Interpretation: the E1 action-execution infrastructure passed, but the identical
open-loop effort sequence is not repeatable enough to claim stable control. The observed
terminal and trajectory spread is a concrete E2/E3 input: use the closed-loop controller,
make reset restore the complete articulation state, and set a measured regression gate
before learned-policy rollout. These numbers are not task success or model quality.

## 4. E2 learned-policy interface smoke

The E1 effort boundary remains available for diagnostics, but it is not the default
learned-policy execution path. A bounded ACT run showed why the distinction matters:

- the checkpoint inferred a small Cartesian action and MoveIt Servo produced only a
  `0.024880 rad` maximum joint-target step;
- observe-only mode computed at most `0.2013 N·m`, so the policy and Servo target were
  not intrinsically explosive;
- applying the same command through the cross-process effort loop produced
  `2.8973 rad` joint and `1.1741 m` EE excursions and triggered E-stop;
- the backend state stream measured in E1 was only `23.959–26.733 Hz`, whereas the
  external controller was configured as a much faster loop. That boundary is not a
  stable substitute for a simulator-local high-rate impedance controller.

The implemented smoke path is therefore:

```text
ACT checkpoint -> bounded ee_delta_gripper -> MoveIt Servo
  -> bounded /joint_target -> isaac_sim_adapter
  -> /isaac/joint_position_cmd -> Isaac-local articulation position drive
```

It validates the policy/observation/action/Servo/Isaac interface without introducing
PyBullet IK. The adapter rejects non-finite, out-of-limit or greater-than-`0.25 rad`
joint steps. The backend also converts Isaac's `panda_hand` observation to the shared
`panda_ee` contract using the URDF local `+Z 0.10 m` fixed transform; the measured
post-fix FK position error was `1.382e-7 m`.

Canonical three-action result:

| Check | Observed result |
|---|---|
| Inference / execution / overall | `PASS / PASS / PASS` |
| Actions | 3 requested, 3 completed |
| Inference latency | 483.656 / 415.279 / 275.665 ms |
| Maximum Servo joint target excursion | 0.072947 rad |
| Maximum observed joint excursion | 0.0729 rad |
| Maximum EE excursion | 0.004123 m |
| Safety | `ok=true`, `estop=false` |
| GPU, full 9.8 s sample window | 8.857% average, 11% peak |
| VRAM | 1683 MiB start, 2232 MiB peak |

Evidence: `evidence/e2_act_isaac_smoke_20260718/summary.json` and the raw reports in
the same directory. This is an interface smoke, not a full pick/place rollout or task
success claim. A stable effort-policy path still requires a simulator-local high-rate
closed-loop controller and a new measured gate.

Run the bounded position-mode smoke with:

```bash
timeout 120s scripts/run_isaac_act_smoke.sh \
  /absolute/path/to/checkpoint.pt \
  /tmp/isaac_act_position_smoke 3 false
```

## 5. Verification boundaries

### Implemented and testable without Isaac

- command validation, clamping, timeout and reset epoch state machine;
- canonical/raw topic wiring and explicit SensorDataQoS;
- five-repeat report math and source-level Isaac effort API checks;
- MuJoCo/default launch behavior remains unchanged.

### Requires real Isaac runtime evidence

- The canonical run above supplies evidence for articulation effort consumption,
  five-repeat differences, 30 Hz renderer pressure, local watchdog and QoS compatibility.
- Results must be re-measured after simulator, driver, control or scene changes; this
  single host/run does not establish a universal threshold.

### Not claimed by E1

- ACT/MLP learned-policy success;
- grasp/lift/place task success;
- stable autonomous grasping, hard real-time, real robot or completed Sim2Real.

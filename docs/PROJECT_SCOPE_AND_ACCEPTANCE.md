# Project Scope and Acceptance

This document is the single-page contract for what this repository claims, how it is accepted, and where the boundary is. Detailed design remains in `ARCHITECTURE_V2.md` and each `SPEC_V2_M*.md`.

Cross-repository handoff contracts are defined in [`INTER_REPO_CONTRACTS.md`](INTER_REPO_CONTRACTS.md), covering the runtime -> dataset-lab episode interface and the training-lab -> runtime policy-export interface.

---

## Project Scope

`ros2-arm-teleoperation-suite` is a software-only ROS 2 Jazzy teleoperation and data-collection stack for a Franka Panda arm in MuJoCo.

In scope:

- Layered teleoperation pipeline: input device -> safety -> MoveIt Servo -> ros2_control -> fieldbus/control backend -> MuJoCo physics.
- `ros2_control` controller/hardware-interface integration, including a Cartesian impedance controller and CANopen DS402 hardware backend.
- SocketCAN/vcan0 fieldbus validation with virtual DS402 servo drives, PDO/SDO/NMT/EMCY traffic, and fault injection.
- MuJoCo physics, force/torque, end-effector/object pose, scene/wrist RGBD, left/right GelSight-like tactile RGB, and gripper state observability.
- Multi-modal LeRobot-style episode recording for ACT / Diffusion Policy data pipelines, including scene/wrist/tactile image fields.
- Portfolio evidence: CLI logs, candump traces, dataset feature dumps, screenshots, and GIFs that are traceable to real runs.

Out of scope:

- Certified real-robot safety, functional-safety compliance, or production deployment.
- Guaranteed real Franka hardware control, real gripper Modbus/RS485 integration, or real CAN bus timing certification.
- Training a production-quality policy or proving sim-to-real transfer.
- Photorealistic rendering, camera calibration parity with physical hardware, or high-fidelity contact modeling.
- Claiming every demo GIF exercises every subsystem. Some demos intentionally use a narrower runtime mode for stability.

---

## Runtime Modes

The project has two control backends. This distinction matters for claims and media evidence.

| Mode | Launch setting | Path | Use case | Evidence |
|---|---|---|---|---|
| Sim-direct | `use_sim:=true` | `canopen_system` writes directly to `/sim/joint_effort_cmd` and reads `/sim/encoder_state` | Fast development, M1 smoke, M4/M6/M7 visual demos, recorder validation | ROS topics, controllers, MuJoCo, dataset checks |
| CAN/vcan0 | `use_sim:=false can_interface:=vcan0` | `canopen_system` <-> SocketCAN `vcan0` <-> `virtual_servo_driver` <-> `/sim/*` <-> MuJoCo | M2 fieldbus acceptance, CAN-visible safety checks | `candump vcan0`, `/servo_drive/status`, EMCY/Quick Stop evidence |
| Real CAN candidate | `use_sim:=false can_interface:=can0` | Same SocketCAN backend, pointed at a physical CAN interface | Future hardware bring-up | Not claimed as completed acceptance |

MuJoCo is always the physics server in this repository. In sim-direct mode it receives commands directly from the hardware interface; in CAN/vcan0 mode it receives commands only after the virtual DS402 drive has consumed CAN frames and published `/sim/joint_effort_cmd`.

Rule of thumb:

- If the claim is "the robot moves / records / produces a grasp GIF", sim-direct mode is acceptable.
- If the claim is "the control path went through CANopen DS402", the run must use `use_sim:=false can_interface:=vcan0` and include `candump` or `/servo_drive/status` evidence.
- M7 GIF is allowed to default to sim-direct mode so grasp recording does not depend on CAN timing. The CAN fieldbus claim is covered by M2 evidence.
- M7 sim-direct demos use a MuJoCo-only grasp assist to keep the cube attached after closed-gripper contact. This improves synthetic-data/demo determinism and is not a claim of high-fidelity contact modeling.

---

## Acceptance Map

| Milestone | Acceptance summary | Verification entry |
|---|---|---|
| M1 ros2_control + MuJoCo | Controllers active, Panda stable under gravity compensation, `/joint_states` near 1 kHz, `/sim/*` backplane alive | `docs/SPEC_V2_M1_CONTROL_SKELETON.md`, `media/m1/*` |
| M2 CANopen DS402 | vcan0 PDO traffic, DS402 Operation Enabled, forward command through CAN, fault injection -> EMCY/Fault | `docs/SPEC_V2_M2_CANOPEN_FIELDBUS.md`, `candump vcan0`, `media/m2/*` |
| M3 impedance controller | Plugin loads active, effort interfaces are valid, tracking error target < 2 mm, 1 kHz update, controller hot-switch | `docs/SPEC_V2_M3_IMPEDANCE_CTRL.md` |
| M4 motion layer | Teleop -> safety -> servo -> `/joint_target` -> controller -> MuJoCo is smooth, heartbeat stable, latency target < 50 ms | `scripts/validate_m4_motion_layer.sh --launch` |
| M5 safety layer | 5 monitors pass, out-of-bounds commands rejected, heartbeat timeout latches E-Stop, reset works, CAN Quick Stop checked in CAN mode | `scripts/validate_m5_safety_layer.sh --launch`, optional `--can` mode |
| M6 perception recorder | scene RGB/depth, wrist RGB, and left/right tactile RGB publish; recorder writes loadable dataset with state/ee/ft/gripper/scene/wrist/tactile/depth/action/timestamps | `scripts/validate_m6_perception_recorder.sh --launch` |
| M7 teleop/synth data | Input driver abstraction works, domain randomization resets scene, object pose is real, batch generation runs, at least one grasp/demo GIF exists | `docs/SPEC_V2_M7_TELEOP_SYNTH.md`, `docs/MEDIA_CAPTURE_PLAN.md` |

Acceptance evidence should be traceable to raw logs or generated artifacts. README media must follow `docs/MEDIA_CAPTURE_PLAN.md`.

---

## Current Known Boundaries

- M7 grasp stability is deterministic in the default sim-direct demo via grasp assist. Before recording, still check wrist camera visibility and `/gripper/state` as described in `MEDIA_CAPTURE_PLAN.md`.
- Wrist and tactile camera data are now part of the recorder contract, but existing older datasets may not contain `observation.images.wrist` or `observation.images.tactile_left/right`.
- `use_sim:=true` does not exercise SocketCAN, by design.
- `use_sim:=false can_interface:=vcan0` validates the CANopen protocol path, not real hardware electrical timing.
- Real `can0`, real gripper hardware, and real robot safety validation are future bring-up work, not current acceptance.

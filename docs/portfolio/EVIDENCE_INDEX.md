# Evidence Index - ros2-arm-teleoperation-suite

阶段 1 证据资产索引。状态只能是 `keep`, `regenerate`, `relabel`, `move_to_legacy`, `archive`, `delete`。

| 资产 | 当前仓库 | 主线/Legacy | 数据来源 | 生成脚本 | 输入产物 | 能证明 | 不能证明 | 状态 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `media/m1/joint_states_hz.png` | 上游 | 主线/data plot | ROS joint state run | media capture plan / script 待确认 | M1 ROS logs | joint state 频率证据 | 实机可靠性、训练效果 | keep |
| `media/m1/m1_control_loop_proof.svg` | 上游 | 主线/design/evidence map | M1 control loop | script 待确认 | M1 docs/logs | 控制链路结构 | benchmark 性能 | relabel |
| `media/m1/panda_gravity_comp.png` | 上游 | 主线/screenshot | Panda gravity compensation run | script 待确认 | ROS/MuJoCo run | gravity compensation demo | 真实 Panda 控制 | relabel |
| `media/m1/rqt_graph_m1.png` | 上游 | 主线/screenshot | ROS graph | manual capture | running ROS graph | node/topic topology | 数据质量或 success rate | keep |
| `media/m2/candump_pdo.png` | 上游 | 主线/support | CAN/vcan evidence | capture script 待确认 | `.media_evidence/m2_20260628_130122/` | CANopen traffic evidence | real hardware CAN guarantee | relabel |
| `media/m2/ds402_state_machine.png` | 上游 | 主线/support | DS402 state evidence | capture script 待确认 | M2 evidence logs | DS402 state transitions | production fieldbus certification | relabel |
| `media/m2/emcy_fault_injection.png` | 上游 | 主线/support | EMCY fault evidence | capture script 待确认 | M2 evidence logs | fault injection handling | full safety certification | relabel |
| `media/m2/m2_canopen_fieldbus_proof.svg` | 上游 | 主线/design/evidence map | M2 fieldbus docs | script 待确认 | M2 docs/logs | CANopen design/evidence map | real Panda deployment | relabel |
| `media/m3/contact_compliance_ft.png` | 上游 | 主线/support | force/contact demo | capture script 待确认 | M3 logs | contact compliance visualization | grasp success in real world | relabel |
| `media/m3/controller_active.png` | 上游 | 主线/screenshot | controller state | manual/script capture 待确认 | ROS controller output | controller active state | closed-loop success | relabel |
| `media/m3/ee_tracking_error.png` | 上游 | 主线/data plot | tracking logs | plotting script 待确认 | M3 logs | EE tracking error | downstream replay quality | keep |
| `media/m4/e2e_latency.png` | 上游 | 主线/data plot | teleop latency logs | plotting script 待确认 | M4 logs | upstream latency | downstream benchmark latency | keep |
| `media/m4/singularity_slowdown.png` | 上游 | 主线/data plot | servo safety behavior | plotting script 待确认 | M4 logs | singularity slowdown behavior | formal safety guarantee | relabel |
| `media/m4/teleop_keyboard.gif` | 上游 | 主线/demo | teleop demo | capture script 待确认 | ROS/MuJoCo run | keyboard teleop software demo | autonomous grasp or real robot | keep |
| `media/m5/estop_and_reset.gif` | 上游 | 主线/demo | safety demo | capture script 待确认 | ROS/MuJoCo run | E-stop/reset behavior | certified safety | relabel |
| `media/m5/safety_diagnostics.png` | 上游 | 主线/screenshot | safety diagnostics | manual/script capture 待确认 | diagnostics output | diagnostics visibility | high reliability claim | relabel |
| `media/m6/camera_rgb_view.png` | 上游 | 主线/screenshot | camera capture | `scripts/capture_m6_media.py` 待确认 | M6 ROS image topics | camera observation exists | training image use in canonical MLP | keep |
| `media/m6/lerobot_dataset_features.png` | 上游 | 主线/screenshot | recorder dataset feature view | `scripts/capture_m6_media.py` 待确认 | `episode_*/train/`, metadata | recorder output shape | midstream release correctness | keep |
| `media/m6/multimodal_sensor_sync_grid.png` | 上游 | 主线/data plot | multimodal sync | `scripts/capture_m6_media.py` 待确认 | M6 sensor streams | sync visualization | downstream sensor fusion success | relabel |
| `media/m6/multimodal_sync.png` | 上游 | 主线/data plot | multimodal sync | `scripts/capture_m6_media.py` 待确认 | M6 sensor streams | sync timing evidence | policy performance | relabel |
| `media/m6/tactile_left_view.png` | 上游 | 主线/screenshot | tactile camera/sensor view | `scripts/capture_m6_media.py` 待确认 | tactile stream | sensor stream availability | canonical MLP uses tactile | relabel |
| `media/m6/tactile_right_view.png` | 上游 | 主线/screenshot | tactile camera/sensor view | `scripts/capture_m6_media.py` 待确认 | tactile stream | sensor stream availability | canonical MLP uses tactile | relabel |
| `media/m6/wrist_camera_view.png` | 上游 | 主线/screenshot | wrist camera view | `scripts/capture_m6_media.py` 待确认 | wrist image topic | wrist observation availability | canonical release has images | relabel |
| `media/m7/domain_randomization_grid.png` | 上游 | 主线/support | domain randomization demo | `scripts/capture_m7_demo.sh` / script 待确认 | M7 demo run | visual randomization example | 泛化保证或 Sim2Real success | relabel |
| `media/m7/grasp_demo.gif` | 上游 | 主线/demo | MuJoCo grasp demo | `scripts/capture_m7_demo.sh` 待确认 | M7 sim run | sim grasp motion demo | real grasp success | keep |
| `media/m7/gripper_closeup.gif` | 上游 | 主线/demo | gripper demo | `scripts/capture_m7_demo.sh` 待确认 | M7 sim run | gripper motion | grasp robustness | relabel |
| `media/m7/gripper_closeup.png` | 上游 | 主线/screenshot | gripper demo | capture script 待确认 | M7 sim run | gripper visual | success rate | relabel |
| `media/m7/policy_inference_log.png` | 上游 | support/screenshot | historical or demo policy log | source script 待确认 | terminal/log | inference log exists if mapped | upstream owns policy training | relabel |
| `media/panda_teleop_trajectories_3d.png` | 上游 | 主线/data plot | Panda trajectories | plotting script 待确认 | `data/episodes_mlp` | trajectory distribution | success rate/Sim2Real | keep |
| `media/three_repo_dataflow_diagram.png` | 上游 | 主线/design | three-repo docs | canonical script should live in midstream | `THREE_REPO_CANONICAL_FACTS.md` | high-level dataflow | run evidence | regenerate |
| `media/three_repo_run_evidence.png` | 上游 | 主线/evidence collage | three-repo run summary | canonical script should live in midstream | canonical metrics + benchmark JSON | summarized run evidence | original raw evidence or real robot | regenerate |



| `media/three_repo_canonical_dataflow.svg` | 上游 | 主线/design | phase-2 canonical facts | manual SVG from midstream canonical source | midstream `THREE_REPO_CANONICAL_FACTS.md` | 三仓职责边界与数据流 | run evidence or real robot capability | keep |

| `media/three_repo_canonical_run_evidence.svg` | 上游 | 主线/evidence summary | phase-2 canonical facts and JSON artifacts | manual SVG from midstream canonical source | canonical manifests, metrics, handoff, latest benchmark JSON | README 级运行证据摘要 | original artifacts or real robot capability | keep |

## Notes

- README 首页建议最多保留 3-5 个核心证据，并把其他 M1-M7 图放到文档或 media appendix。
- `three_repo_*` 公共图应由中游 canonical 源统一生成；上游不应手工维护不同版本。
- M7/domain randomization 只能证明仿真扰动/可视化，不证明泛化或真实 Sim2Real。

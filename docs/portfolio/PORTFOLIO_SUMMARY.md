# PORTFOLIO_SUMMARY —— ros2-arm-teleoperation-suite（三仓闭环 · 上游对外母版）

> **对外主语（2026-07-27 冻结，与中游 canonical 事实一致）**：**具身策略数据治理与分层验证框架** —— 上游执行/采集 → 中游合同/交付/训练/评测 → 下游回放/风险/监控，在同一套 Panda 闭环上为多个策略候选（MLP BC / ACT / SmolVLA / scripted oracle）建立可复现、可审计、防包装的分层判定链路。
>
> 历史范围说明「Panda 机械臂的多仓数据、训练、离线评估与 Sim2Sim / Sim2Real-readiness 验证闭环」保留为范围描述，对外主语以冻结句为准。
>
> **事实基准**：WS2 只读跨仓审计报告（2026-08-05，审计全程只读、未运行任何 ROS/仿真节点）+ 本仓代码、配置、测试与既有运行产物。下文一律区分【已实现】/【已实现（部分验证）】/【已实现（mock/代码级）】/【文档声明】/【推断】；证据不足处如实标注，不编造、不美化。

---

## 1. 仓库定位：三仓闭环的上游

| 项 | 内容 |
| --- | --- |
| 定位 | ROS 2 机械臂执行、控制编排与设备接入层（遥操作 / 任务执行 / 数据采集 / Task GT） |
| 实时 Agent | Task Planning · Motion Planning & Control · Evaluation（双轨） |
| 明确不负责 | 中游合同/数据/训练/handoff；下游 replay harness、风险聚合、HOC |
| 对中游的交接 | `episode_*/train/` + `meta.json`（含 `upstream_gate`、`success`、安全标志） |

三仓闭环：本仓（上游）在线执行与采集 → **并列相关工程** `robot-arm-episode-data-lab`（中游，三仓 canonical 事实集中地）适配/release/训练/评测/handoff → **并列相关工程** `ros2-moveit-pybullet-bridge`（下游）回放/风险/监控。三仓共用 V2.1 规范（中游根 `AGENTS.md` 为 canonical，本仓 `docs/AGENTS.md` 为实现映射）；闭环跑法见中游 `docs/CLOSED_LOOP_RUNBOOK.md`（G0–G3）。

## 2. 技术栈与结构

| 层 | 技术/组件 | 说明 |
| --- | --- | --- |
| 编排 | `teleop_bringup/full_system.launch.py` | 描述、仿真、安全、运动、ros2_control、可选录制一体拉起 |
| 遥操作输入 | `teleop_input` | 键盘/手柄 → 位姿与夹爪命令（`/teleop/cmd_pose`、`/teleop/gripper_cmd`、`/teleop/record_trigger`） |
| 运动层 | MoveIt Servo（`teleop_moveit_config`） | 笛卡尔伺服；**不使用 RRT**（RRT 属 legacy/下游） |
| 控制层 | `cartesian_impedance_controller`（`teleop_controllers`） | 笛卡尔阻抗力矩；仿真 `500 Hz` / 真机路径 `1 kHz`（`control_rate_{sim,real}.yaml`） |
| 安全层 | `safety_monitor` | 限位、通信 watchdog、Hold（零速度）、E-stop 锁存（`/safety/status`、`/safety/estop`） |
| 仿真后端 | `mujoco_sim` + `camera_bridge`（默认）；`isaac_sim_adapter`（外部 Isaac 进程，有界接入） | MuJoCo 为默认批采栈；Isaac 用于有界执行与 Task GT |
| 设备接口 | `canopen_hw_interface` + `virtual_servo_driver`（DS402）；`gripper_driver` | `use_sim:=false` 时启用 CANopen 路径；夹爪为 **Mock**（内存寄存器模拟） |
| 任务与 Gate | `synth_data_gen`（batch FSM + `_validate_episode` + 连续 Task GT） | 批采主轨门禁 |
| 录制 | `lerobot_recorder` | 多模态同步写入 `episode_*/train/` + `meta.json` |
| 辅轨评测 | `grasp_monitor` | `/grasp/status`（`GraspStateEstimator`） |
| 消息 | `teleop_interfaces` | 自定义 msg/srv/action（含 PolicyCommand / PolicyExecutionReport / TaskEvaluationStatus） |

src 包全集（审计 §2.1 核对，与职责无缺失）：`camera_bridge` / `canopen_hw_interface` / `grasp_monitor` / `gripper_driver` / `isaac_sim_adapter`（含 `smolvla_policy_inference_node.py`、`s4_runtime_contract.json`）/ `lerobot_recorder` / `mujoco_sim` / `safety_monitor` / `synth_data_gen` / `teleop_bringup` / `teleop_controllers` / `teleop_description` / `teleop_input` / `teleop_interfaces` / `teleop_moveit_config` / `virtual_servo_driver`。

## 3. 核心能力（逐项证据分类）

| # | 能力 | 分类 | 证据位置 |
| --- | --- | --- | --- |
| 1 | **Task Planning Agent**：批采 batch FSM（Hover → Descend → Close → Lift → Transport → Place → Release）或 L0 遥操作输入；批采前设置 `language_instruction` 与 `upstream_gate:=batch_generator` | 【已实现】FSM 七阶段仅抽样确认 hover/descend/release 等（部分确认） | `src/synth_data_gen/synth_data_gen/batch_generator.py`（`_validate_episode` @L1245、`_discard_recording`、`stop_success`、`_release_gripper_lock`）；`src/teleop_input/teleop_input_node.py` |
| 2 | **Motion Planning & Control Agent**：L1 safety_monitor → L2 moveit_servo 笛卡尔伺服 → L3 cartesian_impedance_controller → L4–L5 CANopen + MuJoCo；无 RRT | 【已实现】 | `src/teleop_moveit_config/config/servo.yaml`、`launch/servo.launch.py`；`src/teleop_controllers/src/cartesian_impedance_controller.cpp` + `config/impedance_params.yaml`；`src/teleop_bringup/config/control_rate_{sim,real}.yaml` |
| 3 | **Evaluation 主轨**：`batch_generator._validate_episode` 判定 lift_delta、bin XY、reset/language 就绪 → `discard` / `stop_success`；仅 accepted episode 落盘 | 【已实现】 | `batch_generator.py:553` 调用、L622 `stop_success`、L625-627 `discard` |
| 4 | **Evaluation 辅轨**：`grasp_monitor_node` 发布 `/grasp/status`；默认 `enable_grasp_monitor:=true` | 【已实现】 | `src/grasp_monitor/grasp_monitor_node.py:407`（`GraspStateEstimator`） |
| 5 | **训练数据硬约束**：`grasp_assist_enabled:=false`（批采默认） | 【已实现（部分验证）】参数存在，审计未逐行核对默认值 | `teleop_bringup/full_system.launch.py`；`batch_generator.py` grasp_assist 相关参数 |
| 6 | **Episode 录制**：`recorder_node` 消费 `/teleop/record_trigger`（start / stop_success / discard），`upstream_gate` 参数（默认 `teleop`） | 【已实现】 | `src/lerobot_recorder/recorder_node.py`；输出 `episode_*/train/` + `meta.json` |
| 7 | **Gate 协议落盘**：`meta.json` 写 `upstream_gate` 与 `success`（见 §4） | 【已实现】有实测运行产物 | `data/e2_red_500hz_seed58_wrist_smoke2_20260723/episode_000000/meta.json` |
| 8 | **安全监控**：watchdog / 限位 / Hold（零速度）/ E-stop 锁存；策略路径 `/policy/runtime_hold`（R2 清 queue 保持位置、R3 清 queue 服从 safety latch；authoritative 模式首目标前检查 pose/gripper publisher 数量） | 【已实现】软件路径；**非**认证硬件安全 | `src/safety_monitor/`；`src/isaac_sim_adapter/policy_runtime.py`（M4 已实现） |
| 9 | **Task GT live mirror**：连续 GT recorder 发布 `/task/evaluation_status`（UNAVAILABLE / RUNNING / PASS / FAIL，`risk_may_override=false`） | 【已实现（代码/文档级，审计未复跑运行）】 | `docs/AGENTS.md` §5；`src/synth_data_gen/` 任务与 Gate 模块 |
| 10 | **Policy Runtime（M1–M6，默认 legacy 路径）**：Backend / Lifecycle / native chunk10-K5 Scheduler；`policy_execution_adapter`（absolute EEF8 / delta EEF7 shadow 裁决）；`policy_runtime_ros`（QoS 映射）；`execution_adapter_mode=legacy|shadow|authoritative` 默认 `legacy`；M2 拒绝 shadow + 非 dry-run；M6 mock PolicyBackend 的 ROS/DDS wiring（RUN→R2 Hold→R3 E-stop + HOC trace）已验证，health 携带 `last_command_sequence` / `trace_run_id` / `episode_id` | 【已实现（mock/代码级）】**限制：authoritative 仅完成代码与 mock contract，未执行在线切流；在线 async double buffer 未实现** | `src/isaac_sim_adapter/policy_runtime.py`、`policy_execution_adapter.py`、`policy_runtime_ros.py`、`smolvla_policy_inference_node.py` |
| 11 | **S4 运行时合同**：`s4_runtime_contract.json`（10 Hz / chunk10 / K5 / replan 0.5 s / `claims_sim2real=false` / `claims_task_success=false`） | 【已实现】注意：该合同与中游 `configs/smolvla_s3/` 双份镜像（md5 相同），审计建议声明权威源（中游），上游为镜像 | `src/isaac_sim_adapter/isaac_sim_adapter/s4_runtime_contract.json` |
| 12 | **CANopen / DS402 接口**：SDO 服务器、DS402 状态机、EMCY 故障注入；`use_sim:=false` 启用；真实 SocketCAN（`can0`）启动参数存在但**未作为验收通过项** | 【已实现（接口/虚拟驱动级）】 | `src/canopen_hw_interface/`、`src/virtual_servo_driver/`；证据图见 `media/m2/`（多标 relabel，见 §5） |
| 13 | **夹爪接口**：`MockModbusClient` | 【已实现（mock）】**内存寄存器模拟**，不经过 TCP Socket，不是真实 Modbus TCP/RTU/RS485 | `src/gripper_driver/` |
| 14 | **采集工具链**：preflight、日常采集、归档、校验 | 【已实现】 | `scripts/run_batch_preflight_smoke.sh`、`scripts/collect_daily_episodes.sh`、`scripts/episode_archive.py`、`scripts/validate_dataset.py data/episodes --min-frames 5` |

**跨仓上下文（非本仓直接证据，权威在中游）**：scripted oracle 在修正物理链上 lift 5/5（中游 E3.5 v2b，证明执行环境可完成任务，不替代 learned policy）；learned policy 有界 Isaac S4（修光权威证据）interface 5/5、GT lift **0/5** → **Hold**；SmolVLA Recovery v3 离线 open-loop **Pass**（`eval_gate_v3` 冻结阈值）。详见 §6/§8 与中游权威文档。

## 4. Gate 协议中的角色（`upstream_gate` / `success`）

本仓是 Gate 协议的**源头层（G0）**：物理门禁在上游完成，中游 `filter_scope=training_split_only` 时只校验 schema 与 training split，**不得**从 `observation.object_pose` 重新推导 lift/place 成败。

| 协议层 | 字段 | 本仓角色 |
| --- | --- | --- |
| 上游 episode `meta.json` | `upstream_gate: batch_generator \| teleop`；`success: true`；安全标志 | 由 `batch_generator` 或 teleop 流程写入；审计实测 `data/e2_red_500hz_seed58_wrist_smoke2_20260723/episode_000000/meta.json` 为 `"upstream_gate": "batch_generator"`、`"success": true`、`action_type: ee_pose_gripper` —— 与规范一致 ✓ |
| 中游 adapted/release manifest | `upstream_gate`、`filter_scope: training_split_only`、`physical_validation_applied: true`、`action_type: ee_delta_gripper` | 中游消费本仓 `upstream_gate` 字段；实测 `data/adapted_panda/manifest.json` 四字段全部吻合（审计 §3.1） |
| 下游 handoff / benchmark | `must_validate` 五项、`benchmark_summary.json` | 下游静态校验 bundle；本仓不参与 |

**双轨评测在 Gate 中的分工**：主轨 `_validate_episode` 决定 `discard` / `stop_success`（决定哪些 episode 进训练集）；辅轨 `grasp_monitor` 提供实时物理监督（`/grasp/status`），批采默认启用；训练数据强制 `grasp_assist_enabled:=false`。

## 5. 证据状态与 EVIDENCE_INDEX 导航

证据资产索引：`docs/portfolio/EVIDENCE_INDEX.md`（本仓唯一既有 portfolio 文档，本次补齐母版前无对外叙事）。

- 资产规模：**33 条**（`media/m1`–`m7` 过程证据 + `three_repo_*` 公共图）。
- 状态分布（如实呈现，不美化）：**keep 11 / relabel 20 / regenerate 2**。
  - **relabel（20 条）**：多为「主线/support」「screenshot/demo」类资产，当前标签与其能证明的范围不匹配，需要重新标注（例如 M2 CAN/EMCY、M3 接触柔顺、M6 触觉/腕部相机、M7 domain randomization）；**大量条目的生成脚本仍标「待确认」**。
  - **regenerate（2 条）**：`three_repo_dataflow_diagram.png`、`three_repo_run_evidence.png` —— 公共图应由中游 canonical 源统一生成，上游不应手工维护不同版本。
  - M7 / domain randomization 类资产只能证明仿真扰动/可视化，**不证明**泛化或真实 Sim2Real（索引 Notes 已声明）。
- 审计结论（WS2 §5-M5）：上游证据索引成熟度低于中游；本母版即为补齐动作之一，**不改变索引内既有标注**。

## 6. 能证明 / 不能证明清单

**能证明（有代码/测试/运行产物）**

- ROS 2 + MuJoCo 的 Panda 控制、编排、采集与上游物理 Gate（批量 episode + `meta.json` 产物存在）。
- 笛卡尔伺服 + 笛卡尔阻抗控制链路（仿真 500 Hz 路径）与双速率配置（`control_rate_{sim,real}.yaml`）。
- 双轨 Evaluation（`_validate_episode` 主轨 + `/grasp/status` 辅轨）与「仅 accepted episode 落盘」。
- Gate 协议字段全链一致性（`upstream_gate` / `success` → manifest → handoff → benchmark，审计实测一致）。
- 软件层安全监控路径：watchdog、限位、Hold（零速度）、E-stop 锁存与 diagnostics 可见性。
- Policy Runtime 的 mock/代码级合同：M2 shadow 保护、M4 `/policy/runtime_hold`、M6 真实 ROS/DDS wiring（RUN→R2→R3 + HOC trace）。
- Episode 多模态录制（图像/触觉/关节流同步写入，`episode_*/train/`）。
- 采集工具链（preflight / daily / archive / validate）与运行产物。

**不能证明（当前项目证据不足，不得声称）**

- 真实 Franka Panda 部署 / 驱动（无实机；`use_sim:=false` 路径与 CAN 证据仅为接口/虚拟驱动级，真实 SocketCAN 未验收）。
- 已完成真实 Sim2Real（只能写 Sim2Sim / Sim2Real-readiness 文档级 SOP）。
- 稳定在线自主抓取（learned policy 有界 Isaac S4 权威 lift 0/5 → Hold）。
- learned-policy task success（S4 `outcome_success 0/5`；S4 首轮近黑 reach 3/5 · grasp 1/5 已被修光复测证伪，标注 Superseded，不得作权威）。
- 离线 loss 提升等同于任务成功率提升。
- 真实 Modbus 夹爪接入（`MockModbusClient` 为内存模拟）；认证硬件安全（软件 Hold/E-stop ≠ 功能安全认证）。
- Policy Runtime `authoritative` 在线切流（仅代码 + mock contract）。
- 上游证据索引中 relabel / 待确认条目的「超出标签的更强主张」（如 grasp 鲁棒性、下游回放质量、policy 训练归属 —— 见 EVIDENCE_INDEX 各条「不能证明」列）。

## 7. 面试可讲点（均有可追溯证据）

1. **双轨评测与 Gate 落盘设计**：为什么批采门禁（`_validate_episode` → discard/stop_success）必须在上游做物理判定、中游 `filter_scope=training_split_only` 不重判 —— 职责单一化避免证据链漂移。
2. **Gate 协议全链一致**：`upstream_gate` / `filter_scope` / `must_validate` 在 meta.json → manifest → handoff → benchmark_summary 逐层可回溯（审计逐字段实测一致）。
3. **分层安全与运行时 Hold 语义**：safety_monitor（watchdog/限位）→ Hold/E-stop；策略路径 `/policy/runtime_hold` 的 R2/R3 行为与 authoritative 首目标前检查；同时主动声明软件路径 ≠ 认证硬件安全。
4. **执行适配器的裁决设计**：`execution_adapter_mode=legacy|shadow|authoritative`、absolute EEF8 / delta EEF7 shadow 裁决、M2 拒绝 shadow+非 dry-run 组合 —— 以及**当前未启用 authoritative 在线切流**的诚实边界。
5. **三次止损（跨仓共享叙事）**：错误 evaluator 隔离（INVALID_EVALUATOR_V0）→ interface 5/5 ≠ 任务成功（continuous GT 0/20）→ 近黑 reach 3/5 主动修光复测证伪为 1/5（Superseded 标注）—— 展示「防包装」的工程文化。
6. **证据纪律**：每份 claim 带 `claims_*=false`、lock SHA、provenance；EVIDENCE_INDEX 用 keep / relabel / regenerate 治理旧证据，公共图主动交还中游 canonical 源。
7. **系统集成深度**：MoveIt Servo + 笛卡尔阻抗双速率、MuJoCo 默认 + Isaac 有界 adapter、多模态录制同步、CANopen/DS402 接口与虚拟伺服、DS402 状态机与 EMCY 故障注入。

## 8. 边界声明（硬约束）

- **Not real robot**：无真实 Franka Panda 部署；**Not Sim2Real**：未完成真实 Sim2Real，Sim2Real-readiness 仅是文档级 SOP；**无稳定在线自主抓取**。
- **SmolVLA 状态（只描述事实，不推进）**：Recovery v3 离线 open-loop **Pass**（`eval_gate_v3` 冻结）≠ 任务成功；有界 Isaac S4 seeds 1–5 已跑（`ran_isaac=true`），interface 5/5、权威修光复测 GT lift **0/5** → **Hold**；首轮近黑产物为 **Superseded / historical**。**默认停止**：不扩种子、不重训、不新增采集；S3 任何继续修复/重训需显式人工批准与外部 GPU；`max_data_fix_retries: 1` 已用尽。ACT 为冻结诊断基线，不继续盲目训练。权威证据：中游 `evidence/smolvla_s4_bounded5_relight_20260724T151711Z/s4_gate.json`。
- **硬件相关**：`MockModbusClient` 为内存寄存器模拟，非真实 Modbus；真实 SocketCAN（`can0`）未作为验收通过项；软件 Hold/E-stop ≠ 认证硬件安全；CAN/EMCY 等证据图为仿真/vcan 接口证据。
- **运行时**：Policy Runtime 默认执行适配 `legacy`；`authoritative` 在线切流未启用；在线 async double buffer 未实现。
- **职责边界**：本仓不负责训练、release、handoff 与下游 replay/risk/HOC；本仓无 RRT（属 legacy/下游）。
- **证据索引诚实性**：20/33 条资产为 relabel、2 条 regenerate、生成脚本大量「待确认」—— 以上为当前事实，不使用超出标签的更强主张。

## 9. 姊妹仓与权威入口

- 本仓母版叙事以中游 canonical 为准，出现口径差异时以中游为准：
  - **并列相关工程**（中游）`robot-arm-episode-data-lab`：`docs/portfolio/THREE_REPO_CANONICAL_FACTS.md`（统一事实源）、`docs/portfolio/FINAL_PROJECT_SUMMARY.md`（统一收口）、`docs/portfolio/BOUNDARY_FREEZE.md`（边界冻结）、`docs/portfolio/EVIDENCE_INDEX.md`、`docs/portfolio/SMOLVLA_RECOVERY_V3_PORTFOLIO.md`。
  - **并列相关工程**（下游）`ros2-moveit-pybullet-bridge`：`docs/portfolio/README.md`（压缩导航）、`INTERVIEW_PREP.md`。
- 三仓规范：V2.1（中游根 `AGENTS.md` canonical）；闭环跑法：中游 `docs/CLOSED_LOOP_RUNBOOK.md`。

## 附：事实基准与审计快照

- 审计报告：`/home/ina/portfolio-audit/ws2_three-repo-closed-loop.md`（2026-08-05 只读审计；未运行任何节点，运行态结论全部来自既有 JSON/log 产物）。
- 上游 Git 快照（审计时点）：HEAD `fb12b674e61aa8f9b1862cffcc0cf6aa2995fdaa`（2026-07-27）；工作区 dirty：10 修改 + 4 未跟踪（`docs/TASK_PHASE_FAILURE_TELEMETRY_CONTRACT.md`、`media/readme_three_repo_overview.svg`、`src/synth_data_gen/synth_data_gen/task_telemetry.py`、`tests/test_task_telemetry.py`）。
- 未确认项（如实保留）：FSM 七阶段完整枚举、`grasp_assist_enabled` 默认值、`eval_gate_v3` 各阈值语义为抽样确认；`task_telemetry.py` 等未跟踪 WIP 未做运行验证。
- 证据分类口径：已实现 / 已实现（部分验证）/ 已实现（mock/代码级）/ 文档声明，代码未确认 / 基于证据的推断 —— 与 AGENTS.md §8.4 回答格式一致。

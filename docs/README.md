# 文档索引

> **新人阅读顺序**：仓库根 [README](../README.md) → [AGENTS.md](./AGENTS.md) →
> [PROJECT_SCOPE_AND_ACCEPTANCE.md](./PROJECT_SCOPE_AND_ACCEPTANCE.md) →
> [INTER_REPO_CONTRACTS.md](./INTER_REPO_CONTRACTS.md) → [ARCHITECTURE_V2.md](./ARCHITECTURE_V2.md)。
>
> `ROADMAP.md` 与 `SPEC_V2_M*.md` 是设计和验收入口，不作为当前完成状态的单一事实源。
> 当前能力以根 README 的「当前状态」、代码和测试为准。

跨三仓项目事实查询与本仓变更影响分析：

```bash
bin/ask-project "上游当前负责什么？"
bin/project-evidence impact --base HEAD~1 --head HEAD
```

入口通过 `EPISODE_DATA_LAB_ROOT` 定位中游 Project Evidence Agent 核心。

---

## ✅ 当前基线文档（V2）

| 文档 | 说明 |
|---|---|
| [ARCHITECTURE_V2.md](./ARCHITECTURE_V2.md) | V2 工业级七层架构：系统图 / 节点图 / Topic 图 / Package 结构 / Launch 架构 |
| [ARCHITECTURAL_DECISION_RECORDS.md](./ARCHITECTURAL_DECISION_RECORDS.md) | 关键架构决策：为什么选 MuJoCo 而非 Gazebo、为什么选 CANopen 而非 EtherCAT |
| [PROJECT_SCOPE_AND_ACCEPTANCE.md](./PROJECT_SCOPE_AND_ACCEPTANCE.md) | 项目边界、非目标、`use_sim`/CAN 模式说明与 M1-M7 验收入口 |
| [GEOMETRY_TIMING_DIAGNOSTICS_AUDIT.md](./GEOMETRY_TIMING_DIAGNOSTICS_AUDIT.md) | 2026-08-13 TF/FK、相机外参与 timestamp skew 只读审计基线 |
| [SIMULATION_GEOMETRY_STAGE1_REPORT.md](./SIMULATION_GEOMETRY_STAGE1_REPORT.md) | Stage 1：独立 FK、cross-model REPORT_ONLY、controller=`panda_link7` 合同冻结 |
| [SIMULATION_GEOMETRY_STAGE2_REPORT.md](./SIMULATION_GEOMETRY_STAGE2_REPORT.md) | Stage 1 live TF closeout + Stage 2 joint-zero/TCP fault injection（诊断副本） |
| [SIMULATION_GEOMETRY_STAGE3_REPORT.md](./SIMULATION_GEOMETRY_STAGE3_REPORT.md) | Stage 3：scene/wrist camera extrinsic authority、TF/renderer 一致、腕部 DESIGN_NOMINAL、`/sim/scene_visual` 非目标+灯光 |
| [SIMULATION_GEOMETRY_STAGE4_REPORT.md](./SIMULATION_GEOMETRY_STAGE4_REPORT.md) | Stage 4：signed publication-time skew、controlled delay copies、SOURCE_TIME UNAVAILABLE |
| [WRIST_RGB_ACCEPTANCE_REPORT.md](./WRIST_RGB_ACCEPTANCE_REPORT.md) | 腕部 RGB：`H_knuckle_z05` 像素 4/4；portfolio 默认打开腕部相机 |
| [CONTROL_FRAME_ALIGNMENT_PROPOSAL.md](./CONTROL_FRAME_ALIGNMENT_PROPOSAL.md) | 可选：阻抗 tip 对齐 `panda_ee` 的提案（未实施） |
| [ROADMAP.md](./ROADMAP.md) | 开发路线图、分支策略、逐里程碑检查清单（M1–M7） |
| [MEDIA_CAPTURE_PLAN.md](./MEDIA_CAPTURE_PLAN.md) | 各里程碑媒体采集计划（截图/GIF/录屏），规定内容、命令、存放路径与嵌入位置 |
| [M7_GRASP_DEBUGGING.md](./M7_GRASP_DEBUGGING.md) | M7 物理抓取调试记录、无 GIF 调参流程、自适应夹持力验收门槛 |

---

## 📋 V2 里程碑细化 SPEC

| 里程碑 | 设计 / 验收文档 | 核心内容 |
|---|---|---|
| **M1** ros2_control + MuJoCo | [SPEC_V2_M1_CONTROL_SKELETON.md](./SPEC_V2_M1_CONTROL_SKELETON.md) | ros2_control 骨架、MuJoCo 物理服务器、joint_state_broadcaster |
| **M2** CANopen DS402 总线 | [SPEC_V2_M2_CANOPEN_FIELDBUS.md](./SPEC_V2_M2_CANOPEN_FIELDBUS.md) | vcan0、DS402 状态机、PDO/SDO/EMCY、虚拟伺服驱动器 |
| **M3** 阻抗控制器 | [SPEC_V2_M3_IMPEDANCE_CTRL.md](./SPEC_V2_M3_IMPEDANCE_CTRL.md) | cartesian_impedance_controller、末端跟踪、接触柔顺 |
| **M4** MoveIt Servo | [SPEC_V2_M4_MOTION_LAYER.md](./SPEC_V2_M4_MOTION_LAYER.md) | MoveIt Servo、笛卡尔→关节、奇异/限位规避 |
| **M5** 安全层 | [SPEC_V2_M5_SAFETY_LAYER.md](./SPEC_V2_M5_SAFETY_LAYER.md) | 监视器、watchdog、E-Stop 与 Quick Stop 验收设计 |
| **M6** 视觉与 Recorder | [SPEC_V2_M6_PERCEPTION_RECORDER.md](./SPEC_V2_M6_PERCEPTION_RECORDER.md) | RGB/Depth、多模态对齐、LeRobot 风格录制 |
| **M7** 遥操作与合成数据 | [SPEC_V2_M7_TELEOP_SYNTH.md](./SPEC_V2_M7_TELEOP_SYNTH.md) | TeleopDriverBase、Domain Randomization、批量 episode |

## 🧩 Simulator Backend 扩展 SPEC

| 文档 | 状态 | 核心内容 |
|---|---|---|
| [SPEC_V2_SIM_BACKENDS_ISAAC.md](./SPEC_V2_SIM_BACKENDS_ISAAC.md) | P0–P4 functional / P5 evidence-only complete | 保留 MuJoCo 默认行为；Isaac adapter、单 episode PoC 和首轮 Sim2Sim 观测分布对比已实测 |
| [SIMULATOR_BACKEND_CONTRACT.md](./SIMULATOR_BACKEND_CONTRACT.md) | P0–P5 evidenced contract | MuJoCo 硬编码审计、稳定 ROS contract、Isaac raw→canonical 映射与能力缺口 |
| [ISAAC_E1_ACTION_EXECUTION.md](./ISAAC_E1_ACTION_EXECUTION.md) | E1 action execution | Isaac effort consumption、latest-value/watchdog/reset/QoS 边界与 5-repeat 验收 |

---

## 🗄 V1 历史存档（参照用）

> V1 为五层教学版（teleop → impedance独立节点 → can_bridge → mujoco → recorder），已停止维护，**仅供架构演进对照**。

| 文档 | 说明 |
|---|---|
| [DESIGN_SPEC.md](./archive/v1/DESIGN_SPEC.md) | V1 五层教学版总体设计规范 |
| [SPEC_M1_CAN_RS485.md](./archive/v1/SPEC_M1_CAN_RS485.md) | V1 CAN/RS485 通信层 |
| [SPEC_M2_MUJOCO_BRIDGE.md](./archive/v1/SPEC_M2_MUJOCO_BRIDGE.md) | V1 MuJoCo 桥接 |
| [SPEC_M3_IMPEDANCE_CTRL.md](./archive/v1/SPEC_M3_IMPEDANCE_CTRL.md) | V1 阻抗控制器（独立节点版） |
| [SPEC_M4_FULL_PIPELINE.md](./archive/v1/SPEC_M4_FULL_PIPELINE.md) | V1 全链路集成 |
| [SPEC_M5_LEROBOT_RECORDER.md](./archive/v1/SPEC_M5_LEROBOT_RECORDER.md) | V1 LeRobot 录制 |

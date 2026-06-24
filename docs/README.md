# 文档索引

> **阅读顺序**：`ARCHITECTURE_V2.md` → `ROADMAP.md` → 对应里程碑 `SPEC_V2_M*.md`

---

## ✅ 当前基线文档（V2）

| 文档 | 说明 |
|---|---|
| [ARCHITECTURE_V2.md](./ARCHITECTURE_V2.md) | V2 工业级七层架构：系统图 / 节点图 / Topic 图 / Package 结构 / Launch 架构 |
| [ROADMAP.md](./ROADMAP.md) | 开发路线图、分支策略、逐里程碑检查清单（M1–M7） |
| [MEDIA_CAPTURE_PLAN.md](./MEDIA_CAPTURE_PLAN.md) | 各里程碑媒体采集计划（截图/GIF/录屏），规定内容、命令、存放路径与嵌入位置 |

---

## 📋 V2 里程碑细化 SPEC

| 里程碑 | 状态 | 文档 | 核心内容 |
|---|---|---|---|
| **M1** ros2_control + MuJoCo | ✅ 完成 | [SPEC_V2_M1_CONTROL_SKELETON.md](./SPEC_V2_M1_CONTROL_SKELETON.md) | ros2_control 骨架、MuJoCo 物理服务器、joint_state_broadcaster @1kHz |
| **M2** CANopen DS402 总线 | ✅ 完成 | [SPEC_V2_M2_CANOPEN_FIELDBUS.md](./SPEC_V2_M2_CANOPEN_FIELDBUS.md) | vcan0、DS402 状态机、PDO/SDO/EMCY、虚拟伺服驱动器 ×7 |
| **M3** 阻抗控制器（插件） | 🔧 进行中 | [SPEC_V2_M3_IMPEDANCE_CTRL.md](./SPEC_V2_M3_IMPEDANCE_CTRL.md) | cartesian_impedance_controller、末端跟踪 <2mm、接触柔顺 |
| **M4** MoveIt Servo 运动层 | 🔧 进行中 | [SPEC_V2_M4_MOTION_LAYER.md](./SPEC_V2_M4_MOTION_LAYER.md) | MoveIt 2 Servo、笛卡尔→关节、奇异/限位规避、端到端 <50ms |
| **M5** 安全层 + E-Stop | 🔲 待开始 | [SPEC_V2_M5_SAFETY_LAYER.md](./SPEC_V2_M5_SAFETY_LAYER.md) | 5 监视器、心跳看门狗、DS402 Quick Stop 闭环 |
| **M6** 视觉 + LeRobot Recorder | 🔲 待开始 | [SPEC_V2_M6_PERCEPTION_RECORDER.md](./SPEC_V2_M6_PERCEPTION_RECORDER.md) | RGB/Depth @30Hz、多模态对齐、LeRobotDataset 导出 |
| **M7** 遥操作设备 + 合成数据 | 📝 规划中 | *(SPEC_V2_M7 待撰写)* | TeleopDriverBase 可插拔接口、Domain Randomization、仿真数据生成 Pipeline |

---

## 🗄 V1 历史存档（参照用）

> V1 为五层教学版（teleop → impedance独立节点 → can_bridge → mujoco → recorder），已停止维护，**仅供架构演进对照**。

| 文档 | 说明 |
|---|---|
| [DESIGN_SPEC.md](./DESIGN_SPEC.md) | V1 五层教学版总体设计规范 |
| [SPEC_M1_CAN_RS485.md](./SPEC_M1_CAN_RS485.md) | V1 CAN/RS485 通信层 |
| [SPEC_M2_MUJOCO_BRIDGE.md](./SPEC_M2_MUJOCO_BRIDGE.md) | V1 MuJoCo 桥接 |
| [SPEC_M3_IMPEDANCE_CTRL.md](./SPEC_M3_IMPEDANCE_CTRL.md) | V1 阻抗控制器（独立节点版） |
| [SPEC_M4_FULL_PIPELINE.md](./SPEC_M4_FULL_PIPELINE.md) | V1 全链路集成 |
| [SPEC_M5_LEROBOT_RECORDER.md](./SPEC_M5_LEROBOT_RECORDER.md) | V1 LeRobot 录制 |

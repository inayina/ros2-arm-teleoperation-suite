# ros2-arm-teleoperation-suite

[English](#english) | [中文](#中文)

---

<a name="english"></a>
## 🇬🇧 English

### Overview

`ros2-arm-teleoperation-suite` is a full-pipeline ROS 2 (Jazzy) robotic arm teleoperation suite, completely based on software simulation (without physical hardware). The **V2 architecture** is designed as an industrial-grade stack (not a teaching demo), mirroring how real industrial arms are built: a dedicated safety layer, decoupled motion/control layers, a `ros2_control` real-time loop, a CANopen DS402 fieldbus driving a simulated servo drive, vision perception, and multi-modal LeRobot data recording.

> **Architecture spec: [`docs/ARCHITECTURE_V2.md`](docs/ARCHITECTURE_V2.md)** (mermaid diagrams, node/topic graphs, package layout, launch design, M1–M6 milestones). V1 design is archived in [`docs/DESIGN_SPEC.md`](docs/DESIGN_SPEC.md).

### Key Features (V2 · 7 layers)

1. **L0 Teleop Input**: Keyboard / SpaceMouse / gamepad / Quest 3 → `/teleop/cmd_pose` + heartbeat. All devices share a **pluggable `TeleopDriverBase` interface** — swapping input hardware requires zero changes to downstream layers.
2. **L1 Safety Layer (C++)**: `safety_monitor` with Joint / Workspace / Velocity limit monitors, communication watchdog, and a latching E-Stop wired to DS402 Quick Stop. Outputs `/safe_master_pose` only when all checks pass.
3. **L2 Motion Layer**: MoveIt 2 Servo for Cartesian→joint servoing with singularity / joint-limit avoidance, emitting `/joint_target` (decoupled from control).
4. **L3 Control Layer (`ros2_control`, 1kHz)**: Cartesian impedance controller as a `controller_interface` plugin + `joint_state_broadcaster`, hot-swappable with `joint_trajectory_controller`.
5. **L4 Fieldbus / Drive**: `canopen_system` hardware interface over vcan0 (CANopen DS402 PDO/SDO/NMT/EMCY) → `virtual_servo_driver` simulating DS402 state machine, encoder feedback, and fault states.
6. **L5 Physics Simulation**: `mujoco` v3 (Franka Panda) as a pure physics server + virtual cameras; ground-truth vs. fieldbus-measured state separation.
7. **L6 Perception + L7 Recording**: `camera_bridge` (RGB/Depth) and a multi-modal `lerobot_recorder` (state, ee_pose, ft, gripper, rgb, depth, action, timestamp) → LeRobot dataset for ACT / Diffusion Policy.

### System Architecture (V2)

```mermaid
flowchart TB
    TI["L0 teleop_input<br/>keyboard / gamepad / Quest3"]
    SM["L1 safety_monitor (C++)<br/>JointLimit · Workspace · Velocity<br/>Watchdog · E-Stop"]
    SV["L2 moveit_servo<br/>Cartesian → Joint"]
    RC["L3 ros2_control (1kHz)<br/>cartesian_impedance_controller<br/>+ joint_state_broadcaster + canopen_system"]
    FB["L4 vcan0 (CANopen DS402)<br/>+ virtual_servo_driver ×7"]
    MJ["L5 mujoco_sim<br/>physics + FT + virtual cameras"]
    CAM["L6 camera_bridge<br/>RGB + Depth"]
    REC["L7 lerobot_recorder<br/>multi-modal → LeRobot Dataset"]

    TI -->|"/teleop/cmd_pose + heartbeat"| SM
    SM -->|"/safe_master_pose"| SV
    SM -.->|"/safety/estop → DS402 Quick Stop"| RC
    SV -->|"/joint_target"| RC
    RC <-->|"CAN frames"| FB
    FB <-->|"/sim/* backplane"| MJ
    MJ -->|"/ft_sensor, /ee_pose, /joint_states"| REC
    MJ --> CAM
    CAM -->|"/camera/color, /camera/depth"| REC

    style SM fill:#5c1a1a,stroke:#ff4a4a,color:#fde8e8
    style SV fill:#1a3a2a,stroke:#4aff8a,color:#e8fdf0
    style RC fill:#1a2a5c,stroke:#4a7aff,color:#e8eefd
    style FB fill:#3a2a1a,stroke:#ffaa4a,color:#fdf3e8
    style MJ fill:#2a1a3a,stroke:#aa4aff,color:#f3e8fd
    style CAM fill:#1a3a3a,stroke:#4affff,color:#e8fdfd
    style REC fill:#3a1a2a,stroke:#ff4aaa,color:#fde8f3
```

> Full layered diagrams (node graph, topic graph, launch architecture) are in [`docs/ARCHITECTURE_V2.md`](docs/ARCHITECTURE_V2.md).

### End-to-End Pipeline: Teleoperation → Training → Sim2Sim Deployment

```
[Teleop Device]          [MuJoCo Simulation]
      │                          │
      ▼                          ▼
 /teleop/cmd_pose  →  Safety → Servo → Impedance → CAN → Physics
                                                            │
                                          LeRobot Dataset ←┘
                                                  │
                              ACT / Diffusion Policy Training
                                                  │
                              Policy Inference Node (ROS 2)
                                                  │
                                    MuJoCo Sim2Sim Validation
```

The suite covers the complete loop: **data collection → dataset → policy training → sim deployment**. Domain Randomization (object poses, friction, mass) in MuJoCo ensures dataset diversity for robust policy learning.

### Quick Start

ROS 2 Jazzy should be run with the system Python 3.12 environment (`/usr/bin/python3` + `/opt/ros/jazzy`). Do not run `ros2 launch` from the conda `ros2-teleop` environment; keep conda for LeRobot data processing, training, and notebooks.

```bash
# 1. Source ROS 2
source /opt/ros/jazzy/setup.bash

# 2. Setup virtual CAN interface
bash scripts/setup_vcan.sh

# 3. Install dependencies
bash scripts/install_deps.sh

# 4. Build the workspace
colcon build

# 5. Source workspace environment
source install/setup.bash

# 6. Launch the full system (sim mode, impedance controller)
ros2 launch teleop_bringup full_system.launch.py

# Variants
ros2 launch teleop_bringup m1_control_sim.launch.py                 # M1 smoke: ros2_control + MuJoCo
ros2 launch teleop_bringup full_system.launch.py controller:=forward        # M1/M2 torque path
ros2 launch teleop_bringup full_system.launch.py use_sim:=false can_interface:=can0  # real CAN
ros2 launch teleop_bringup full_system.launch.py record:=true               # enable recorder

# M4 validation / cleanup
bash scripts/validate_m4_motion_layer.sh --launch   # launch stack + run acceptance checks
bash scripts/stop_stack.sh                          # tear down lingering background nodes
```

---

<a name="中文"></a>
## 🇨🇳 中文

### 项目概述

`ros2-arm-teleoperation-suite` 是一套基于 ROS 2 (Jazzy) 的机械臂遥操作全链路系统，无实体硬件、纯软件仿真。**V2 架构**以「工业级机械臂软件栈」为目标重构（而非教学演示）：独立安全层、运动/控制解耦、`ros2_control` 实时主循环、CANopen DS402 现场总线驱动虚拟伺服、视觉感知层、多模态 LeRobot 数据录制。

> **架构规范见 [`docs/ARCHITECTURE_V2.md`](docs/ARCHITECTURE_V2.md)**（Mermaid 架构图、节点图、Topic 图、Package 结构、Launch 架构、M1–M6 里程碑）。V1 设计存档于 [`docs/DESIGN_SPEC.md`](docs/DESIGN_SPEC.md)。

### 核心特性（V2 · 七层）

1. **L0 遥操作输入**：键盘 / SpaceMouse / 手柄 / Quest3 → `/teleop/cmd_pose` + 心跳。所有设备共享**可插拔 `TeleopDriverBase` 接口**，切换输入设备无需改动下游任何层。
2. **L1 安全层（C++）**：`safety_monitor` 集成关节/工作空间/速度限位监视器、通信看门狗、可锁存 E-Stop（联动 DS402 Quick Stop）；全部检查通过才输出 `/safe_master_pose`。
3. **L2 运动层**：MoveIt 2 Servo 笛卡尔→关节伺服，自带奇异点/关节限位规避，输出 `/joint_target`（与控制解耦）。
4. **L3 控制层（`ros2_control`，1kHz）**：笛卡尔阻抗控制器作为 `controller_interface` 插件 + `joint_state_broadcaster`，可与 `joint_trajectory_controller` 热切换。
5. **L4 现场总线/驱动**：`canopen_system` 硬件接口经 vcan0（CANopen DS402 PDO/SDO/NMT/EMCY）→ `virtual_servo_driver` 仿真 DS402 状态机、编码器反馈、故障态。
6. **L5 物理仿真**：`mujoco` v3（Franka Panda）作为纯物理服务器 + 虚拟相机；区分仿真真值与总线测得值。
7. **L6 感知 + L7 录制**：`camera_bridge`（RGB/Depth）+ 多模态 `lerobot_recorder`（state / ee_pose / ft / gripper / rgb / depth / action / timestamp）→ LeRobot 数据集，兼容 ACT / Diffusion Policy。

### 系统架构（V2）

```mermaid
flowchart TB
    TI["L0 teleop_input<br/>键盘 / 手柄 / Quest3"]
    SM["L1 safety_monitor (C++)<br/>关节 · 工作空间 · 速度<br/>看门狗 · E-Stop"]
    SV["L2 moveit_servo<br/>笛卡尔 → 关节"]
    RC["L3 ros2_control (1kHz)<br/>cartesian_impedance_controller<br/>+ joint_state_broadcaster + canopen_system"]
    FB["L4 vcan0 (CANopen DS402)<br/>+ virtual_servo_driver ×7"]
    MJ["L5 mujoco_sim<br/>物理 + FT + 虚拟相机"]
    CAM["L6 camera_bridge<br/>RGB + Depth"]
    REC["L7 lerobot_recorder<br/>多模态 → LeRobot Dataset"]

    TI -->|"/teleop/cmd_pose + 心跳"| SM
    SM -->|"/safe_master_pose"| SV
    SM -.->|"/safety/estop → DS402 Quick Stop"| RC
    SV -->|"/joint_target"| RC
    RC <-->|"CAN 帧"| FB
    FB <-->|"/sim/* 背板"| MJ
    MJ -->|"/ft_sensor, /ee_pose, /joint_states"| REC
    MJ --> CAM
    CAM -->|"/camera/color, /camera/depth"| REC

    style SM fill:#5c1a1a,stroke:#ff4a4a,color:#fde8e8
    style SV fill:#1a3a2a,stroke:#4aff8a,color:#e8fdf0
    style RC fill:#1a2a5c,stroke:#4a7aff,color:#e8eefd
    style FB fill:#3a2a1a,stroke:#ffaa4a,color:#fdf3e8
    style MJ fill:#2a1a3a,stroke:#aa4aff,color:#f3e8fd
    style CAM fill:#1a3a3a,stroke:#4affff,color:#e8fdfd
    style REC fill:#3a1a2a,stroke:#ff4aaa,color:#fde8f3
```

> 完整分层图（节点图、Topic 图、Launch 架构）见 [`docs/ARCHITECTURE_V2.md`](docs/ARCHITECTURE_V2.md)。

### 端到端 Pipeline：遥操作 → 训练 → Sim2Sim 部署

```
[遥操作设备]                    [MuJoCo 仿真]
      │                              │
      ▼                              ▼
 /teleop/cmd_pose → 安全层 → Servo → 阻抗控制 → CAN → 物理引擎
                                                        │
                                        LeRobot Dataset ←┘
                                                │
                            ACT / Diffusion Policy 训练
                                                │
                            策略推理节点（ROS 2）
                                                │
                                  MuJoCo Sim2Sim 验证
```

全链路覆盖：**数据采集 → 数据集 → 策略训练 → 仿真部署**。MuJoCo 中的 Domain Randomization（物体位姿、摩擦力、质量）确保数据集多样性，提升策略泛化能力。

### 快速开始

ROS 2 Jazzy 主运行环境使用系统 Python 3.12（`/usr/bin/python3` + `/opt/ros/jazzy`）。不要在 conda `ros2-teleop` 环境里运行 `ros2 launch`；conda 仅用于 LeRobot 数据处理、训练和 notebook。

```bash
# 1. Source ROS 2
source /opt/ros/jazzy/setup.bash

# 2. 配置虚拟 CAN 环境
bash scripts/setup_vcan.sh

# 3. 安装依赖
bash scripts/install_deps.sh

# 4. 编译工作空间
colcon build

# 5. Source 工作空间环境
source install/setup.bash

# 6. 一键启动全链路系统（仿真模式 + 阻抗控制器）
ros2 launch teleop_bringup full_system.launch.py

# 常用变体
ros2 launch teleop_bringup m1_control_sim.launch.py                 # M1 验证：ros2_control + MuJoCo
ros2 launch teleop_bringup full_system.launch.py controller:=forward        # M1/M2 力矩直通
ros2 launch teleop_bringup full_system.launch.py use_sim:=false can_interface:=can0  # 实体 CAN
ros2 launch teleop_bringup full_system.launch.py record:=true               # 启用录制

# M4 验收 / 清理
bash scripts/validate_m4_motion_layer.sh --launch   # 自动起栈 + 采集验收指标
bash scripts/stop_stack.sh                          # 开发结束后清理后台节点
```

### 演示

> 采集计划见 [`docs/MEDIA_CAPTURE_PLAN.md`](docs/MEDIA_CAPTURE_PLAN.md)。M1/M2 已补齐可视化证明图与运行证据 PNG。

#### M1 — MuJoCo Panda 重力补偿

![M1 ros2_control + MuJoCo 闭环视觉证明](media/m1/m1_control_loop_proof.svg)

验证点：`m1_control_sim.launch.py` 启动后，`forward_effort_controller`、`canopen_system(use_sim:=true)`、`mujoco_sim`、`joint_state_broadcaster` 形成最小闭环；`/sim/joint_effort_cmd` 与 `/sim/encoder_state` 贯通，`/joint_states` 目标频率 ≥ 950 Hz，MuJoCo 中 Panda 在重力补偿下保持站立。

| 节点拓扑 | Panda 站立 | 频率/激活日志 |
|---|---|---|
| ![M1 rqt_graph 节点拓扑](media/m1/rqt_graph_m1.png) | ![M1 Panda 重力补偿站立](media/m1/panda_gravity_comp.png) | ![M1 joint_states 频率验证](media/m1/joint_states_hz.png) |

#### M2 — CANopen DS402 总线

![M2 CANopen DS402 现场总线视觉证明](media/m2/m2_canopen_fieldbus_proof.svg)

验证点：`m2_fieldbus.launch.py` 将 M1 直连仿真替换为 `vcan0`；`canopen_system(use_sim:=false)` 发送 RPDO/SYNC，`virtual_servo_driver ×7` 进入 `Operation Enabled`，TPDO 回传编码器状态并驱动 `/joint_states`；故障注入后 `candump` 可见 EMCY 帧且 `/servo_drive/status` 进入 `Fault`。

| candump PDO/SYNC | DS402 状态机 | Fault/EMCY 路径 |
|---|---|---|
| ![M2 candump 周期 PDO 帧](media/m2/candump_pdo.png) | ![M2 DS402 状态机](media/m2/ds402_state_machine.png) | ![M2 EMCY 故障路径](media/m2/emcy_fault_injection.png) |

#### M4 — 键盘遥操作端到端（主演示）

*(待补充 `media/m4/teleop_keyboard.gif` — 键盘控制机械臂实时运动 GIF)*

#### M5 — E-Stop 安全闭环

*(待补充 `media/m5/estop_and_reset.gif` — E-Stop 触发与复位流程 GIF)*

#### M6 — LeRobot 数据集

*(待补充 `media/m6/lerobot_dataset_features.png` — 多模态 Episode 字段结构)*

#### M7 — 夹爪抓取 Demo（最终演示）

*(待补充 `media/m7/grasp_demo.gif` — 仿真夹爪抓取任务全流程 ≥15s)*


### 开发者文档

请参阅 [`docs/`](docs/) 目录获取详细的设计规范与各里程碑技术文档。完整索引见 [`docs/README.md`](docs/README.md)。

**V2 当前基线：**
- [ARCHITECTURE_V2.md](docs/ARCHITECTURE_V2.md)：V2 工业级七层架构规范
- [ROADMAP.md](docs/ROADMAP.md)：开发路线图（M1–M7）

**V2 里程碑 SPEC：**
- [SPEC_V2_M1_CONTROL_SKELETON.md](docs/SPEC_V2_M1_CONTROL_SKELETON.md)：✅ ros2_control 骨架 + MuJoCo
- [SPEC_V2_M2_CANOPEN_FIELDBUS.md](docs/SPEC_V2_M2_CANOPEN_FIELDBUS.md)：✅ CANopen DS402 总线
- [SPEC_V2_M3_IMPEDANCE_CTRL.md](docs/SPEC_V2_M3_IMPEDANCE_CTRL.md)：🔧 笛卡尔阻抗控制器
- [SPEC_V2_M4_MOTION_LAYER.md](docs/SPEC_V2_M4_MOTION_LAYER.md)：🔧 MoveIt Servo 运动层
- [SPEC_V2_M5_SAFETY_LAYER.md](docs/SPEC_V2_M5_SAFETY_LAYER.md)：🔲 安全层 + E-Stop
- [SPEC_V2_M6_PERCEPTION_RECORDER.md](docs/SPEC_V2_M6_PERCEPTION_RECORDER.md)：🔲 视觉 + LeRobot Recorder

**V1 存档（参照用）：** [DESIGN_SPEC.md](docs/DESIGN_SPEC.md) 及各 `SPEC_M*.md`

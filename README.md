# ros2-arm-teleoperation-suite

[English](#english) | [中文](#中文)

---

<a name="english"></a>
## 🇬🇧 English

### Overview

`ros2-arm-teleoperation-suite` is a full-pipeline ROS 2 (Jazzy) robotic arm teleoperation suite, completely based on software simulation (without physical hardware). It is designed to demonstrate the complete 5-layer architecture of an embodied AI teleoperation system.

### Key Features

1. **Teleop Input Layer**: Converts keyboard/gamepad inputs to target pose commands (with pre-reserved interfaces for Quest 3 VR).
2. **Control Layer (C++)**: 6-DOF Cartesian impedance controller with KDL-based Inverse Kinematics and adaptive stiffness for contact compliance.
3. **CAN / RS485 Bridge Layer**: Virtual CAN bus (vcan0) with CANopen DS402 PDO frame encoding/decoding, and pymodbus-based Modbus RTU simulation for the gripper.
4. **Physics Simulation Layer**: 1kHz physics simulation using `mujoco` v3 and the Franka Panda model.
5. **Data Recording Layer**: Records teleoperation episodes in the standard LeRobot (HuggingFace `datasets`) format, ready to be consumed by ACT / Diffusion Policy training pipelines.

### System Architecture

```mermaid
flowchart TB
    subgraph INPUT["Layer 1 · Teleop Input Layer"]
        KBD["⌨️ Keyboard / 🎮 Gamepad"]
        QUEST["🥽 Quest3 VR (Reserved)"]
        TI["teleop_input\n(Python)"]
        KBD --> TI
        QUEST -.->|"future"| TI
    end

    subgraph CTRL["Layer 2 · Impedance Control Layer (C++)"]
        IC["impedance_controller\n(C++ / MultiThreadedExecutor)"]
        IK["KDL ChainIkSolver\nPose → Joint Angles"]
        IMP["Impedance Control\nτ = Jᵀ·[K(xd-x) + D(ẋd-ẋ)] + τ_g"]
        IC --> IK --> IMP
    end

    subgraph CAN_LAYER["Layer 3 · Communication Layer"]
        CB["can_bridge\n(Python)"]
        RS["rs485_bridge\nModbus RTU (Python)"]
        VCAN["🔌 vcan0\nVirtual CAN Bus\n(CANopen DS402 PDO)"]
        CB <-->|"CAN Frames"| VCAN
    end

    subgraph SIM["Layer 4 · Physics Simulation Layer"]
        MJ["mujoco_sim\n(Python / MuJoCo v3)"]
        PANDA["🦾 Franka Panda\nfranka_panda.xml\n1kHz Physics Step"]
        MJ --> PANDA
    end

    subgraph REC["Layer 5 · Data Recording Layer"]
        LR["lerobot_recorder\n(Python / HuggingFace datasets)"]
        DS["💾 data/episodes/\nhf_dataset format\nCompatible with ACT / Diffusion Policy"]
        LR --> DS
    end

    TI -->|"/master_pose\nPoseStamped @ 50Hz"| IC
    IMP -->|"/joint_torque_cmd\nJointState"| CB
    CB -->|"/joint_states\nEncoder Feedback"| MJ
    RS -->|"/gripper_state\nFloat32"| LR
    MJ -->|"/joint_states @ 100Hz"| IC
    MJ -->|"/ft_sensor\nWrenchStamped"| IC
    MJ -->|"/joint_states"| LR
    TI -->|"/gripper_cmd\nFloat32"| RS

    style INPUT fill:#1a3a5c,stroke:#4a9eff,color:#e8f4fd
    style CTRL fill:#1a3a2a,stroke:#4aff8a,color:#e8fdf0
    style CAN_LAYER fill:#3a2a1a,stroke:#ffaa4a,color:#fdf3e8
    style SIM fill:#2a1a3a,stroke:#aa4aff,color:#f3e8fd
    style REC fill:#3a1a2a,stroke:#ff4aaa,color:#fde8f3
```

### Quick Start

```bash
# 1. Setup virtual CAN interface
bash scripts/setup_vcan.sh

# 2. Install dependencies
bash scripts/install_deps.sh

# 3. Build the workspace
colcon build

# 4. Source environment
source install/setup.bash

# 5. Launch the full system
ros2 launch launch/full_system.launch.py
```

---

<a name="中文"></a>
## 🇨🇳 中文

### 项目概述

`ros2-arm-teleoperation-suite` 是一套基于 ROS 2 (Jazzy) 的机械臂遥操作全链路系统。在无实体硬件的条件下，纯基于软件仿真完整体现具身智能遥操作系统的五层核心架构。

### 核心特性

1. **遥操作输入层**：支持键盘/手柄输入，转换为末端位姿指令（预留 Quest 3 接口）。
2. **阻抗控制层（C++）**：基于 KDL 实现六维笛卡尔阻抗控制器，支持接触力自适应刚度调节（柔顺控制）。
3. **总线通信层**：基于 `vcan0` 虚拟 CAN 总线实现 CANopen DS402 PDO 帧编解码；基于 `pymodbus` 仿真 RS485 Modbus RTU 夹爪控制。
4. **物理仿真层**：基于 `mujoco` v3 引擎运行 Franka Panda 机械臂，1kHz 高频物理步进。
5. **数据录制层**：支持将遥操作记录为 HuggingFace `datasets` 格式（LeRobot 兼容），无缝接入具身智能模型（ACT / Diffusion Policy）训练管线。

### 系统架构

```mermaid
flowchart TB
    subgraph INPUT["Layer 1 · 遥操作输入层"]
        KBD["⌨️ 键盘 / 🎮 手柄"]
        QUEST["🥽 Quest3 VR（预留接口）"]
        TI["teleop_input\n(Python)"]
        KBD --> TI
        QUEST -.->|"future"| TI
    end

    subgraph CTRL["Layer 2 · 阻抗控制层 (C++)"]
        IC["impedance_controller\n(C++ / MultiThreadedExecutor)"]
        IK["KDL ChainIkSolver\n末端位姿 → 关节角"]
        IMP["阻抗控制律\nτ = Jᵀ·[K(xd-x) + D(ẋd-ẋ)] + τ_g"]
        IC --> IK --> IMP
    end

    subgraph CAN_LAYER["Layer 3 · 通信层"]
        CB["can_bridge\n(Python)"]
        RS["rs485_bridge\nModbus RTU (Python)"]
        VCAN["🔌 vcan0\n虚拟 CAN 总线\n(CANopen DS402 PDO)"]
        CB <-->|"CAN帧"| VCAN
    end

    subgraph SIM["Layer 4 · 物理仿真层"]
        MJ["mujoco_sim\n(Python / MuJoCo v3)"]
        PANDA["🦾 Franka Panda\nfranka_panda.xml\n1kHz 物理步进"]
        MJ --> PANDA
    end

    subgraph REC["Layer 5 · 数据录制层"]
        LR["lerobot_recorder\n(Python / HuggingFace datasets)"]
        DS["💾 data/episodes/\nhf_dataset 格式\n兼容 ACT / Diffusion Policy"]
        LR --> DS
    end

    TI -->|"/master_pose\nPoseStamped @ 50Hz"| IC
    IMP -->|"/joint_torque_cmd\nJointState"| CB
    CB -->|"/joint_states\n编码器反馈"| MJ
    RS -->|"/gripper_state\nFloat32"| LR
    MJ -->|"/joint_states @ 100Hz"| IC
    MJ -->|"/ft_sensor\nWrenchStamped"| IC
    MJ -->|"/joint_states"| LR
    TI -->|"/gripper_cmd\nFloat32"| RS

    style INPUT fill:#1a3a5c,stroke:#4a9eff,color:#e8f4fd
    style CTRL fill:#1a3a2a,stroke:#4aff8a,color:#e8fdf0
    style CAN_LAYER fill:#3a2a1a,stroke:#ffaa4a,color:#fdf3e8
    style SIM fill:#2a1a3a,stroke:#aa4aff,color:#f3e8fd
    style REC fill:#3a1a2a,stroke:#ff4aaa,color:#fde8f3
```

### 快速开始

```bash
# 1. 配置虚拟 CAN 环境
bash scripts/setup_vcan.sh

# 2. 安装依赖
bash scripts/install_deps.sh

# 3. 编译工作空间
colcon build

# 4. Source 环境
source install/setup.bash

# 5. 一键启动全链路系统
ros2 launch launch/full_system.launch.py
```

### 演示视频

*(演示 GIF 或视频占位 - 待补充至 `media/` 目录)*

### 开发者文档

请参阅 `docs/` 目录获取详细的设计规范与各里程碑技术文档：
- [DESIGN_SPEC.md](docs/DESIGN_SPEC.md): 整体设计规范
- [ROADMAP.md](docs/ROADMAP.md): 开发路线图
- [SPEC_M1_CAN_RS485.md](docs/SPEC_M1_CAN_RS485.md): CAN/RS485 通信层规范
- [SPEC_M2_MUJOCO_BRIDGE.md](docs/SPEC_M2_MUJOCO_BRIDGE.md): MuJoCo 桥接层规范
- [SPEC_M3_IMPEDANCE_CTRL.md](docs/SPEC_M3_IMPEDANCE_CTRL.md): C++ 阻抗控制器规范
- [SPEC_M4_FULL_PIPELINE.md](docs/SPEC_M4_FULL_PIPELINE.md): 全链路集成规范
- [SPEC_M5_LEROBOT_RECORDER.md](docs/SPEC_M5_LEROBOT_RECORDER.md): LeRobot 数据录制层规范

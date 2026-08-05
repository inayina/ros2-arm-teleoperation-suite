# ros2-arm-teleoperation-suite

**ROS 2 机械臂执行、控制编排与设备接入层**

把任务意图、遥操作或策略动作送进 ROS 2 执行链：编排节点、MoveIt Servo、控制器插件、仿真或设备侧接口，并反馈执行状态、安全状态与任务真值。Episode 录制是这条执行链的产物，不是本仓唯一目的。

面向：**机器人系统软件工程师｜ROS 2、C++、Linux、设备通信、执行监督与系统验证**。

---

## 解决什么系统问题

机械臂系统软件需要回答的是：

- 意图如何进入可控的 ROS 2 执行路径；
- 控制命令如何落到仿真或设备接口；
- 心跳丢失、限位触发时如何 Hold 或停止；
- 执行过程中状态、安全和任务真值如何被观测与记录。

本仓把这些责任放在同一条在线链路中，而不是把“采集数据”和“控制机器人”拆成互不相关的脚本。

---

## 在三仓架构中的位置

```text
ros2-arm-teleoperation-suite（本仓 · 上游）
  在线执行 · 控制 · 设备接口 · 采集 · Task GT
                     │ raw episode
                     ▼
robot-arm-episode-data-lab（中游）
  合同 · Release · 训练 · 离线评测 · Handoff
                     │ actions / reports
                     ▼
ros2-moveit-pybullet-bridge（下游）
  Replay · Monitor · Risk · Safety · HOC
```

<p align="center">
  <img src="media/readme_three_repo_overview.svg" alt="Franka Panda 三仓执行、交付与验证架构" width="100%">
</p>

本仓只负责在线执行与上游物理结果。数据合同、Release、训练和离线 Gate 在中游；命令重放、风险聚合与 HOC 在下游。三仓边界见中游 [BOUNDARY_FREEZE.md](https://github.com/inayina/robot-arm-episode-data-lab/blob/main/docs/portfolio/BOUNDARY_FREEZE.md)。

---

## 输入 · 处理 · 输出

| 方向 | 内容 |
| --- | --- |
| **输入** | 任务目标（batch FSM）、遥操作（键盘/手柄）、或策略动作（Isaac/MuJoCo 有界推理路径） |
| **处理** | ROS 2 编排 → MoveIt Servo → 笛卡尔阻抗控制 → MuJoCo（默认）或 Isaac adapter → 安全监视与 Task GT |
| **输出** | 关节/末端/夹爪/图像状态；`/safety/status` 与 E-stop；连续 Task GT；`episode_*/train/` + `meta.json` |

```text
任务意图 / 遥操作 / 策略动作
        ↓
ROS 2 执行、控制与设备接入
        ↓
状态、Episode 与任务真值
```

---

## 核心模块

| 模块 | 路径 | 职责 |
| --- | --- | --- |
| 系统编排 | `src/teleop_bringup/` | `full_system.launch.py`：描述、仿真、安全、运动、ros2_control、可选录制 |
| 遥操作输入 | `src/teleop_input/` | 键盘/手柄 → 位姿与夹爪命令 |
| MoveIt Servo | `src/teleop_moveit_config/` | 笛卡尔伺服（约 125 Hz 关节目标） |
| 阻抗控制 | `src/teleop_controllers/` | `CartesianImpedanceController`；仿真控制环 500 Hz（`control_rate_sim.yaml`） |
| 安全监视 | `src/safety_monitor/` | 限位、通信 watchdog、Hold（零速度）与 E-stop 锁存 |
| MuJoCo 后端 | `src/mujoco_sim/` + `camera_bridge` | 默认物理与相机；主采集路径 |
| Isaac 适配 | `src/isaac_sim_adapter/` | 外部 Isaac 进程的 ROS 适配与有界策略运行时合同 |
| 虚拟伺服 / CAN | `src/virtual_servo_driver/`、`canopen_hw_interface/` | DS402 虚拟驱动与 CANopen HW；`use_sim:=false` 时启用 |
| 夹爪接口 | `src/gripper_driver/` | **Mock**：内存寄存器模拟，不是真实 Modbus |
| 任务与 Gate | `src/synth_data_gen/` | batch FSM；`_validate_episode` 主轨 Gate；连续 Task GT |
| Episode 录制 | `src/lerobot_recorder/` | 多模态同步写入；消费 Task GT 与录制触发 |
| 辅轨监视 | `src/grasp_monitor/` | `/grasp/status`；训练数据须 `grasp_assist_enabled:=false` |

---

## 正常执行链

```text
任务 / 遥操作 / 策略动作
  → ROS 2 编排（teleop_bringup）
  → MoveIt Servo / 笛卡尔阻抗控制器
  → MuJoCo（默认）或 Isaac adapter + 外部 Isaac
  → 关节/末端状态、safety status、连续 Task GT
  → lerobot_recorder → episode_*/train/ + meta.json
```

上游物理 Gate（`batch_generator` 或 teleop 流程）在 `meta.json` 中写入 `upstream_gate` 与 `success` 等字段。中游在 `filter_scope=training_split_only` 时**不再**从 `object_pose` 重判 lift/place。

## 异常 / 故障链

```text
遥操作心跳中断或限位触发
  → safety_monitor 通信 watchdog / 监视器失败
  → Hold（发布零速度）或锁存 E-stop（/safety/estop）
  → 控制器置零力矩；CANopen/仿真侧停止有效努力
  → 执行与安全状态写入 diagnostics / episode meta
```

策略路径另有 `/policy/runtime_hold`（运行时 Hold 合同）。软件 Hold/E-stop 是仿真与接口验证路径，**不是**认证的功能安全或硬件安全证据。

<p align="center">
  <img src="https://raw.githubusercontent.com/inayina/robot-arm-episode-data-lab/main/docs/portfolio/portfolio_control_safety_stack.svg" alt="从策略目标到控制、设备接口与安全反馈的执行链" width="100%">
</p>

<p align="center"><sub>控制与安全链特写：已实现仿真预检与接口代码路径；实体总线、物理急停和现场验收仍为 Hardware No-Go。</sub></p>

---

## 当前已验证状态

- ROS 2 + MuJoCo 的 Panda 控制、编排、采集与上游物理 Gate 已实现。
- MoveIt Servo、阻抗控制、safety monitor、Task GT、episode 录制有代码与测试证据。
- Isaac 以 adapter + 外部进程方式接入；用于有界执行与 Task GT，不是默认批采栈。
- Policy runtime 合同（chunk、限幅、Hold/E-stop、trace）与 mock/ROS wiring 已验证；默认执行适配仍为 `legacy`；authoritative 在线切流**未启用**。
- Scripted oracle 在修正物理链上 lift **5/5**：证明执行环境能完成任务，不能替代 learned policy。
- Learned policy 有界 Isaac S4（修光权威证据）：interface 5/5，GT lift **0/5** → **Hold**。
- **没有**真实 Franka Panda 部署；**没有**完成 Sim2Real；MuJoCo/Isaac 不是实机证据。

权威跨仓状态与证据索引见中游 [FINAL_PROJECT_SUMMARY.md](https://github.com/inayina/robot-arm-episode-data-lab/blob/main/docs/portfolio/FINAL_PROJECT_SUMMARY.md)。

---

## 快速开始

环境与验收范围见 [PROJECT_SCOPE_AND_ACCEPTANCE.md](docs/PROJECT_SCOPE_AND_ACCEPTANCE.md)。常用仿真采集入口：

```bash
colcon build --symlink-install \
  --packages-select lerobot_recorder teleop_bringup mujoco_sim

source install/setup.bash

ros2 launch teleop_bringup full_system.launch.py \
  record:=true \
  capture_mode:=portfolio \
  camera_rate:=10.0 \
  camera_width:=320 \
  camera_height:=240 \
  auto_record_seconds:=15.0 \
  auto_record_delay_s:=22.0
```

该示例会自动结束录制，不要当作长期常驻服务。

```bash
bin/ask-project "上游 safety monitor 如何触发 Hold 或 E-stop？"
```

中游不在默认路径时：`export EPISODE_DATA_LAB_ROOT=/path/to/robot-arm-episode-data-lab`。

---

## 目录导航

| 路径 | 用途 |
| --- | --- |
| `src/teleop_bringup/` | 系统启动与节点编排 |
| `src/teleop_moveit_config/` | MoveIt / Servo |
| `src/teleop_controllers/` | 控制器插件 |
| `src/safety_monitor/` | 安全监视与 E-stop |
| `src/mujoco_sim/` | 默认仿真后端 |
| `src/isaac_sim_adapter/` | Isaac 适配与策略运行时 |
| `src/canopen_hw_interface/` | CANopen 硬件接口 |
| `src/virtual_servo_driver/` | DS402 虚拟伺服 |
| `src/gripper_driver/` | 夹爪驱动（Mock Modbus） |
| `src/synth_data_gen/` | 批采与 Task GT / Gate |
| `src/lerobot_recorder/` | Episode 录制 |
| `scripts/` | 校验脚本、Isaac/MuJoCo S4 入口 |
| `docs/` | 架构、接口、验收 |
| `media/` | 仿真与验收过程可视化（可选） |
| `tests/` | 启动、Gate、安全、runtime 合同测试 |
| `docs/portfolio/` | 对外作品集母版（[入口](docs/portfolio/README.md) · [PORTFOLIO_SUMMARY](docs/portfolio/PORTFOLIO_SUMMARY.md) · [EVIDENCE_INDEX](docs/portfolio/EVIDENCE_INDEX.md)） |

---

## 跨仓接口

| 交接 | 内容 |
| --- | --- |
| → 中游 | `episode_*/train/` + `meta.json`（含 `upstream_gate`、`success`、安全标志） |
| ← 中游 | 运行时合同镜像（如 S4 JSON）；策略权重不在本仓训练 |
| ↔ 下游 | 共享安全/策略话题语义；下游不拥有上游 Task GT 结论 |

接口说明：[INTER_REPO_CONTRACTS.md](docs/INTER_REPO_CONTRACTS.md)。闭环跑法：中游 [CLOSED_LOOP_RUNBOOK.md](https://github.com/inayina/robot-arm-episode-data-lab/blob/main/docs/CLOSED_LOOP_RUNBOOK.md)。

---

## 边界与未完成事项

**本仓不负责：** schema 适配、Release、训练、离线 Gate、PyBullet replay、distribution monitor、risk 聚合、HOC。

**必须保持的真实性边界：**

- 无真实 Panda 部署；无完成的 Sim2Real。
- Open-loop Pass ≠ 闭环任务成功；Interface Pass ≠ Reach/Grasp/Lift 成功。
- `MockModbusClient` 是**内存寄存器模拟**：不经过 TCP Socket，不是真实 Modbus TCP Server，不是 Modbus RTU/RS485，不构成真实夹爪接入证据。
- 真实 Modbus、香橙派边端、`robot-control-runtime` 等属于**尚未接入**的下一阶段集成方向，代码不在本仓。
- 真实 SocketCAN（`can0`）路径存在启动参数，但**未作为验收通过项**。
- 软件 Hold/E-stop ≠ 认证硬件安全。

进一步阅读：[上游 Agent 映射](docs/AGENTS.md) · [架构](docs/ARCHITECTURE_V2.md) · [项目范围与验收](docs/PROJECT_SCOPE_AND_ACCEPTANCE.md)

---

## 面向招聘者的 30 秒摘要

这是三仓 Panda 系统的**上游执行层**：用 ROS 2 把遥操作/任务/策略动作编排进 MoveIt Servo 与阻抗控制，接入 MuJoCo（及有界 Isaac），用 safety monitor 处理超时与 E-stop，用 Task GT 判定物理结果，并把执行过程录成带 Gate 的 episode。训练与 handoff 在中游；replay 与风险观测在下游。当前无实机、无 Sim2Real；learned policy 有界闭环仍为 Hold。

---

## English Brief

**Upstream ROS 2 arm execution, control orchestration, and device-interface layer** in a three-repo Franka Panda stack. It routes teleop, task FSM, or bounded policy actions through MoveIt Servo and impedance control into MuJoCo (default) or an Isaac adapter, publishes safety and continuous task GT, and records gated episodes.

It does not own data release, training, offline gates, or downstream replay/risk/HOC. `MockModbusClient` is an in-memory register mock, not real Modbus TCP/RTU. No real Panda deployment or completed Sim2Real. Open-loop or interface Pass is not task success; learned-policy bounded Isaac remains Hold (lift 0/5).

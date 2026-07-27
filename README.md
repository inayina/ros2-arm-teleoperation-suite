# ros2-arm-teleoperation-suite

这个仓库负责让 Franka Panda 在仿真里真正“动起来”，并把发生过的事情可靠地记录下来。

它把遥操作输入、批量任务或策略动作送进 ROS 2 控制栈，通过 MoveIt Servo、阻抗控制和 MuJoCo/Isaac 执行机械臂，同时记录 episode、检查安全状态，并用连续任务真值判断是否真的 reach、grasp、lift。训练和数据发布不在这里做；这个仓库专注的是采集与在线执行。

> 可以把它理解为三仓系统的“现场”：动作在这里执行，传感器数据在这里产生，物理任务结果也在这里确认。

## 它解决什么问题

机器人学习需要的不只是关节轨迹，还需要知道轨迹来自什么任务、是否安全、物体有没有真的被抓起，以及采集过程是否使用了会污染训练数据的辅助逻辑。

本仓把这些责任放在同一条实时链路中：

```text
任务目标 / 遥操作 / 策略动作
              │
              ▼
Task planning → MoveIt Servo → impedance control → MuJoCo / Isaac
              │                    │
              │                    ├─ safety / execution status
              │                    └─ continuous task GT
              ▼
episode recorder → episode_*/train/ + meta.json
```

最终得到的不是一个“看起来动过”的 demo，而是一批带来源、状态和物理 Gate 的 episode，以及可以被中游继续处理的明确输入。

## 在三仓系统中的位置

```text
本仓（上游）
  控制 · 仿真 · 采集 · 在线执行 · task GT
             │ raw episode
             ▼
robot-arm-episode-data-lab（中游）
  合同 · 数据 · 训练 · 离线评测 · handoff
             │ actions / reports
             ▼
ros2-moveit-pybullet-bridge（下游）
  replay · monitor · risk · HOC
```

三仓边界的统一说明见中游 [BOUNDARY_FREEZE.md](https://github.com/inayina/robot-arm-episode-data-lab/blob/main/docs/portfolio/BOUNDARY_FREEZE.md)。

## 你会在这里找到什么

- **遥操作与批量采集**：既可以人工控制 Panda，也可以按任务 FSM 批量生成 episode。
- **分层控制**：上层笛卡尔目标通过 MoveIt Servo 和阻抗控制落到仿真机器人。
- **两套仿真环境**：MuJoCo 用于主要采集链，Isaac 用于有界策略执行与任务真值评测。
- **episode 录制**：同步保存关节、末端位姿、夹爪、图像和任务元数据。
- **物理 Gate**：由上游判断 episode 是否成功、安全停止或驱动故障；中游不会从 object pose 再猜一次。
- **Policy Runtime**：在线 inference、chunk scheduler 和 execution adapter 位于本仓；当前默认仍保持受控、非 authoritative 的接入方式。

训练数据必须关闭 grasp assist：`grasp_assist_enabled:=false`。

## 当前状态

- ROS 2 / MuJoCo 的 Panda 控制、采集和 episode Gate 已实现。
- Task、Motion 和 Evaluation Agent 已映射到具体节点和配置。
- Policy Runtime 已具备 chunk10/K5、同步 replan、执行限幅、Hold/E-stop 与 trace 合同。
- Async double-buffer 目前只有中游离线 bench，尚未接入本仓在线节点。
- Scripted oracle 在修正物理链后可完成 lift 5/5；这只证明执行环境具备完成任务的能力。
- Learned policy 的 bounded Isaac 结果仍是 lift 0/5 → Hold。
- 当前没有真实 Panda 部署，也没有完成 Sim2Real。

## 快速开始

环境要求与完整参数见 [PROJECT_SCOPE_AND_ACCEPTANCE.md](docs/PROJECT_SCOPE_AND_ACCEPTANCE.md)。常用的仿真采集入口是：

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

这条命令会自动结束录制；不要把 README 中的示例当成长期常驻服务配置。

只想检查纯 CPU 逻辑或了解项目事实时，可以先从测试和检索入口开始：

```bash
bin/ask-project "上游如何产生一个可用于训练的 Panda episode？"
bin/project-evidence impact --base HEAD~1 --head HEAD
```

如果中游仓不在默认位置，设置：

```bash
export EPISODE_DATA_LAB_ROOT=/path/to/robot-arm-episode-data-lab
```

## 目录怎么读

| 路径 | 用途 |
| --- | --- |
| `src/teleop_bringup/` | 系统启动与节点编排 |
| `src/mujoco_sim/` | MuJoCo Panda 仿真 |
| `src/synth_data_gen/` | batch generation 与上游物理 Gate |
| `src/lerobot_recorder/` | episode 录制和多模态同步 |
| `src/teleop_moveit_config/` | MoveIt / Servo 配置与启动 |
| `src/teleop_controllers/` | 控制器插件，包括笛卡尔阻抗控制 |
| `src/isaac_sim_adapter/` | Isaac 有界执行与 runtime 合同镜像 |
| `docs/` | 架构、接口、验收与运行手册 |
| `media/` | 仿真和采集过程的辅助可视化 |

## 边界

本仓不负责：

- 数据 schema 适配、release、训练和离线模型评测；这些属于中游。
- 下游 PyBullet replay、distribution monitor、risk 和 HOC。
- 真实机械臂驱动、生产安全认证或已经完成的 Sim2Real。
- 用“命令执行完”替代物体真的被 lift 的任务真值。

## 进一步阅读

- [上游 Agent 与模块映射](docs/AGENTS.md)
- [三仓数据接口](docs/INTER_REPO_CONTRACTS.md)
- [项目范围与验收](docs/PROJECT_SCOPE_AND_ACCEPTANCE.md)
- [架构说明](docs/ARCHITECTURE_V2.md)
- [闭环运行手册](https://github.com/inayina/robot-arm-episode-data-lab/blob/main/docs/CLOSED_LOOP_RUNBOOK.md)

## English brief

This is the upstream execution and collection surface of a three-repo Franka Panda system. It runs ROS 2 control and MuJoCo/Isaac simulation, records expert episodes, applies the upstream physical gate, and owns online inference, scheduling, execution adaptation, and task ground truth.

It does not own data releases, policy training, downstream replay/risk validation, real-robot deployment, or completed Sim2Real. A completed command sequence is not considered task success unless the runtime ground truth confirms the physical outcome.

# 上游 Agent 实现映射 (AGENTS.md)

Canonical 总览：中游仓 `robot-arm-episode-data-lab/AGENTS.md` V2.1。

本文件描述 **ros2-arm-teleoperation-suite** 内的实时 Agent 实现与默认约束。

---

## 1. Task Planning Agent

| 项 | 值 |
|----|-----|
| **实现** | `src/synth_data_gen/synth_data_gen/batch_generator.py`（批采）<br>`src/teleop_input/teleop_input_node.py`（遥操作） |
| **输入** | `language_instruction`、目标物体名、bin 配置 |
| **输出** | `/teleop/cmd_pose`、`/teleop/gripper_cmd`、`/teleop/record_trigger` |
| **FSM** | Hover → Descend → Close → Lift → Transport → Place → Release |

批采每集开始前设置 recorder：
- `language_instruction`
- `upstream_gate:=batch_generator`

---

## 2. Motion Planning & Control Agent

| 层 | 包 / 节点 | 职责 |
|----|-----------|------|
| L1 | `safety_monitor` | 限位、watchdog、E-stop |
| L2 | `moveit_servo` / `servo_node` | 笛卡尔伺服 → `/joint_target` |
| L3 | `cartesian_impedance_controller` | 1kHz 阻抗力矩 |
| L4–L5 | CANopen + `mujoco_sim` | 仿真物理 |

**注意**：实时栈使用 MoveIt Servo 笛卡尔跟踪，**不使用 RRT**。

---

## 3. Evaluation Agent（双轨）

### 主轨 — 批采门禁（决定 discard / stop_success）
- **实现**：`batch_generator._validate_episode`
- **判定**：lift_delta、bin XY、reset/language 参数是否就绪
- **落盘**：仅 accepted episode；`meta.json` → `upstream_gate=batch_generator`

### 辅轨 — 实时物理监督
- **实现**：`grasp_monitor_node`（`GraspStateEstimator`）
- **话题**：`/grasp/status`
- **默认**：`full_system.launch.py` → `enable_grasp_monitor:=true`

### 安全监督
- **实现**：`safety_monitor` → `/safety/status`、`/safety/estop`

---

## 4. 数据采集 Agent

| 项 | 值 |
|----|-----|
| **实现** | `lerobot_recorder/recorder_node.py` |
| **触发** | `/teleop/record_trigger`：`start` / `stop_success` / `discard` |
| **参数** | `upstream_gate`（默认 `teleop`；批采时设为 `batch_generator`） |
| **输出** | `episode_*/train/` + `meta.json` |

---

## 5. 默认 Launch 约束（训练级批采）

`teleop_bringup/full_system.launch.py` 默认：

```yaml
grasp_assist_enabled: false      # 禁止模拟吸附进训练集
enable_grasp_monitor: true       # 辅轨评测
```

Preflight：`scripts/run_batch_preflight_smoke.sh`

日常追加：`scripts/collect_daily_episodes.sh` → 持久库 `data/episodes/`

归档工具：`scripts/episode_archive.py`（`import` / `status` / `next-index`）

Validation：`scripts/validate_dataset.py data/episodes --min-frames 5`

---

## 6. 本仓不负责

- Schema 适配、release、策略训练（中游）
- PyBullet policy replay（下游）

契约：[docs/INTER_REPO_CONTRACTS.md](INTER_REPO_CONTRACTS.md)

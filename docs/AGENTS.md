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
| L3 | `cartesian_impedance_controller` | 阻抗力矩（仿真 `500 Hz` / 真机 `1 kHz`） |
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

## 5. Policy Runtime M1–M6（默认 legacy）

- **合同来源**：中游 `panda_policy_runtime_v1` SHA lock。
- **消息**：`teleop_interfaces/{PolicyCommand,PolicyExecutionReport,TaskEvaluationStatus}`。
- **实现**：`isaac_sim_adapter/policy_runtime.py`（Backend、Lifecycle、native chunk10 / K5 Scheduler）、`policy_execution_adapter.py`（absolute EEF8 / delta EEF7 shadow 裁决）与 `policy_runtime_ros.py`（QoS、command/health/report ROS 映射）。
- **接入**：`smolvla_policy_inference_node.py` 参数 `execution_adapter_mode=legacy|shadow|authoritative`，节点默认 `legacy`；旧 `policy_runtime_shadow_enabled` 仅保留兼容。S4 专用脚本显式选择 `authoritative`。
- **已实现输出**：开启且 `dry_run=true` 后发布 shadow `/policy/command`、`/policy/runtime_health` 与 `/policy/execution_report`；M2 节点拒绝 shadow + 非 dry-run 组合。
- **Task GT live mirror**：canonical Isaac/MuJoCo continuous GT recorder 发布 `/task/evaluation_status`；等待 privileged object pose 时为 `UNAVAILABLE`，运行中为 `RUNNING`，结束发布 `PASS/FAIL`，且 `risk_may_override=false`。
- **已验证**：canonical S4 已有 telemetry 750 actions 的 bounded/clip parity；CPU、schema 与 mock ROS report tests 通过。
- **M4 已实现**：订阅 `/policy/runtime_hold`；R2 清 queue 并在 authoritative 模式保持当前位置，R3 清 queue 并服从 safety latch；authoritative 模式在首个目标前检查 pose/gripper publisher count。
- **M6 已验证**：mock PolicyBackend 的真实 ROS/DDS wiring 已完成 RUN→R2 Hold→R3 E-stop 与 HOC trace；health 现显式携带 `last_command_sequence`、`trace_run_id`、`episode_id`，避免跨 topic 到达顺序造成误关联。
- **异步 chunk 与远程推理已实现（非 Isaac 验收）**：`policy_runtime_async_chunk_enabled=true` 时，单独 worker 执行 native `predict_action_chunk`；latest-only pending/result slot 防止陈旧 backlog；控制定时器独立消费 K=5；可选异步 synthetic 双相机 warmup 的结果永不进入动作队列；queue underrun / stale 在 authoritative 路径保持当前位置并等待新鲜 chunk。新增 `RemoteSmolVlaPolicyBackend` 与 loopback HTTP `smolvla_remote_inference_v1`，health/report 包含请求、完成、失败、warmup、pending/result drop 与 underrun 计数。
- **当前限制**：远程服务通过 SSH 隧道的 5 次真实双相机请求 p50≈323 ms、p95≈325 ms，证明协议吞吐进入 500 ms 重规划窗；该证据不包含 Isaac，也不代表在线切流、实时性 Gate 或抓取成功。若单 chunk 推理慢于 K=5 的 0.5 s 消费窗，异步结构仍会 fail-closed Hold，不能凭队列弥补算力缺口。
- **执行权威**：节点默认仍为 `legacy`，便于回滚；S4 脚本是显式例外。不得把 authoritative 可选代码路径或异步测试写成已经在线接管或任务成功。

---

## 6. 默认 Launch 约束（训练级批采）

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

## 7. 本仓不负责

- Schema 适配、release、策略训练（中游）
- PyBullet policy replay（下游）

契约：[docs/INTER_REPO_CONTRACTS.md](INTER_REPO_CONTRACTS.md)

---

## 8. Project Evidence Agent 集成

Project Evidence Agent 的 registry、检索、audit 和 impact 核心由中游
`robot-arm-episode-data-lab/project_knowledge/` 维护；本仓只提供薄入口，不重复实现知识检索逻辑。

```bash
# 项目事实查询
bin/ask-project "上游 batch_generator 物理门禁如何实现？"

# 三仓 audit
bin/project-evidence audit --json-out /tmp/project-audit.json --markdown-out /tmp/project-audit.md

# 本仓 Git 影响分析；wrapper 自动注入 upstream repository 名
bin/project-evidence impact --base HEAD~1 --head HEAD
```

若中游不在自动 fallback 路径，设置：

```bash
export EPISODE_DATA_LAB_ROOT=/path/to/robot-arm-episode-data-lab
```

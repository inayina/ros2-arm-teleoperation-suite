# 三仓数据闭环与交接契约速查

用于 `ros2-arm-teleoperation-suite`、`robot-arm-episode-data-lab`、`ros2-moveit-pybullet-bridge` 之间的职责边界、字段契约和排错入口。

## 仓库职责

| 仓库 | 角色 | Owns | Does not own |
|---|---|---|---|
| `ros2-arm-teleoperation-suite` | 上游运行时与采集 | ROS2/MuJoCo、批量采集、raw HF episodes、仿真验证、recorder schema | 数据清洗发布、训练 run、checkpoint 质量声明 |
| `robot-arm-episode-data-lab` | 中游数据+训练仓库 | raw import、schema 适配、过滤、splits、release manifest、dataset inspection、ACT/Diffusion 训练、checkpoint/export | ROS2 runtime、MuJoCo 控制链路、raw episode 采集 |
| `/home/ina/ros2_ws/src/ros2-moveit-pybullet-bridge` | 下游 bridge/runtime | MoveIt/PyBullet bridge、策略回放/部署验证、运行时监控、ROS topic 适配 | raw episode 录制、数据清洗 release、批量采集、策略训练 |

## Contract A: Runtime -> Dataset Lab

上游 recorder 写一个 episode 一个 HuggingFace Dataset：

```text
data/episodes/
└── episode_{episode_index:06d}/
    └── train/
        ├── dataset_info.json
        ├── state.json
        └── data-*.arrow
```

上游 raw frame 必备字段：

| Field | Shape/type | 说明 |
|---|---|---|
| `observation.state` | float32 `[7]` | Panda measured joint state |
| `observation.gripper` | float32 `[1]` | 夹爪开合/状态 |
| `action` | float32 `[8]` | 7 维 arm action + 1 维 gripper command |
| `observation.ee_pose` | float32 `[7]` | `[x, y, z, qx, qy, qz, qw]` |
| `observation.object_pose` | float32 `[7]` | 目标物体 privileged pose，用于过滤/调试 |
| `observation.ft` | float32 `[6]` | 末端 force/torque |
| `observation.images.scene` | uint8 `[H, W, 3]` | scene RGB |
| `observation.images.wrist` | uint8 `[H, W, 3]` | wrist RGB |
| `observation.images.tactile_left/right` | uint8 `[H, W, 3]` | 左/右 GelSight-like 触觉 RGB |
| `observation.depth.scene` | float32 `[H, W]` | scene depth |
| `timestamp` | float64 | 同步时间戳 |
| `episode_index`, `frame_index` | int64 | episode/frame 索引 |
| `done` | bool | 终止帧 |
| `task` | string | 任务标签 |
| `language_instruction` | string | 自然语言任务描述 |
| `success` | bool | 是否为可训练成功示范 |

可选过滤字段：`safety_estop`、`drive_fault`、`failure_reason`。

默认交给中游前必须满足：

- `python3 scripts/validate_dataset.py <episode_root> --min-frames 5` 通过。
- `language_instruction` 非空。
- 默认训练候选帧全部 `success=true`。
- 无 `safety_estop=true` 或 `drive_fault=true` 污染帧。
- 批量采集前已经完成 smoke、小样本和混合 soak 成功率门槛。

## Contract B: Dataset/Training Lab -> Downstream Bridge

中游 data lab 负责显式适配 release schema，并在同仓内完成 ACT/Diffusion 训练、评估和 checkpoint/export。下游 bridge 消费中游产物：

- 可将上游 `observation.state[7] + observation.gripper[1]` 合成为训练 `state[8]`。
- 可将上游绝对动作派生为训练动作，例如 `ee_delta_gripper[7]`。
- release manifest 应记录 raw source、schema、splits、filter rules、language/action/state 维度、是否含 `success` 标签。
- training manifest 应记录 dataset release、policy type、训练配置、normalization、metrics、checkpoint 路径。
- 默认 release/filter rules 应要求 `success=true`，并排除 `safety_estop=true`、`drive_fault=true`。

不要在上游或下游 bridge 静默修改字段维度；任何 `action[8] -> action[7]`、`state[7] -> state[8]` 都必须在中游 adapter/release manifest 中可追踪。

## Contract C: Middle Repo Policy Artifact -> Downstream Bridge

中游 `/home/ina/robot-sim-lab/robot-arm-episode-data-lab` 导出 runtime-facing policy artifact。下游 `/home/ina/ros2_ws/src/ros2-moveit-pybullet-bridge` 消费 artifact 做回放、部署验证和监控。artifact 至少应包含：

```text
checkpoints/<run_name>/
├── checkpoint.pt
├── policy_config.yaml
├── normalization.json
├── metrics.json
└── manifest.yaml
```

`manifest.yaml` 至少声明：

- `policy.type`: `act` 或 `diffusion_policy`
- `policy.checkpoint`
- `policy.config`
- `policy.normalization`
- `dataset.schema`
- `dataset.source_manifest`
- `io.observation_keys`
- `io.action_key`
- `io.action_dim`
- `runtime.publish`

下游 bridge 只消费 immutable artifact 做加载、回放、MoveIt/PyBullet 适配和运行时验证；训练质量、benchmark 数字、checkpoint 选择属于中游数据+训练仓库，不属于上游采集仓或下游 bridge。

## Contract D: Feedback Loop -> Upstream

三仓必须形成闭环优化：中游和下游的结果需要反向影响上游采集、仿真、recorder 和 validation，但回流物必须是轻量、可审计、可版本化的反馈 artifact，而不是数据本体。

允许回流到上游本仓：

- `reports/` 或 `docs/` 下的质量报告：失败类别、每类成功率、过滤原因、schema 缺失字段、动作维度问题、语言模板问题。
- 采集配置建议：目标物体分布、domain randomization 范围、相机/触觉视角、episode 长度、任务语言模板、validation 阈值。
- 下游 replay/部署验证摘要：MoveIt 规划失败类型、动作超限、频率不匹配、坐标系或 quaternion 顺序问题、夹爪命令兼容性问题。
- 接口契约和 manifest 示例：schema version、policy IO requirements、runtime topic mapping、normalization key 约定。
- 极小 fixtures：只用于单测或回归的 tiny episode / mock manifest / failure case，不作为训练数据。

推荐模板：

- `docs/templates/feedback_report.yaml`
- `docs/templates/collection_tuning_suggestion.yaml`
- `docs/templates/downstream_replay_summary.yaml`

不要回流到上游本仓：

- 中游清洗后的完整 dataset release、splits、parquet/arrow 大文件或统计缓存。
- ACT/Diffusion checkpoint 大文件、训练日志、wandb 目录、tensorboard 全量记录。
- 下游 replay 生成的大规模运行日志、视频或 rollout 数据。
- 中游为了训练派生出的完整 `state[8]`、`ee_delta_gripper[7]` 数据副本。

回流落点建议：

- 采集/成功率问题：更新 `docs/sorting_dev_guide.md`、`batch_generator` 参数或 validation 阈值。
- schema/接口问题：更新 `docs/INTER_REPO_CONTRACTS.md`、`references/three_repo_handoff.md` 和小型 schema fixture。
- 控制/部署问题：更新 controller/safety/recorder 的 runtime 参数、topic 契约或下游 bridge 适配说明。

## 常见边界错误

- 在上游本仓直接跑大训练或写训练报告。
- 在中游 data lab 放 ROS2 节点、MuJoCo 控制逻辑或 recorder runtime。
- 在下游 bridge 清洗 raw episode、补录数据、改 release schema 或训练策略。
- 把 `action[8]` 静默截断成 `action[7]`。
- 把 `observation.state[7]` 当成训练 `state[8]`，但没有合并 gripper。
- 低成功率时直接开 100+ 或过夜批采。
- 用 dataset 目录存在来证明可训练，而没有跑 validation 和成功率门槛。
- 把回流闭环做成“大文件搬回上游”，导致采集仓变成数据湖或训练产物仓。
- `use_sim:=true` 的 MuJoCo 演示里声称走了 CANopen/DS402 硬件证据。

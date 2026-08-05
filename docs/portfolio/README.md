# 作品集入口 —— ros2-arm-teleoperation-suite（三仓闭环 · 上游）

三仓闭环的**上游（在线执行层）**对外作品集入口。对外主语与口径以中游 canonical 为准（冻结主语：**具身策略数据治理与分层验证框架**）。

## 导航

| 文档 | 用途 |
| --- | --- |
| [PORTFOLIO_SUMMARY.md](PORTFOLIO_SUMMARY.md) | **对外母版**：仓库定位、技术栈、核心能力（逐项证据分类）、Gate 协议角色、证据状态、能证明/不能证明、面试可讲点、边界声明 |
| [EVIDENCE_INDEX.md](EVIDENCE_INDEX.md) | 证据资产索引：33 条资产，keep / relabel / regenerate 状态与「script 待确认」如实标注，含每条「能证明 / 不能证明」列 |

## 30 秒摘要

三仓 Panda 系统的上游执行层：把遥操作、任务 FSM 或有界策略动作编排进 MoveIt Servo 与笛卡尔阻抗控制，接入 MuJoCo（默认）与有界 Isaac adapter，用 safety monitor 处理 watchdog 限位与 E-stop，用双轨 Evaluation（批采门禁 + grasp_monitor）与连续 Task GT 判定物理结果，并把执行过程录成带 `upstream_gate` / `success` 字段的 episode 交给中游。训练、release、handoff 在中游；回放与风险观测在下游。

## 边界速记

- Not task success · Not Sim2Real · Not real robot（无真实 Panda 部署）。
- SmolVLA：Recovery v3 离线 open-loop Pass（冻结 `eval_gate_v3`）≠ 任务成功；有界 Isaac S4 权威 GT lift **0/5** → **Hold**；**默认停止**（不扩种子、不重训、不新增采集）。
- Policy Runtime 默认执行适配 `legacy`；`authoritative` 在线切流未启用。
- `MockModbusClient` 为内存寄存器模拟；真实 SocketCAN 未验收；软件 Hold/E-stop ≠ 认证硬件安全。

## 权威入口（中游 canonical，口径冲突时以中游为准）

- [THREE_REPO_CANONICAL_FACTS.md](https://github.com/inayina/robot-arm-episode-data-lab/blob/main/docs/portfolio/THREE_REPO_CANONICAL_FACTS.md) —— 三仓统一事实源
- [FINAL_PROJECT_SUMMARY.md](https://github.com/inayina/robot-arm-episode-data-lab/blob/main/docs/portfolio/FINAL_PROJECT_SUMMARY.md) —— 统一收口
- [BOUNDARY_FREEZE.md](https://github.com/inayina/robot-arm-episode-data-lab/blob/main/docs/portfolio/BOUNDARY_FREEZE.md) —— 边界冻结与提交冻结
- [EVIDENCE_INDEX.md](https://github.com/inayina/robot-arm-episode-data-lab/blob/main/docs/portfolio/EVIDENCE_INDEX.md) —— 中游证据索引

三仓规范 V2.1：中游根 `AGENTS.md`（canonical）；本仓实现映射：`../AGENTS.md`。

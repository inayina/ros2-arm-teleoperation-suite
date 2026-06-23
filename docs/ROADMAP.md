# 开发路线图：`ros2-arm-teleoperation-suite`

**版本**：v0.1  
**更新日期**：2026-06-23  
**目标**：6 周内完成可演示的五层机械臂遥操作全链路系统

---

## 总览

```
M0  M1          M2          M3              M4      M5    M6
│   │           │           │               │       │     │
▼   ▼           ▼           ▼               ▼       ▼     ▼
预适配  CAN/RS485   MuJoCo桥接  阻抗控制器     全链路   录制  收尾
(done) [Week 1]  [Week 2]   [Week 3-4]    [Week 5][W5]  [W6]
```

---

## 里程碑总表

| 里程碑 | 分支 | 核心目标 | 验收标准 | 预计用时 |
|---|---|---|---|---|
| **M0** | *(episode-data-lab 项目)* | LeRobot hf_dataset 预适配 | `export_to_lerobot.py` dry-run 通过 | **Done ✅** |
| **M1** | `feat/can-rs485-layer` | CAN + RS485 通信层 | vcan0 收发帧，Modbus RTU 通 | Week 1 |
| **M2** | `feat/mujoco-ros2-bridge` | MuJoCo × ROS2 桥接 | Panda 响应 `/joint_torque_cmd` | Week 2 |
| **M3** | `feat/impedance-controller` | C++ 阻抗控制器 | 末端误差 < 2mm，柔顺响应可见 | Week 3-4 |
| **M4** | `feat/full-pipeline` | 全链路贯通 | 键盘→CAN→MuJoCo→反馈，闭环稳定 | Week 5 |
| **M5** | `feat/lerobot-recorder` | LeRobot 数据录制 | 50 步 Episode 可 load_from_disk | Week 5 |
| **M6** | `feat/polish` | 收尾发布 | README + 演示视频 + Bullet Points | Week 6 |

---

## 分支策略

```
main
├── feat/can-rs485-layer      ← M1 开发分支
├── feat/mujoco-ros2-bridge   ← M2 开发分支
├── feat/impedance-controller ← M3 开发分支
├── feat/full-pipeline        ← M4 集成分支（rebase from M1/M2/M3）
├── feat/lerobot-recorder     ← M5 开发分支
└── feat/polish               ← M6 收尾分支
```

**合并规则**：
- 每个 feat 分支 PR → main，保持 main 始终可运行
- M4 `full-pipeline` 从 M1/M2/M3 merge 后集成测试
- commit 格式：`type(scope): message`，例如 `feat(can_bridge): add PDO encoder feedback`

---

## 分支 SPEC 文件索引

| 文件 | 对应里程碑 |
|---|---|
| [SPEC_M1_CAN_RS485.md](./SPEC_M1_CAN_RS485.md) | M1：CAN + RS485 通信层 |
| [SPEC_M2_MUJOCO_BRIDGE.md](./SPEC_M2_MUJOCO_BRIDGE.md) | M2：MuJoCo × ROS2 桥接 |
| [SPEC_M3_IMPEDANCE_CTRL.md](./SPEC_M3_IMPEDANCE_CTRL.md) | M3：C++ 阻抗控制器 |
| [SPEC_M4_FULL_PIPELINE.md](./SPEC_M4_FULL_PIPELINE.md) | M4：全链路集成 |
| [SPEC_M5_LEROBOT_RECORDER.md](./SPEC_M5_LEROBOT_RECORDER.md) | M5：LeRobot 数据录制 |

---

## 开发检查清单（逐周）

### Week 1 — M1: CAN / RS485 层

- [ ] `scripts/setup_vcan.sh` 可成功创建 vcan0 接口
- [ ] `can_bridge_node.py` 启动，能向 vcan0 发送 PDO 帧
- [ ] `can_bridge_node.py` 能接收 vcan0 回环帧并发布 `/joint_states`
- [ ] `rs485_modbus_node.py` 启动 Modbus TCP Server 仿真
- [ ] `/gripper_cmd` 写入 → Modbus 寄存器 `0x0040` 变化
- [ ] `/gripper_state` 正确回读寄存器 `0x0041`
- [ ] `tests/test_can_bridge.py` 全通过

### Week 2 — M2: MuJoCo 桥接

- [ ] `config/models/franka_panda.xml` 下载就位
- [ ] `mujoco_sim_node.py` 启动，MuJoCo viewer 弹出
- [ ] 发布 `/joint_torque_cmd` → Panda 关节运动可见
- [ ] `/ft_sensor` 在末端接触时有非零力矩输出
- [ ] `/joint_states` 以 100Hz 稳定发布
- [ ] 运行 10 秒无崩溃

### Week 3-4 — M3: 阻抗控制器

- [ ] `impedance_controller_node.cpp` 编译通过（colcon build）
- [ ] KDL Chain 从 URDF 正确构建，7 个关节
- [ ] 给定目标位姿，末端跟踪误差 < 2mm（见 SPEC_M3）
- [ ] 接触力 > 5N 时自动切换柔顺模式
- [ ] `tests/test_impedance_controller.cpp` GTest 全通过
- [ ] 控制频率 ≥ 500Hz（MultiThreadedExecutor 验证）

### Week 5 — M4: 全链路 + M5: 录制

- [ ] `launch/full_system.launch.py` 一键启动所有 5 个节点
- [ ] 键盘 W/A/S/D 输入 → MuJoCo Panda 运动
- [ ] 按 G → 夹爪开合
- [ ] 端到端延迟 < 50ms（`/master_pose` → MuJoCo 渲染）
- [ ] 按 R 开始录制 → 按 R 停止 → `data/episodes/` 生成文件
- [ ] `datasets.load_from_disk()` 读取 Episode，字段完整
- [ ] 录制 50 步 Episode 文件大小合理（< 10MB）

### Week 6 — M6: 收尾

- [ ] `README.md` 中英双语，包含架构图、演示 GIF
- [ ] 简历 Bullet Points 填写量化数据（延迟 ms、误差 mm）
- [ ] `media/` 目录包含演示视频
- [ ] `requirements.txt` 锁版本，`scripts/install_deps.sh` 可用

---

## 关键风险与应对

| 风险 | 影响 | 应对 |
|---|---|---|
| KDL IK 收敛慢 | M3 控制频率不足 | 降维到位置控制，阻抗层叠加 |
| MuJoCo viewer 无 GPU | M2 渲染卡顿 | 使用 `mujoco.viewer` offscreen 模式 |
| vcan0 内核模块缺失 | M1 全阻塞 | 预先 `apt install linux-modules-extra-$(uname -r)` |
| colcon 构建 KDL 失败 | M3 全阻塞 | 备选：用 Python 版 KDL（pykdl-utils）先通路 |

---

*路线图随开发进度更新，每个里程碑合并到 main 后勾选对应条目。*

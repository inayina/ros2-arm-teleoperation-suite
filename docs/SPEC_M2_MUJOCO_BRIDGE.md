> 🗄 **归档（V1）**。V2 中 MuJoCo 为纯物理服务器，见 [`ARCHITECTURE_V2.md`](./ARCHITECTURE_V2.md) §6.4–6.5。

# SPEC M2: MuJoCo × ROS2 桥接

**分支**：`feat/mujoco-ros2-bridge`  
**里程碑**：M2  
**预计用时**：Week 2（5 天）  
**负责模块**：`src/mujoco_sim/`

---

## 1. 目标

构建 MuJoCo v3 物理仿真与 ROS2 之间的双向数据桥：

1. ROS2 发布的关节力矩指令 → 驱动 Franka Panda 关节运动
2. MuJoCo 物理引擎输出的关节状态 + 末端接触力 → 发布到 ROS2 话题

---

## 2. 技术选型与理由

### 2.1 物理仿真引擎：MuJoCo v3

| 方案 | 优点 | 缺点 | 结论 |
|---|---|---|---|
| **MuJoCo v3** ✅ | Google DeepMind 维护；接触力模型精确；LeRobot 官方生态；Python API 简洁 | XML 格式学习曲线 | **选用** |
| PyBullet | 已有项目使用经验 | 接触力模型不精确；已停止维护 | 否（已在 M0 项目用过） |
| Gazebo / ROS2 gz_sim | ROS2 原生集成 | 配置复杂；资源消耗大 | 否 |
| Isaac Sim | 渲染真实 | 需 GPU；License 限制 | 否 |

**选用理由**：MuJoCo 的 `mjData.sensor()` 接口能精确读取末端六维力矩，这是阻抗控制器（M3）的关键输入。且 `franka_panda.xml` 在 mujoco_menagerie 有官方维护模型。

### 2.2 ROS2 与 MuJoCo 的线程模型：单进程双线程

| 方案 | 优点 | 缺点 | 结论 |
|---|---|---|---|
| **单进程：MuJoCo 主循环 + rclpy spin 线程** ✅ | 零 IPC 延迟；状态共享简单 | 需手动管理线程安全 | **选用** |
| 独立进程 + ROS2 话题通信 | 解耦好 | 进程间通信引入 > 1ms 额外延迟 | 否 |
| ros2_control 框架 | 标准化 | 配置复杂；过度工程 | 备选（M6 可选升级） |

**选用理由**：控制回路延迟敏感，ROS2 话题的序列化/反序列化开销在 1ms 量级。单进程共享内存的方式能将 MuJoCo → 控制器的延迟压到 < 0.1ms。

### 2.3 MuJoCo Viewer：`mujoco.viewer.launch_passive`

```python
# 推荐方式：非阻塞 passive viewer，主循环保持控制权
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
```

- `launch_passive` 在子线程中渲染，主线程控制物理步进
- 无 GPU 环境可加 `--no-render` 参数跳过 viewer，headless 运行

---

## 3. 接口定义

### 3.1 mujoco_sim_node 订阅

| Topic | 类型 | 说明 |
|---|---|---|
| `/joint_torque_cmd` | `sensor_msgs/JointState` | `name` = 关节名，`effort` = 力矩（N·m） |

### 3.2 mujoco_sim_node 发布

| Topic | 类型 | 频率 | 说明 |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | 100Hz | 7 关节位置/速度 |
| `/ft_sensor` | `geometry_msgs/WrenchStamped` | 100Hz | 末端六维力矩 |

### 3.3 关节名映射（Franka Panda）

```python
JOINT_NAMES = [
    "panda_joint1", "panda_joint2", "panda_joint3",
    "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7",
]

# MuJoCo actuator 名（franka_panda.xml 中定义）
ACTUATOR_NAMES = [
    "actuator1", "actuator2", "actuator3",
    "actuator4", "actuator5", "actuator6", "actuator7",
]
```

---

## 4. 文件清单

```
src/mujoco_sim/
├── package.xml
├── setup.py
└── mujoco_sim/
    ├── __init__.py
    └── mujoco_sim_node.py        ← 核心节点

config/
└── models/
    └── franka_panda.xml           ← 从 mujoco_menagerie 获取
    └── franka_panda_scene.xml     ← 桌面 + 物体场景（M5 录制用）
```

---

## 5. 关键实现细节

### 5.1 物理步进与 ROS2 发布循环

```python
PHYSICS_FREQ = 1000   # Hz，MuJoCo 步进频率
PUBLISH_FREQ = 100    # Hz，ROS2 话题发布频率
PUBLISH_EVERY = PHYSICS_FREQ // PUBLISH_FREQ  # 每 10 步发布一次

step_count = 0
while rclpy.ok() and viewer.is_running():
    # 1. 应用最新力矩指令（来自 ROS2 回调，有锁保护）
    with self._cmd_lock:
        if self._latest_cmd is not None:
            self._apply_torque_cmd(self._latest_cmd)

    # 2. 物理步进（1kHz）
    mujoco.mj_step(self.model, self.data)
    step_count += 1

    # 3. 以 100Hz 发布 ROS2 话题
    if step_count % PUBLISH_EVERY == 0:
        self._publish_joint_states()
        self._publish_ft_sensor()

    # 4. 同步 viewer（渲染频率由 viewer 自动节流到 60Hz）
    viewer.sync()
```

### 5.2 末端接触力读取

```python
def _publish_ft_sensor(self) -> None:
    """读取 MuJoCo 传感器数据并发布 WrenchStamped。"""
    # franka_panda.xml 需定义 ee_force_sensor 和 ee_torque_sensor
    force  = self.data.sensor("ee_force_sensor").data.copy()   # (3,) N
    torque = self.data.sensor("ee_torque_sensor").data.copy()  # (3,) N·m

    msg = WrenchStamped()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = "panda_ee"
    msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = force
    msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = torque
    self._ft_pub.publish(msg)
```

### 5.3 Franka Panda 模型获取

```bash
# 下载 franka_panda.xml（mujoco_menagerie 官方）
wget -q https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/main/franka_emika_panda/panda.xml \
     -O config/models/franka_panda.xml
wget -q https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/main/franka_emika_panda/scene.xml \
     -O config/models/franka_panda_scene.xml
# 同时下载 assets/（mesh 文件）
```

> ⚠️ **注意**：`franka_panda.xml` 默认没有末端力传感器定义，需手动在 `<sensor>` 块添加：
> ```xml
> <sensor>
>   <force  name="ee_force_sensor"  site="attachment_site"/>
>   <torque name="ee_torque_sensor" site="attachment_site"/>
> </sensor>
> ```

---

## 6. 验收标准

### 必须通过（阻塞合并）

| # | 验收项 | 验证方法 |
|---|---|---|
| AC-1 | `mujoco_sim_node.py` 启动后 MuJoCo viewer 弹出，Franka Panda 静止站立 | 视觉确认 |
| AC-2 | 向 `/joint_torque_cmd` 发布非零力矩（joint1 = 10 N·m），Panda 关节 1 开始旋转 | `ros2 topic pub` + 视觉确认 |
| AC-3 | `/joint_states` 以 100Hz 稳定发布（`ros2 topic hz` 测量 95–105Hz） | `ros2 topic hz /joint_states` |
| AC-4 | `/ft_sensor` 在 Panda 末端与地面接触时输出非零力（fz > 5N） | 手动下压末端后 `ros2 topic echo` |
| AC-5 | 节点运行 60 秒无崩溃、无内存泄漏增长 | `top` 监控 RSS |
| AC-6 | `--no-render` 参数下可 headless 运行（不弹 viewer），适用于 CI | 命令行参数测试 |

### 加分项

- [ ] 发布频率在满负载（M3 阻抗控制器运行时）仍 ≥ 95Hz
- [ ] 支持加载自定义场景 XML（桌面 + 物体）为 M5 录制做准备
- [ ] `mjModel` 和 `mjData` 正确序列化为 state，支持重置到初始状态

---

## 7. 面试话术关键点

> "MuJoCo 节点的核心设计决策是**线程模型**：物理步进在主线程以 1kHz 运行，ROS2 spin 在独立线程处理订阅回调，两者通过带锁的共享变量传递力矩指令。这样既保证了物理仿真的时间精度，又不会让 ROS2 的回调调度影响仿真频率——这和实际机器人控制器的实时/非实时分层设计是同一个思路。"

> 🗄 **归档（V1）**。V2 七层集成与 launch 架构见 [`ARCHITECTURE_V2.md`](./ARCHITECTURE_V2.md) §5。

# SPEC M4: 全链路集成

**分支**：`feat/full-pipeline`  
**里程碑**：M4  
**预计用时**：Week 5 前半段（3 天）  
**前置依赖**：M1（CAN/RS485）、M2（MuJoCo）、M3（阻抗控制器）全部合并到 main

---

## 1. 目标

将 M1/M2/M3 三个独立模块集成为可演示的全链路系统：

```
键盘输入 → /master_pose → 阻抗控制器 → /joint_torque_cmd
        → CAN Bridge → vcan0 → MuJoCo → /joint_states + /ft_sensor
        → 反馈回控制器（闭环）
```

---

## 2. 技术选型与理由

### 2.1 启动框架：ROS2 Launch（Python API）

| 方案 | 优点 | 缺点 | 结论 |
|---|---|---|---|
| **`full_system.launch.py`** ✅ | 声明式；节点参数集中管理；可条件启动 | — | **选用** |
| 多个 shell 脚本手动启动 | 简单 | 无法统一管理生命周期 | 否 |
| Docker Compose | 隔离好 | 与 ROS2 DDS 路由复杂 | 否 |

### 2.2 延迟测量：ROS2 Header Timestamp 差分

```python
# 在 mujoco_sim_node 中，计算从 /master_pose 到渲染的延迟
latency_ms = (now - master_pose_msg.header.stamp).nanoseconds / 1e6
```

**选用理由**：不引入外部工具，利用 ROS2 消息头时间戳做端到端延迟跟踪，是 ROS2 性能分析的标准方法。

### 2.3 `teleop_input` 节点：pynput 键盘监听

| 方案 | 优点 | 缺点 | 结论 |
|---|---|---|---|
| **pynput** ✅ | 跨平台；无 root；非阻塞 | — | **选用** |
| curses | 标准库 | 需独占终端 | 否 |
| pygame | 功能多 | 依赖重 | 否 |

---

## 3. 系统集成设计

### 3.1 节点启动顺序与依赖

```
1. setup_vcan.sh          （预启动脚本，非 ROS2 节点）
2. mujoco_sim_node        （最先启动，等待仿真稳定 2s）
3. can_bridge_node        （依赖 vcan0 接口存在）
4. rs485_modbus_node      （独立，无硬依赖）
5. impedance_controller   （依赖 /joint_states 和 /ft_sensor 已发布）
6. teleop_input_node      （最后启动，等用户准备好）
```

### 3.2 `full_system.launch.py` 结构

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 0. 初始化 vcan0
        ExecuteProcess(cmd=["bash", "scripts/setup_vcan.sh"]),

        # 1. MuJoCo 仿真
        Node(package="mujoco_sim", executable="mujoco_sim_node",
             parameters=["config/controller_params.yaml"]),

        # 2. CAN Bridge（延迟 1s 启动，等 vcan0 ready）
        TimerAction(period=1.0, actions=[
            Node(package="can_bridge", executable="can_bridge_node",
                 parameters=["config/can_config.yaml"]),
        ]),

        # 3. RS485 Modbus
        Node(package="can_bridge", executable="rs485_modbus_node"),

        # 4. 阻抗控制器（延迟 2s 启动，等 MuJoCo 发布 /joint_states）
        TimerAction(period=2.0, actions=[
            Node(package="impedance_controller",
                 executable="impedance_controller_node",
                 parameters=["config/controller_params.yaml"]),
        ]),

        # 5. 遥操作输入（延迟 3s 启动，等用户准备）
        TimerAction(period=3.0, actions=[
            Node(package="teleop_input", executable="teleop_input_node"),
        ]),
    ])
```

### 3.3 teleop_input 键盘映射

```python
KEYBOARD_MAP = {
    Key.w:     ("x", +1),   # 末端 +X
    Key.s:     ("x", -1),   # 末端 -X
    Key.a:     ("y", -1),   # 末端 -Y
    Key.d:     ("y", +1),   # 末端 +Y
    Key.q:     ("z", +1),   # 末端 +Z（上升）
    Key.e:     ("z", -1),   # 末端 -Z（下降）
    Key.up:    ("pitch", +1),
    Key.down:  ("pitch", -1),
    Key.left:  ("yaw", +1),
    Key.right: ("yaw", -1),
}
STEP_SIZE = 0.005   # m（每次按键末端移动 5mm）
ROT_STEP  = 0.05    # rad（每次按键旋转 ~3°）

# 特殊键
# G: /gripper_cmd 切换 0.0 ↔ 1.0
# R: 触发 /episode/status → "record_start" / "record_stop"
# Space: 急停，发布零速度 /master_pose（保持当前位置）
```

---

## 4. 端到端延迟预算

| 段落 | 预算 | 实现方式 |
|---|---|---|
| 键盘事件 → `/master_pose` 发布 | < 5ms | pynput callback 直接 publish |
| `/master_pose` → 阻抗控制器计算完成 | < 2ms | C++ KDL + Eigen，500Hz 控制循环 |
| `/joint_torque_cmd` → CAN 帧发送 | < 1ms | python-can socket 直发 |
| vcan0 回环 → `/joint_states` 更新 | < 2ms | 接收线程 + publish |
| `/joint_states` → MuJoCo 步进 → viewer 渲染 | < 10ms | 1kHz 步进，viewer 60Hz 刷新 |
| **总端到端延迟目标** | **< 20ms** | — |

---

## 5. 验收标准

### 必须通过（阻塞合并）

| # | 验收项 | 验证方法 | 指标 |
|---|---|---|---|
| AC-1 | `ros2 launch launch/full_system.launch.py` 一键启动，5 个节点全部 active | `ros2 node list` | 5/5 节点 |
| AC-2 | 按键 W/S 后，MuJoCo viewer 中 Panda 末端沿 X 轴移动可见 | 视觉确认 | — |
| AC-3 | 按键 G，MuJoCo 夹爪开合，`/gripper_state` 值变化 | `ros2 topic echo` | 0 ↔ 1 |
| AC-4 | 端到端延迟（从 `/master_pose` 发布到 MuJoCo `/joint_states` 更新）< 50ms | Header timestamp 差分 | < 50ms |
| AC-5 | 闭环运行 60 秒，无力矩发散（`/joint_torque_cmd` effort 始终 < 100 N·m） | `ros2 topic echo` 监控 | 稳定 |
| AC-6 | 按 Space 急停后，末端停止移动，不继续漂移 | 视觉确认 | — |

### 加分项

- [ ] 延迟实测 < 20ms（达到预算目标）
- [ ] `rqt_graph` 可正确显示完整节点图
- [ ] 支持 `--no-render` 无头模式启动（CI 友好）

---

## 6. 面试话术关键点

> "全链路集成的最大挑战是**启动时序依赖**：MuJoCo 节点需要 ~1 秒初始化模型，阻抗控制器需要等到 `/joint_states` 开始发布才能做正运动学。我用 ROS2 launch 的 TimerAction 做了分级延迟启动，而不是在代码里写 sleep，这样启动逻辑是声明式的，便于调整。"

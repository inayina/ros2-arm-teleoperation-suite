# SPEC V2-M4: MoveIt Servo 运动层

**分支**：`feat/v2-motion-layer`  
**依赖**：M3（`feat/v2-impedance-controller` 已合入 main）  
**核心目标**：引入 MoveIt 2 Servo 实现笛卡尔→关节实时伺服，完成从键盘输入到 MuJoCo 物理仿真的端到端链路，验证 < 50ms 全链路延迟，并启用奇异点/关节限位自动减速。  
**预计工作量**：5~7 天

> V1 对照：V1 中阻抗控制器内部硬编码 IK，运动与控制耦合。V2 拆出独立的 Motion Layer（MoveIt Servo），只负责「去哪」（参考轨迹），控制层（阻抗控制器）只负责「怎么伺服」（力矩/阻抗），两层解耦。

---

## 1. 目标

1. 配置 `teleop_moveit_config`（`servo.yaml`、`kinematics.yaml`、SRDF）
2. 启动 `servo_node`：订阅 `/safe_master_pose`（pose tracking 模式），输出 `/joint_target`
3. 实现 `teleop_input_node`：键盘/手柄 → `/teleop/cmd_pose` + `/teleop/heartbeat`
4. M5 前临时：`teleop_input` 输出直通 `/safe_master_pose`（安全层占位）
5. 验证端到端链路：键盘 → servo → 阻抗控制器 → CAN → MuJoCo，延迟 < 50ms
6. 奇异点/关节限位附近自动减速验证

---

## 2. 包清单（M4 引入/修改）

```
src/
├── teleop_moveit_config/           ← [NEW] L2 MoveIt Servo 运动层配置包
│   ├── config/
│   │   ├── servo.yaml              # MoveIt Servo 核心参数（发布速率、限幅等）
│   │   ├── kinematics.yaml         # KDL/TRAC-IK 求解器配置
│   │   ├── joint_limits.yaml       # 关节速度/加速度限制（servo 用）
│   │   ├── panda.srdf              # MoveIt SRDF（规划组定义）
│   │   └── panda_simple_controllers.yaml  # servo 输出控制器适配
│   ├── launch/
│   │   ├── servo.launch.py         # servo_node 独立启动
│   │   └── moveit.launch.py        # MoveIt 完整栈（可选，调试用）
│   ├── CMakeLists.txt
│   └── package.xml
│
├── teleop_input/                   ← [NEW] L0 遥操作输入节点（Python）
│   ├── teleop_input/
│   │   ├── __init__.py
│   │   ├── teleop_input_node.py    # 键盘/手柄/Quest3 → /teleop/cmd_pose + heartbeat
│   │   └── keyboard_reader.py      # curses 非阻塞键盘读取
│   ├── config/
│   │   └── teleop_config.yaml      # 步长、速度限制、映射
│   ├── package.xml
│   └── setup.py
│
└── teleop_bringup/
    └── launch/
        ├── motion.launch.py        # [NEW] servo_node 启动
        ├── full_system.launch.py   # [NEW] 顶层一键启动（include 所有子 launch）
        └── m4_pipeline_test.launch.py  # [NEW] M4 端到端验证 launch
```

---

## 3. 接口定义

### 3.1 `teleop_input_node` 发布话题

| Topic | 类型 | 频率 | QoS | 说明 |
|---|---|---|---|---|
| `/teleop/cmd_pose` | `geometry_msgs/PoseStamped` | 100 Hz | Best Effort | 主端期望末端位姿（世界坐标系） |
| `/teleop/heartbeat` | `std_msgs/Header` | 50 Hz | Reliable | 通信活跃性（M5 看门狗用） |
| `/teleop/gripper_cmd` | `std_msgs/Float64` | event | Reliable | 夹爪开合（0~1） |
| `/teleop/record_trigger` | `std_msgs/String` | event | Reliable | `"start"`/`"stop"` 录制 |

### 3.2 键盘映射

| 键 | 动作 | 位移量 |
|---|---|---|
| `W/S` | X 轴前进/后退 | ±5 mm/按键 |
| `A/D` | Y 轴左移/右移 | ±5 mm/按键 |
| `Q/E` | Z 轴上升/下降 | ±5 mm/按键 |
| `I/K` | 绕 X 轴旋转 | ±3°/按键 |
| `J/L` | 绕 Y 轴旋转 | ±3°/按键 |
| `U/O` | 绕 Z 轴旋转 | ±3°/按键 |
| `G` | 夹爪开/关（切换） | — |
| `R` | 开始录制 Episode | — |
| `T` | 停止录制 Episode | — |
| `Space` | 回到 Home 位姿 | — |
| `Esc` | 急停（手动） | — |

### 3.3 `servo_node` 话题（MoveIt Servo 标准接口）

| Topic | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/safe_master_pose` | `geometry_msgs/PoseStamped` | 订阅 | 安全层输出（M5 前直通） |
| `/joint_target` | `trajectory_msgs/JointTrajectory` | 发布 | 输出给阻抗控制器，125 Hz |

### 3.4 M5 前临时直通（`teleop_input_node` 内部）

```python
# M5 安全层就位前，直接将 /teleop/cmd_pose 重发为 /safe_master_pose
# （生产环境 M5 安全层会替换这条路径）
self._safe_pub = self.create_publisher(
    PoseStamped, "/safe_master_pose", 10)

def _on_cmd_pose(self, msg: PoseStamped):
    # TODO M5: 此处将改为由 safety_monitor 过滤
    self._safe_pub.publish(msg)   # 临时直通
```

---

## 4. 关键配置文件

### 4.1 `servo.yaml`（MoveIt Servo 核心参数）

```yaml
moveit_servo:
  # 发布输出类型：joint_trajectory（给阻抗控制器）
  command_out_type: trajectory_msgs/JointTrajectory
  publish_period: 0.008   # 125 Hz

  # 输入类型：PoseStamped（pose tracking 模式）
  command_in_type: geometry_msgs/PoseStamped
  cartesian_command_in_topic: /safe_master_pose

  # 输出话题
  command_out_topic: /joint_target

  # 关节状态
  joint_topic: /joint_states

  # 规划组（SRDF 中定义）
  move_group_name: panda_arm

  # 安全参数
  scale:
    linear:  0.4    # 线速度缩放（m/s）
    rotational: 0.8  # 角速度缩放（rad/s）

  # 奇异点处理
  lower_singularity_threshold: 17.0  # 开始减速
  hard_stop_singularity_threshold: 5.0  # 停止运动

  # 关节限位
  joint_limit_margins: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]  # rad

  # 碰撞检查（可选，需 PlanningScene）
  check_collisions: false   # M4 先禁用，M5 后按需开启

  # 线程
  num_outgoing_solver_threads: 2
```

### 4.2 `kinematics.yaml`（KDL 求解器）

```yaml
panda_arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005   # 5ms 超时（保证 < 10ms IK 延迟）
  kinematics_solver_attempts: 3
```

### 4.3 `panda.srdf`（简化版，仅规划组定义）

```xml
<?xml version="1.0"?>
<robot name="panda">
  <group name="panda_arm">
    <chain base_link="panda_link0" tip_link="panda_hand"/>
  </group>
  <group_state name="home" group="panda_arm">
    <joint name="panda_joint1" value="0"/>
    <joint name="panda_joint2" value="-0.785"/>
    <joint name="panda_joint3" value="0"/>
    <joint name="panda_joint4" value="-2.356"/>
    <joint name="panda_joint5" value="0"/>
    <joint name="panda_joint6" value="1.571"/>
    <joint name="panda_joint7" value="0.785"/>
  </group_state>
  <disable_collisions link1="panda_link0" link2="panda_link1" reason="Adjacent"/>
  <!-- ... 其余自碰撞禁用对 ... -->
</robot>
```

---

## 5. 关键实现细节

### 5.1 `teleop_input_node.py` 骨架

```python
class TeleopInputNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("teleop_input")

        # 发布者
        self._cmd_pub   = self.create_publisher(PoseStamped, "/teleop/cmd_pose", 10)
        self._hb_pub    = self.create_publisher(Header, "/teleop/heartbeat", 10)
        self._safe_pub  = self.create_publisher(PoseStamped, "/safe_master_pose", 10)  # M5 前临时
        self._grip_pub  = self.create_publisher(Float64, "/teleop/gripper_cmd", 10)
        self._rec_pub   = self.create_publisher(String, "/teleop/record_trigger", 10)

        # 当前期望位姿（从 home 出发）
        self._current_pose = self._home_pose()

        # 心跳定时器（50 Hz）
        self.create_timer(0.02, self._publish_heartbeat)

        # 键盘读取定时器（100 Hz）
        self.create_timer(0.01, self._read_keyboard_and_publish)

        self._keyboard = KeyboardReader()

    def _read_keyboard_and_publish(self):
        key = self._keyboard.get_key()
        if key is None:
            return

        delta = self._key_to_delta(key)   # 返回 (dx, dy, dz, droll, dpitch, dyaw)
        self._apply_delta(delta)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "panda_link0"
        msg.pose = self._current_pose
        self._cmd_pub.publish(msg)
        self._safe_pub.publish(msg)        # M5 前临时直通
```

### 5.2 延迟测量方案

```bash
# 在两端打时间戳，用 ros2_tracing 或手动测量
# 方法 1：ros2 topic delay（测 /joint_states 相对于 /safe_master_pose 的延迟）
ros2 topic delay /joint_states

# 方法 2：自定义时间戳测量节点
# 在 /teleop/cmd_pose 和 /joint_states 上加 header.stamp，计算差值

# 方法 3：candump 时间戳（CAN 层延迟）
candump -t a vcan0 | head -20
```

### 5.3 全链路 Launch 组合（`full_system.launch.py`）

```python
# 启动顺序（RegisterEventHandler 串接）：
# 1. description.launch.py   → robot_description + TF
# 2. simulation.launch.py    → MuJoCo 就绪
# 3. fieldbus.launch.py      → vcan0 + virtual_servo_driver ×7
# 4. ros2_control.launch.py  → controller_manager + spawners
# 5. safety.launch.py        → safety_monitor（M5）；M4 阶段跳过
# 6. motion.launch.py        → servo_node
# 7. recording.launch.py     → lerobot_recorder（record:=true 时）

launch_arguments = [
    DeclareLaunchArgument("use_sim",       default_value="true"),
    DeclareLaunchArgument("can_interface", default_value="vcan0"),
    DeclareLaunchArgument("controller",    default_value="impedance"),
    DeclareLaunchArgument("record",        default_value="false"),
    DeclareLaunchArgument("headless",      default_value="false"),
    DeclareLaunchArgument("servo_mode",    default_value="pose"),
]
```

---

## 6. 验收标准

### 必须通过（阻塞合并至 main）

| # | 验收项 | 验证方法 |
|---|---|---|
| AC-1 | `servo_node` 启动，订阅 `/safe_master_pose`，输出 `/joint_target` @125Hz | `ros2 topic hz /joint_target` |
| AC-2 | 键盘 W/S/A/D/Q/E 控制 Panda 末端在 MuJoCo viewer 中平滑移动 | 视觉确认 |
| AC-3 | 端到端延迟（键盘按键 → MuJoCo 关节实际运动）< 50ms | `ros2 topic delay /joint_states` 或自定义测量 |
| AC-4 | 接近关节限位时 servo 自动减速（`/joint_target` 幅值减小） | `ros2 topic echo /joint_target` 观察 |
| AC-5 | 接近奇异点时 servo 自动减速（不发散、不锁死） | 手动将末端移到奇异点附近观察 |
| AC-6 | `ros2 launch teleop_bringup full_system.launch.py` 一键起全栈 | 命令行执行，无报错 |
| AC-7 | 心跳 `/teleop/heartbeat` @50Hz 稳定 | `ros2 topic hz /teleop/heartbeat` |

### 加分项

- [ ] `servo_mode:=twist` 模式（TwistStamped 输入）也可工作
- [ ] Quest3 手柄输入（通过 `/dev/input` 读取）
- [ ] 多段键盘连续输入轨迹平滑（速度滤波，避免跳变）

---

## 7. 常用调试命令

```bash
# 构建 M4 相关包
colcon build --packages-select teleop_moveit_config teleop_input teleop_bringup
source install/setup.bash

# 单独启动 servo_node（调试用）
ros2 launch teleop_moveit_config servo.launch.py

# 全链路启动（M4 完整栈）
ros2 launch teleop_bringup full_system.launch.py

# 手动发 /safe_master_pose 测试 servo 响应
ros2 topic pub /safe_master_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: panda_link0}, pose: {position: {x: 0.4, y: 0.0, z: 0.4}, \
  orientation: {w: 1.0}}}" --once

# 观察 servo 输出
ros2 topic hz /joint_target
ros2 topic echo /joint_target --once

# 测量端到端延迟
ros2 topic delay /joint_states

# 检查 servo 状态（MoveIt Servo 状态话题）
ros2 topic echo /servo_server/status --once

# 启动键盘遥操作
ros2 run teleop_input teleop_input_node
```

---

## 8. 关键风险与应对

| 风险 | 应对 |
|---|---|
| MoveIt Servo 配置门槛高（SRDF/yaml 字段繁多） | 以 `panda_moveit_config` 官方模板为基础裁剪，只保留 servo 必需字段 |
| KDL IK 在某些位姿下失败（无解） | 设置 `kinematics_solver_attempts: 3`；超时返回上一有效解 |
| `/safe_master_pose` 与 servo 输入类型不匹配 | 确认 servo `command_in_type: geometry_msgs/PoseStamped`，frame_id 一致 |
| 全链路延迟超 50ms | 用 `ros2 topic delay` 逐层定位瓶颈；DDS QoS 改为 Best Effort；减少中间节点缓冲 |

---

*本文件为 V2-M4 细化 SPEC；架构基线见 [`ARCHITECTURE_V2.md`](./ARCHITECTURE_V2.md) §6.2，里程碑总览见 [`ROADMAP.md`](./ROADMAP.md)。*

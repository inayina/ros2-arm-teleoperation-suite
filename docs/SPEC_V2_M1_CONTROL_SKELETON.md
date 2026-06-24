# SPEC V2-M1: ros2_control 骨架 + MuJoCo 物理服务器

**分支**：`feat/v2-control-skeleton`  
**依赖**：无（M1 是最底层）  
**核心目标**：建立 ros2_control 实时框架 + MuJoCo 物理引擎，打通 `/sim/*` 内部背板，使 Panda 能在 1kHz 控制循环下靠重力补偿站立。  
**预计工作量**：5~7 天

> V1 对照：V1 中 MuJoCo 直接扮演「驱动器」角色，控制器作为独立节点订阅/发布 Topic。V2 将 ros2_control 作为实时主循环，MuJoCo 退化为纯物理服务器，两者通过 `/sim/*` 背板解耦。

---

## 1. 目标

1. 创建 `teleop_interfaces`（自定义 msg/srv）、`teleop_description`（URDF + ros2_control 标签）
2. 实现 `canopen_hw_interface`（`SystemInterface` 插件，M1 阶段「直连 sim」模式，不走真实 CAN）
3. 实现 `mujoco_sim_node`：1kHz 物理步进 + `/sim/joint_effort_cmd` ↔ `/sim/encoder_state` 背板
4. 使用 `joint_state_broadcaster` + `forward_command_controller` 验证控制链路
5. 一键 `ros2 launch teleop_bringup m1_control_sim.launch.py` 起全栈

---

## 2. 包清单（M1 引入）

```
src/
├── teleop_interfaces/              ← [NEW] 自定义消息/服务包（纯接口，最先构建）
│   ├── msg/
│   │   ├── SafetyStatus.msg        # 安全层状态（M5 使用，M1 阶段先定义）
│   │   └── DriveStatus.msg         # DS402 驱动器状态（M2 使用，M1 阶段先定义）
│   ├── srv/
│   │   └── TriggerEstop.srv        # 急停触发服务
│   ├── CMakeLists.txt
│   └── package.xml
│
├── teleop_description/             ← [NEW] 机器人描述包
│   ├── urdf/
│   │   ├── panda.urdf.xacro        # Franka Panda URDF（含 ros2_control 标签）
│   │   └── panda.ros2_control.xacro  # canopen_system SystemInterface 挂载
│   ├── config/
│   │   └── joint_limits.yaml       # Panda 软限位（soft limit + margin）
│   ├── launch/
│   │   └── description.launch.py   # robot_state_publisher 启动
│   └── meshes/                     # Franka Panda STL/DAE 网格
│
├── canopen_hw_interface/           ← [NEW] ros2_control SystemInterface 插件
│   ├── include/canopen_hw_interface/
│   │   └── canopen_system.hpp
│   ├── src/
│   │   └── canopen_system.cpp
│   ├── hardware_plugin.xml         # pluginlib 导出
│   ├── config/
│   │   └── can_config.yaml         # M1: sim_direct=true; M2: 切 vcan0
│   ├── CMakeLists.txt
│   └── package.xml
│
├── mujoco_sim/                     ← [MODIFY] 物理引擎节点（V1 基础升级）
│   └── mujoco_sim/
│       ├── mujoco_sim_node.py      # 1kHz 步进 + /sim/* 背板
│       └── virtual_camera.py       # 虚拟相机（M6 用，M1 先放桩）
│
└── teleop_bringup/                 ← [NEW] Launch 编排包
    ├── launch/
    │   ├── m1_control_sim.launch.py          # M1 最小验证 launch
    │   ├── description.launch.py
    │   ├── ros2_control.launch.py
    │   └── simulation.launch.py
    ├── config/
    │   └── ros2_controllers.yaml    # controller_manager 控制器配置
    ├── CMakeLists.txt
    └── package.xml

config/
└── models/
    ├── franka_panda.xml            # mujoco_menagerie 官方模型（含 FT sensor 扩展）
    └── franka_panda_scene.xml      # 桌面场景（M6 录制用）
```

---

## 3. 接口定义

### 3.1 `/sim/*` 内部背板（M1 核心接口）

| Topic | 类型 | 方向 | 频率 | QoS |
|---|---|---|---|---|
| `/sim/joint_effort_cmd` | `std_msgs/Float64MultiArray` | `canopen_hw_interface` → `mujoco_sim` | 1000 Hz | Best Effort |
| `/sim/encoder_state` | `sensor_msgs/JointState` | `mujoco_sim` → `canopen_hw_interface` | 1000 Hz | Best Effort |

> M1「直连 sim」模式：`canopen_hw_interface` 不走 CAN，直接订阅/发布 `/sim/*`。M2 切入真实 CAN 时，这两个话题改为由 `virtual_servo_driver` 桥接。

### 3.2 ros2_control 对外话题

| Topic | 类型 | 发布者 | 频率 | QoS |
|---|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_broadcaster` | 1000 Hz | Best Effort |
| `/dynamic_joint_states` | `control_msgs/DynamicJointState` | `joint_state_broadcaster` | 1000 Hz | Best Effort |

### 3.3 teleop_interfaces 消息定义

**`SafetyStatus.msg`**：
```
# Header
std_msgs/Header header
# 安全状态标志
bool estop_active
bool joint_limit_violated
bool workspace_limit_violated
bool velocity_limit_violated
bool comm_watchdog_timeout
# 详情
string message
```

**`DriveStatus.msg`**：
```
# Header
std_msgs/Header header
# 驱动器状态（每关节）
uint8[] node_ids
string[] ds402_state         # "OperationEnabled" / "Fault" 等
float64[] actual_torque_nm
float64[] actual_position_rad
uint16[] fault_code          # DS402 EMCY fault code
```

**`TriggerEstop.srv`**：
```
string reason    # 急停原因（记录日志用）
---
bool success
string message
```

### 3.4 硬件接口（ros2_control command/state interfaces）

```
command_interfaces:
  - <joint_name>/effort        # 力矩命令（7 关节 × 1）

state_interfaces:
  - <joint_name>/position      # 关节角（来自编码器/MuJoCo）
  - <joint_name>/velocity      # 关节速度
  - <joint_name>/effort        # 实际力矩反馈
```

---

## 4. 关键实现细节

### 4.1 URDF `ros2_control` 标签（`panda.ros2_control.xacro`）

```xml
<ros2_control name="panda_canopen_system" type="system">
  <hardware>
    <plugin>canopen_hw_interface/CanopenSystem</plugin>
    <param name="can_interface">vcan0</param>
    <param name="sim_direct">true</param>   <!-- M1: 直连 sim，不走 CAN -->
    <param name="physics_freq">1000</param>
  </hardware>

  <!-- 7 个关节，依次 node_id = 1~7 -->
  <joint name="panda_joint1">
    <param name="node_id">1</param>
    <command_interface name="effort">
      <param name="min">-87.0</param>
      <param name="max"> 87.0</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  <!-- ... panda_joint2 ~ panda_joint7 同上 ... -->
</ros2_control>
```

### 4.2 CanopenSystem M1「直连 sim」实现骨架（C++）

```cpp
// canopen_system.hpp
class CanopenSystem : public hardware_interface::SystemInterface {
public:
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // M1: 直连模式，直接走 /sim/* 话题
  bool sim_direct_{true};
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr encoder_sub_;

  std::array<double, 7> hw_commands_effort_{};
  std::array<double, 7> hw_states_position_{};
  std::array<double, 7> hw_states_velocity_{};
  std::array<double, 7> hw_states_effort_{};
  std::mutex state_mutex_;
};
```

```cpp
// canopen_system.cpp — write() 核心逻辑（M1 直连）
hardware_interface::return_type CanopenSystem::write(
    const rclcpp::Time &, const rclcpp::Duration &)
{
  if (sim_direct_) {
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data.assign(hw_commands_effort_.begin(), hw_commands_effort_.end());
    effort_pub_->publish(msg);
  }
  // M2: else { /* encode RPDO, send via SocketCAN */ }
  return hardware_interface::return_type::OK;
}
```

### 4.3 MuJoCo 物理节点（`mujoco_sim_node.py`）

```python
PHYSICS_FREQ  = 1000  # Hz
PUBLISH_FREQ  = 100   # Hz（/sim/encoder_state 发布频率）
PUBLISH_EVERY = PHYSICS_FREQ // PUBLISH_FREQ

class MujocoSimNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("mujoco_sim")
        self.model = mujoco.MjModel.from_xml_path("config/models/franka_panda.xml")
        self.data  = mujoco.MjData(self.model)

        # /sim/joint_effort_cmd 订阅（来自 canopen_hw_interface）
        self.create_subscription(
            Float64MultiArray, "/sim/joint_effort_cmd",
            self._on_effort_cmd, qos_profile_sensor_data)

        # /sim/encoder_state 发布（回传给 canopen_hw_interface）
        self._enc_pub = self.create_publisher(
            JointState, "/sim/encoder_state", qos_profile_sensor_data)

        # /ft_sensor 发布（给阻抗控制器 M3 用）
        self._ft_pub  = self.create_publisher(
            WrenchStamped, "/ft_sensor", qos_profile_sensor_data)

        self._cmd_lock = threading.Lock()
        self._latest_effort = [0.0] * 7

    def run_physics_loop(self):
        """在独立线程中以 1kHz 步进物理引擎"""
        step = 0
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while rclpy.ok() and viewer.is_running():
                with self._cmd_lock:
                    for i, tau in enumerate(self._latest_effort):
                        self.data.actuator(f"actuator{i+1}").ctrl[0] = tau

                mujoco.mj_step(self.model, self.data)
                step += 1

                if step % PUBLISH_EVERY == 0:
                    self._publish_encoder_state()
                    self._publish_ft_sensor()

                viewer.sync()
```

### 4.4 重力补偿（使 Panda 静止站立）

```python
def _gravity_compensation(self) -> list[float]:
    """计算重力补偿力矩（MuJoCo 提供 qfrc_bias）"""
    return list(self.data.qfrc_bias[:7])  # N·m，抵消重力
```

在控制器未下指令时，`mujoco_sim_node` 自动施加重力补偿力矩，使 Panda 保持初始姿态——这是 M1 的核心验收指标之一。

### 4.5 `ros2_controllers.yaml`（controller_manager 配置）

```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    forward_command_controller:
      type: forward_command_controller/ForwardCommandController
    # M3 引入：
    # cartesian_impedance_controller:
    #   type: teleop_controllers/CartesianImpedanceController

forward_command_controller:
  ros__parameters:
    joints:
      - panda_joint1
      - panda_joint2
      - panda_joint3
      - panda_joint4
      - panda_joint5
      - panda_joint6
      - panda_joint7
    interface_name: effort
```

### 4.6 M1 最小 Launch（`m1_control_sim.launch.py`）

启动顺序（`RegisterEventHandler` 串接）：
1. `robot_state_publisher`（发布 `robot_description` + TF）
2. `mujoco_sim_node`（物理引擎就绪，`/sim/*` 背板上线）
3. `controller_manager`（加载 `canopen_system`，`sim_direct:=true`）
4. Spawner：`joint_state_broadcaster` → `forward_command_controller`

---

## 5. 验收标准

### 必须通过（阻塞合并至 main）

| # | 验收项 | 验证命令 |
|---|---|---|
| AC-1 | `colcon build` 无 error（Warning 允许） | `colcon build --packages-select teleop_interfaces teleop_description canopen_hw_interface mujoco_sim teleop_bringup` |
| AC-2 | `joint_state_broadcaster` 处于 `active` 状态 | `ros2 control list_controllers` |
| AC-3 | `forward_command_controller` 处于 `active` 状态 | `ros2 control list_controllers` |
| AC-4 | `/joint_states` 以 ≥ 950 Hz 稳定发布（7 关节，position/velocity/effort 完整） | `ros2 topic hz /joint_states` + `ros2 topic echo /joint_states --once` |
| AC-5 | Panda 在 MuJoCo viewer 中靠重力补偿站立（不坍塌，关节角稳定） | 视觉确认 |
| AC-6 | `/sim/joint_effort_cmd` 与 `/sim/encoder_state` 均有数据 | `ros2 topic hz /sim/encoder_state` |
| AC-7 | `m1_control_sim.launch.py` 一键启动，60 秒无崩溃 | `ros2 launch teleop_bringup m1_control_sim.launch.py` |
| AC-8 | `canopen_hw_interface` 与 `mujoco_sim` 硬件接口均被识别 | `ros2 control list_hardware_components` |

### 加分项（不阻塞合并）

- [ ] `headless:=true` 参数下 MuJoCo 无 GUI 运行（CI 用）
- [ ] `forward_command_controller` 发布一个非零力矩，Panda 实际运动
- [ ] `robot_description` 在 rviz2 正确可视化，TF 树完整

---

## 6. 常用调试命令

```bash
# 构建 M1 相关包
colcon build --packages-select teleop_interfaces teleop_description \
    canopen_hw_interface mujoco_sim teleop_bringup
source install/setup.bash

# 启动 M1 最小栈
ros2 launch teleop_bringup m1_control_sim.launch.py

# 验证控制器状态
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 control list_hardware_components

# 验证话题
ros2 topic hz /joint_states
ros2 topic hz /sim/encoder_state
ros2 topic echo /joint_states --once

# 向 forward_command_controller 发布力矩测试（joint1 = 5 N·m）
ros2 topic pub /forward_command_controller/commands \
    std_msgs/msg/Float64MultiArray \
    "{data: [5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# 检查 TF 树
ros2 run tf2_tools view_frames
```

---

## 7. 关键风险与应对

| 风险 | 应对 |
|---|---|
| `ros2_control` SystemInterface 实时性不足（update() 抖动） | 先用 `mock_components` 直连验证逻辑，确认 1kHz 后再切 canopen_hw_interface |
| MuJoCo `qfrc_bias` 重力补偿与 URDF 不一致导致 Panda 漂移 | 确认 MuJoCo XML 与 URDF 质量参数一致；必要时用 `mujoco.mj_inverse()` 校验 |
| `canopen_system` 插件找不到（pluginlib 路径问题） | 检查 `hardware_plugin.xml` 与 `package.xml` 中的 `<export>` 标签 |

---

*本文件为 V2-M1 细化 SPEC；架构基线见 [`ARCHITECTURE_V2.md`](./ARCHITECTURE_V2.md) §6 和 §7，里程碑总览见 [`ROADMAP.md`](./ROADMAP.md)。*

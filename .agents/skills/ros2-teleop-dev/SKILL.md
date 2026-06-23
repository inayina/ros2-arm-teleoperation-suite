---
name: ros2-teleop-dev
description: Development assistant for ros2-arm-teleoperation-suite. Provides project architecture, ROS2 topic map, milestone acceptance criteria, debug commands, and key implementation patterns for the 5-layer robotic teleoperation system (teleop input → Cartesian impedance control → CAN bus → MuJoCo sim → LeRobot recorder).
---

# ros2-arm-teleoperation-suite 开发助手

## 项目概况

- **目标**：机械臂遥操作全链路系统（无实体硬件，纯软件仿真）
- **机器人**：Franka Panda（MuJoCo v3 + mujoco_menagerie XML）
- **ROS2 发行版**：Jazzy
- **Python 环境**：conda `ros2-teleop`（`/home/ina/miniforge3/envs/ros2-teleop`）
- **项目路径**：`/home/ina/dev/ros2-arm-teleoperation-suite`
- **兄弟项目**：`/home/ina/robot-sim-lab/robot-arm-episode-data-lab`（LeRobot 数据预处理）
- **主分支**：`main`

---

## 五层架构速查

```
Layer 1  teleop_input         Python   /master_pose (PoseStamped @ 50Hz)
   ↓
Layer 2  impedance_controller  C++17   /joint_torque_cmd (JointState @ 500Hz)
   ↓
Layer 3a can_bridge            Python  vcan0 ↔ CANopen DS402 PDO
Layer 3b rs485_modbus          Python  Modbus TCP → /gripper_state
   ↓
Layer 4  mujoco_sim            Python  /joint_states + /ft_sensor @ 100Hz
   ↓
Layer 5  lerobot_recorder      Python  data/episodes/ (HF datasets Arrow)
```

---

## ROS2 话题全表

| Topic | 类型 | 发布者 | 订阅者 | QoS |
|---|---|---|---|---|
| `/master_pose` | `geometry_msgs/PoseStamped` | teleop_input | impedance_controller | Best Effort |
| `/joint_torque_cmd` | `sensor_msgs/JointState` | impedance_controller | can_bridge | Best Effort |
| `/joint_states` | `sensor_msgs/JointState` | mujoco_sim | impedance_controller, lerobot_recorder | Best Effort |
| `/ft_sensor` | `geometry_msgs/WrenchStamped` | mujoco_sim | impedance_controller | Best Effort |
| `/gripper_cmd` | `std_msgs/Float32` | teleop_input | rs485_bridge | Reliable |
| `/gripper_state` | `std_msgs/Float32` | rs485_bridge | lerobot_recorder | Best Effort |
| `/episode/status` | `std_msgs/String` | teleop_input | lerobot_recorder | Reliable |

---

## 里程碑与分支

| 里程碑 | 分支 | 状态 | 关键验收 |
|---|---|---|---|
| M0 | episode-data-lab 项目 | ✅ Done | `export_to_lerobot.py` dry-run 通过 |
| M1 | `feat/can-rs485-layer` | 🔲 | `candump vcan0` 抓到 PDO 帧；pytest 通过 |
| M2 | `feat/mujoco-ros2-bridge` | 🔲 | Panda viewer 响应 `/joint_torque_cmd` |
| M3 | `feat/impedance-controller` | 🔲 | 末端误差 < 2mm；500Hz 控制频率 |
| M4 | `feat/full-pipeline` | 🔲 | 一键 launch，端到端延迟 < 50ms |
| M5 | `feat/lerobot-recorder` | 🔲 | 50 帧 Episode，load_from_disk 可读 |
| M6 | `feat/polish` | 🔲 | README + 演示视频 |

---

## 常用调试命令速查

### ROS2 话题

```bash
ros2 topic list                          # 列出所有话题
ros2 topic hz /joint_states              # 测量发布频率
ros2 topic delay /joint_states           # 测量消息延迟
ros2 topic echo /ft_sensor --once        # 单次查看消息内容
ros2 topic info --verbose /master_pose   # 查看 QoS 设置
ros2 node list                           # 查看所有节点
ros2 node info /impedance_controller     # 查看节点订阅/发布
```

### CAN 总线

```bash
# 初始化 vcan0
bash scripts/setup_vcan.sh

# 监听 vcan0 所有帧
candump vcan0

# 发送测试帧（模拟驱动器编码器反馈，关节 1）
cansend vcan0 181#0010000000000000

# 发送关节 0 力矩指令帧（1.0 N·m = 1000 counts）
cansend vcan0 200#E8030000000000000

# 检查 vcan0 状态
ip link show vcan0
```

### colcon 构建

```bash
# 只构建阻抗控制器包
colcon build --packages-select impedance_controller

# 构建并跑测试
colcon build && colcon test --packages-select impedance_controller
colcon test-result --verbose

# Source 环境
source install/setup.bash
```

### conda 环境

```bash
conda activate ros2-teleop
# Python 依赖路径：/home/ina/miniforge3/envs/ros2-teleop/bin/python
```

### MuJoCo

```bash
# 无 GPU 时 headless 运行（跳过 viewer）
python src/mujoco_sim/mujoco_sim/mujoco_sim_node.py --no-render

# 下载 Franka Panda 模型
wget -q https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/main/franka_emika_panda/panda.xml \
     -O config/models/franka_panda.xml
```

---

## 关键实现模式

### C++ 阻抗控制器 CallbackGroup 模板

```cpp
// 构造函数中
cb_group_control_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
cb_group_sensor_ = create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);

auto opt_control = rclcpp::SubscriptionOptions();
opt_control.callback_group = cb_group_control_;
auto opt_sensor = rclcpp::SubscriptionOptions();
opt_sensor.callback_group = cb_group_sensor_;

// 控制指令订阅（串行）
sub_master_pose_ = create_subscription<PoseStamped>(
    "/master_pose", 10, std::bind(&ImpedanceController::on_master_pose, this, _1),
    opt_control);

// 传感器订阅（并发）
sub_joint_states_ = create_subscription<JointState>(
    "/joint_states", 10, std::bind(&ImpedanceController::on_joint_states, this, _1),
    opt_sensor);
sub_ft_ = create_subscription<WrenchStamped>(
    "/ft_sensor", 10, std::bind(&ImpedanceController::on_ft, this, _1),
    opt_sensor);

// 启动时使用
auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
exec->add_node(node);
exec->spin();
```

### Python MuJoCo 物理循环模板

```python
PHYSICS_FREQ  = 1000  # Hz
PUBLISH_FREQ  = 100   # Hz
PUBLISH_EVERY = PHYSICS_FREQ // PUBLISH_FREQ

step = 0
with mujoco.viewer.launch_passive(model, data) as viewer:
    while rclpy.ok() and viewer.is_running():
        with cmd_lock:
            if latest_cmd:
                apply_torque(data, latest_cmd)
        mujoco.mj_step(model, data)
        step += 1
        if step % PUBLISH_EVERY == 0:
            publish_joint_states(data)
            publish_ft_sensor(data)
        viewer.sync()
```

### CANopen PDO 编解码

```python
import struct
TORQUE_SCALE = 0.001  # N·m/bit
COUNTS_PER_REV = 4096

def pack_torque_pdo(torque_nm: float) -> bytes:
    raw = int(torque_nm / TORQUE_SCALE)
    raw = max(-32768, min(32767, raw))
    return struct.pack("<h6x", raw)

def unpack_encoder_pdo(data: bytes) -> float:
    counts = struct.unpack("<i4x", data)[0]
    return counts / COUNTS_PER_REV * 2 * 3.14159265
```

### LeRobot Episode 写入模板

```python
from datasets import Dataset, Features, Sequence, Value

EPISODE_FEATURES = Features({
    "observation.state":       Sequence(Value("float32"), length=7),
    "action":                  Sequence(Value("float32"), length=7),
    "observation.ee_pose":     Sequence(Value("float32"), length=7),
    "observation.object_pose": Sequence(Value("float32"), length=7),
    "timestamp":               Value("float64"),
    "episode_index":           Value("int64"),
    "frame_index":             Value("int64"),
    "done":                    Value("bool"),
    "language_instruction":    Value("string"),
    "success":                 Value("bool"),
})

ds = Dataset.from_list(buffer, features=EPISODE_FEATURES)
ds.save_to_disk(f"data/episodes/episode_{idx:06d}/train")
```

---

## 参见

- 详细架构设计：[docs/DESIGN_SPEC.md](../docs/DESIGN_SPEC.md)
- 开发路线图与周检查清单：[docs/ROADMAP.md](../docs/ROADMAP.md)
- M1 CAN/RS485 详细 SPEC：[docs/SPEC_M1_CAN_RS485.md](../docs/SPEC_M1_CAN_RS485.md)
- M2 MuJoCo 桥接 SPEC：[docs/SPEC_M2_MUJOCO_BRIDGE.md](../docs/SPEC_M2_MUJOCO_BRIDGE.md)
- M3 阻抗控制器 SPEC：[docs/SPEC_M3_IMPEDANCE_CTRL.md](../docs/SPEC_M3_IMPEDANCE_CTRL.md)
- M4 全链路集成 SPEC：[docs/SPEC_M4_FULL_PIPELINE.md](../docs/SPEC_M4_FULL_PIPELINE.md)
- M5 LeRobot 录制 SPEC：[docs/SPEC_M5_LEROBOT_RECORDER.md](../docs/SPEC_M5_LEROBOT_RECORDER.md)

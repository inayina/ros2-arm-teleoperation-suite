---
name: ros2-teleop-dev
description: Development assistant for ros2-arm-teleoperation-suite. Provides project architecture, ROS2 topic map, milestone acceptance criteria, debug commands, and key implementation patterns for the V2 industrial-grade teleoperation stack (teleop → safety layer → MoveIt Servo motion layer → ros2_control impedance controller → CANopen DS402 fieldbus → virtual servo drive → MuJoCo sim → camera bridge → LeRobot recorder).
---

# ros2-arm-teleoperation-suite 开发助手

## 项目概况

- **目标**：工业级机械臂遥操作全链路平台（无实体硬件，纯软件仿真）
- **架构**：**V2**（见 `docs/ARCHITECTURE_V2.md`）；V1 五层设计存档于 `docs/DESIGN_SPEC.md`
- **机器人**：Franka Panda（MuJoCo v3 + mujoco_menagerie XML）
- **ROS2 发行版**：Jazzy（含 ros2_control / MoveIt 2 Servo）
- **Python 环境**：ROS2/Jazzy 主运行环境使用系统 Python 3.12（`/usr/bin/python3` + `/opt/ros/jazzy`）；conda 仅用于 LeRobot 数据处理、训练或 notebook，不用于 `ros2 launch`
- **项目路径**：`/home/ina/dev/ros2-arm-teleoperation-suite`
- **兄弟项目**：`/home/ina/robot-sim-lab/robot-arm-episode-data-lab`（LeRobot 数据预处理）
- **主分支**：`main`

---

## V2 七层架构速查

```
L0 teleop_input        Python  /teleop/cmd_pose + /teleop/heartbeat
   ↓
L1 safety_monitor      C++     /safe_master_pose（JointLimit·Workspace·Velocity·Watchdog·E-Stop）
   ↓
L2 moveit_servo        C++     /joint_target（笛卡尔→关节, 奇异/限位规避）
   ↓
L3 ros2_control        C++     controller_manager @1kHz
     · cartesian_impedance_controller（controller_interface 插件, effort 命令）
     · joint_state_broadcaster → /joint_states
     · canopen_system（SystemInterface 硬件接口）
   ↓ vcan0 (CANopen DS402: RPDO/TPDO/SDO/NMT/EMCY)
L4 virtual_servo_driver Python DS402 状态机 + 电流环 + 编码器 + 故障注入 ×7
   ↓ /sim/joint_effort_cmd  ↑ /sim/encoder_state
L5 mujoco_sim          Python  物理引擎 + FT 真值 + 虚拟相机 @1kHz/100Hz
   ↓
L6 camera_bridge       Python  Scene/Wrist RGBD + 左/右 GelSight-like 视触觉 @30Hz
L7 lerobot_recorder    Python  多模态对齐 → LeRobot Dataset
```

> 核心原则：MuJoCo 提供**仿真真值**；机器人栈只通过 CANopen 编码器反馈「测得」状态（measured-state），两者分离。

---

## ROS2 话题全表（V2）

| Topic | 类型 | 发布者 | 订阅者 | QoS |
|---|---|---|---|---|
| `/teleop/cmd_pose` | `geometry_msgs/PoseStamped` | teleop_input | safety_monitor | Best Effort |
| `/teleop/heartbeat` | `std_msgs/Header` | teleop_input | safety_monitor | Reliable |
| `/teleop/gripper_cmd` | `std_msgs/Float64` | teleop_input | gripper_driver | Reliable |
| `/safe_master_pose` | `geometry_msgs/PoseStamped` | safety_monitor | servo_node | Reliable |
| `/safety/estop` | `std_msgs/Bool` | safety_monitor | canopen_system | Reliable + Transient Local |
| `/safety/status` | `teleop_interfaces/SafetyStatus` | safety_monitor | recorder | Reliable |
| `/joint_target` | `trajectory_msgs/JointTrajectory` | servo_node | cartesian_impedance_controller | Reliable |
| `/joint_states` | `sensor_msgs/JointState` | joint_state_broadcaster | servo, impedance, recorder | Best Effort |
| `/sim/joint_effort_cmd` | `std_msgs/Float64MultiArray` | virtual_servo_driver | mujoco_sim | Best Effort |
| `/sim/encoder_state` | `sensor_msgs/JointState` | mujoco_sim | virtual_servo_driver | Best Effort |
| `/servo_drive/status` | `teleop_interfaces/DriveStatus` | virtual_servo_driver | recorder | Reliable |
| `/ft_sensor` | `geometry_msgs/WrenchStamped` | mujoco_sim | impedance ctrl, recorder | Best Effort |
| `/ee_pose` | `geometry_msgs/PoseStamped` | mujoco_sim | recorder | Best Effort |
| `/sim/object_pose` | `geometry_msgs/PoseStamped` | mujoco_sim | camera_bridge, recorder | Best Effort |
| `/camera/color/image_raw` | `sensor_msgs/Image` | camera_bridge | recorder | Best Effort |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | camera_bridge | recorder | Best Effort |
| `/camera/wrist/color/image_raw` | `sensor_msgs/Image` | camera_bridge | recorder | Best Effort |
| `/camera/tactile_left/image_raw` | `sensor_msgs/Image` | camera_bridge | recorder | Best Effort |
| `/camera/tactile_right/image_raw` | `sensor_msgs/Image` | camera_bridge | recorder | Best Effort |
| `/gripper/state` | `std_msgs/Float64` | gripper_driver | recorder | Best Effort |

> CAN 帧（vcan0）不是 ROS Topic：RPDO `0x200+id` / TPDO `0x180+id` / SDO `0x600·0x580+id` / NMT `0x000` / SYNC `0x080` / EMCY `0x080+id`。用 `candump vcan0` 抓帧。

---

## 里程碑与分支（V2）

| 里程碑 | 分支 | 状态 | 关键验收 |
|---|---|---|---|
| M1 | `feat/v2-control-skeleton` | ✅ | `ros2 control list_controllers` 显示 broadcaster active；Panda 重力补偿站立；`/joint_states` @1kHz |
| M2 | `feat/v2-canopen-fieldbus` | ✅ | `candump vcan0` 抓到周期 PDO；DS402 到 Operation Enabled；故障注入→EMCY |
| M3 | `feat/v2-impedance-controller` | ✅ | 阻抗插件被加载 active；末端误差 < 2mm；`update()` 1kHz |
| M4 | `feat/v2-motion-layer` | ✅ | 键盘→servo→阻抗→CAN→MuJoCo 端到端 < 50ms；奇异/限位自动减速 |
| M5 | `feat/v2-safety-layer` | ✅ | 5 监视器单测过；心跳超时 100ms→E-Stop→DS402 Quick Stop；可复位 |
| M6 | `feat/v2-perception-recorder` | ✅ | RGB/Depth @30Hz；多模态 LeRobotDataset 可 load，字段完整 |
| M7 | `feat/v2-teleop-synth-data` | ✅ | TeleopDriverBase 接口可插拔；合成数据批量生成；Domain Randomization 验证 |

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

### ros2_control（V2）

```bash
ros2 control list_controllers                 # 查看控制器状态 (active/inactive)
ros2 control list_hardware_interfaces         # 查看 command/state interfaces
ros2 control list_hardware_components          # 查看 canopen_system 硬件组件
# 热切换：阻抗控制器 ↔ JointTrajectoryController
ros2 control switch_controllers \
  --deactivate cartesian_impedance_controller \
  --activate joint_trajectory_controller
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

### Python / conda 环境

```bash
# ROS2 Jazzy 主运行环境：使用系统 Python 3.12，避免 conda Python 与 rclpy ABI 不匹配
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# conda ros2-teleop 可用于 LeRobot 数据处理、训练或 notebook；
# 不要在该 conda 环境中运行 ros2 launch / rclpy 节点。
```

### MuJoCo

```bash
# 无 GPU 时 headless 运行（跳过 viewer）
python src/mujoco_sim/mujoco_sim/mujoco_sim_node.py --no-render

# 下载 Franka Panda 模型
wget -q https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/main/franka_emika_panda/panda.xml \
     -O config/models/franka_panda.xml

# 运行 M7 全系统抓取演示并录制 GIF（一键启动全链路）
bash scripts/capture_m7_demo.sh
```

### M7 MuJoCo sim-direct grasp 调试

边界：M7 抓取 GIF / 合成数据演示默认只看 `use_sim:=true` 的 MuJoCo sim-direct 路径；不要切到 CANopen / DS402。CAN 证据单独用 M2/M5 验收。

```bash
# 开启 M7 抓取演示，默认打印 MuJoCo contact/object/gripper 调试日志
M7_CONTACT_DEBUG=true bash scripts/capture_m7_demo.sh

# 只观察核心话题
ros2 topic echo /sim/object_pose --once
ros2 topic echo /gripper/state --once
ros2 topic echo /teleop/gripper_cmd --once
ros2 topic echo /ft_sensor --once

# 直接检查 XML 里的 object/finger 接触参数
python3 - <<'PY'
import mujoco
m = mujoco.MjModel.from_xml_path("config/models/franka_panda.xml")
for name in ["target_object_geom", "floor"]:
    gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name)
    print(name, "size", m.geom_size[gid], "friction", m.geom_friction[gid],
          "condim", m.geom_condim[gid], "solref", m.geom_solref[gid],
          "solimp", m.geom_solimp[gid])
for body in ["left_finger", "right_finger"]:
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body)
    print(body, "mass", m.body_mass[bid], "inertia", m.body_inertia[bid])
PY
```

日志判读：
- `finger_object_contacts > 0`：指尖和物体已经发生 MuJoCo contact；若仍滑落，优先调 `friction`、`condim`、`solref/solimp` 和物体质量。
- `object_contacts > 0` 但 `finger_object_contacts == 0`：物体多半只碰到地面或手掌，检查 `batch_generator` 的 `pick_height_offset`、物体姿态和 wrist camera 视角。
- `contacts_total == 0` 且 `ee_object_dist` 很大：先查 `/sim/object_pose` 是否更新、Servo 是否跟踪到 hover/pick pose。
- `gripper_cmd=0` 但 `gripper_qpos` 不收敛：查 `gripper_motor` actuator、`panda_finger_joint*` 映射和 joint range。

### M6/M7 视触觉链路调试

面试叙事：本项目能讲成 `MuJoCo 指尖触觉相机 → camera_bridge GelSight-like 光度立体彩色图 → lerobot_recorder 多模态同步落盘`。这条线贴合视触觉感知与灵巧操作岗位，但要明确是软件仿真链路，不宣称真实 GF515 硬件精度。

关键文件：
- `config/models/franka_panda.xml`：`left_tactile_camera` / `right_tactile_camera` 嵌入左右指尖内侧。
- `src/camera_bridge/camera_bridge/camera_bridge_node.py`：订阅 `/sim/object_pose`，渲染前同步本地 `target_object_joint`；`tactile_mode` 读取 depth，计算形变梯度并生成 RGB photometric-stereo 风格触觉图。
- `src/lerobot_recorder/lerobot_recorder/time_sync.py`：把 scene、wrist、左右 tactile、joint、ee、ft、object pose 放入 `ApproximateTimeSynchronizer`。
- `src/lerobot_recorder/lerobot_recorder/recorder_node.py`：写入 `observation.images.tactile_left/right`。

```bash
# 检查左右触觉图像话题是否在线
ros2 topic hz /camera/tactile_left/image_raw --window 50
ros2 topic hz /camera/tactile_right/image_raw --window 50
ros2 topic echo /sim/object_pose --once

# 检查触觉桥参数、topic 名和 /sim/object_pose 订阅
ros2 node info /left_tactile_bridge
ros2 node info /right_tactile_bridge

# 检查 recorder 是否写入触觉字段
python3 - <<'PY'
from datasets import load_from_disk
ds = load_from_disk("data/episodes/episode_000000/train")
print(ds.features)
for key in ["observation.images.tactile_left", "observation.images.tactile_right"]:
    assert key in ds.features, key
PY

# 采集真实 M6 面试图片；只认 ROS 图像话题或 recorder 数据集里的真实帧
python3 scripts/capture_m6_media.py \
  --output media/m6 \
  --dataset .m6_validation/episodes/episode_000000/train \
  --timeout 8

# 一键 M6 验收 + 自动刷新 media/m6/*.png
bash scripts/validate_m6_perception_recorder.sh --launch

# 面试前只补真实 MuJoCo/EGL 图片时，可放宽速率/帧数阈值；不要用示意图顶替
MUJOCO_GL=egl M6_MIN_CAMERA_HZ=2 M6_MIN_FRAMES=5 \
  bash scripts/validate_m6_perception_recorder.sh --launch
```

注意：`scripts/generate_m5_m7_media.py` 只能画示意图，不能作为 M6 scene/wrist/tactile 的 Core 证据。看 `media/m6/capture_manifest.json` 的 `fresh_files`，确认本次真实刷新了 `camera_rgb_view.png`、`wrist_camera_view.png`、`tactile_left_view.png`、`tactile_right_view.png` 和 `lerobot_dataset_features.png`。
如果日志中出现 `synthetic fallback`，这批图片只能说明 topic 在线，不能当作真实 MuJoCo 渲染图；真实图应看到 `camera_bridge up (... MuJoCo renderer ...)`。
`media/m6/multimodal_sync.png` 要由 `capture_m6_media.py` 从 LeRobotDataset 生成；它是行级同步摘要，不是手绘 topic 曲线。

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
from datasets import Array3D, Dataset, Features, Sequence, Value

EPISODE_FEATURES = Features({
    "observation.state":       Sequence(Value("float32"), length=7),
    "action":                  Sequence(Value("float32"), length=7),
    "observation.ee_pose":     Sequence(Value("float32"), length=7),
    "observation.object_pose": Sequence(Value("float32"), length=7),
    "observation.images.tactile_left":  Array3D(dtype="uint8", shape=(240, 320, 3)),
    "observation.images.tactile_right": Array3D(dtype="uint8", shape=(240, 320, 3)),
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

- **V2 工业级架构（当前基线）**：[docs/ARCHITECTURE_V2.md](../docs/ARCHITECTURE_V2.md)
- **开发路线图与里程碑检查清单**：[docs/ROADMAP.md](../docs/ROADMAP.md)

### V2 里程碑细化 SPEC（当前）

- **M1** ros2_control 骨架 + MuJoCo：[docs/SPEC_V2_M1_CONTROL_SKELETON.md](../docs/SPEC_V2_M1_CONTROL_SKELETON.md)
- **M2** CANopen DS402 现场总线：[docs/SPEC_V2_M2_CANOPEN_FIELDBUS.md](../docs/SPEC_V2_M2_CANOPEN_FIELDBUS.md)
- **M3** 笛卡尔阻抗控制器（插件）：[docs/SPEC_V2_M3_IMPEDANCE_CTRL.md](../docs/SPEC_V2_M3_IMPEDANCE_CTRL.md)
- **M4** MoveIt Servo 运动层：[docs/SPEC_V2_M4_MOTION_LAYER.md](../docs/SPEC_V2_M4_MOTION_LAYER.md)
- **M5** 安全层 + E-Stop 闭环：[docs/SPEC_V2_M5_SAFETY_LAYER.md](../docs/SPEC_V2_M5_SAFETY_LAYER.md)
- **M6** 视觉感知 + LeRobot Recorder：[docs/SPEC_V2_M6_PERCEPTION_RECORDER.md](../docs/SPEC_V2_M6_PERCEPTION_RECORDER.md)
- **M7** 遥操作设备可插拔 + 合成数据：[docs/SPEC_V2_M7_TELEOP_SYNTH.md](../docs/SPEC_V2_M7_TELEOP_SYNTH.md)
- **策略部署流程**：[docs/POLICY_DEPLOYMENT.md](../docs/POLICY_DEPLOYMENT.md)

### V1 历史存档 SPEC（参照用）

- 整体设计规范（V1）：[docs/DESIGN_SPEC.md](../docs/DESIGN_SPEC.md)
- M1 CAN/RS485 SPEC（V1）：[docs/SPEC_M1_CAN_RS485.md](../docs/SPEC_M1_CAN_RS485.md)
- M2 MuJoCo 桥接 SPEC（V1）：[docs/SPEC_M2_MUJOCO_BRIDGE.md](../docs/SPEC_M2_MUJOCO_BRIDGE.md)
- M3 阻抗控制器 SPEC（V1）：[docs/SPEC_M3_IMPEDANCE_CTRL.md](../docs/SPEC_M3_IMPEDANCE_CTRL.md)
- M4 全链路集成 SPEC（V1）：[docs/SPEC_M4_FULL_PIPELINE.md](../docs/SPEC_M4_FULL_PIPELINE.md)
- M5 LeRobot 录制 SPEC（V1）：[docs/SPEC_M5_LEROBOT_RECORDER.md](../docs/SPEC_M5_LEROBOT_RECORDER.md)

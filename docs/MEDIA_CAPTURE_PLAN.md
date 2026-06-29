# 媒体采集计划（Portfolio Evidence Plan）

**目的**：为 README、简历和技术面试准备少量但可信的作品集证据，而不是为每个里程碑补齐全套 GUI 截图。
**存放路径**：`media/<milestone>/`。
**原始证据路径**：`.media_evidence/<timestamp>/`（已在 `.gitignore` 中忽略，用来追溯 PNG/GIF 的来源）。
**格式规范**：静态图用 PNG，主演示用 GIF（≤5MB）或 MP4（放 `media/demo/`，不默认进 git）。

> 新原则：README 只展示能支撑作品集叙事的核心证据。rqt、viewer、plot 等 GUI 截图可以作为补充，但不再是阻塞项；没有窗口时，可以用真实 ROS2 命令输出、MuJoCo offscreen render、`ros2 node info`/`ros2 topic` 原始日志生成证据图。

---

## 证据分级

| 等级 | 目标 | README 使用方式 |
|---|---|---|
| Core | 证明系统真的跑通，能支撑作品集主叙事 | 优先嵌入 README |
| Support | 解释架构、协议、数据结构 | 可嵌入 spec 或 README 表格 |
| Optional | GUI 观感补充，例如 rqt_graph、rqt_plot、rqt_robot_monitor | 有窗口时再采，不阻塞 |

## 最小证据链

| 证据点 | 文件 | 等级 | 当前状态 |
|---|---|---|---|
| M1 ros2_control + MuJoCo 闭环 | `media/m1/panda_gravity_comp.png`, `media/m1/joint_states_hz.png`, `media/m1/rqt_graph_m1.png` | Core | 已用真实 M1 运行证据刷新 |
| M2 CANopen DS402 现场总线 | `media/m2/candump_pdo.png`, `media/m2/ds402_state_machine.png`, `media/m2/emcy_fault_injection.png` | Core | 已用真实 vcan0 candump/DS402/EMCY 刷新 |
| M4 或 M7 主演示 | `media/m4/teleop_keyboard.gif` 或 `media/m7/grasp_demo.gif` | Core | M7 已用真实 MuJoCo scene camera capture 刷新；不能用示意 GIF 替代 |
| M7 夹爪近景 | `media/m7/gripper_closeup.gif`, `media/m7/gripper_closeup.png` | Core | 已用真实 wrist camera capture 刷新，用于说明夹爪闭合、接触和保持 |
| M6 视觉/视触觉 + LeRobot 数据闭环 | `media/m6/camera_rgb_view.png`, `media/m6/wrist_camera_view.png`, `media/m6/tactile_left_view.png`, `media/m6/tactile_right_view.png`, `media/m6/lerobot_dataset_features.png` | Core | 必须用 `capture_m6_media.py` 重采真实 ROS/topic 或 dataset 帧，不能沿用旧示意图 |
| V2 架构解释 | `media/m1/m1_control_loop_proof.svg`, `media/m2/m2_canopen_fieldbus_proof.svg` | Support | 可保留为说明图 |
| M3 阻抗控制细节 | `media/m3/controller_active.png`, `media/m3/ee_tracking_error.png` | Optional | 有真实曲线再补 |
| M5 安全层细节 | `media/m5/estop_and_reset.gif`, `media/m5/safety_diagnostics.png` | Optional | 有完整演示再补 |

## README 嵌入门槛

- 来源必须是真实运行：ROS2 CLI、MuJoCo offscreen/viewer、SocketCAN/candump、LeRobot dataset load、或基于这些原始输出生成的证据图。
- `scripts/generate_m5_m7_media.py` 只能生成示意/占位材料，不得作为 README 或作品集的 Core 证据来源。
- 图中不能包含 `surrogate`、`placeholder`、`required capture`、`still needed`、`replace with real` 等占位文案。
- 文件内容必须和验收点一致，例如 `emcy_fault_injection.png` 必须能看到 EMCY 或 Fault 注入证据。
- 每张生成图都应能追溯到 `.media_evidence/<timestamp>/`、运行日志或生成脚本。

---

## M1 - ros2_control + MuJoCo 闭环

**作品集目标**：证明 M1 不是静态设计，而是实际启动了 MuJoCo、`controller_manager`、`joint_state_broadcaster`，并输出约 1kHz 的 `/joint_states`。

当前 M1 证据来源：

```bash
.media_evidence/m1_20260628_123549/
```

采集命令：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 control list_controllers
timeout 8s ros2 topic hz /joint_states --window 100
ros2 node info /canopen_hw_backplane
ros2 node info /mujoco_sim
ros2 node info /joint_state_broadcaster
```

| 文件 | 内容 | 说明 |
|---|---|---|
| `media/m1/panda_gravity_comp.png` | MuJoCo Panda ready pose | 来自 `config/models/franka_panda.xml` 的 offscreen render，适合无窗口环境 |
| `media/m1/joint_states_hz.png` | `list_controllers` + `/joint_states` 频率 | 来自真实 ROS2 CLI 输出，显示 998-1001 Hz |
| `media/m1/rqt_graph_m1.png` | M1 live ROS graph proof | 由 `ros2 node info` 关系渲染；不要求实际 rqt 窗口 |
| `media/m1/m1_control_loop_proof.svg` | 最小闭环说明图 | Support 图，可用于解释链路 |

README 可直接嵌入 M1 三张 PNG，或只嵌入 `panda_gravity_comp.png` + `joint_states_hz.png`。

---

## M2 - CANopen DS402 现场总线

**作品集目标**：证明控制链路经过真实 SocketCAN/vcan0，而不是只在 ROS topic 里闭环。

必须保留的证据：

| 文件 | 内容 |
|---|---|
| `media/m2/candump_pdo.png` | `candump -L vcan0` 中可见 SYNC、RPDO1、TPDO1、TPDO2 周期帧 |
| `media/m2/ds402_state_machine.png` | `/servo_drive/status` 确认 7 轴均进入 DS402 Operation Enabled |
| `media/m2/emcy_fault_injection.png` | `/inject_fault_joint1` 后可见 EMCY `0x081` 和 node 1 Fault 状态 |

当前 M2 证据来源：

```bash
.media_evidence/m2_20260628_130122/
```

采集命令：

```bash
ros2 launch teleop_description description.launch.py use_sim:=false can_interface:=vcan0
ros2 launch teleop_bringup fieldbus.launch.py use_sim:=false can_interface:=vcan0
ros2 launch teleop_bringup simulation.launch.py headless:=true randomize:=false model_path:=config/models/franka_panda.xml camera_rate:=5.0
ros2 launch teleop_bringup ros2_control.launch.py use_sim:=false can_interface:=vcan0 controller:=forward
timeout 5s candump -L vcan0
ros2 topic echo /servo_drive/status --once
ros2 service call /inject_fault_joint1 std_srvs/srv/Trigger "{}"
```

关键结果：

- `candump_pdo.log`：105,805 帧 / 4.999s，包含 `0x201-0x207`、`0x181-0x187`、`0x281-0x287` 和 `0x080`。
- `drive_status_once_before_fault.txt`：7 轴 `ds402_state: 4`、`statusword: 39`、`controlword: 15`、`fault_code: 0`。
- `candump_emcy.log`：故障注入后捕获 `vcan0 081#10320000000000`；`drive_status_once_after_fault_correct.txt` 显示 node 1 `ds402_state: 7`、`fault_code: 12816`。

推荐原始命令：

```bash
bash scripts/setup_vcan.sh
ros2 launch teleop_bringup m2_fieldbus.launch.py
timeout 5s candump vcan0
ros2 topic echo /servo_drive/status --once
```

如果没有 GUI，优先保存 raw txt，再生成终端式 PNG。

---

## M4/M7 - 主演示 GIF（二选一优先）

**作品集目标**：让面试官一眼看到系统会动。M7 抓取演示优先于 M4 键盘遥操作；如果 M7 暂时不稳，就先用 M4。

| 文件 | 内容 | 优先级 |
|---|---|---|
| `media/m7/grasp_demo.gif` | MuJoCo 中完成接近、夹取、抬起 | 最高 |
| `media/m7/gripper_closeup.gif` | wrist/夹爪近景，确认指尖闭合、接触力和方块保持 | 最高 |
| `media/m4/teleop_keyboard.gif` | 键盘/遥操作输入驱动机械臂 | 备选 |
| `media/m4/e2e_latency.png` | 延迟测量输出 | Support |

录制建议：

```bash
bash scripts/capture_m7_demo.sh
# 或
bash scripts/validate_m4_motion_layer.sh --launch
```

说明：`capture_m7_demo.sh` 默认使用 `full_system.launch.py` 的 `use_sim:=true` sim-direct 路径，以提高抓取 GIF 录制稳定性。脚本会同时录制 scene 主视角 `grasp_demo.gif` 和 wrist 近景 `gripper_closeup.gif`。这些 GIF 只证明运动/抓取/视觉链路，不作为 CANopen 现场总线证据；CAN 证据以 M2 的 `candump`/DS402/EMCY 和 M5 的 Quick Stop CAN 模式验收为准。

M7 录制前健康检查：

```bash
ros2 topic hz /camera/color/image_raw --window 50
ros2 topic hz /camera/wrist/color/image_raw --window 50
ros2 topic echo /gripper/state --once
ros2 topic pub --once /teleop/gripper_cmd std_msgs/msg/Float64 "{data: 0.0}"
ros2 topic echo /gripper/state --once
ros2 topic pub --once /teleop/gripper_cmd std_msgs/msg/Float64 "{data: 1.0}"
ros2 topic echo /gripper/state --once
```

通过条件：

- scene 与 wrist RGB 都稳定发布；M7 GIF 优先使用 scene 视角，wrist 视角用于确认夹爪和物体接触。
- `/teleop/gripper_cmd` 发 `0.0` 后 `/gripper/state` 应向闭合变化，发 `1.0` 后应向打开变化。
- wrist 画面中能看到夹爪指尖和目标方块；如果只看到远景或空画面，先修相机位姿/话题再录制。
- 抓取后抬升阶段方块应保持在夹爪中；默认 sim-direct demo 启用 MuJoCo grasp assist，用于稳定合成数据和作品集 GIF。

M7 失败回退：

- 如果抓取物理不稳但运动链路稳定，先录 `media/m4/teleop_keyboard.gif` 作为主展示。
- 如果 scene GIF 看不清夹爪，使用 `media/m7/gripper_closeup.gif` 作为 README 近景补充；它由 `capture_m7_demo.sh` 同步录制。
- 如果 recorder 已生成 episode，但 GIF 失败，优先保存 `dataset.features`、`/gripper/state` 日志和 wrist 单帧，避免重跑时失去可追溯证据。

没有桌面窗口时，先用 recorder/offscreen 帧生成 GIF；GUI 录屏只是加分项。

---

## M6 - 视觉 + LeRobot 数据闭环

**作品集目标**：证明系统不是只做控制，还能输出训练数据。

| 文件 | 内容 |
|---|---|
| `media/m6/camera_rgb_view.png` | MuJoCo scene camera 或 `/camera/color/image_raw` 的真实帧 |
| `media/m6/wrist_camera_view.png` | wrist/夹爪近景 `/camera/wrist/color/image_raw` 的真实帧，确认方块和指尖可见 |
| `media/m6/tactile_left_view.png` | 左指尖 `/camera/tactile_left/image_raw` 的真实帧，确认 GelSight-like 视触觉输出 |
| `media/m6/tactile_right_view.png` | 右指尖 `/camera/tactile_right/image_raw` 的真实帧，确认 GelSight-like 视触觉输出 |
| `media/m6/lerobot_dataset_features.png` | `load_from_disk(...).features` 输出，必须包含 `observation.images.wrist` 与 `observation.images.tactile_left/right` |
| `media/m6/multimodal_sync.png` | Optional，基于真实 LeRobotDataset 的同步帧摘要，不使用示意曲线 |

推荐命令：

```bash
bash scripts/validate_m6_perception_recorder.sh --launch

# 面试补真实图片：用 MuJoCo EGL renderer，允许 headless 多相机低帧率
MUJOCO_GL=egl M6_MIN_CAMERA_HZ=2 M6_MIN_FRAMES=5 \
  bash scripts/validate_m6_perception_recorder.sh --launch

# 如果 M6 stack 已经在运行，可单独刷新真实图片
python3 scripts/capture_m6_media.py \
  --output media/m6 \
  --dataset .m6_validation/episodes/episode_000000/train \
  --timeout 8
```

M6 录制前健康检查：

```bash
ros2 topic hz /camera/color/image_raw --window 50
ros2 topic hz /camera/wrist/color/image_raw --window 50
ros2 topic hz /camera/tactile_left/image_raw --window 50
ros2 topic hz /camera/tactile_right/image_raw --window 50
```

通过条件：

- scene、wrist、left tactile、right tactile 图像均稳定接近 30Hz。
- 数据集 features 中包含 `observation.images.scene`、`observation.images.wrist`、`observation.images.tactile_left`、`observation.images.tactile_right`、`observation.depth.scene`。
- tactile 图像必须来自真实 `camera_bridge` 话题或 fallback 触觉动画，不能用 `scripts/generate_m5_m7_media.py` 的示意图替代。
- `media/m6/capture_manifest.json` 的 `fresh_files` 必须包含上述 5 个 Core 文件；如果目录里只有旧 PNG 但 manifest 没有刷新，不能用于面试/README。
- 真实 MuJoCo 图片要求 launch 日志里能看到 `camera_bridge up (... MuJoCo renderer ...)`；如果出现 `synthetic fallback`，只能作为 topic 在线证据，不作为真实渲染截图。
- `multimodal_sync.png` 应由 `capture_m6_media.py` 从 `.m6_validation/episodes/episode_000000/train` 生成；它证明 recorder 行级同步和字段完整，不宣称原始 topic jitter 曲线。

---

## Optional GUI 证据

这些图可以提升观感，但不再阻塞 README：

| 工具 | 用途 |
|---|---|
| `rqt_graph` | 直观看节点/话题拓扑 |
| `rqt_plot` | M3 末端误差、接触力曲线 |
| `rqt_robot_monitor` | M5 安全诊断面板 |
| MuJoCo viewer 录屏 | 替代 offscreen render，观感更自然 |

如果当前屏幕没有窗口或远程会话无法截图，直接跳过 GUI，保留真实 CLI/offscreen 证据即可。

---

## 原始证据采集

通用采集脚本：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash scripts/collect_media_evidence.sh
```

脚本会把 `ros2 topic hz`、`ros2 control list_controllers`、`candump`、dataset features 等输出保存到 `.media_evidence/<timestamp>/`。这些 raw txt 不进 git，但用于生成可追溯的 PNG/GIF。

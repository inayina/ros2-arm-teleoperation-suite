# 媒体采集计划（Media Capture Plan）

**目的**：为 README 和技术演示提供视觉证明，按里程碑逐步补充。  
**存放路径**：`media/<milestone>/`，文件命名见各节。  
**格式规范**：截图用 PNG，动图用 GIF（≤5MB），录屏用 MP4（`media/demo/`）。

---

## 总览

| 里程碑 | 必拍内容 | 优先级 | 状态 |
|---|---|---|---|
| M1 | MuJoCo Panda 站立 + rqt_graph | ⭐⭐⭐ | ✅ 已补齐 |
| M2 | candump PDO 帧 + DS402 状态机 | ⭐⭐⭐ | ✅ 已补齐 |
| M3 | 末端跟踪误差曲线 + 接触柔顺 | ⭐⭐⭐ | ⬜ 待补 |
| M4 | 键盘遥操作端到端 GIF | ⭐⭐⭐ | ⬜ 待补 |
| M5 | E-Stop 触发 + 复位流程 | ⭐⭐ | ⬜ 待补 |
| M6 | LeRobot Episode 数据结构 + 相机画面 | ⭐⭐ | ⬜ 待补 |
| M7 | 抓取任务 Demo GIF（核心演示） | ⭐⭐⭐ | ⬜ 待补 |

---

## M1 — ros2_control + MuJoCo

**触发时机**：`ros2 launch teleop_bringup m1_control_sim.launch.py` 成功运行后

### ✅ M1-0：闭环视觉证明图
- **内容**：`forward_effort_controller → canopen_system(use_sim:=true) → /sim/* → mujoco_sim → joint_state_broadcaster → /joint_states` 最小闭环
- **文件名**：`media/m1/m1_control_loop_proof.svg`
- **嵌入位置**：README.md `### 演示` 区域；`docs/SPEC_V2_M1_CONTROL_SKELETON.md`
- **状态**：已补充

### 📸 M1-1：MuJoCo Panda 重力补偿站立
- **内容**：MuJoCo viewer 窗口，Panda 在重力补偿下保持竖直站立
- **命令**：
  ```bash
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  ros2 launch teleop_bringup m1_control_sim.launch.py
  ```
- **截取方式**：截 MuJoCo viewer 窗口（含机械臂全身），PNG
- **文件名**：`media/m1/panda_gravity_comp.png`
- **嵌入位置**：README.md `### 演示` 区域（M1 行）
- **状态**：已补充

### 📸 M1-2：rqt_graph 节点拓扑图
- **内容**：`/joint_states`、`/sim/joint_effort_cmd`、`/sim/encoder_state` 连线清晰可见
- **命令**：
  ```bash
  ros2 run rqt_graph rqt_graph
  # 勾选 "Nodes/Topics (all)"，取消 "Dead sinks" / "Leaf topics"
  ```
- **截取方式**：截 rqt_graph 窗口，PNG
- **文件名**：`media/m1/rqt_graph_m1.png`
- **嵌入位置**：README.md M1 行 / `docs/SPEC_V2_M1_CONTROL_SKELETON.md`
- **状态**：已补充

### 📸 M1-3：`/joint_states` 频率验证
- **内容**：终端输出 `ros2 topic hz /joint_states`，显示 ~1000 Hz
- **命令**：
  ```bash
  ros2 topic hz /joint_states
  ```
- **截取方式**：终端截图（包含 `average rate: 1000.xxx` 那行），PNG
- **文件名**：`media/m1/joint_states_hz.png`
- **状态**：已补充（当前图使用 M1 launch 日志中的 `controller_manager update rate is 1000 Hz` 与 `joint_state_broadcaster` 激活记录）

---

## M2 — CANopen DS402 现场总线

**触发时机**：`candump vcan0` 可见周期 PDO 帧，DS402 到 Operation Enabled

### ✅ M2-0：现场总线视觉证明图
- **内容**：`canopen_system(use_sim:=false) → vcan0 RPDO/SYNC → virtual_servo_driver ×7 → /sim/* → TPDO → /joint_states`，并包含 EMCY 故障注入路径
- **文件名**：`media/m2/m2_canopen_fieldbus_proof.svg`
- **嵌入位置**：README.md `### 演示` 区域；`docs/SPEC_V2_M2_CANOPEN_FIELDBUS.md`
- **状态**：已补充

### 📸 M2-1：candump 周期 PDO 帧
- **内容**：终端滚动显示 vcan0 上的 RPDO/TPDO 周期帧（`0x180+id`、`0x200+id`）
- **命令**：
  ```bash
  candump vcan0 | head -40
  ```
- **截取方式**：终端截图，显示至少 3 个关节的帧，PNG
- **文件名**：`media/m2/candump_pdo.png`
- **嵌入位置**：README.md M2 行 / `docs/SPEC_V2_M2_CANOPEN_FIELDBUS.md`
- **状态**：已补充

### 📸 M2-2：DS402 状态机转换日志
- **内容**：终端日志显示 `Switch On Disabled → Ready → Switched On → Operation Enabled`
- **截取方式**：终端截图，PNG
- **文件名**：`media/m2/ds402_state_machine.png`
- **状态**：已补充

### 📸 M2-3：故障注入 EMCY 帧
- **内容**：`candump` 显示 EMCY 帧（`0x080+id`），同时 `virtual_servo_driver` 日志显示进入 `Fault`
- **文件名**：`media/m2/emcy_fault_injection.png`
- **状态**：已补充（当前图记录 `/servo_drive/status` 已锁存的 `Fault` 状态；未主动调用新的 `inject_fault` 服务）

---

## M3 — 笛卡尔阻抗控制器

**触发时机**：阻抗控制器 active，末端跟踪误差可量测

### 📈 M3-1：末端跟踪误差实时曲线
- **内容**：`rqt_plot` 显示末端位置误差 ≤ 2mm（x/y/z 三条线）
- **命令**：
  ```bash
  ros2 run rqt_plot rqt_plot /ee_pose/pose/position/x /ee_pose/pose/position/y
  ```
- **截取方式**：截 rqt_plot 窗口，曲线稳定后截图，PNG
- **文件名**：`media/m3/ee_tracking_error.png`
- **嵌入位置**：README.md M3 行

### 📈 M3-2：接触柔顺力曲线
- **内容**：`rqt_plot` 显示 `/ft_sensor` 在接触瞬间力矩变化，控制器自动降刚度
- **文件名**：`media/m3/contact_compliance_ft.png`

### 📸 M3-3：控制器 active 状态
- **内容**：`ros2 control list_controllers` 输出，`cartesian_impedance_controller [active]`
- **文件名**：`media/m3/controller_active.png`

---

## M4 — MoveIt Servo 运动层 ⭐ 核心演示

**触发时机**：键盘→伺服→阻抗→CAN→MuJoCo 端到端可用

### 🎬 M4-1：键盘遥操作端到端 GIF（主演示）
- **内容**：左半屏终端键盘操作，右半屏 MuJoCo viewer 机械臂跟随移动
- **工具**：`peek`（`sudo apt install peek`）或 `ffmpeg` + `convert`
  ```bash
  # 录屏为 mp4，再转 gif
  ffmpeg -i teleop_demo.mp4 -vf "fps=15,scale=720:-1" -loop 0 media/m4/teleop_keyboard.gif
  ```
- **时长**：15–30 秒，展示 XYZ 三个方向移动
- **文件名**：`media/m4/teleop_keyboard.gif`
- **嵌入位置**：**README.md 演示区域顶部**（最重要的一张）

### 📸 M4-2：端到端延迟测量
- **内容**：`ros2 topic delay /joint_target` 输出，显示 < 50ms
- **文件名**：`media/m4/e2e_latency.png`

### 📸 M4-3：奇异点/限位减速截图
- **内容**：接近关节限位时 `servo_node` 日志显示自动减速警告
- **文件名**：`media/m4/singularity_slowdown.png`

---

## M5 — 安全层 + E-Stop

**触发时机**：5 个监视器单测通过，E-Stop 闭环可演示

### 🎬 M5-1：E-Stop 触发 + 复位 GIF
- **内容**：发送超限指令 → 安全层拒绝 → `/safety/estop` → 力矩归零 → `/safety/reset` 复位
- **文件名**：`media/m5/estop_and_reset.gif`
- **嵌入位置**：README.md M5 行

### 📸 M5-2：rqt_robot_monitor 安全诊断
- **内容**：rqt_robot_monitor 显示 5 个子监视器全部 OK
- **命令**：
  ```bash
  ros2 run rqt_robot_monitor rqt_robot_monitor
  ```
- **文件名**：`media/m5/safety_diagnostics.png`

---

## M6 — 视觉 + LeRobot Recorder

**触发时机**：camera_bridge + recorder 运行，Episode 可录制

### 📸 M6-1：RGB/Depth 相机画面
- **内容**：`rqt_image_view` 显示 `/camera/color/image_raw`（机器人工作空间视角）
- **命令**：
  ```bash
  ros2 run rqt_image_view rqt_image_view /camera/color/image_raw
  ```
- **文件名**：`media/m6/camera_rgb_view.png`

### 📸 M6-2：LeRobot Dataset 数据结构
- **内容**：Python 终端打印 `dataset.features`，显示所有字段（state/ee/ft/rgb/depth/action）
- **命令**：
  ```python
  from datasets import load_from_disk
  ds = load_from_disk("data/episodes/episode_000000/train")
  print(ds.features)
  ```
- **文件名**：`media/m6/lerobot_dataset_features.png`
- **嵌入位置**：README.md M6 行 / `docs/SPEC_V2_M6_PERCEPTION_RECORDER.md`

### 📸 M6-3：多模态录制时序对齐
- **内容**：`rqt_plot` 显示关节状态、FT 传感器、相机时间戳对齐曲线
- **文件名**：`media/m6/multimodal_sync.png`

---

## M7 — 遥操作设备 + 合成数据（核心演示）

**触发时机**：Domain Randomization 数据生成可跑，抓取场景完成

### 🎬 M7-1：抓取任务 Demo GIF（最终演示）
- **内容**：机械臂在 MuJoCo 中完成夹爪抓取物体全流程（≥15秒）
- **录制要求**：
  - 包含物体（cube/sphere）和 Panda 末端 + 夹爪
  - 展示抓取过程：接近 → 对准 → 夹取 → 抬起
  - 终端同时显示 `/ft_sensor` 数值变化（证明接触感知）
- **文件名**：`media/m7/grasp_demo.gif`
- **嵌入位置**：**README.md 演示区域顶部**，替换 M4 GIF 或并排

### 🎬 M7-2：Domain Randomization 数据多样性展示
- **内容**：批量生成不同物体位置的仿真截图拼图（3x3 grid）
- **文件名**：`media/m7/domain_randomization_grid.png`

### 📸 M7-3：策略部署验证
- **内容**：策略推理节点运行日志，显示推理延迟和动作输出
- **文件名**：`media/m7/policy_inference_log.png`

---

## README 嵌入计划

> 按此顺序将图片嵌入 `README.md` 的 `### 演示` 区域：

```markdown
### 演示

#### 系统架构总览
![rqt_graph 节点拓扑](media/m1/rqt_graph_m1.png)

#### M1 — Panda 重力补偿（仿真）
![M1 ros2_control + MuJoCo 闭环视觉证明](media/m1/m1_control_loop_proof.svg)
![Panda 在 MuJoCo 中重力补偿站立](media/m1/panda_gravity_comp.png)

#### M2 — CANopen DS402 总线
![M2 CANopen DS402 现场总线视觉证明](media/m2/m2_canopen_fieldbus_proof.svg)
![candump 周期 PDO 帧](media/m2/candump_pdo.png)

#### M4 — 键盘遥操作端到端（主演示）
![键盘控制机械臂实时运动](media/m4/teleop_keyboard.gif)

#### M5 — E-Stop 安全闭环
![E-Stop 触发与复位流程](media/m5/estop_and_reset.gif)

#### M6 — LeRobot 数据集
![多模态 Episode 数据结构](media/m6/lerobot_dataset_features.png)

#### M7 — 夹爪抓取 Demo
![仿真夹爪抓取任务全流程](media/m7/grasp_demo.gif)
```

---

## 录制工具参考

```bash
# 安装截图/录屏工具
sudo apt install peek            # GUI GIF 录制
sudo apt install ffmpeg          # 命令行录屏

# 录屏 → GIF（推荐流程）
ffmpeg -video_size 1280x720 -framerate 30 -f x11grab -i :0.0 output.mp4
ffmpeg -i output.mp4 -vf "fps=15,scale=720:-1:flags=lanczos" -loop 0 output.gif

# 压缩 GIF（保持 ≤5MB）
gifsicle -O3 --colors 128 output.gif -o output_compressed.gif
```

# XBOX / Gamepad 遥操作使用指南

本项目内置了对主流手柄（如 Xbox / PS 虚拟或实体手柄）的支持。基于 `inputs` 库直接读取事件并将其转化为机械臂的末端笛卡尔位姿增量。

---

## 1. 环境依赖与权限配置

由于手柄驱动通过直接读取 Linux 的输入设备事件文件（`/dev/input/event*`）进行工作，您需要安装 Python 依赖库并对用户授权：

### 1.1 安装 inputs 库
在您运行 ROS 2 节点的 Python 环境中安装：
```bash
pip install inputs
```

### 1.2 授权当前用户读取输入设备 (极其重要)
在 Linux 系统中，默认情况下非 root 用户没有读取 `/dev/input/` 设备事件的权限。
请运行以下命令将当前用户加入 `input` 用户组，以免发生初始化失败或无法识别手柄的问题：
```bash
sudo usermod -aG input $USER
```
> [!IMPORTANT]
> 执行完组别修改命令后，您需要**注销并重新登录系统**（或者重启终端、运行 `su - $USER`），以使组权限生效。

---

## 2. 轴与按键映射 (Keymap Layout)

系统支持【平移模式】与【旋转模式】的一键切换，默认处于平移模式。通过**按住左扳机键 (LT / L2)** 可以临时切换进入旋转模式：

### 2.1 摇杆控制模式 (LT 切换)

| 控制模式 | 摇杆输入 | 对应机械臂动作 | 控制逻辑 |
| :--- | :--- | :--- | :--- |
| **平移模式**<br>(松开 LT) | **左摇杆 (左右)** | 沿 Y 轴平移 (Left/Right) | 左右推动 |
| | **左摇杆 (上下)** | 沿 X 轴平移 (Forward/Backward) | 上下推动 |
| | **右摇杆 (上下)** | 沿 Z 轴平移 (Up/Down) | 上下推动 |
| | **右摇杆 (左右)** | *无/保留* | |
| **旋转模式**<br>(按住 LT > 0.5) | **左摇杆 (左右)** | 沿 X 轴翻滚 (Roll) | 左右推动 |
| | **左摇杆 (上下)** | 沿 Y 轴俯仰 (Pitch) | 上下推动 |
| | **右摇杆 (左右)** | 沿 Z 轴自转 (Yaw) | 左右旋转 |
| | **右摇杆 (上下)** | *无/保留* | |

### 2.2 功能按键

| 按键名称 | 手柄物理按键 | 对应功能 | 说明 |
| :--- | :--- | :--- | :--- |
| **A / 交叉键** | `BTN_SOUTH` | 打开夹爪 | 向 `/teleop/gripper_cmd` 发送 `1.0` |
| **B / 圆圈键** | `BTN_EAST` | 合上夹爪 | 向 `/teleop/gripper_cmd` 发送 `0.0` |
| **X / 方块键** | `BTN_WEST` | 夹爪开/合切换 | 保留 toggle 模式 |
| **Y / 三角键** | `BTN_NORTH` | 回到初始位姿 (Home) | 单击重置机械臂为初始 Pose |
| **R1 / RB 键** | `BTN_TR` | 启动数据集录制 | 向 `/teleop/record_trigger` 发送 "start" |
| **L1 / LB 键** | `BTN_TL` | 结束数据集录制 | 向 `/teleop/record_trigger` 发送 "stop" |

---

## 3. 运行指南

### 3.1 推荐方式：随全链路一起启动 Xbox 驱动

不要先启动默认键盘遥操作再另开一个手柄节点。系统里只能有一个 `/teleop/cmd_pose` 主控发布者，否则两个输入源会互相覆盖，机械臂看起来就像“不动”。

```bash
ros2 launch teleop_bringup full_system.launch.py \
  headless:=false \
  teleop_driver:=gamepad
```

此时控制台应该会输出：`teleop_input driver 'gamepad' ready (...)`。

### 3.2 备选方式：单独启动手柄节点

如果你想在另一个终端单独跑手柄节点，启动全系统时必须关闭内置 teleop：

终端 1：
```bash
ros2 launch teleop_bringup full_system.launch.py \
  headless:=false \
  start_teleop:=false
```

终端 2：
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_input teleop_input_node --ros-args -p driver_type:=gamepad
```

* 拿起您的 Xbox 手柄，尝试轻推摇杆，机械臂末端即会开始响应动作。

---

## 4. 故障排查 (Troubleshooting)

1. **终端提示 `inputs not installed` 或 `Driver failed to initialize`**
   * 请确认是否在当前 ROS 2 执行的 Python 环境下运行了 `pip install inputs`。
   * 可以通过命令 `python3 -c "import inputs; print(inputs.devices.gamepads)"` 检查 Python 能否检测到手柄。如果输出为空列表 `[]`，说明系统未接入手柄或当前终端没有读取权限。

2. **手柄有输入，但机械臂无任何运动响应**
   * **有多个 teleop 发布者互相覆盖**：检查 `/teleop/cmd_pose` 是否只有 1 个发布者：
     ```bash
     ros2 topic info /teleop/cmd_pose
     ```
     如果 Publisher count 大于 1，请停掉多余的 `teleop_input_node`，或用 `start_teleop:=false` 启动全系统后再单独跑手柄节点。
   * **安全层触发挂起**：请检查命令行输出，确认是否因为动作过快或超界触发了 `safety_monitor` 的 E-STOP。若已触发 E-STOP，需要发送重置服务：
     ```bash
     ros2 service call /safety/reset std_srvs/srv/Trigger "{}"
     ```
     也可以先确认当前状态：
     ```bash
     ros2 topic echo /safety/estop --once
     ```
     如果输出 `data: true`，手臂控制器会强制零力矩，夹爪仍可能响应。
   * **未发布心跳**：检查手柄节点是否在正常发布心跳 `/teleop/heartbeat`（频率应在 ~50Hz）。
   * **仿真状态异常**：如果日志里出现 `TF_NAN_INPUT`、`Very close to a singularity`，请重启全系统。当前 fallback 仿真会从合法 ready pose 启动并过滤 NaN，避免再次污染 Servo。

3. **机械臂运动有轻微漂移**
   * 项目内置了死区过滤（Deadzone = 0.15）。如果您的手柄存在摇杆漂移且超出了 15% 的范围，可以修改 [gamepad_driver.py](file:///home/ina/dev/ros2-arm-teleoperation-suite/src/teleop_input/teleop_input/gamepad_driver.py#L71) 中的阈值进行微调。

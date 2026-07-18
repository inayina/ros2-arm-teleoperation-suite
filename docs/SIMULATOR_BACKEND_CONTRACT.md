# Simulator Backend Contract（P0 审计基线）

**状态**：P0–P4 implemented and functionally verified / P5 evidence-only comparison completed  
**审计日期**：2026-07-17  
**适用仓库**：`ros2-arm-teleoperation-suite`  
**默认实现**：MuJoCo  
**新增实现**：Isaac Sim 6.0 external runtime + ROS-only adapter

> 本文记录代码审计得到的当前事实和 P1 稳定边界。它不声明 Isaac 已完成项目采集。

---

## A. 当前实际数据流与依赖关系

### A.1 控制与状态链

```text
teleop_input / batch_generator
  → safety_monitor
  → MoveIt Servo
  → cartesian_impedance_controller / ros2_control
  → canopen_hw_interface(use_sim=true)
  → /sim/joint_effort_cmd
  → MuJoCo backend
  → /sim/encoder_state
  → canopen_hw_interface
  → /joint_states
```

代码证据：

- `src/mujoco_sim/mujoco_sim/mujoco_sim_node.py`：订阅 effort，发布 encoder/EE/FT/object/gripper，提供 reset。
- `src/teleop_bringup/launch/full_system.launch.py`：按 description → simulation → fieldbus/recording → safety → motion → ros2_control 编排。
- `src/teleop_bringup/launch/backends/mujoco.launch.py`：P1 后独占 MuJoCo 节点与 MuJoCo camera bridge 启动细节。

### A.2 采集与门禁链

```text
batch_generator
  → 选择 target_object_name
  → /sim/reset_scene
  → /teleop/record_trigger=start
  → FSM: hover/descend/close/lift/transport/place/release
  → _validate_episode
  → /lerobot_recorder/end_episode (commit/discard)
  → episode_*/train + meta.json
  → robot-arm-episode-data-lab
```

主门禁仍由上游 `batch_generator._validate_episode` 负责。中游只消费 raw episode/schema，不重新推导 lift/place 成败。

### A.3 观测链

```text
MuJoCo physics
  ├─ /ee_pose
  ├─ /ft_sensor
  ├─ /sim/object_pose
  ├─ /gripper/state
  └─ state synchronized into MuJoCo renderer
        → camera_bridge
        → scene/wrist/tactile RGB + optional depth
        → lerobot_recorder.MultiModalSync
```

`camera_bridge` 当前不是 backend-neutral renderer：它加载同一 MuJoCo XML，复制 joint/object/gripper 状态并调用 MuJoCo renderer。P1 只把它封装进 MuJoCo backend launch，不重写实现。

---

## B. MuJoCo 硬编码审计

### B.1 Launch 与包依赖

| 文件 | 审计结果 | P1 处理 |
|---|---|---|
| `src/teleop_bringup/launch/full_system.launch.py` | 原来固定 include `simulation.launch.py`，顶层暴露 XML、renderer 和 MuJoCo grasp 参数 | 增加 `sim_backend`，默认 MuJoCo；旧参数保留兼容 |
| `src/teleop_bringup/launch/simulation.launch.py` | 原来直接启动 `mujoco_sim` 和四个 camera bridge | 改为 backend selector，不再直接包含 MuJoCo Node |
| `src/teleop_bringup/launch/backends/mujoco.launch.py` | P1 新增 | 承接原 MuJoCo 节点和 camera 配置 |
| `src/teleop_bringup/launch/m1_control_sim.launch.py` | MuJoCo M1 专用验收、XML 默认路径 | 保持；通过 selector 默认值兼容 |
| `src/teleop_bringup/launch/m2_fieldbus.launch.py` | MuJoCo/CANopen 专用验收、XML 默认路径 | 保持；不是通用 full-system 入口 |
| `src/teleop_bringup/launch/fieldbus.launch.py` | 注释与 sim path 指向 MuJoCo | 行为保持；P2 再校正文案/能力 |
| `src/teleop_bringup/package.xml` | `exec_depend` 包含 `mujoco_sim`、`camera_bridge` | 保持默认后端可安装；P2 不把 Isaac Python 包加入此处 |

### B.2 Batch generator

| 位置/接口 | 审计结果 | P1 处理 |
|---|---|---|
| `target_object_name` | 既是 batch 输入，也是 simulator runtime 参数 | 保持语义 |
| `/mujoco_sim/set_parameters` | 原来由 `_set_node_parameter('/mujoco_sim', ...)` 固定调用 | 改为参数 `simulator_node_name`，默认 `/mujoco_sim` |
| `/sim/reset_scene` | backend-neutral 名称，`std_srvs/Trigger` | 保持 |
| `/sim/object_pose` | backend-neutral privileged observation | 保持 |
| `/ee_pose`、TF | FSM 到达判断和运动初态 | 保持 |
| `/lerobot_recorder/end_episode` | accepted/discard 落盘边界 | 保持 |
| `_validate_episode` 错误文本 | 原来写 “mujoco target_object_name” | 改为 “simulator target_object_name” |

P1 使用可配置节点名是兼容过渡，不把 ROS parameter service 宣称为最终跨后端对象选择 API。P2 应评估专用 `SetTargetObject`/scene-control service。

### B.3 MuJoCo implementation internals（不抽象）

以下内容应继续留在 `src/mujoco_sim/`：

- `mujoco.MjModel.from_xml_path`、`MjData`、`mj_step`、`mj_forward`；
- Panda joint/actuator/site/sensor 名称映射；
- `config/models/franka_panda.xml`；
- `DomainRandomizer` 对 body/geom/camera/light 的 MuJoCo API 修改；
- MuJoCo viewer、fallback integrator；
- grasp assist、contact hold、adaptive gripper force 和 contact debug；
- `virtual_camera.py` 的 `mujoco.Renderer`。

这些是 backend 实现细节，不应为了统一 Isaac USD/OmniGraph 而抽成共同 Python 基类。

### B.4 Camera bridge

当前直接耦合：

- `camera_bridge_node.py` 导入 `mujoco` 与 `mujoco_sim.virtual_camera`；
- 参数 `model_path` 默认 `config/models/franka_panda.xml`；
- 参数 `use_mujoco_renderer`；
- `object_sync.py` 固定 MuJoCo object→joint 命名和默认 `/mujoco_sim` 参数节点；
- scene/wrist/tactile bridge 都加载 MuJoCo 模型并根据 ROS 状态同步 renderer。

P1 决策：不改 camera bridge；只将它视为 MuJoCo backend 的 perception implementation。Isaac P2/P3 应直接发布相同 camera topic，或提供独立 Isaac image adapter。

### B.5 Recorder 与 telemetry

- `lerobot_recorder/time_sync.py` 的同步输入名称已经 backend-neutral，但注释仍提 MuJoCo。
- `lerobot_recorder/system_telemetry.py` 的进程匹配表包含 `mujoco_sim`。它只影响 telemetry 标签，不影响 frame schema；P2 扩展为 backend-aware。
- recorder frame schema、action semantics、`upstream_gate` 和 `EndEpisode` 不依赖 MuJoCo API，P1 保持不动。

### B.6 其他 MuJoCo 专用路径

下列路径属于专用测试、证据脚本或明确的 MuJoCo 实现，P1 不改：

- `config/models/franka_panda.xml`
- `src/mujoco_sim/**`
- `tests/test_domain_randomizer.py`
- `tests/test_mujoco_sim_fallback_fk.py`
- `tests/test_camera_bridge_object_mapping.py`
- `scripts/run_batch_preflight_smoke.sh`
- `scripts/validate_m6_perception_recorder.sh`
- `scripts/validate_m7_grasp_monitor.sh`
- `scripts/capture_m7_demo.sh`
- `scripts/collect_media_evidence.sh`
- `scripts/stop_stack.sh`

---

## C. Simulator Backend ROS Contract

### C.1 必需 runtime 接口

| 名称 | ROS 类型 | 当前 QoS | producer → consumer | 语义/约束 |
|---|---|---|---|---|
| `/sim/joint_effort_cmd` | `std_msgs/Float64MultiArray` | sensor-data | control/HW → backend | 前 7 项为 Panda joint1..7，单位 N·m |
| `/sim/encoder_state` | `sensor_msgs/JointState` | sensor-data | backend → simulated HW | name 必须含 Panda 7 joints；position rad、velocity rad/s、effort N·m |
| `/sim/object_pose` | `geometry_msgs/PoseStamped` | depth 10 / recorder sensor-data | backend → batch/camera/recorder | 当前 canonical frame=`world`，位置 m，四元数 ROS xyzw |
| `/sim/reset_scene` | `std_srvs/Trigger` | service | batch → backend | 成功后恢复 Panda/object 状态并重新开始发布；失败必须 `success=false` |
| `/ee_pose` | `geometry_msgs/PoseStamped` | depth 10 / recorder sensor-data | backend → batch/recorder | 当前 frame=`panda_link0`，位置 m，四元数 xyzw |
| `/ft_sensor` | `geometry_msgs/WrenchStamped` | depth 10 / recorder sensor-data | backend → controller/recorder | force N、torque N·m；frame 必须明确。MuJoCo=`panda_ee`，Isaac P4=`panda_hand` incoming-joint local frame |
| `/joint_states` | `sensor_msgs/JointState` | runtime-defined / recorder sensor-data | ros2_control → motion/camera/recorder | 上层 measured state；backend 不得绕过 runtime 语义 |
| `/tf`, `/tf_static` | TF2 | TF defaults | robot_state_publisher/runtime → consumers | 至少包含 `panda_link0`→`panda_ee` |

### C.2 相机 contract

| Topic | 类型 | frame |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` (`rgb8`) | `scene_camera_optical_frame` |
| `/camera/depth/image_raw` | `sensor_msgs/Image` (`32FC1`, m) | `scene_camera_optical_frame` |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | `scene_camera_optical_frame` |
| `/camera/wrist/color/image_raw` | `sensor_msgs/Image` (`rgb8`) | `wrist_camera_optical_frame` |
| `/camera/wrist/depth/image_raw` | `sensor_msgs/Image` (`32FC1`, m) | `wrist_camera_optical_frame` |
| `/camera/wrist/color/camera_info` | `sensor_msgs/CameraInfo` | `wrist_camera_optical_frame` |

Scene camera 是 `capture_mode=portfolio` 的当前必需视觉输入；wrist/depth/tactile 由 launch capability 控制。缺失接口不得以伪造零图像冒充真实 Isaac sensor。

### C.3 Recorder contract

| 名称 | 类型 | 语义 |
|---|---|---|
| `/teleop/record_trigger` | `std_msgs/String` | `start`, `stop_success`, `discard` 等兼容触发 |
| `/lerobot_recorder/end_episode` | `teleop_interfaces/srv/EndEpisode` | commit/discard 与返回 dataset path/frame count |

P4 保持 raw frame fields/action 维度和中游 schema 不变，只将 `simulator_backend`、`simulator_version`、`scene_id` 作为可选顶层 provenance 写入 `meta.json`。旧 episode 缺少这些字段仍受支持。

### C.4 backend control plane 过渡项

`target_object_name` 目前通过 ROS parameter service 设置。P1 暴露 batch 参数：

```yaml
simulator_node_name: /mujoco_sim
```

未来 Isaac adapter 可使用自己的节点名，但必须实现同名 string 参数，直到专用 scene-control service 取代该过渡契约。

---

## D. 应抽象与应保持实现专用的边界

### 应抽象

- 顶层 `sim_backend` 选择；
- backend 启动入口；
- `/sim/*`、EE、FT、joint state、TF、camera 和 recorder ROS contract；
- batch 对 simulator control node 的寻址；
- P2 之后的 capability/diagnostics/provenance。

### 不应抽象

- MuJoCo XML 与 Isaac USD scene；
- MuJoCo API 与 Isaac/Omniverse API；
- 两个 renderer 的内部实现；
- 各 backend 的 domain randomization 实现；
- MuJoCo grasp assist/contact debug；
- Isaac extension/OmniGraph 启动细节；
- 中游 schema adapter 和下游 PyBullet runtime。

---

## E. P0/P1 文件范围

### 新增

- `docs/SIMULATOR_BACKEND_CONTRACT.md`
- `src/teleop_bringup/launch/backends/mujoco.launch.py`
- `tests/test_sim_backend_launch.py`

### 修改

- `docs/SPEC_V2_SIM_BACKENDS_ISAAC.md`
- `docs/README.md`
- `src/teleop_bringup/launch/full_system.launch.py`
- `src/teleop_bringup/launch/simulation.launch.py`
- `src/synth_data_gen/synth_data_gen/batch_generator.py`
- `tests/test_batch_generator_validation.py`

### 保持不动

- `src/mujoco_sim/**`
- `src/camera_bridge/**`
- `src/lerobot_recorder/**`
- 中游与下游全部运行代码
- raw episode/schema/action semantics

---

## F. P0/P1 验收证据要求

```bash
python3 -m pytest -q tests/test_sim_backend_launch.py \
  tests/test_batch_generator_validation.py

colcon build --symlink-install --packages-select \
  teleop_bringup synth_data_gen camera_bridge lerobot_recorder mujoco_sim

source install/setup.bash
ros2 launch teleop_bringup full_system.launch.py --show-args

timeout 60s ros2 launch teleop_bringup full_system.launch.py \
  sim_backend:=mujoco start_teleop:=false record:=false \
  enable_grasp_monitor:=false capture_mode:=training headless:=true

colcon test --packages-select \
  teleop_bringup synth_data_gen camera_bridge lerobot_recorder mujoco_sim
colcon test-result --verbose
```

必须额外验证：

1. 不传 `sim_backend` 与显式 `mujoco` 都能启动 `/mujoco_sim`；
2. `sim_backend:=isaac` 在 P1 明确报 P2 未实现，不静默继续；
3. `sim_backend:=pybullet` 被 choices 拒绝；
4. `batch_generator.py` 不存在固定 `_set_node_parameter('/mujoco_sim', ...)` 调用；
5. 测试后清理所有 ROS/MuJoCo/recorder 进程。

---

## G. 已知风险与后续决策

- `simulator_node_name` 是 P1 兼容过渡；P2 需要决定专用 scene-control service。
- 顶层仍保留 MuJoCo 参数以保证命令兼容；P2 不应继续增加 backend 专属顶层参数。
- camera bridge 和 telemetry 尚未 backend-neutral；它们已被明确隔离/记录，不阻塞 P1。
- P1 不改变 recorder，因此没有 `meta.json` provenance；这是刻意保持 schema 稳定。
- Isaac 的 joint/object/reset/scene RGB/EE/FT 已完成 P3/P4 最小验证；wrist/depth、effort command、randomization、batch/grasp gate 尚未实现或验证。

---

## H. Isaac P2–P4 实际映射

| Raw Isaac interface | Canonical interface | P4 状态 |
|---|---|---|
| `/isaac/joint_states` | `/sim/encoder_state`（仅 Panda 7 arm joints） | 已实测 |
| finger positions in raw joint state | `/gripper/state`（mean/0.04，clamp 0..1） | 已实测录入 episode |
| `/isaac/object_pose` | `/sim/object_pose` | 已实测 |
| `/isaac/reset_scene_cmd` + done | `/sim/reset_scene` | 已实测 success |
| `/isaac/ee_pose` | `/ee_pose` | 已实测 |
| `/isaac/ft_sensor` | `/ft_sensor` | 已实测；panda_hand incoming-joint reaction，local frame |
| `/isaac/camera/color/image_raw` | `/camera/color/image_raw` + `/camera/scene/image_raw` | 已实测 320×240 |

相机 helper 使用 simulation time，而 recorder 其余链路使用 ROS system time；adapter 在 canonical camera boundary 重写 header stamp，避免跨时钟域同步失败。Isaac runtime 始终由隔离 venv/远程环境管理，普通 ROS workspace 只依赖 `isaac_sim_adapter`。

# SPEC V2: Simulator Backend Contract 与 Isaac Sim 渐进接入

**状态**：P0–P4 functionally verified / P5 evidence-only comparison completed（package lint baseline 另见第 10 节）  
**实施范围**：上游 `ros2-arm-teleoperation-suite`  
**关联仓库**：中游 `robot-arm-episode-data-lab`、下游 `ros2-moveit-pybullet-bridge`  
**默认后端**：MuJoCo  
**新增候选后端**：Isaac Sim 6.0  
**原则**：先契约、后适配；保持 MuJoCo 默认行为；不重构三仓职责边界

> 本文是实施与验收规范，不是完成状态声明。当前能力必须以代码、测试和实际运行证据为准。

---

## 1. 背景与直接结论

项目保持三仓分工：

| 仓库 | 职责 | 本计划是否修改 |
|---|---|---|
| `ros2-arm-teleoperation-suite` | ROS 2 仿真、遥操作、batch generation、episode recorder、upstream physical gate | 是，唯一主要实施仓库 |
| `robot-arm-episode-data-lab` | raw episode adapter、schema validation、release、EDA、训练与 handoff | 第一阶段不修改 |
| `ros2-moveit-pybullet-bridge` | PyBullet replay、tracking、distribution monitoring、risk benchmark | 不修改 |

MuJoCo 继续承担本地快速开发、数据采集和回归测试。Isaac Sim 是上游新增的高保真仿真后端，不替换 MuJoCo，不与下游 PyBullet 合并，也不改变中游训练主线。

目标数据流：

```text
MuJoCo backend ─┐
                 ├─ ROS simulation contract
Isaac backend ──┘
                         ↓
             batch generator / teleoperation
                         ↓
                  lerobot_recorder
                         ↓
          raw episode + meta.json contract
                         ↓
          robot-arm-episode-data-lab
```

---

## 2. 目标与非目标

### 2.1 目标

1. 在 `full_system.launch.py` 增加 `sim_backend:=mujoco|isaac`。
2. 保持 `sim_backend:=mujoco` 为默认值，使现有命令和测试不回归。
3. 将当前 MuJoCo 启动逻辑拆成独立 backend launch。
4. 让 MuJoCo 与 Isaac 对外提供相同或可适配的 ROS contract。
5. 移除 `batch_generator` 对固定 `/mujoco_sim` 节点名的依赖。
6. 尽量保持 recorder、raw episode 和中游 schema 不变。
7. 以一个 Isaac raw episode 通过现有 recorder 与中游 adapter/schema 检查作为阶段性闭环。

### 2.2 非目标

- 不删除 MuJoCo。
- 不把 PyBullet 变成上游采集后端。
- 不合并或大规模重构三个仓库。
- 不在 P0/P1 修改中游训练、release 或下游 replay 主线。
- 不要求第一阶段完成大规模 Isaac 数据采集。
- 不把安装成功、场景启动或 `/joint_states` 发布描述为完整采集已可用。
- 不虚构尚未运行的 Isaac 接口、帧率、稳定性或 episode 验证结果。

---

## 3. 当前状态与 P0 初步证据

### 3.1 已实现且有代码证据

- MuJoCo 节点已发布 `/sim/encoder_state`、`/sim/object_pose`、`/ee_pose`、`/ft_sensor`，并提供 `/sim/reset_scene`。
- recorder 已消费 `/joint_states`、`/ee_pose`、`/ft_sensor`、`/sim/object_pose` 和相机话题。
- batch generator 已调用 `/sim/reset_scene`、读取对象/末端位姿并调用 recorder 的 `EndEpisode` 服务。
- raw episode 已有 `meta.json`，当前至少包含 `upstream_gate`、`success` 等门禁信息。

### 3.2 已确认的硬编码锚点

下列位置是 P0 的起始清单，不代表全量审计已经完成：

| 文件/位置 | 当前耦合 |
|---|---|
| `src/teleop_bringup/launch/full_system.launch.py:70` | 固定 include `simulation.launch.py` |
| `src/teleop_bringup/launch/full_system.launch.py:141` | 默认 `config/models/franka_panda.xml` |
| `src/teleop_bringup/launch/full_system.launch.py:143` | `headless` 描述直接指向 MuJoCo |
| `src/teleop_bringup/launch/simulation.launch.py:59` | 直接启动 `mujoco_sim/mujoco_sim_node` |
| `src/teleop_bringup/launch/simulation.launch.py:64` | 将 MuJoCo `model_path` 传给节点 |
| `src/synth_data_gen/synth_data_gen/batch_generator.py:208` | 固定向 `/mujoco_sim` 设置 `target_object_name` |
| `src/camera_bridge/camera_bridge/camera_bridge_node.py:18` | 直接导入 MuJoCo 和 `mujoco_sim.virtual_camera` |
| `src/camera_bridge/camera_bridge/camera_bridge_node.py:35` | 默认 MuJoCo XML `model_path` |
| `src/camera_bridge/camera_bridge/object_sync.py:8` | 固定 `MUJOCO_SIM_PARAM_NODE=/mujoco_sim` |
| `src/lerobot_recorder/lerobot_recorder/system_telemetry.py:15` | telemetry 进程映射固定包含 `mujoco_sim` |
| `src/teleop_bringup/package.xml:32` | bringup 对 `mujoco_sim` 有运行依赖 |

### 3.3 本机独立 Isaac 可行性证据

Isaac Sim 6.0 位于仓库外的独立 Python 3.12 venv。P2–P4 已进一步验证项目 adapter、最小 ROS contract 和一个 PoC episode；这仍不代表大规模 Isaac 采集、抓取成功率或生产稳定性已经验证。

本机 RTX PRO 500 Laptop 约 6 GB VRAM，低于兼容性检查器门槛。计划必须允许：

- MuJoCo 继续承担日常开发和完整回归；
- Isaac 本地只运行低画质、单 Panda、单 episode 概念验证；
- Isaac 环境与普通 ROS 2 工作区解耦；
- 重负载验证迁移到远程或更高显存 GPU。

---

## 4. Simulator Backend Contract

### 4.1 稳定命名原则

上层节点依赖 topic/service contract，不依赖 backend 进程名。backend 节点名称可以不同，但 launch 必须保证对外接口稳定。

`batch_generator` 不得通过 `/mujoco_sim/set_parameters` 选择对象。优先采用 backend-neutral service；在该 service 落地前，可用 launch 注入的 `simulator_node_name` 作为兼容过渡，但不得把新的固定 Isaac 节点名写入业务逻辑。

### 4.2 必需接口

| 接口 | 类型 | 方向 | 语义 |
|---|---|---|---|
| `/sim/joint_effort_cmd` | `std_msgs/Float64MultiArray` | runtime → backend | Panda 关节力矩命令，关节顺序必须固定并记录 |
| `/sim/encoder_state` | `sensor_msgs/JointState` | backend → virtual drive/runtime | 仿真编码器状态；name/position/velocity 必须一致 |
| `/sim/object_pose` | `geometry_msgs/PoseStamped` | backend → batch/recorder | 当前目标对象世界位姿，frame_id 必须明确 |
| `/sim/reset_scene` | `std_srvs/Trigger`（P1 兼容） | batch → backend | 将机械臂、对象与随机化状态恢复到可采集起点 |
| `/ee_pose` | `geometry_msgs/PoseStamped` | backend/runtime → recorder | 末端位姿，四元数采用 ROS `xyzw` |
| `/ft_sensor` | `geometry_msgs/WrenchStamped` | backend → controller/recorder | 末端六维力/力矩，frame_id 与单位固定 |
| `/joint_states` | `sensor_msgs/JointState` | ros2_control/runtime → consumers | 上层测得关节状态，不应被 recorder 改写 |
| `/tf`、`/tf_static` | TF2 | runtime/backend → consumers | Panda link tree 与相机/世界坐标关系 |
| `/teleop/record_trigger` | `std_msgs/String` | teleop/batch → recorder | 保持 `start`/`stop` 兼容路径 |
| `/lerobot_recorder/end_episode` | `teleop_interfaces/srv/EndEpisode` | batch → recorder | 明确 commit/discard/stop_success 等终止语义 |

### 4.3 相机接口

至少保持当前 recorder 使用的话题名称和消息类型：

- `/camera/color/image_raw`
- `/camera/depth/image_raw`
- `/camera/color/camera_info`
- wrist camera 对应的现有话题

MuJoCo 可以继续使用现有 `camera_bridge`/renderer。Isaac 应直接发布或通过轻量 adapter 映射为相同 ROS Image/CameraInfo contract。不要为了统一实现而强迫两个 backend 共用 renderer 代码。

### 4.4 backend 能力与可选接口

不同 backend 的能力必须显式声明，不允许静默伪造：

| 能力 | MuJoCo | Isaac P2/P3 预期 | 缺失时行为 |
|---|---|---|---|
| joint state / TF | required | required | 启动失败 |
| object pose / reset | required | P3 required | batch generation 禁用 |
| scene RGB | supported | P3 最小子集 | portfolio capture 禁用或启动失败 |
| wrist RGB/depth | supported/configurable | optional | metadata 标记 unavailable |
| FT | supported | adapter 后验证 | training capture 不得伪造零值 |
| domain randomization | MuJoCo 实现 | 后续逐项映射 | 记录实际启用能力 |
| grasp validation | batch 主轨 + monitor 辅轨 | P4 前验证 | 不宣称成功采集 |

### 4.5 参数与 provenance

P1 引入：

```text
sim_backend:=mujoco|isaac       # 默认 mujoco
```

backend 专属参数应留在各自 launch 中，例如 MuJoCo XML、MuJoCo renderer 和 Isaac USD/extension 配置，不继续膨胀顶层公共参数。

`meta.json` 可向后兼容地增加可选字段：

```json
{
  "simulator_backend": "mujoco",
  "simulator_version": "3.x",
  "scene_id": "panda_tabletop_v1"
}
```

兼容规则：

1. 旧 episode 缺少这些字段仍必须能被现有 adapter/validator 读取。
2. 新字段只能补充 provenance，不改变 `success` 或 `upstream_gate` 语义。
3. 中游不得通过 backend 名称重新推导物理成功与否。
4. P4 已将这些字段作为可选顶层 provenance 写入；旧 episode 缺失字段仍可读取。

---

## 5. 文件变更策略

### 5.1 建议新增

| 文件 | 阶段 | 作用 |
|---|---|---|
| `docs/SIMULATOR_BACKEND_CONTRACT.md` | P0 | 经过审计确认的最终 ROS contract/单位/QoS/frame 规范 |
| `src/teleop_bringup/launch/backends/mujoco.launch.py` | P1 | 从现有 simulation launch 搬出的 MuJoCo 启动逻辑 |
| `src/teleop_bringup/launch/backends/isaac.launch.py` | P2 | Isaac 外部进程/adapter 的启动入口或连接入口 |
| `src/isaac_sim_adapter/`（名称待 P0 确认） | P2 | Isaac topic/service/TF 适配；不包含 Isaac 安装本体 |
| backend contract/launch tests | P0/P1/P2 | 默认行为、错误 backend、接口映射回归 |

### 5.2 建议修改

| 文件 | 最小修改 |
|---|---|
| `src/teleop_bringup/launch/full_system.launch.py` | 声明并透传 `sim_backend`；默认 `mujoco` |
| `src/teleop_bringup/launch/simulation.launch.py` | 仅负责 backend 选择/兼容入口，不直接承载 MuJoCo 细节 |
| `src/synth_data_gen/synth_data_gen/batch_generator.py` | 去掉固定 `/mujoco_sim` 参数服务依赖 |
| `src/teleop_bringup/package.xml` | P2 时按实际 adapter 依赖调整，不提前引入 Isaac Python 依赖 |
| `src/lerobot_recorder/...` | 仅 P4 增加可选 provenance，保持 frame schema |
| `docs/README.md`、`docs/INTER_REPO_CONTRACTS.md` | 登记 SPEC；P4 时补 provenance 契约 |

### 5.3 第一阶段保持不动

- `robot-arm-episode-data-lab` 的 adapter、release、EDA、MLP/ACT 主线。
- `ros2-moveit-pybullet-bridge` 的 PyBullet replay、monitor 和 benchmark。
- raw frame 的 state/action/observation 语义。
- MuJoCo 物理实现、domain randomization、grasp assist 算法本体。
- 现有 camera bridge 的 MuJoCo renderer 实现；P1 只隔离启动边界。

不过度抽象原则：只抽象上层确实跨 backend 使用的 ROS 接口和 launch 选择。MuJoCo XML/API、Isaac USD/OmniGraph、各自 renderer 与随机化实现应保留在 backend 内部，不创建统一 Python simulator 基类。

---

## 6. 分阶段实施计划

### P0：仓库审计与接口契约文档

**行为变化**：无。

任务：

1. 完整审计 launch、节点名、XML/model_path、reset、目标对象参数、pose/joint/FT、camera renderer、domain randomization 和 grasp validation。
2. 记录所有 `mujoco_sim`、MuJoCo XML/API 和 renderer 路径及调用方。
3. 确认 topic 类型、frame_id、单位、QoS、频率和启动依赖。
4. 写出 `SIMULATOR_BACKEND_CONTRACT.md`，区分 required/optional capability。
5. 建立 P1 的 launch 静态测试和回归命令基线。

验收证据：

```bash
rg -n "mujoco_sim|MuJoCo|mujoco|franka_panda.xml|model_path" \
  src docs config tests
colcon test --packages-select teleop_bringup synth_data_gen camera_bridge \
  lerobot_recorder mujoco_sim
colcon test-result --verbose
```

产物：硬编码清单、contract 文档、测试基线日志。P0 结束时仍使用原启动链。

### P1：增加 backend 参数并拆分 MuJoCo launch

**行为变化**：只改变内部 launch 组织；默认运行结果不变。

任务：

1. 增加 `sim_backend`，合法值仅 `mujoco|isaac`，默认 `mujoco`。
2. 将现有 MuJoCo 节点和 MuJoCo camera bridge 配置搬入 `backends/mujoco.launch.py`。
3. 保持 `simulation.launch.py` 为兼容入口。
4. batch generator 改用 backend-neutral 目标选择接口或参数化节点名过渡方案。
5. 未安装 Isaac 时，默认 MuJoCo 路径不得加载 Isaac 包。

验收命令：

```bash
colcon build --symlink-install --packages-select \
  teleop_bringup synth_data_gen camera_bridge lerobot_recorder mujoco_sim
source install/setup.bash
ros2 launch teleop_bringup full_system.launch.py --show-args
timeout 60s ros2 launch teleop_bringup full_system.launch.py \
  sim_backend:=mujoco start_teleop:=false record:=false headless:=true
colcon test --packages-select teleop_bringup synth_data_gen camera_bridge \
  lerobot_recorder mujoco_sim
colcon test-result --verbose
```

可验证证据：默认命令与显式 `sim_backend:=mujoco` 启动相同节点和接口；未知 backend 明确失败；测试全部通过；无后台残留。

### P2：Isaac backend 骨架与接口适配

任务：

1. 新增 Isaac backend launch/adapter，连接仓库外 Isaac 运行环境。
2. 先实现 capability 声明、joint state/TF 映射与清晰的 unavailable 错误。
3. 普通 ROS 2 workspace 不安装 `isaacsim[all]`，不 import Isaac 私有 Python 环境。
4. 支持 Isaac 在本机或远程 GPU 启动，上游只通过 ROS 2 contract 连接。

验收：launch 可选择 `isaac`；未启动 Isaac 时快速、可诊断地失败；MuJoCo 回归仍通过。本阶段不验收完整 episode。

### P3：Isaac Panda 场景最小接口

任务：在单 Panda、轻量场景中验证以下最小子集：

- joint state；
- Panda TF tree；
- target object pose；
- deterministic reset；
- scene 或 wrist camera 至少一个。

验收示例：

```bash
ros2 topic list
timeout 20s ros2 topic echo /joint_states --once
timeout 20s ros2 topic echo /sim/object_pose --once
timeout 20s ros2 service call /sim/reset_scene std_srvs/srv/Trigger "{}"
timeout 20s ros2 run tf2_ros tf2_echo panda_link0 panda_ee
```

证据必须包含实际启动日志、topic/service 输出，以及 reset 前后可解释的状态变化。headless 验证可用有界运行日志代替截图。

### P4：一个 Isaac raw episode

任务：

1. 使用现有 recorder 完成一个 Isaac episode。
2. 保持 required frame fields、action 语义和 upstream gate。
3. 增加并测试可选 provenance 字段。
4. 用上游 validator 和中游现有 adapter/schema 验证。

验收证据：一个可追溯的 `episode_*`、`meta.json`、视频/帧计数一致，以及中游 adapter/schema PASS。raw contract 与训练 schema 维度不同，必须先适配，不能把 raw 目录直接按训练 schema 的预期失败算作回归。

### P5：MuJoCo 与 Isaac Sim2Sim 分布对比

使用同一任务配置、种子策略和 schema，对比：

- joint/EE trajectory；
- object pose 与成功门禁；
- force/torque；
- camera observation；
- tracking error、缺失率、时序和 episode-level 分布。

结果必须标记为 Sim2Sim evidence，不外推为真实机械臂成功率或已完成 Sim2Real。

---

## 7. 回归与兼容性风险

| 风险 | 影响 | 控制措施 |
|---|---|---|
| launch 参数重命名 | 现有脚本无法启动 | P1 保留旧参数和默认值，增加 launch tests |
| backend 节点名泄漏 | batch 只能控制某一 simulator | 业务逻辑只依赖 contract；节点名仅作短期参数化过渡 |
| joint 顺序/单位不同 | 控制错误或数据污染 | contract 固定 name/order/unit，并在 adapter 边界断言 |
| TF/frame 不一致 | EE/object pose 无法比较 | 明确 world/base/camera frame 与四元数顺序 |
| reset 语义不一致 | episode 不可复现 | 定义 reset 后状态、settle 和首次发布要求 |
| Isaac FT/相机缺失时填零 | 训练数据静默污染 | capability fail-fast 或 metadata 标记 unavailable |
| provenance 字段变 required | 老 episode 无法读取 | 新字段保持 optional，增加 legacy fixture 测试 |
| Isaac 环境污染 ROS workspace | Python/extension 冲突 | Isaac 独立 venv/容器/远程进程，ROS contract 解耦 |
| 6 GB VRAM 性能不足 | OOM 或帧率过低 | 本地轻量 PoC；大场景转远程 GPU；MuJoCo 承担回归 |
| 为统一而重写 camera/randomizer | 大范围回归 | backend 内保留原生实现，只统一输出 contract |

---

## 8. 第一阶段最小 PR 的准确范围

第一阶段 PR 只包含 P0/P1：

### 必须包含

- 本 SPEC 与最终 backend contract 文档。
- `full_system.launch.py` 的 `sim_backend` 参数，默认 `mujoco`。
- MuJoCo backend launch 拆分和 `simulation.launch.py` 兼容入口。
- batch generator 去除固定 `/mujoco_sim` 依赖。
- 默认、显式 MuJoCo、非法 backend 的 launch tests。
- 当前相关 MuJoCo、camera、batch、recorder 测试通过的日志。

### 明确不包含

- Isaac 完整 adapter 或大场景。
- recorder frame schema 改造。
- 中游训练/release 修改。
- 下游 PyBullet replay 修改。
- Isaac 大规模采集或性能结论。
- P2-P5 的未验证完成声明。

### 合并闸门

只有同时满足以下条件才能合并：

1. 不传 `sim_backend` 时仍走 MuJoCo；
2. 现有 MuJoCo 启动命令可用；
3. 现有相关测试全部通过；
4. batch generator 业务代码不再出现固定 `/mujoco_sim`；
5. 三仓代码边界没有变化；
6. 测试结束后无 ROS 2/MuJoCo/recorder 后台进程残留。

---

## 9. 决策点

开始 P1 前，P0 必须给出并确认以下决策：

1. 目标对象切换采用新 service，还是短期使用 `simulator_node_name` 参数过渡；
2. `/sim/object_pose` 的 canonical frame；
3. Isaac 是否直接发布 camera topics，还是通过独立 image adapter；
4. FT 在 P3/P4 是 required 还是 capability-gated optional；
5. Isaac 由本地 launch 拉起，还是默认连接仓库外已运行实例。

在这些决策确认前，不开始 P1 之外的 Isaac adapter 实现。

---

## 10. P0/P1 实施记录（2026-07-17）

已完成：

- P0 硬编码审计与 `SIMULATOR_BACKEND_CONTRACT.md`；
- `full_system.launch.py` 增加 `sim_backend`，默认 `mujoco`；
- 原 MuJoCo 启动逻辑迁移到 `launch/backends/mujoco.launch.py`；
- `simulation.launch.py` 改为 backend selector；
- batch generator 通过 `simulator_node_name` 参数寻址 simulator control node；
- Isaac 与非法 backend 均 fail-fast；
- 中游、下游、recorder schema 和 MuJoCo 实现未修改。

实际证据：

- 相关新增/修改单测：16 passed；
- 仓库 pytest：63 个普通测试通过，宿主 DDS 权限下 M1 launch test 另行通过（合计 64）；
- 5 个相关包构建成功；
- 默认命令和显式 `sim_backend:=mujoco` 均实际加载 MuJoCo XML 并启动节点；
- `sim_backend:=isaac` 明确返回 P2 尚未实现；`pybullet` 被 choices 拒绝。

已知非功能阻塞：`teleop_bringup` 的 ament lint 在 P1 前已有全包 quote-style、copyright 和 docstring 基线失败；P1 没有扩大范围去机械格式化全部历史 launch。功能测试与 ROS runtime 验证通过，但在清理该历史 lint 基线前，不能声称 `colcon test-result` 全绿。

---

## 11. P2–P4 实施与实测记录（2026-07-17）

### 11.1 已实现

- `src/isaac_sim_adapter/`：普通 ROS 2 环境中的轻量 adapter，不 import Isaac Python。
- `scripts/isaac_panda_backend.py`：由隔离的 Isaac Sim 6.0 venv 运行单 Panda、单红方块和 320×240 scene RGB。
- `launch/backends/isaac.launch.py`：只启动 adapter，并在 raw joint stream 缺失时 fail-fast。
- raw `/isaac/*` 映射为 `/sim/encoder_state`、`/sim/object_pose`、`/sim/reset_scene`、`/ee_pose`、`/ft_sensor`、scene camera 与归一化 gripper state。
- Isaac joint state 经过现有 simulated hardware/ros2_control 形成 `/joint_states`，TF 仍由现有 `robot_state_publisher` 产生。
- recorder 可选 provenance：`simulator_backend`、`simulator_version`、`scene_id`。

### 11.2 P3 实测证据

- GPU：RTX PRO 500 Blackwell Laptop，6113 MiB；驱动 580.159.03。
- Isaac：`isaacsim 6.0.0.0`，headless 场景完成 38,553 物理帧/150 秒，无 CUDA OOM。
- raw joint 包含 7 arm + 2 finger joints；canonical `/sim/encoder_state` 固定为 7 arm joints。
- `/sim/object_pose` 实测约 `[0.4500, 0, 0.0400]`；`/sim/reset_scene` 返回 `success=True`。
- scene image 实测 320×240；backend diagnostics 显示 joint/object/camera active。
- `/joint_states` 实测 7 joints；`panda_link0 -> panda_hand` TF 连续可解。

### 11.3 P4 实测证据

- 输出：`/tmp/isaac_p4_episode`（临时验证产物，不提交仓库）。
- recorder：25 帧、5 秒、约 4.804 Hz，LeRobot v2.1 parquet + scene MP4。
- ffprobe：320×240、5 fps、25 video packets，与 parquet/meta frame count 一致。
- `meta.json`：`simulator_backend=isaac`、`simulator_version=6.0.0.0`、`scene_id=p3_single_panda_red_box_v1`。
- FT 不是零占位：使用 Isaac `panda_hand` incoming-joint local reaction wrench；首帧示例 force 约 `[4.03, -2.77, 3.03]` N。
- raw 维度保持现有契约：`observation.state[7]`、`action[8]`。
- 中游现有 adapter 使用 `--derive-ee-delta-action` 后得到 `state[8]`、`action[7]`；`inspect_dataset.py` 对 1 episode / 25 frames 返回 `Status: PASS`。

### 11.4 未实现/未验证

- `/sim/joint_effort_cmd` 尚未驱动 Isaac articulation，因此不能用现有控制链执行 Isaac 抓取任务。
- wrist RGB/depth、camera info、Isaac domain randomization、grasp validation 和 batch generation 未完成。
- P4 是固定 PoC action 的 recorder/schema 验证，不是任务成功率或物理抓取成功证据。
- Isaac 闭环任务执行尚未实现，因此 P5 只能比较 matched recorder 输入下的观测分布，不能比较两后端抓取成功率。

### 11.5 P5 实测证据（EVIDENCE_ONLY）

- 使用相同 recorder 配置采集 MuJoCo 与 Isaac 各 1 个 raw episode：25 帧、5 Hz、320×240 scene RGB，并对两侧发布完全相同的固定 `action[8]`。
- 两侧 existing adapter/schema 均通过：各 1 episode / 25 frames；raw action 逐维一致，trajectory L2 RMSE 为 0。
- 观测并未物理对齐：joint state trajectory L2 RMSE 约 1.551，EE pose 约 0.405，object pose 约 0.123；object position 均值偏移约 `[0.100, 0.070, 0.015]` m。
- 相机抽样的平均亮度约为 MuJoCo 0.214、Isaac 0.419；这反映当前视角、灯光和材质差异，不是视觉质量排名。
- MuJoCo 与 Isaac 的 FT 发布 frame/语义仍不同，FT 数值仅作诊断，不可设置阈值门禁。
- 详细可复现报告与工具位于中游仓库 `robot-arm-episode-data-lab`：`docs/portfolio/SIM2SIM_ISAAC_P5_EVIDENCE.md` 与 `training/scripts/compare_sim_backends.py`。
- 结论状态固定为 `EVIDENCE_ONLY`：没有建立 calibrated gate，不外推抓取成功率、Sim2Real 或真实机械臂表现。

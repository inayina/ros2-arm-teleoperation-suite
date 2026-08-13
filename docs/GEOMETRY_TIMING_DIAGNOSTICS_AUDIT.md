# 仿真几何、相机外参与时序诊断审计基线

| 元数据 | 值 |
|---|---|
| 状态 | Audit baseline + Stage 1 implemented (REPORT_ONLY) |
| 审计日期 | 2026-08-13 |
| 适用仓库 | `ros2-arm-teleoperation-suite` |
| 审计快照 | `f3a760774d02aabf6a6bdd2993a53e1738b867b5` |
| 审计阶段 | 只读审计已落盘；Stage 1 FK/TF/SIM_GT comparator 已实现；未进入 Stage 2+ |
| Stage 1 入口 | [SIMULATION_GEOMETRY_STAGE1_REPORT.md](./SIMULATION_GEOMETRY_STAGE1_REPORT.md) / `teleop_diagnostics` |
| 运行边界 | Stage 1 证据为离线 CLI；未常驻 ROS；无 fault injection / camera / timestamp |
| 物理证据 | `PHYSICAL=NOT_RUN/UNAVAILABLE` |

> 本文固化 2026-08-13 对 TF/frame、URDF/MuJoCo/FK、joint zero/kinematic
> offset、camera extrinsic/hand-eye、timestamp skew 和 diagnostics 的代码与测试审计。
> 它是一个带 Git 快照的审计基线，不是运行验收报告，也不表示后续计划已经实现。

---

## 0. 结论与证据口径

### 0.1 直接结论

当前仓库已经具备仿真执行、Robot State Publisher TF、MuJoCo 末端位姿、相机图像和
recorder 同步链路；但尚未形成把这些来源交叉验证的几何与时序诊断闭环。

当前能够确认的是“各子系统分别存在并有局部测试”，不能确认以下更强主张：

- TF、控制器解析 FK、URDF/KDL FK 与 MuJoCo EE ground truth 在同一关节状态下一致；
- Image Header 中的 camera optical frame 在 TF 树中存在；
- renderer、CameraInfo、TF 与 camera extrinsic 使用同一权威合同；
- recorder 行内数据来自同一物理/仿真 acquisition time；
- 已完成 joint-zero、hand-eye 或真实相机标定；
- 仿真 residual 可以外推为真实机械臂、Sim2Real 或任务成功证据。

### 0.2 证据分类

| 分类 | 本文语义 |
|---|---|
| `MODEL` | URDF、KDL、解析 FK、fallback FK 或配置推导结果 |
| `SIM_GT` | 真 MuJoCo backend 的 `site_xpos/site_xmat` 等仿真内部状态 |
| `INJECTED_FAULT` | 未来仅在诊断副本中施加的已知偏置 |
| `ESTIMATED` | 未来 solver/estimator 输出；不等于 ground truth |
| `PHYSICAL` | 真实传感器/机械臂证据；本次为 `NOT_RUN/UNAVAILABLE` |

输入状态必须单独记录为：

```text
AVAILABLE | STALE | MISSING | INVALID | UNAVAILABLE
```

初版诊断若没有冻结阈值，应输出 `REPORT_ONLY` 或 `INSUFFICIENT_DATA`，不得用没有依据的
阈值生成 `PASS`。输入 unavailable、stale、backend unknown 或 provenance 不明时不得
fail-open。

---

## A. Current State

| 能力 | 当前状态 | 当前证据是否充分 |
|---|---|---|
| `world → panda_link0 → ... → panda_ee` TF | 已实现 | TF 结构充分；跨模型数值一致性不足 |
| MuJoCo `/ee_pose` | 已实现 | 真 MuJoCo 时为 `SIM_GT`；fallback 时仅为 `MODEL` |
| URDF/MuJoCo nominal geometry | 源文件看起来对齐 | 缺少独立、自动化、多姿态一致性测试 |
| 控制器解析 FK/Jacobian | 已实现 | 与当前 `panda_ee` 参考点存在合同差异 |
| scene/wrist camera rendering | 已实现 | camera frame 只在 Header 字符串中，未进入 TF 树 |
| CameraInfo | 已实现 | 没有证明与 XML camera/TF 完全一致 |
| recorder 时间同步 | latest-sample + slop 已实现 | 没有 signed skew 或 source-time 证据 |
| 几何/标定 diagnostics | 未实现 | 当前项目证据不足，无法确认 |
| joint-zero/model/TCP fault injection | 未实现 | 当前项目证据不足，无法确认 |
| 真实相机/机械臂标定 | 未实现、未运行 | `PHYSICAL=NOT_RUN/UNAVAILABLE` |

仓库责任保持不变：该诊断属于上游执行与采集链；不把 schema/release/training 实现复制到
中游，也不把 replay/risk ownership 移入本仓。

---

## B. Evidence

### B.1 TF 与 frame authority

当前模型主链为：

```text
world
└── panda_link0
    └── panda_link1 ... panda_link7
        └── panda_hand
            └── panda_ee
```

代码证据：

- `world → panda_link0` 是单位固定变换：
  [`src/teleop_description/urdf/panda.urdf.xacro`](../src/teleop_description/urdf/panda.urdf.xacro#L39-L46)。
- 7 个 revolute joint 的 origin、axis 与 limits：
  [`panda.urdf.xacro`](../src/teleop_description/urdf/panda.urdf.xacro#L54-L122)。
- `panda_link7 → panda_hand`：`xyz="0 0 0.107"`、yaw `-π/4`：
  [`panda.urdf.xacro`](../src/teleop_description/urdf/panda.urdf.xacro#L124-L130)。
- `panda_hand → panda_ee`：z `0.10 m`：
  [`panda.urdf.xacro`](../src/teleop_description/urdf/panda.urdf.xacro#L131-L136)。
- `robot_state_publisher` 展开该 xacro：
  [`src/teleop_description/launch/description.launch.py`](../src/teleop_description/launch/description.launch.py#L14-L36)。
- `/joint_states` 来自 `joint_state_broadcaster`：
  [`src/teleop_bringup/config/controllers.yaml`](../src/teleop_bringup/config/controllers.yaml#L1-L14)。

当前仓库没有名为 `base` 或 `base_link` 的 canonical Panda base；实际名称是
`panda_link0`，末端名称是 `panda_ee`。

固定 joints 由 Robot State Publisher 发布到 `/tf_static`；revolute joints 根据
`/joint_states` 发布到 `/tf`。MuJoCo 节点本身没有 `TransformBroadcaster`。

当前 [`docs/ARCHITECTURE_V2.md`](./ARCHITECTURE_V2.md#L82-L92) 内部存在一处冲突：
节点图暗示 MuJoCo 发布 `/tf`，而当前代码不支持该说法；同一文档的
[publisher 表](./ARCHITECTURE_V2.md#L228-L239) 又把 `/tf` 正确归给
`robot_state_publisher`。代码事实优先。

Camera frame 状态：

- `scene_camera_optical_frame`、`wrist_camera_optical_frame` 和 tactile optical frame
  作为 launch 参数及 Image/CameraInfo `frame_id` 存在；
- URDF 中没有对应 link/joint；
- 相关包中没有 static/dynamic camera TF broadcaster；
- object pose 通过 `/sim/object_pose` 的 `PoseStamped` 表达，也没有 object TF。

因此当前 TF 树不能回答 `panda_link0 ↔ camera optical frame` 或
`panda_ee ↔ camera optical frame` 的查询。

### B.2 URDF、MuJoCo 与 FK

#### B.2.1 URDF 与 MuJoCo nominal chain

MuJoCo XML 的 Panda body/joint chain 与 URDF nominal geometry 基本对应：

- Panda joint/body chain：
  [`config/models/franka_panda.xml`](../config/models/franka_panda.xml#L137-L205)。
- hand 固定变换：
  [`franka_panda.xml`](../config/models/franka_panda.xml#L205-L212)。
- `panda_ee` site：
  [`franka_panda.xml`](../config/models/franka_panda.xml#L239)。

MuJoCo 节点根据 `panda_ee` site 的 `site_xpos/site_xmat` 发布 `/ee_pose`：

- 参数与 site 名称：
  [`src/mujoco_sim/mujoco_sim/mujoco_sim_node.py`](../src/mujoco_sim/mujoco_sim/mujoco_sim_node.py#L135-L145)。
- site lookup：
  [`mujoco_sim_node.py`](../src/mujoco_sim/mujoco_sim/mujoco_sim_node.py#L435-L438)。
- `/ee_pose` publish：
  [`mujoco_sim_node.py`](../src/mujoco_sim/mujoco_sim/mujoco_sim_node.py#L568-L605)。

真实 MuJoCo path 中，该值是 `SIM_GT`。但 MuJoCo 不可用时，同一话题会由手写
`fallback_ee_transform()` 生成：

- fallback FK：
  [`mujoco_sim_node.py`](../src/mujoco_sim/mujoco_sim/mujoco_sim_node.py#L50-L60)。
- fallback backend 调用：
  [`mujoco_sim_node.py`](../src/mujoco_sim/mujoco_sim/mujoco_sim_node.py#L123-L129)。

因此 `/ee_pose` 的话题名和 message type 不足以证明 evidence class。消费者必须知道当前
backend provenance；否则可能把 `MODEL` 错标成 `SIM_GT`。

另一个潜在合同风险是：`site_xpos` 是 MuJoCo world coordinates，而消息 Header 当前写
`panda_link0`。由于现有 `world → panda_link0` 是 identity，数值当前可对齐；一旦 base
不再位于 world 原点，该隐含前提就会失效。

#### B.2.2 只读数值 probe

审计期间对现有模型做了未落盘、非自动化的只读 probe：

| q | MuJoCo `panda_ee` | fallback FK | residual |
|---|---|---|---|
| 全零 | `[0.088, 0.000, 0.826]` | `[0.088, 0.000, 0.826]` | 约 `1.3e-16 m` |
| ready | `[0.30701957, 0.000, 0.49026956]` | `[0.30701957, 0.000, 0.49026956]` | 约 `1.8e-16 m` |

这只能说明当前 MuJoCo XML 与重复相同常数的 fallback FK 在两个样本上吻合。它不是独立
URDF/KDL/MoveIt 验证，也没有形成 repository test，不能升级为 cross-model gate。

#### B.2.3 控制器解析 FK 参考点差异

阻抗控制器维护另一套解析 DH/FK/Jacobian：

- DH 参数：
  [`src/teleop_controllers/src/impedance_math.cpp`](../src/teleop_controllers/src/impedance_math.cpp#L34-L43)。
- FK：
  [`impedance_math.cpp`](../src/teleop_controllers/src/impedance_math.cpp#L69-L80)。
- 控制循环使用该 FK/Jacobian：
  [`src/teleop_controllers/src/cartesian_impedance_controller.cpp`](../src/teleop_controllers/src/cartesian_impedance_controller.cpp#L270-L279)。
- 单元测试将零位 z 锁为 `1.033 m`：
  [`src/teleop_controllers/test/test_impedance_math.cpp`](../src/teleop_controllers/test/test_impedance_math.cpp#L42-L52)。

当前 URDF/MuJoCo `panda_ee` 零位 z 为 `0.826 m`；控制器解析 FK 为 `1.033 m`，相差
`0.207 m`。解析 FK 只累计 7 个 DH transform，没有显式应用
`panda_link7 → panda_hand → panda_ee` 两段 fixed transform。

这是一个直接的 reference-frame/model-contract mismatch 证据，但不是“最终 TCP 必然偏差
0.207 m”的证据：控制器 current/desired 均使用同一内部 FK。实现前必须先确认控制目标
应当是 `panda_link7`、`panda_hand` 还是 `panda_ee`，再决定修正 FK 还是更正命名和合同。

MoveIt Servo 则明确使用 `panda_ee`：
[`src/teleop_moveit_config/config/servo.yaml`](../src/teleop_moveit_config/config/servo.yaml#L28-L34)。

#### B.2.4 joint zero / model offset 当前状态

在 `src/`、`config/`、`tests/` 和当前文档中检索 joint-zero、encoder offset、joint offset、
extrinsic calibration 与 hand-eye，未找到以下直接实现证据：

- joint zero offset 的显式配置和 provenance；
- joint origin/link geometry/TCP offset 的受控注入；
- 基于多姿态 residual 的 offset estimator；
- 可辨识性、conditioning 或 ambiguity 报告。

因此当前结论是：**当前项目证据不足，无法确认已实现 joint-zero/model-offset 诊断或标定。**

### B.3 Camera extrinsic / hand-eye

MuJoCo XML 中存在两类相机：

- scene camera：`worldbody` 直接子项，world-fixed / eye-to-hand：
  [`franka_panda.xml`](../config/models/franka_panda.xml#L121)。
- wrist camera：位于 hand body 下，eye-in-hand：
  [`franka_panda.xml`](../config/models/franka_panda.xml#L239)。

默认 full stack 启用 scene camera、关闭 wrist camera；对应 launch 位于：
[`src/teleop_bringup/launch/backends/mujoco.launch.py`](../src/teleop_bringup/launch/backends/mujoco.launch.py#L44-L54)。

Camera bridge 的关键行为：

- 独立加载一份 MuJoCo model：
  [`src/camera_bridge/camera_bridge/camera_bridge_node.py`](../src/camera_bridge/camera_bridge/camera_bridge_node.py#L120-L153)。
- 订阅 joint/gripper/object/EE：
  [`camera_bridge_node.py`](../src/camera_bridge/camera_bridge/camera_bridge_node.py#L96-L103)。
- callbacks 只保留最新值，不保留输入 Header stamp：
  [`camera_bridge_node.py`](../src/camera_bridge/camera_bridge/camera_bridge_node.py#L221-L255)。
- render 前把最新缓存写入独立 model：
  [`camera_bridge_node.py`](../src/camera_bridge/camera_bridge/camera_bridge_node.py#L257-L276)。
- Image Header 只写配置的 `frame_id` 字符串：
  [`camera_bridge_node.py`](../src/camera_bridge/camera_bridge/camera_bridge_node.py#L497-L506)。

CameraInfo 的 K/P 根据独立 `fovy_deg` 和 image size 计算：

- CameraInfo publish：
  [`camera_bridge_node.py`](../src/camera_bridge/camera_bridge/camera_bridge_node.py#L278-L293)。
- intrinsics 计算：
  [`src/mujoco_sim/mujoco_sim/virtual_camera.py`](../src/mujoco_sim/mujoco_sim/virtual_camera.py#L11-L31)。

当前没有单一 SE(3) authority 同时约束 XML renderer pose、TF、Image Header 和 recorder；
也没有显式测试 MuJoCo camera axes 到 ROS optical convention 的转换。

Domain randomization 还存在实际链路断点：

- 主 simulator 的 `DomainRandomizer` 会修改其 model camera pose：
  [`src/mujoco_sim/mujoco_sim/domain_randomizer.py`](../src/mujoco_sim/mujoco_sim/domain_randomizer.py#L155-L216)。
- 默认 camera noise 位于：
  [`config/randomization.yaml`](../config/randomization.yaml#L1-L12)。
- backend launch 只把 randomization 参数传给 `mujoco_sim`，未传给独立 camera bridge：
  [`mujoco.launch.py`](../src/teleop_bringup/launch/backends/mujoco.launch.py#L66-L85)。

实际图像由 camera bridge 的独立 model 渲染，因此当前配置的 camera pose randomization
不会作用到实际图像流。Standalone randomizer test 不能证明 rendered image 已使用该扰动。

当前能够确认仿真 XML 里存在 eye-to-hand/eye-in-hand 相机；不能确认已完成仿真
hand-eye consistency validation，更不能确认真实相机标定。

### B.4 Timestamp 与多模态同步

当前项目没有 `/clock` 或 `use_sim_time` 的统一仿真时间合同。MuJoCo 使用 ROS timer 驱动
固定 physics step，但消息 Header 使用各 publisher 的 ROS `now()`。

| 数据 | 当前 timestamp 语义 | 可比较性 |
|---|---|---|
| `/ft_sensor`、`/ee_pose`、`/sim/object_pose` | 同一 MuJoCo publish tick | 三者共享 publish event |
| `/sim/encoder_state` | MuJoCo node publish time | 原 stamp 随后未被 `/joint_states` 保留 |
| `/joint_states` | controller/joint-state broadcaster 时间 | 只能视作下游 publication time |
| RGB/depth/CameraInfo | camera bridge 自己的 tick | 同一 camera node 内共享 stamp |
| `/teleop/cmd_pose` | command publish event | 不是 actuator application time |
| gripper command | `Float64` 无 Header | 无 source stamp |
| Task GT | batch generator 观察/发布时间 | 不是 source sensor acquisition time |
| recorder frame `timestamp` | scene color stamp，缺失时退到 joint stamp | 单一行时间，不保留全部 source stamps |

Camera bridge 在同步 model 和 render 前取得自己的时间戳：
[`camera_bridge_node.py`](../src/camera_bridge/camera_bridge/camera_bridge_node.py#L295-L336)。
它渲染的是若干 callbacks 的 latest cache，因此 joint、gripper、object 可能来自不同消息时刻。

Recorder `MultiModalSync` 是 camera-driven latest-sample 策略：

- 数据结构和同步策略：
  [`src/lerobot_recorder/lerobot_recorder/time_sync.py`](../src/lerobot_recorder/lerobot_recorder/time_sync.py#L8-L15)。
- subscriptions/cache：
  [`time_sync.py`](../src/lerobot_recorder/lerobot_recorder/time_sync.py#L41-L85)。
- required modalities：
  [`time_sync.py`](../src/lerobot_recorder/lerobot_recorder/time_sync.py#L87-L110)。
- age/reject diagnostics：
  [`time_sync.py`](../src/lerobot_recorder/lerobot_recorder/time_sync.py#L132-L144)。
- absolute slop rejection：
  [`time_sync.py`](../src/lerobot_recorder/lerobot_recorder/time_sync.py#L149-L158)。

当前同步器没有 signed skew、nearest-neighbour queue、插值/外推、每帧原始 source stamp、
physics step/sample ID，也没有把 skew 与 joint/EE 速度关联。

现有 ROS Header 在同一主机、同一 ROS time source 下可以计算 publication-time delta；但消息
的采样语义不同，不能把该 delta 直接解释为真实 acquisition-time skew。高速运动时 latest
cache 会把时间错位转化为空间错位，当前尚无量化证据。

### B.5 已有 diagnostics 与证据资产

已有 ROS diagnostics 主要覆盖 safety/watchdog/limits：
[`src/safety_monitor/src/safety_monitor_node.cpp`](../src/safety_monitor/src/safety_monitor_node.cpp#L354-L454)。

Recorder diagnostics 主要输出 modality age 和 reject count：
[`src/lerobot_recorder/lerobot_recorder/recorder_node.py`](../src/lerobot_recorder/lerobot_recorder/recorder_node.py#L413-L456)。

当前没有：

- `geometry_diagnostics.json/csv`；
- TF completeness/authority/staleness report；
- cross-model FK residual；
- joint-zero/TCP/model-bias injection report；
- camera extrinsic perturbation report；
- timestamp signed-skew CSV。

[`docs/portfolio/EVIDENCE_INDEX.md`](./portfolio/EVIDENCE_INDEX.md) 中的 camera/synchronization
截图可以证明相应字段或画面存在，不能证明 raw timestamp skew、外参标定或 cross-model
geometry consistency。

### B.6 2026-08-13 定向测试记录

Python 定向测试命令：

```bash
source install/setup.bash
pytest -q \
  tests/test_mujoco_sim_fallback_fk.py \
  tests/test_camera_bridge_object_mapping.py \
  tests/test_scene_layout.py \
  tests/test_domain_randomizer.py \
  tests/test_time_sync_scene_only.py \
  tests/test_lerobot_recorder.py
```

结果：`28 passed, 1 failed in 0.83s`。

失败项：

```text
tests/test_lerobot_recorder.py::TestLeRobotRecorder::
test_frame_separates_gripper_observation_and_command
```

失败原因：测试通过 `RecorderNode.__new__()` 手工构造实例，但 fixture 没有初始化
`_task_phase_valid`；当前 `_on_frame()` 读取该字段时抛出 `AttributeError`。这是当前测试基线
缺陷，不是 geometry/timestamp gate 失败；本次审计未获授权修复。

控制器 GTest 命令：

```bash
build/teleop_controllers/test_impedance_math
build/teleop_controllers/test_impedance_controller
```

结果：

- `test_impedance_math`：7/7 passed；
- `test_impedance_controller`：10/10 passed。

这些 GTest 证明控制器内部 FK/Jacobian/control-law 的自洽性，不能证明其 reference frame 与
URDF `panda_ee` 或 MuJoCo site 一致。

---

## C. Gaps

### P0 — 建立可信观察边界

1. **独立 cross-model FK/TF/SIM_GT 比较**
   - 使用 URDF/KDL 或 MoveIt RobotModel 计算独立 FK；
   - 同 q 比较 URDF FK、实时 TF、MuJoCo site GT；
   - fallback `/ee_pose` 不得标成 `SIM_GT`。

2. **冻结阻抗控制器的末端参考 frame 合同**
   - 明确控制目标是 link7、hand 还是 `panda_ee`；
   - 不允许在合同未确认前直接增加 `0.207 m` 补偿。

3. **建立 camera frame/extrinsic authority**
   - Image Header 声明的 optical frame 必须能在 TF 中解析；
   - renderer、TF、Header 和 CameraInfo 必须来自同一 nominal/effective contract。

4. **Provenance fail-closed**
   - backend unknown、input missing/stale/invalid 时输出 `INSUFFICIENT_DATA` 或
     `ERROR_INPUT`；
   - `PHYSICAL` 始终显式为 `NOT_RUN/UNAVAILABLE`，直到真实证据存在。

### P1 — 诊断能力

1. joint zero、joint origin/link geometry 与 TCP offset 的受控注入；
2. camera extrinsic translation/rotation perturbation，并作用到实际 renderer；
3. timestamp signed skew、out-of-order、stale 和 controlled delay；
4. 修复 recorder 当前测试 fixture，建立干净的回归基线。

### P2 — 有证据后再扩展

1. 由 nominal distribution 与失败成本支持的 PASS/WARN/ERROR thresholds；
2. joint-offset 或 hand-eye solver；
3. residual trend/dashboard；
4. 真实相机、标定板、真机关节零位与物理验证。

P2 solver 当前不应直接实现：尚无具备可辨识性的多姿态观测合同、标定目标检测、噪声模型
和 physical dataset。能优化出数值不等于完成标定。

---

## D. JD Mapping

| 机器人系统软件能力 | 当前项目直接证据 | 补齐后的可审计证据 |
|---|---|---|
| ROS 2 TF/frame 调试 | RSP、URDF、实时 TF 已实现 | TF completeness、authority、stale/invalid report |
| 运动学/模型一致性 | URDF、MuJoCo、解析 FK 三套来源 | 同 q 多姿态 residual 与 frame contract |
| 控制链问题定位 | 控制器内部 FK/Jacobian 已测试 | link7/hand/TCP 偏差的证据化归因 |
| Camera extrinsic/hand-eye | XML 中有 scene/wrist camera | renderer–TF–Header–CameraInfo 一致性与扰动证据 |
| 多模态时间同步 | recorder slop gate | signed skew、delay injection、运动敏感性 |
| 故障注入 | safety fault path 已存在 | joint/TCP/extrinsic/time fault matrix |
| Evidence engineering | 已有 evidence index | JSON/CSV manifest、evidence class 与 unavailable 状态 |

该方向可支持“机器人执行链几何与时序诊断能力”，不能支持“真实标定完成”“真机部署”或
“功能安全认证”。

---

## E. Proposed Architecture

建议新增一个窄职责 `teleop_diagnostics` 包：

```text
teleop_description
  └─ nominal model + camera extrinsic contract
          │
          ├─ robot_state_publisher → MODEL TF
          ├─ camera_bridge → effective renderer extrinsic + camera TF
          └─ mujoco_sim → SIM_GT EE + backend provenance
                                │
teleop_diagnostics              │
  ├─ independent URDF/KDL FK ───┤
  ├─ TF/model/GT comparator
  ├─ diagnostic-only fault injector
  ├─ timestamp skew tracker
  └─ JSON/CSV evidence writer
```

新增包的理由是该观察者横跨 TF、JointState、MuJoCo GT、CameraInfo 与 timestamps；把它塞入
`teleop_description` 会污染 resource ownership，塞入 `camera_bridge` 会扩大 renderer 职责，
塞入 `safety_monitor` 又会把非安全级推断混进 safety path。

约束：

- fault injection 只作用于诊断副本，不发布被污染的 `/joint_states` 或 control command；
- 初版只支持 kinematic 参数，不把 inertial/dynamics 偏差混入 FK 诊断；
- `SUSPECTED` 只表达 residual pattern，不宣称唯一根因；
- 时间诊断首先标为 `PUBLISH_TIME_SKEW`；建立统一 acquisition/physics time 后才允许
  `SOURCE_TIME_SKEW`；
- diagnostics 不拥有控制权、safety latch 或 recorder commit/discard 决策。

---

## F. File Plan

以下只是建议清单，本文落盘时均未实施。

### 建议新增

```text
src/teleop_diagnostics/package.xml
src/teleop_diagnostics/CMakeLists.txt
src/teleop_diagnostics/include/teleop_diagnostics/model_consistency.hpp
src/teleop_diagnostics/src/model_consistency.cpp
src/teleop_diagnostics/src/geometry_diagnostics_node.cpp
src/teleop_diagnostics/config/diagnostics.yaml
src/teleop_diagnostics/launch/geometry_diagnostics.launch.py
src/teleop_description/config/camera_extrinsics.yaml
tests/test_cross_model_fk_consistency.py
tests/test_geometry_fault_injection.py
tests/test_camera_extrinsic_contract.py
tests/test_timestamp_skew_diagnostics.py
docs/SIMULATION_GEOMETRY_TIMING_DIAGNOSTICS.md
```

### 建议修改

| 文件 | 计划用途 | 限制 |
|---|---|---|
| `mujoco_sim_node.py` | 输出 backend/provenance；区分 `SIM_GT` 与 fallback `MODEL` | 不改变默认控制行为 |
| `camera_bridge_node.py` | 消费统一 extrinsic、发布 effective camera TF、保留 source stamps | 不实现 calibration solver |
| `domain_randomizer.py` | 将 camera perturbation 作用到实际 renderer | 必须报告 effective transform |
| `full_system.launch.py` | 增加默认关闭的 diagnostics 开关 | 默认 launch 行为保持 |
| `impedance_math.cpp` | 合同确认后对齐或重命名 FK reference | 不先猜测后修改 |
| `time_sync.py` | 暴露 signed deltas | 首版不改变 recorder gate |

不修改中游 release/schema/training 和下游 replay/risk 实现。

---

## G. Test Plan

### G.1 Nominal geometry

- q=zero、ready、关节限位附近、固定 seed 随机有效姿态；
- URDF/KDL FK vs Robot State Publisher TF vs MuJoCo `panda_ee`；
- translation residual 与 rotation geodesic residual 分开；
- NaN、错误 joint 数、未知 joint、无 MuJoCo backend 时 fail-closed；
- 初版只 report，不设经验阈值。

### G.2 Controller frame contract

- 同 q 比较 analytic FK 与 link7、hand、`panda_ee`；
- 验证 Jacobian reference point；
- 任何控制器修改前后运行现有 17 个 GTest 和新增 cross-model tests。

### G.3 Fault injection

| 注入 | 预期观察 |
|---|---|
| 单关节 `+δ` zero offset | residual 随姿态改变并具有 joint-dependent pattern |
| joint origin/link length bias | residual 沿下游链传播 |
| TCP x/y/z offset | EE residual 呈末端局部坐标一致模式 |
| 多故障组合 | 允许 ambiguous，不强行唯一归因 |
| 无注入 | 不得产生 `SUSPECTED` |

### G.4 Camera extrinsic

- 零扰动时 contract/TF/renderer pose 一致；
- 已知平移和单轴旋转扰动；
- scene camera 验证 `world → optical`；
- wrist camera 验证 `panda_hand → optical` 且随机器人运动；
- ROS optical convention 与 CameraInfo K/P；
- randomization 后 TF 与 renderer 使用同一 effective extrinsic。

### G.5 Timestamp

- `±10/30/50/100 ms` controlled delay；
- stale、out-of-order、duplicate、missing stamps；
- static 与高速运动场景；
- image–joint、image–EE、image–object signed p50/p95/max；
- publication-time 与 source-time 指标分开；
- unavailable modality 不得生成 pass。

### G.6 Evidence outputs

```text
run_manifest.json
geometry_diagnostics.json
geometry_samples.csv
timestamp_skew.csv
```

每份证据必须记录 commit、backend、launch args、evidence class、input status 和
`PHYSICAL=NOT_RUN/UNAVAILABLE`。

---

## H. Scope Boundaries

本文及下一阶段默认不授权：

- 真实机械臂部署；
- 真实 camera calibration、hand-eye 或 joint-zero calibration；
- 把 MuJoCo GT 写成 physical ground truth；
- 把 fallback FK 写成 `SIM_GT`；
- 通用 calibration/retry framework；
- 三仓 schema/action/release/handoff 变更；
- diagnostics 直接改变控制命令或 safety decision；
- 在没有 nominal distribution 和失败成本前设置阈值；
- 将 residual 小、测试 pass 或仿真截图写成任务成功、Sim2Real 或真机证据。

---

## I. Recommended Implementation Order

### Stage 1 — TF/FK authority 与 nominal report（P0）

- 独立 URDF/KDL FK；
- TF、MuJoCo GT、controller FK 对比；
- backend provenance；
- 冻结 link7/hand/`panda_ee` 合同。

**退出门禁**：多姿态报告可复现；未知 backend fail-closed；仍保持 report-only。

### Stage 2 — 受控几何故障注入（P1）

- joint zero、kinematic origin/link、TCP offset；
- residual pattern 与 ambiguity 说明。

**退出门禁**：无注入不误报；已知注入可观测；异常输入不 pass。

### Stage 3 — Camera extrinsic contract（P1）

- 统一 XML/TF/Header/CameraInfo/renderer；
- 修复 camera randomization 未作用于 rendered stream；
- 分别验证 scene eye-to-hand 和 wrist eye-in-hand。

**退出门禁**：effective transform 可回溯，renderer 与 TF 使用同一合同。

### Stage 4 — Timestamp skew diagnostics（P1）

- signed publication skew；
- controlled delay/out-of-order；
- 运动敏感性；
- 再决定是否增加 physics sample ID/source-time 合同。

**退出门禁**：报告明确 skew 语义；missing/stale/unavailable 不会产生 pass。

Calibration solver 不进入上述四阶段。只有观察链稳定、数据具备可辨识性且存在独立验证集
后，才能单独立项。

---

## 审计停止点

截至本文快照：

- 审计证据已经落盘；
- diagnostics implementation 尚未开始；
- 没有修改代码、接口、launch、配置或测试；
- 没有生成新的运行时/物理证据；
- 下一步必须由用户显式授权具体实施阶段。

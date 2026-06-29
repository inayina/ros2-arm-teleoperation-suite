# SPEC V2-M7: 遥操作设备可插拔 + 合成数据生成 Pipeline

**分支**：`feat/v2-teleop-synth-data`  
**依赖**：M6（`feat/v2-perception-recorder`）  
**核心目标**：解耦遥操作输入设备，引入 `TeleopDriverBase` 以支持多种遥操作硬件（键盘、SpaceMouse）；在 MuJoCo 仿真端加入 Domain Randomization（域随机化），并实现批量合成数据（Synthetic Data）生成的自动化脚本；为从仿真走向算法打通最后一公里，补齐 `POLICY_DEPLOYMENT.md` 流程文档。  
**预计工作量**：5~7 天

> V1 对照：V1 无明确的输入层抽象（强耦合键盘），无 Domain Randomization 与合成数据脚本。V2 将输入设备完全插件化，全面支持端到端算法的数据集多样性要求。

---

## 1. 目标

1. **输入设备解耦**：重构 `teleop_input`，抽象出 `TeleopDriverBase`，实现按配置加载驱动。
2. **多设备支持**：迁移原键盘逻辑至 `KeyboardDriver`；新增 `SpaceMouseDriver` 框架桩代码与接线文档。
3. **仿真器域随机化**：在 `mujoco_sim` 中支持基于 YAML 配置的 Domain Randomization（相机位姿、物体质量、摩擦力、光照等随机化）。
4. **真实状态导出**：从 MuJoCo 提取目标物体的真实 6-DOF 位姿，真实反映在数据记录中（替换原占位符）。
5. **自动化合成脚本**：新建 `synth_data_gen` 包，提供可指定随机种子（Seed）和生成规模的自动化 Episode 批量采集能力。
6. **算法部署指引**：撰写 `docs/POLICY_DEPLOYMENT.md`，阐述 "数据集生成 -> 策略训练 -> 仿真推理" 的全链路闭环验证路径。

---

## 2. 包清单（M7 引入/修改）

```
src/
├── teleop_input/                   ← [MODIFY] 重构输入驱动
│   ├── teleop_input/
│   │   ├── __init__.py
│   │   ├── teleop_input_node.py    # [MODIFY] 改为通过参数动态加载 Driver
│   │   ├── driver_base.py          # [NEW] TeleopDriverBase 抽象类
│   │   ├── keyboard_driver.py      # [NEW] 键盘实现（迁移自原有逻辑）
│   │   └── spacemouse_driver.py    # [NEW] 3Dconnexion SpaceMouse 桩代码
│   ├── config/
│   │   └── input_params.yaml       # [NEW] 输入设备选择与参数配置
│   ├── package.xml
│   └── setup.py
│
├── mujoco_sim/                     ← [MODIFY] 域随机化与位姿输出
│   ├── mujoco_sim/
│   │   ├── mujoco_sim_node.py      # [MODIFY] 输出 /sim/object_pose
│   │   └── domain_randomizer.py    # [NEW] 解析配置并在每次 Reset 时应用随机化
│   └── config/
│       └── randomization.yaml      # [NEW] 随机化范围定义
│
├── synth_data_gen/                 ← [NEW] 合成数据批量流水线
│   ├── synth_data_gen/
│   │   ├── __init__.py
│   │   └── batch_generator.py      # ROS2 动作回放与自动录制控制
│   ├── package.xml
│   └── setup.py
│
└── teleop_bringup/
    └── launch/
        └── data_collection.launch.py # [NEW] 带随机化与自动录制的专有 Launch

docs/
├── SPEC_V2_M7_TELEOP_SYNTH.md      # [NEW] 本文档
└── POLICY_DEPLOYMENT.md            # [NEW] 算法部署与推理文档
```

---

## 3. 遥操作设备抽象 (`teleop_input` 重构)

### 3.1 `TeleopDriverBase` 接口

定义在 `driver_base.py` 中，要求所有遥操作设备驱动继承并实现：

```python
from abc import ABC, abstractmethod
from typing import Tuple, Optional

class TeleopDriverBase(ABC):
    """遥操作设备驱动基类"""

    @abstractmethod
    def initialize(self) -> bool:
        """设备初始化，返回是否成功"""
        pass

    @abstractmethod
    def get_pose_delta(self) -> Tuple[list, list]:
        """
        获取一个周期的位姿增量
        :return: (position_delta_xyz, rpy_delta)
        """
        pass

    @abstractmethod
    def get_gripper_cmd(self) -> Optional[float]:
        """
        获取夹爪指令（如触发变化则返回浮点数，否则返回 None）
        :return: 0.0(闭合) - 1.0(完全张开) 或 None
        """
        pass

    @abstractmethod
    def get_record_trigger(self) -> Optional[str]:
        """获取录制触发器指令 ("start", "stop", None)"""
        pass

    @abstractmethod
    def close(self) -> None:
        """释放资源"""
        pass
```

### 3.2 节点重构 (`teleop_input_node.py`)

- 移除硬编码的按键逻辑。
- 通过 ROS2 参数 `driver_type` 选择加载 `KeyboardDriver` 或 `SpaceMouseDriver`。
- 在定时器（cmd_rate）中调用 `driver.get_pose_delta()` 并累加发布到 `/teleop/cmd_pose`。

---

## 4. 仿真器域随机化 (`mujoco_sim`)

### 4.1 随机化配置 (`randomization.yaml`)

```yaml
domain_randomization:
  enabled: true
  seed: 42
  camera:
    scene:
      pos_noise: [-0.05, 0.05]   # xyz 偏移范围 (m)
      rot_noise: [-5.0, 5.0]     # rpy 偏移范围 (deg)
  object:
    mass_range: [0.1, 0.5]       # 质量范围 (kg)
    friction_range: [0.5, 1.2]   # 滑动摩擦系数范围
    initial_pos_range:
      x: [0.3, 0.6]
      y: [-0.2, 0.2]
  lighting:
    diffuse_noise: [-0.1, 0.1]
```

### 4.2 `/sim/object_pose` 输出

在 `mujoco_sim_node.py` 中：
- 识别目标物体（例如通过 `mjOBJ_BODY` 名称查找 `target_object`）。
- 提取真实的全局坐标和四元数。
- 以 100Hz 频率发布到 `/sim/object_pose` (`geometry_msgs/PoseStamped`)。

### 4.3 MuJoCo Grasp Assist（sim-direct Demo）

M7 的作品集 GIF 和合成数据 smoke run 默认走 `use_sim:=true` sim-direct 路径。为避免小方块在简化接触模型中因为摩擦/接触参数抖动而掉落，`mujoco_sim_node.py` 提供可参数化的 grasp assist：

- `grasp_assist_enabled`：默认 `true`。
- `grasp_assist_close_threshold`：夹爪闭合到该阈值以下时允许捕获目标。
- `grasp_assist_release_threshold`：夹爪打开到该阈值以上时释放目标。
- `grasp_assist_capture_radius`：末端与目标物体的捕获半径。

该机制仅用于仿真数据和作品集演示的确定性，不声明高保真接触物理。CANopen 与 E-Stop 证据仍由 M2/M5 验收覆盖。

---

## 5. 合成数据生成流水线 (`synth_data_gen`)

为了训练具有鲁棒性的策略模型，我们需要自动化运行预定轨迹或随机动作来生成大量的仿真 Episode 数据。

### 5.1 `batch_generator.py` 工作流

1. **启动与握手**：等待仿真器、ROS2 控制器及录制节点上线。
2. **循环执行 Episode** (1...N)：
   - 通过自定义服务 `/sim/reset_scene`（需在 `mujoco_sim` 新增）打乱场景并应用 Domain Randomization。
   - 向 `/teleop/record_trigger` 发布 `"start"`。
   - 读取一段预录制的动作回放（或通过插值执行固定拾取抓取逻辑），发送至 `/teleop/cmd_pose`。
   - 动作执行完毕，向 `/teleop/record_trigger` 发布 `"stop"`。
   - 等待文件写盘完成，记录成功。
3. **输出汇总**：统计成功生成的 Episode 数量和总帧数。

---

## 6. LeRobot 数据格式适配升级

### 6.1 对齐物体位姿

修改 `lerobot_recorder/recorder_node.py`：
- 订阅 `/sim/object_pose` (`geometry_msgs/PoseStamped`)。
- 将其加入 `ApproximateTimeSynchronizer` 的多模态对齐队列（如果物体静止可能需要缓存上一帧，或者让 mujoco_sim 以高频发送）。
- 写入 `EPISODE_FEATURES` 的 `observation.object_pose`，替换原来的全零占位符。

---

## 7. 验收标准

### 必须通过（阻塞合并至 main）

| # | 验收项 | 验证方式 |
|---|---|---|
| AC-1 | `TeleopDriverBase` 解耦成功，使用 `keyboard` 驱动时遥控行为与 M6 完全一致。 | `ros2 run teleop_input teleop_input_node --ros-args -p driver_type:=keyboard` 验证移动。 |
| AC-2 | `mujoco_sim` 解析 `randomization.yaml` 并成功在每次调用 reset 服务时改变目标物体的位置和颜色/相机位姿。 | 调用 `/sim/reset_scene` 服务并观察 rqt_image_view。 |
| AC-3 | `mujoco_sim` 稳定发布 `/sim/object_pose`，录制得到的 LeRobot Dataset 的 `observation.object_pose` 有效且包含正确位移。 | 录制后运行 dataset 检查脚本，确认 object_pose 非零且变化。 |
| AC-4 | `batch_generator.py` 能无人值守连续生成 10 个 Episode 的数据集。 | 运行 `ros2 run synth_data_gen batch_generator --episodes 10`。 |
| AC-5 | `docs/POLICY_DEPLOYMENT.md` 包含清晰的（1）模型训练命令、（2）策略推理节点挂载方式。 | 代码审查 / 个人走读验证。 |
| AC-6 | 提供至少一段夹爪在仿真中完成完整抓取任务的演示 GIF，并提供夹爪近景（包含在 README 中）。 | 检查 `media/m7/grasp_demo.gif`、`media/m7/gripper_closeup.gif` 及 README 引用。 |

---

## 8. 常用调试命令

```bash
# 启动带有域随机化支持的仿真
ros2 launch teleop_bringup data_collection.launch.py randomize:=true

# 切换输入设备驱动（测试桩）
ros2 run teleop_input teleop_input_node --ros-args -p driver_type:=spacemouse

# 手动触发仿真器域随机化重置
ros2 service call /sim/reset_scene std_srvs/srv/Trigger "{}"

# 监控真实物体位姿
ros2 topic echo /sim/object_pose

# 启动批量数据合成（指定种子）
ros2 run synth_data_gen batch_generator --episodes 50 --seed 2026
```

# 多目标语言条件分类抓取（Multi-Object Sorting）设计规格说明书 (Spec)

本设计规格说明书旨在规范**“多颜色形状物体 ── 语言指令分类抓取与放置”**任务在三个解耦仓库中的协同开发接口与技术实现细节，确保系统在数据流、通信总线和算法层面的闭环一致性。

---

## 1. 架构变更与接口契约 (Interface Contract)

在多目标分类任务中，数据流中新增了**“自然语言指令 (Language Instruction)”**这一关键维度。三个仓库的接口契约定义如下：

```
【上游：采集端】 ──(数据集 Parquet 写入 language_instruction)──▶ 【中游：训练端】
                                                                      │
                                                                 (模型导出 Policy)
                                                                      │
                                                                      ▼
【下游：网页端 HOC】 ◀──(输入指令文本)── 【下游：PolicyRunner】 ◀───────┘
```

### 1.1 数据集 Schema 变更（中游与上游对齐）
在中游的机器人数据 Schema 配置（`configs/robot_schemas/panda_multi_task.yaml`）中，新增以下元数据与特征定义：
```yaml
features:
  observation.image: { type: "image", shape: [3, 224, 224] }
  observation.tactile: { type: "image", shape: [1, 64, 64] }
  observation.state: { type: "vector", shape: [7] }
  action: { type: "vector", shape: [7] }
  language_instruction: { type: "string" }  # 新增自然语言指令字段
```

### 1.2 下游 ROS 2 接口变更（HOC 与 PolicyRunner 对齐）
在下游的 `bridge_monitor_msgs` 中，新增控制与触发服务，用于从网页端发送语言指令：
* **`SetLanguageTask.srv`** (自定义服务):
  ```protobuf
  string instruction  # 例如 "pick up the green sphere"
  ---
  bool success
  string message
  ```

---

## 2. 仓库一：上游 `ros2-arm-teleoperation-suite` 开发细节

上游核心任务是：**支持多物体场景渲染、随机化位置生成，并在录制时写入语言文本。**

### 2.1 物理场景升级 (`config/models/franka_panda.xml`)
- **变更点**：移除单一的 `target_object`。
- **新增内容**：
  - 定义三个独立物体：`object_red_box` (方块)、`object_blue_cylinder` (圆柱)、`object_green_sphere` (球体)。
  - 定义两个目标筐：`bin_left` (位于 `[0.4, -0.2, 0.02]`)，`bin_right` (位于 `[0.4, 0.2, 0.02]`)。

### 2.2 仿真节点位置随机化 (`src/mujoco_sim/mujoco_sim/mujoco_sim_node.py`)
- **变更点**：每次重置（Reset）环境时，不仅随机化机器人的关节角度，还需随机化这三个物体在桌面上的 X-Y 坐标和 Z 轴朝向。
- **防止重叠算法**：引入泊松圆盘采样（Poisson Disk Sampling）或简单的距离约束排查，确保三个物体初始距离不小于 10 厘米，防止模型初始状态发生穿模或重叠碰撞。

### 2.3 录制器元数据写入 (`src/lerobot_recorder/lerobot_recorder/recorder_node.py`)
- **变更点**：
  - 启动录制时，节点通过 ROS 2 Parameter 或 Service 接收当前的 `language_instruction` 参数。
  - 在写入 Parquet 文件时，将该字符串广播复制到当前 Episode 每一帧的 `language_instruction` 列中。
  - 批量采集成功提交时，每帧写入 `success=true`；失败尝试通过 `discard` 触发器丢弃，不进入训练数据目录。

### 2.4 自动数据生成原理与失败回滚 (Heuristic Oracle & Rollback)

为了量产大模型所需的数百段数据，项目摒弃了繁琐的人工手动遥操作，采用 **“特权教师-学生模型训练框架（Privileged Teacher-Student Framework）”**，通过专家启发式脚本进行数据量产。

#### 1️⃣ 自动生成原理：专家启发式脚本 (Heuristic Oracle)
自动生成脚本 `batch_generator.py` 扮演了一个“开挂的机器人专家教师”角色：
- **获取特权真值 (Privileged State)**：脚本订阅了仿真器内部透露出来的 `/sim/object_pose` 话题，直接获知物体的绝对 3D 坐标。
- **自动控制与录制**：脚本基于特权坐标计算出平滑的三维抓取轨迹，并控制机械臂自动执行抓取。在动作开始时自动发布 `/teleop/record_trigger` (`start`) 开始录制，抓取抬升动作结束后自动发布 `stop` 停止录制。
- **自动重置环境**：录完一段后，调用重置服务打乱物体位置，自动开启下一段录制。
- **结果**：开发者只需要输入启动命令，去喝杯咖啡，电脑就会在后台自动录好 500 段完美对齐、100% 成功的抓取数据集。

#### 2️⃣ 自动抓取校验与失败丢弃回滚 (Auto-Validation & Rollback)
在行为克隆（Imitation Learning）中，**训练数据必须 100% 都是成功抓取的专家演示**。如果将“抓取失败的尝试过程”喂给网络，模型就会学到错误的动作。

因此，`batch_generator.py` 内置了以下闭环防错回滚逻辑：
- **多重物理状态判定 (Multi-modal Verification)**：在机械臂执行完“抬升 (Lift)”动作后，脚本在后台以 **特权模式** 校验任务是否成功：
  - **坐标判定 (Height Check)**：检查被操作物体的 $z$ 轴坐标是否高于桌面阈值（如 $z_{object} > 0.1\text{m}$）。如果依然在桌面附近（$z_{object} \approx 0.02\text{m}$），则判定为脱手失败。
  - **接触判定 (Contact Check)**：读取夹爪传感器，检查左右手指与目标物体之间是否有持续的接触力。
- **失败自动回滚与覆盖 (Rollback & Discard)**：
  - **判定为 Success**：正常停止录制，将当前的 `episode_xx` 保存，索引加 1，继续录制下一个。
  - **判定为 Failed**：
    1. 立即发布停止录制信号。
    2. 自动删除刚刚录制失败的 `episode_xx` 临时数据文件夹（执行磁盘目录回滚，清除垃圾数据）。
    3. 触发 `/sim/reset_scene` 服务打乱物体，**不增加成功 Episode 索引**，原地重新采集当前关卡，直到成功录满设定的 Episode 数量为止。
    4. 输出日志警报：`[BatchGenerator] Grab failed in episode 12. Rollback folder and retrying...`

---

## 3. 仓库二：中游 `robot-arm-episode-data-lab` 开发细节

中游核心任务是：**在模型训练中引入文本特征嵌入（Text Embedding），训练多任务策略。**

### 3.1 文本特征提取器 (Text Encoder)
- **实现方案**：在训练和推理时，使用轻量化的预训练大模型分词器（如 **DistilBERT-base-uncased** 或 **CLIP Text Encoder**）对指令字符串进行编码。
- **输出格式**：将长度可变的文本转换为固定维度的向量（例如 $1 \times 512$ 维度的 Text Embedding）。

### 3.2 模仿学习网络（ACT）架构升级 (`training/models/act.py`)
- **架构变更**：
  - 原 ACT 仅输入图像特征（ResNet 输出）和当前关节角度（State）。
  - **新架构**：引入 **Cross-Attention（交叉注意力）** 机制。将提取出的 $512$ 维 Text Embedding 作为 Query，与视觉特征和状态特征（Keys & Values）进行交叉融合。
  - **效果**：使神经网络在解码动作时，能够根据语言指令特征，选择性地将“注意力”聚焦到特定颜色和形状的图像像素上。

---

## 4. 仓库三：下游 `ros2-moveit-pybullet-bridge` 开发细节

下游核心任务是：**加载多物体场景，支持网页端发送任务指令，并回放 Policy。**

### 4.1 PyBullet 场景对齐 (`pybullet_bridge/pybullet_bridge/pybullet_bridge_node.py`)
- **变更点**：
  - 下游 PyBullet 节点必须在初始化时，读取与上游一致的 `franka_panda.xml` 或加载对应的 URDF 模型，确保物理碰撞体和颜色材质与 MuJoCo 100% 对齐。
  - 订阅上游同步过来的物体初始位置，保持双端状态同步。

### 4.2 策略运行器升级 (`risk_engine/risk_engine/policy_runner.py`)
- **变更点**：
  - 提供 `SetLanguageTask` 服务。
  - 当接收到 HOC 网页端发送的指令后，调用预训练好的多任务 Policy，将指令转化为 Embedding 喂给神经网络，开始循环输出期望力矩/角度。

### 4.3 网页控制台（HOC）交互升级 (`hoc_console/`)
- **前端新增组件**：在 React 界面上增加一个 **“任务下发面板”**。
- **交互方式**：
  - 包含一个下拉菜单（可以选择预设的分类任务，如 `"抓取红盒子送入左箱"`）和一个自由输入文本框。
  - 点击“发送指令”按钮，通过 WebSocket 向 `hoc_server` 发送请求，桥接调用 ROS 2 服务，启动 Policy 运行.
  - ECharts 实时图表在回放时展示当前注意力机制（Attention Heatmap）的聚焦权重。

---

## 5. 验证与交付计划 (Verification Plan)

### 5.1 数据校验验证
- 运行 `python3 scripts/validate_dataset.py <episode_root> --min-frames 5`，必须验证 `language_instruction` 字段存在且内容不为空、`success=true`、无 `safety_estop` 与 `drive_fault` 污染帧。

### 5.2 策略收敛与烟雾测试 (ACT Smoke Test)
- 运行 `python3 training/scripts/train_act_smoke.py --schema configs/robot_schemas/panda_multi_task.yaml`。
- 验证模型能够正常向前传播（Forward Pass）并输出动作，Loss 在 10 个 Epoch 内呈下降趋势。

### 5.3 跨仿真回放与急停验证
- 在下游启动 HOC 控制台与 PolicyRunner。
- 下发指令 `"pick up the green sphere"`。
- 人为在 PyBullet 中将绿球坐标移走（模拟真实偏差），验证 `dist_monitor` 能够灵敏地检测到 KL 散度超标，网页端红屏报警，并在 Policy 控制器失控前触发刹车服务。

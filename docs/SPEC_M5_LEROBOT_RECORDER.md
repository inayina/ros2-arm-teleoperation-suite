# SPEC M5: LeRobot 数据录制节点

**分支**：`feat/lerobot-recorder`  
**里程碑**：M5  
**预计用时**：Week 5 后半段（2 天）  
**前置依赖**：M4 全链路贯通

---

## 1. 目标

实现 ROS2 录制节点，在遥操作过程中实时缓存数据，并以 **HuggingFace `datasets` hf_dataset 格式**保存 Episode，直接对接 ACT / Diffusion Policy 训练管线。

---

## 2. 技术选型与理由

### 2.1 存储格式：HuggingFace datasets（Arrow 格式）

| 方案 | 优点 | 缺点 | 结论 |
|---|---|---|---|
| **HuggingFace datasets Arrow** ✅ | 直接兼容 ACT/Diffusion Policy；`load_from_disk()` 零依赖；可与 `robot-arm-episode-data-lab` 数据合并 | — | **选用** |
| LeRobot v2.1 原生格式（Parquet） | HuggingFace Hub 原生 | 需 lerobot 库；格式较重 | 备选（`export_lerobot_style.py` 已实现） |
| HDF5（.h5 文件） | 本地存储效率高 | 需额外转换才能训练 | 否（`robot-arm-episode-data-lab` 已用） |
| ROS2 Bag | ROS 生态原生 | 训练管线不直接支持 | 否 |

**选用理由**：M0 已验证 `datasets.save_to_disk()` / `load_from_disk()` 的格式，与 `robot-arm-episode-data-lab` 的 `export_to_lerobot.py` 输出完全对齐，两个项目的数据可直接 `concatenate_datasets()` 合并扩充训练集。

### 2.2 图像处理：Pillow（PIL）

**选用理由**：MuJoCo offscreen render 输出 uint8 RGB numpy array，PIL 直接 `Image.fromarray()` 处理，不引入额外依赖。如需压缩存储，`PIL.Image.tobytes()` 可转为 bytes 列。

### 2.3 录制触发：ROS2 话题 `/episode/status`

| 方案 | 优点 | 缺点 | 结论 |
|---|---|---|---|
| **ROS2 话题触发** ✅ | 解耦；任何节点都可触发录制 | — | **选用** |
| 键盘信号在录制节点内监听 | 简单 | 耦合；不能远程触发 | 否 |

---

## 3. 数据格式规范

### 3.1 每步数据字段

```python
step = {
    # 观测
    "observation.state":        np.float32, shape=(7,),   # 关节位置
    "observation.ee_pose":      np.float32, shape=(7,),   # 末端位姿 [x,y,z,qx,qy,qz,qw]
    "observation.object_pose":  np.float32, shape=(7,),   # 物体位姿（可选，MuJoCo 读取）
    # 图像（可选，--include-images 开启）
    "observation.images.top":   bytes,                    # PNG bytes
    # 动作
    "action":                   np.float32, shape=(7,),   # 目标关节位置（来自 /master_pose 转换）
    # 元数据
    "timestamp":                float64,                  # 相对时间 (s)
    "episode_index":            int64,
    "frame_index":              int64,
    "done":                     bool,                     # 仅末帧为 True
    "language_instruction":     str,                      # 如 "pick up the cube"
    "success":                  bool,                     # 录制结束时由用户标注
}
```

### 3.2 目录结构

```
data/episodes/
├── episode_000001/
│   ├── train/
│   │   ├── dataset_info.json
│   │   └── data-00000-of-00001.arrow
│   └── export_info.json
├── episode_000002/
│   └── ...
└── merged/                    ← 合并后的完整数据集（可选）
    ├── train/
    └── val/
```

---

## 4. 节点设计

### 4.1 状态机

```
IDLE ──(R键/status="record_start")──▶ RECORDING
RECORDING ──(R键/status="record_stop")──▶ SAVING
SAVING ──(写入完成)──▶ IDLE

RECORDING 期间：每次 /joint_states 回调时缓存一帧
SAVING：datasets.save_to_disk() 写入磁盘
```

### 4.2 核心代码结构

```python
class LeRobotRecorderNode(Node):
    def __init__(self):
        # 订阅
        self.sub_joint = self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 10)
        self.sub_ft    = self.create_subscription(
            WrenchStamped, "/ft_sensor", self._on_ft, 10)
        self.sub_status = self.create_subscription(
            String, "/episode/status", self._on_status, 10)

        # 录制缓冲（线程安全）
        self._buffer: list[dict] = []
        self._recording = False
        self._episode_index = 0
        self._t0: float | None = None

    def _on_joint_states(self, msg: JointState) -> None:
        if not self._recording:
            return
        t = self.get_clock().now().nanoseconds / 1e9
        frame = {
            "observation.state": list(msg.position),
            "frame_index":       len(self._buffer),
            "timestamp":         t - self._t0,
            # action 从 /master_pose 转换得到（另一个订阅）
        }
        self._buffer.append(frame)

    def _on_status(self, msg: String) -> None:
        if msg.data == "record_start" and not self._recording:
            self._start_recording()
        elif msg.data == "record_stop" and self._recording:
            self._stop_and_save()

    def _stop_and_save(self) -> None:
        self._recording = False
        # 补全 done 字段
        if self._buffer:
            self._buffer[-1]["done"] = True
        # 异步写入（不阻塞 ROS2 spin）
        threading.Thread(target=self._save_episode, daemon=True).start()

    def _save_episode(self) -> None:
        ds = Dataset.from_list(self._buffer, features=EPISODE_FEATURES)
        out = Path(f"data/episodes/episode_{self._episode_index:06d}/train")
        ds.save_to_disk(str(out))
        self.get_logger().info(f"Saved episode {self._episode_index} → {out}")
        self._episode_index += 1
        self._buffer = []
```

---

## 5. 文件清单

```
src/lerobot_recorder/
├── package.xml
├── setup.py
└── lerobot_recorder/
    ├── __init__.py
    ├── recorder_node.py          ← 录制节点主体
    └── episode_features.py       ← HuggingFace Features schema 定义

scripts/
└── merge_episodes.py             ← 将多个 episode 合并为一个 DatasetDict

tests/
└── test_lerobot_format.py        ← 录制格式验证
```

---

## 6. 验收标准

### 必须通过（阻塞合并）

| # | 验收项 | 验证方法 | 指标 |
|---|---|---|---|
| AC-1 | 按 R 开始录制，`/episode/status` 发布 "record_start"，节点日志显示 "Recording started" | 键盘 + `ros2 topic echo` | — |
| AC-2 | 录制 50 帧后按 R 停止，`data/episodes/episode_000001/train/` 目录生成 | `ls` 确认 | 目录存在 |
| AC-3 | `datasets.load_from_disk("data/episodes/episode_000001/train")` 成功加载，`len(ds) == 50` | Python 脚本 | == 50 |
| AC-4 | 加载后字段完整：`observation.state`, `action`, `timestamp`, `episode_index`, `frame_index`, `done`, `language_instruction` | `ds.features` | 7 个字段 |
| AC-5 | 最后一帧 `done == True`，其余帧 `done == False` | `ds[-1]["done"]` | True |
| AC-6 | `tests/test_lerobot_format.py` pytest 全通过 | `pytest tests/` | 0 failures |
| AC-7 | 单个 50 帧 Episode 文件大小 < 5MB（不含图像） | `du -sh` | < 5MB |

### 加分项

- [ ] `--include-images` 模式下，图像字节正确存入 `observation.images.top`
- [ ] `scripts/merge_episodes.py` 可将 10 个 episode 合并为一个 DatasetDict（train/val 分割）
- [ ] 录制节点崩溃重启后，episode_index 从上次断点自动续编

---

## 7. 与 robot-arm-episode-data-lab 数据对齐

本节点输出格式与 `robot-arm-episode-data-lab` 的 `export_to_lerobot.py` 完全对齐：

| 字段 | 本项目（MuJoCo） | episode-data-lab（PyBullet） |
|---|---|---|
| `observation.state` | Panda 7 关节位置 | Kuka IIWA 7 关节位置 |
| `action` | 目标关节位置（7D） | 目标关节位置（7D） |
| `observation.ee_pose` | KDL 正运动学结果（7D） | PyBullet `getLinkState`（7D） |
| `timestamp` | ROS2 clock（float64） | step / fps（float64） |
| `done` | 末帧为 True | 末帧为 True |

两个数据集可直接 `concatenate_datasets([ds1, ds2])` 合并，扩大训练数据量。

---

## 8. 面试话术关键点

> "录制节点的关键设计是**异步写入**：`save_to_disk()` 是 IO 密集操作，如果在 ROS2 回调里同步调用，会阻塞整个 spin 循环导致漏帧。我把写入逻辑放到 daemon 子线程里，录制完成后立即恢复响应，这个模式和 ROS2 Bag 的异步写入是相同的思路。"

# SPEC V2-M6: 视觉感知 + 多模态 LeRobot Recorder + 收尾

**分支**：`feat/v2-perception-recorder`  
**依赖**：M5（`feat/v2-safety-layer` 已合入 main）  
**核心目标**：实现 MuJoCo 虚拟相机 RGB/Depth 图像发布（`camera_bridge`），以及多模态时间对齐录制（`lerobot_recorder`），输出兼容 ACT/Diffusion Policy 训练管线的 `LeRobotDataset`；并完成整体收尾（README 更新、架构图刷新、演示视频）。  
**预计工作量**：6~8 天

> V1 对照：V1 只录制 joint_state + ee_pose，无视觉数据。V2 新增 RGB/Depth 双路图像，用 `message_filters.ApproximateTimeSynchronizer` 对齐多模态，录制结果直接对接 LeRobot v2 格式。

---

## 1. 目标

1. 在 `franka_panda.xml` 中添加虚拟相机（`scene` 固定相机 + `wrist` 腕部相机）
2. 实现 `camera_bridge_node`：从 MuJoCo offscreen renderer 读取 RGB/Depth，发布到 ROS2 @30Hz
3. 实现 `lerobot_recorder_node`：多模态时间同步 + LeRobot v2 格式写盘
4. `teleop/record_trigger` 控制录制开始/停止
5. 录制结果可被 `LeRobotDataset.load()` 正常加载，字段完整
6. 项目收尾：更新 README、SKILL.md、ARCHITECTURE_V2.md，补充演示 GIF/视频

---

## 2. 包清单（M6 引入/修改）

```
src/
├── camera_bridge/                  ← [NEW] L6 视觉感知（Python）
│   ├── camera_bridge/
│   │   ├── __init__.py
│   │   └── camera_bridge_node.py   # MuJoCo offscreen renderer → ROS2 Image
│   ├── package.xml
│   └── setup.py
│
├── lerobot_recorder/               ← [NEW] L7 多模态录制（Python）
│   ├── lerobot_recorder/
│   │   ├── __init__.py
│   │   ├── recorder_node.py        # ROS2 节点 + message_filters 时间同步
│   │   ├── time_sync.py            # ApproximateTimeSynchronizer 封装
│   │   └── lerobot_writer.py       # LeRobotDataset v2 格式写盘
│   ├── package.xml
│   └── setup.py
│
├── mujoco_sim/                     ← [MODIFY] 添加虚拟相机渲染
│   └── mujoco_sim/
│       ├── mujoco_sim_node.py      # 添加 /mujoco/scene_rgb、/mujoco/scene_depth 内部接口
│       └── virtual_camera.py       # [NEW] offscreen renderer 封装
│
└── teleop_bringup/
    └── launch/
        ├── simulation.launch.py    # [MODIFY] 含 camera_bridge
        └── recording.launch.py     # [NEW] lerobot_recorder 启动

config/
└── models/
    └── franka_panda.xml            # [MODIFY] 添加 <camera> 和 <sensor> 定义
```

---

## 3. MuJoCo 虚拟相机配置

### 3.1 `franka_panda.xml` 相机添加

```xml
<!-- 在 <worldbody> 末尾添加固定场景相机 -->
<body name="scene_camera_mount" pos="1.0 0.0 0.8">
  <camera name="scene" pos="0 0 0" xyaxes="0 -1 0 0 0 1" fovy="60"/>
</body>

<!-- 在 panda_hand 末尾添加腕部相机 -->
<body name="wrist_camera_mount" pos="0 0 0.05" parent="panda_hand">
  <camera name="wrist" pos="0 0 0" xyaxes="1 0 0 0 0 -1" fovy="80"/>
</body>

<!-- 在 <sensor> 块添加 FT 传感器（如未添加） -->
<sensor>
  <force  name="ee_force_sensor"  site="attachment_site"/>
  <torque name="ee_torque_sensor" site="attachment_site"/>
</sensor>
```

### 3.2 相机内参（用于 `camera_info` 发布）

```python
# scene 相机（固定，640×480，fovy=60°）
SCENE_WIDTH, SCENE_HEIGHT = 640, 480
SCENE_FOV_DEG = 60.0

# 从 fovy 计算焦距
f = (SCENE_HEIGHT / 2.0) / math.tan(math.radians(SCENE_FOV_DEG / 2.0))
K_scene = [f, 0, SCENE_WIDTH/2,
           0, f, SCENE_HEIGHT/2,
           0, 0, 1]

# wrist 相机（腕部，320×240，fovy=80°）
WRIST_WIDTH, WRIST_HEIGHT = 320, 240
WRIST_FOV_DEG = 80.0
```

---

## 4. `camera_bridge_node.py` 实现

### 4.1 MuJoCo Offscreen Renderer

```python
class VirtualCamera:
    """封装 MuJoCo offscreen renderer，输出 RGB + Depth"""

    def __init__(self, model: mujoco.MjModel, camera_name: str,
                 width: int, height: int):
        self.renderer = mujoco.Renderer(model, height=height, width=width)
        self.camera_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)

    def render(self, data: mujoco.MjData) -> tuple[np.ndarray, np.ndarray]:
        """返回 RGB (H, W, 3) uint8 和 Depth (H, W) float32 (meters)"""
        # RGB
        self.renderer.update_scene(data, camera=self.camera_id)
        rgb = self.renderer.render()   # (H, W, 3) uint8

        # Depth（需启用 depth rendering）
        self.renderer.enable_depth_rendering()
        self.renderer.update_scene(data, camera=self.camera_id)
        depth_raw = self.renderer.render()   # (H, W) float32，范围 0~1（归一化深度）
        self.renderer.disable_depth_rendering()

        # 归一化深度 → 真实距离（m）
        extent = data.model.stat.extent
        near   = data.model.vis.map.znear * extent
        far    = data.model.vis.map.zfar  * extent
        depth_m = near / (1.0 - depth_raw * (1.0 - near / far))

        return rgb, depth_m
```

### 4.2 `camera_bridge_node.py` 骨架

```python
class CameraBridgeNode(rclpy.node.Node):
    def __init__(self, model, data_ref):
        super().__init__("camera_bridge")

        # 发布者
        self._scene_rgb_pub    = self.create_publisher(Image, "/camera/color/image_raw", 10)
        self._scene_depth_pub  = self.create_publisher(Image, "/camera/depth/image_raw", 10)
        self._scene_info_pub   = self.create_publisher(CameraInfo, "/camera/color/camera_info", 10)

        # 虚拟相机（scene + wrist）
        self._scene_cam = VirtualCamera(model, "scene", 640, 480)
        self._bridge    = CvBridge()

        # 30 Hz 定时器
        self.create_timer(1.0 / 30.0, self._publish_frames)

    def _publish_frames(self):
        stamp = self.get_clock().now().to_msg()

        with DATA_LOCK:   # mujoco_sim_node 共享 data 的锁
            rgb, depth = self._scene_cam.render(SHARED_DATA)

        # RGB → sensor_msgs/Image (rgb8)
        rgb_msg = self._bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = "scene_camera"
        self._scene_rgb_pub.publish(rgb_msg)

        # Depth → sensor_msgs/Image (32FC1, meters)
        depth_msg = self._bridge.cv2_to_imgmsg(depth, encoding="32FC1")
        depth_msg.header = rgb_msg.header
        self._scene_depth_pub.publish(depth_msg)

        # CameraInfo
        info = build_camera_info(stamp, "scene_camera", 640, 480, K_scene)
        self._scene_info_pub.publish(info)
```

---

## 5. `lerobot_recorder_node.py` 实现

### 5.1 LeRobot v2 数据格式

```python
from datasets import Dataset, Features, Sequence, Value, Array3D, Array2D

EPISODE_FEATURES = Features({
    # 本体感知观测
    "observation.state":           Sequence(Value("float32"), length=7),   # /joint_states position
    "observation.ee_pose":         Sequence(Value("float32"), length=7),   # /ee_pose (xyz+quat)
    "observation.ft":              Sequence(Value("float32"), length=6),   # /ft_sensor (fx fy fz tx ty tz)
    "observation.gripper":         Sequence(Value("float32"), length=1),   # /gripper/state

    # 视觉观测
    "observation.images.scene":    Array3D(dtype="uint8", shape=(480, 640, 3)),   # /camera/color
    "observation.images.wrist":    Array3D(dtype="uint8", shape=(240, 320, 3)),   # 腕部相机（可选）
    "observation.depth.scene":     Array2D(dtype="float32", shape=(480, 640)),    # /camera/depth

    # 动作（遥操作指令）
    "action":                      Sequence(Value("float32"), length=8),   # ee_pose(7) + gripper(1)

    # 元数据
    "timestamp":                   Value("float64"),
    "episode_index":               Value("int64"),
    "frame_index":                 Value("int64"),
    "done":                        Value("bool"),
    "task":                        Value("string"),

    # 安全元数据（用于过滤污染片段）
    "safety_estop":                Value("bool"),     # /safety/status.estop_active
    "drive_fault":                 Value("bool"),     # /servo_drive/status 中有任意 Fault
})
```

### 5.2 多模态时间同步（`time_sync.py`）

```python
import message_filters
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import PoseStamped, WrenchStamped

class MultiModalSync:
    """用 ApproximateTimeSynchronizer 对齐多模态话题"""

    SLOP_SEC = 0.05   # 50ms 对齐容差（相机 @30Hz，相邻帧间隔 33ms）

    def __init__(self, node: rclpy.node.Node, callback):
        self._joint_sub  = message_filters.Subscriber(node, JointState, "/joint_states")
        self._ee_sub     = message_filters.Subscriber(node, PoseStamped, "/ee_pose")
        self._ft_sub     = message_filters.Subscriber(node, WrenchStamped, "/ft_sensor")
        self._rgb_sub    = message_filters.Subscriber(node, Image, "/camera/color/image_raw")
        self._depth_sub  = message_filters.Subscriber(node, Image, "/camera/depth/image_raw")

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._joint_sub, self._ee_sub, self._ft_sub,
             self._rgb_sub, self._depth_sub],
            queue_size=30,
            slop=self.SLOP_SEC,
        )
        self._sync.registerCallback(callback)
```

### 5.3 `recorder_node.py` 骨架

```python
class LeRobotRecorderNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("lerobot_recorder")

        self._recording = False
        self._buffer: list[dict] = []
        self._episode_idx = 0
        self._frame_idx   = 0

        # 时间同步器
        self._sync = MultiModalSync(self, self._on_synchronized)

        # 录制触发
        self.create_subscription(
            String, "/teleop/record_trigger", self._on_trigger, 10)

        # 安全状态（元数据）
        self._latest_safety = False
        self.create_subscription(
            SafetyStatus, "/safety/status",
            lambda m: setattr(self, '_latest_safety', m.estop_active), 10)

    def _on_synchronized(self, joint_msg, ee_msg, ft_msg, rgb_msg, depth_msg):
        if not self._recording:
            return

        rgb_np   = self._bridge.imgmsg_to_cv2(rgb_msg, "rgb8")
        depth_np = self._bridge.imgmsg_to_cv2(depth_msg, "32FC1")

        frame = {
            "observation.state":        list(joint_msg.position[:7]),
            "observation.ee_pose":      [ee_msg.pose.position.x, ee_msg.pose.position.y,
                                         ee_msg.pose.position.z,
                                         ee_msg.pose.orientation.x, ee_msg.pose.orientation.y,
                                         ee_msg.pose.orientation.z, ee_msg.pose.orientation.w],
            "observation.ft":           [ft_msg.wrench.force.x,  ft_msg.wrench.force.y,
                                         ft_msg.wrench.force.z,  ft_msg.wrench.torque.x,
                                         ft_msg.wrench.torque.y, ft_msg.wrench.torque.z],
            "observation.gripper":      [self._latest_gripper],
            "observation.images.scene": rgb_np,
            "observation.depth.scene":  depth_np,
            "action":                   self._latest_action,   # 来自 /teleop/cmd_pose
            "timestamp":                rclpy.time.Time.from_msg(joint_msg.header.stamp).nanoseconds * 1e-9,
            "episode_index":            self._episode_idx,
            "frame_index":              self._frame_idx,
            "done":                     False,
            "task":                     self._current_task,
            "safety_estop":             self._latest_safety,
            "drive_fault":              self._latest_drive_fault,
        }
        self._buffer.append(frame)
        self._frame_idx += 1

    def _save_episode(self):
        """将缓冲写入 LeRobot Dataset 格式"""
        if self._buffer:
            self._buffer[-1]["done"] = True  # 最后一帧标记 done
            ds = Dataset.from_list(self._buffer, features=EPISODE_FEATURES)
            save_path = f"data/episodes/episode_{self._episode_idx:06d}/train"
            ds.save_to_disk(save_path)
            self.get_logger().info(f"Episode {self._episode_idx} saved: {len(self._buffer)} frames → {save_path}")
            self._episode_idx += 1
            self._buffer.clear()
            self._frame_idx = 0
```

---

## 6. 接口定义

### 6.1 `camera_bridge` 发布话题

| Topic | 类型 | 频率 | 说明 |
|---|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` (rgb8) | 30 Hz | scene 相机 RGB |
| `/camera/depth/image_raw` | `sensor_msgs/Image` (32FC1) | 30 Hz | scene 相机深度（m） |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | 相机内参 |

### 6.2 `lerobot_recorder` 订阅话题

| Topic | 类型 | 频率 | 用途 |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | 100 Hz | 本体感知 |
| `/ee_pose` | `geometry_msgs/PoseStamped` | 100 Hz | 末端位姿 |
| `/ft_sensor` | `geometry_msgs/WrenchStamped` | 100 Hz | 接触力 |
| `/camera/color/image_raw` | `sensor_msgs/Image` | 30 Hz | RGB 图像（时间同步主频） |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 30 Hz | 深度图像 |
| `/gripper/state` | `std_msgs/Float64` | 20 Hz | 夹爪状态 |
| `/safety/status` | `teleop_interfaces/SafetyStatus` | 50 Hz | 安全元数据 |
| `/servo_drive/status` | `teleop_interfaces/DriveStatus` | 50 Hz | 驱动器故障元数据 |
| `/teleop/record_trigger` | `std_msgs/String` | event | `"start"`/`"stop"` |
| `/teleop/cmd_pose` | `geometry_msgs/PoseStamped` | 100 Hz | 动作标签（action） |

---

## 7. 验收标准

### 必须通过（阻塞合并至 main）

| # | 验收项 | 验证命令 |
|---|---|---|
| AC-1 | `/camera/color/image_raw` + `/camera/depth/image_raw` @30Hz 稳定发布 | `ros2 topic hz /camera/color/image_raw` |
| AC-2 | MuJoCo viewer 中相机画面与 `/camera/color/image_raw` 内容一致（rqt_image_view 确认） | `ros2 run rqt_image_view rqt_image_view` |
| AC-3 | 录制 30 秒 Episode（发 `"start"` → 操作 → 发 `"stop"`），生成 `data/episodes/episode_000000/train/` | 按步骤录制 |
| AC-4 | `LeRobotDataset.load("data/episodes/episode_000000/train")` 成功加载，字段完整（state/ee/ft/gripper/rgb/depth/action/ts） | `python -c "from datasets import load_from_disk; ds=load_from_disk('...'); print(ds.features)"` |
| AC-5 | `ApproximateTimeSynchronizer` 对齐率 > 90%（30 秒录制不少于 800 帧） | 计算 `len(ds)` |
| AC-6 | `safety_estop` / `drive_fault` 字段在 E-Stop 片段中正确标记 `True` | 触发 E-Stop 录制后检查字段 |
| AC-7 | 录制数据可被 ACT 配置直接消费（或 `LeRobotDataset.from_preloaded()` 无报错） | 运行 ACT dataloader 验证 |
| AC-8 | README / ARCHITECTURE_V2.md 更新，`feat/v2-perception-recorder` PR 描述完整 | 代码审查 |

### 加分项

- [ ] 腕部相机（wrist）图像同时录制（`observation.images.wrist`）
- [ ] Episode 可视化脚本（逐帧播放 RGB + joint_states）
- [ ] HuggingFace Hub 上传脚本（`ds.push_to_hub()`）
- [ ] 演示视频 GIF（MuJoCo viewer + 遥操作 + 录制流程）

---

## 8. 常用调试命令

```bash
# 构建 M6 相关包
colcon build --packages-select camera_bridge lerobot_recorder
source install/setup.bash

# 单独测试相机桥接
ros2 run camera_bridge camera_bridge_node

# 查看相机图像（需 rqt_image_view）
ros2 run rqt_image_view rqt_image_view /camera/color/image_raw

# 查看深度图（伪彩色）
ros2 run rqt_image_view rqt_image_view /camera/depth/image_raw

# 启动录制节点
ros2 run lerobot_recorder recorder_node

# 开始/停止录制
ros2 topic pub /teleop/record_trigger std_msgs/msg/String "{data: 'start'}" --once
ros2 topic pub /teleop/record_trigger std_msgs/msg/String "{data: 'stop'}" --once

# 验证录制结果
python3 - <<EOF
from datasets import load_from_disk
ds = load_from_disk("data/episodes/episode_000000/train")
print(f"帧数: {len(ds)}")
print(f"字段: {list(ds.features.keys())}")
print(f"第一帧 state: {ds[0]['observation.state']}")
print(f"图像形状: {ds[0]['observation.images.scene'].shape}")
EOF

# 全链路启动（含 Recorder）
ros2 launch teleop_bringup full_system.launch.py record:=true headless:=false
```

---

## 9. 项目收尾清单

| 收尾项 | 负责文件 | 说明 |
|---|---|---|
| 更新 README.md 演示视频/GIF 占位 | `README.md` | 替换 `media/` 目录中的演示内容 |
| ARCHITECTURE_V2.md 刷新（M1~M6 全部 ✅） | `docs/ARCHITECTURE_V2.md` | 里程碑表格状态更新 |
| ROADMAP.md 勾选 M6 检查清单 | `docs/ROADMAP.md` | 标记全部完成 |
| SKILL.md 参见链接更新 | `.agents/skills/ros2-teleop-dev/SKILL.md` | 添加 V2 SPEC 链接 |
| 作品集叙事更新 | `ARCHITECTURE_V2.md §8` | 补充实测数据（延迟/频率/误差） |

---

## 10. 关键风险与应对

| 风险 | 应对 |
|---|---|
| MuJoCo offscreen renderer 无 GPU 时 30Hz 帧率不够 | `headless:=true` + `mujoco.Renderer` 使用 EGL/osmesa；降分辨率（320×240）兜底 |
| `ApproximateTimeSynchronizer` 对齐率低（多模态频率差异大） | 统一以 30Hz（相机频率）为基准；`joint_states`/`ft_sensor` 先降采样到 30Hz 再同步 |
| LeRobot dataset 图像字段大（640×480 uint8 × N 帧） | 分 Episode 写盘（每 Episode 独立目录）；图像用 PNG 压缩存储（Arrow LargeBinary） |
| `cv_bridge` 与 Python 3.12/conda 环境不兼容 | 使用 `numpy` 直接操作 msgdata：`np.frombuffer(msg.data, dtype=np.uint8).reshape(...)` |

---

*本文件为 V2-M6 细化 SPEC；架构基线见 [`ARCHITECTURE_V2.md`](./ARCHITECTURE_V2.md) §6.5–6.6，里程碑总览见 [`ROADMAP.md`](./ROADMAP.md)。*

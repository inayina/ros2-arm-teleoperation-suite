# 上游仓库：多目标分类抓取采集开发指南 (Upstream Sorting Dev Guide)

本指南针对上游 `ros2-arm-teleoperation-suite` 仓库，规范多目标分类抓取任务的**物理场景配置、防重叠随机化、以及自动化数据采集**的具体开发细节与执行命令。

---

## 1. 本地开发与构建

确保仅编译与数据采集和仿真相关的包，避免无关包的影响：
```bash
colcon build --symlink-install --packages-select \
  teleop_interfaces \
  mujoco_sim \
  lerobot_recorder \
  synth_data_gen \
  teleop_bringup
```

---

## 2. 物理场景配置 (`config/models/franka_panda.xml`)

开发阶段需在 `franka_panda.xml` 的 `<worldbody>` 末尾，替换原有的单一 `target_object`，添加多形状、多颜色的独立物体几何体定义：
```xml
<!-- 红色方块 -->
<body name="object_red_box" pos="0.35 -0.1 0.05">
  <freejoint name="red_box_joint" />
  <geom name="red_box_geom" type="box" size="0.025 0.025 0.025" rgba="0.9 0.1 0.1 1" mass="0.04" friction="1.0 0.05 0.01" />
</body>

<!-- 蓝色圆柱 -->
<body name="object_blue_cylinder" pos="0.4 0.1 0.05">
  <freejoint name="blue_cylinder_joint" />
  <geom name="blue_cylinder_geom" type="cylinder" size="0.02 0.03" rgba="0.1 0.1 0.9 1" mass="0.05" friction="1.2 0.05 0.01" />
</body>

<!-- 绿色球体 -->
<body name="object_green_sphere" pos="0.45 0.0 0.05">
  <freejoint name="green_sphere_joint" />
  <geom name="green_sphere_geom" type="sphere" size="0.025" rgba="0.1 0.9 0.1 1" mass="0.03" friction="0.8 0.05 0.01" />
</body>

<!-- 左侧收集筐 (红/绿色) -->
<body name="bin_left" pos="0.4 -0.25 0.01" static="true">
  <geom name="bin_left_base" type="box" size="0.08 0.08 0.01" rgba="0.2 0.2 0.2 0.5" />
  <geom name="bin_left_wall1" type="box" size="0.08 0.005 0.03" pos="0 0.075 0.02" rgba="0.2 0.2 0.2 0.8" />
  <!-- 其他四面墙以此类推 -->
</body>

<!-- 右侧收集筐 (蓝色) -->
<body name="bin_right" pos="0.4 0.25 0.01" static="true">
  <geom name="bin_right_base" type="box" size="0.08 0.08 0.01" rgba="0.2 0.2 0.2 0.5" />
  <geom name="bin_right_wall1" type="box" size="0.08 0.005 0.03" pos="0 0.075 0.02" rgba="0.2 0.2 0.2 0.8" />
</body>
```

---

## 3. 位置随机化实现 (`src/mujoco_sim/mujoco_sim/mujoco_sim_node.py`)

在 `mujoco_sim_node` 的 `reset_scene` 服务回调函数中，添加以下逻辑以随机化物体初始位置，避免重叠：
```python
def _handle_reset_scene(self, request, response):
    # 1. 关节角度随机初始化...
    
    # 2. 物体位置随机化 (泊松圆盘采样)
    min_dist = 0.10  # 物体间最小间距 10cm
    workspace_x = [0.35, 0.48]
    workspace_y = [-0.15, 0.15]
    
    objects = ["object_red_box", "object_blue_cylinder", "object_green_sphere"]
    placed_poses = []
    
    import random
    for name in objects:
        while True:
            # 随机采样一个 X, Y 坐标
            test_x = random.uniform(*workspace_x)
            test_y = random.uniform(*workspace_y)
            # 校验与已放置物体的距离
            if all(math.hypot(test_x - px, test_y - py) >= min_dist for px, py in placed_poses):
                placed_poses.append((test_x, test_y))
                # 调用 MuJoCo API 设置物体位置
                obj_id = self.model.body(name).id
                self.data.xpos[obj_id] = [test_x, test_y, 0.03]  # Z 轴贴近桌面
                break
    
    response.success = True
    return response
```

---

## 4. 自动化批量采集命令

你可以通过配置 `synth_data_gen` 自动生成不同任务的数据集。当前批量采集已经接入训练前闸门：

- `batch_generator` 以“成功 episode 数”为目标计数，失败会向 recorder 发送 `discard`，不会落盘。
- 默认 `validation_mode:=place`，要求目标物体曾被抬起并最终落在目标筐 XY 范围内。
- recorder 会在每帧写入 `language_instruction` 与 `success` 字段，供 ACT/Diffusion 数据管线过滤。

### 4.0 批量采集前成功率准入

不要在抓取/放置参数没调稳时直接开 100+ episode。正式批采前按以下门槛预飞：

| 阶段 | 规模 | 准入线 |
|---|---:|---|
| 单条 smoke | 红方块、蓝圆柱、绿球各 1 条，`max_attempts_per_episode:=1` | 3/3 一次通过 |
| 单目标小样本 | 每类目标 20 accepted episodes，`max_attempts_per_episode:=3` | 每类 `accepted / attempts >= 0.90`，无连续 3 次 rejected |
| 混合任务 soak | 不传 `target_object_name`，循环采 60 accepted episodes | 整体 `accepted / attempts >= 0.90`，任一类估算不低于 0.85 |
| 大批/过夜采集 | 500+ episodes 前 | 最近 60-100 条 soak `>= 0.95`，且 `validate_dataset.py` 全 PASS |

`batch_generator` 日志会打印 `Batch progress: X/Y accepted after Z attempts`，用 `X / Z` 估算当前窗口成功率。低于门槛时先调 `pick_height_offset`、`hover_height`、`lift_target_z`、夹爪接触/摩擦参数、筐坐标和 `bin_xy_tolerance`，不要硬开大批。

### 4.1 录制“抓取红方块放左箱”的任务：
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch teleop_bringup full_system.launch.py \
  use_sim:=true \
  headless:=true \
  record:=true \
  output_dir:=config/lerobot/datasets/raw/panda_pick_red \
  randomize:=true \
  watchdog_timeout:=2.0 &

python3 scripts/publish_dummy_heartbeat.py --rate 50 &
ros2 service call /safety/reset std_srvs/srv/Trigger "{}"

# 启动自动生成脚本，指定抓取目标、语言标签和录制数量
ros2 run synth_data_gen batch_generator \
  --ros-args \
  -p target_object_name:=object_red_box \
  -p language_instruction:="pick up the red box and place it in the left bin" \
  -p validation_mode:=place \
  -p episodes:=100

python3 scripts/validate_dataset.py config/lerobot/datasets/raw/panda_pick_red --min-frames 5
```

### 4.2 录制“抓取绿球放左箱”的任务：
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch teleop_bringup full_system.launch.py \
  use_sim:=true \
  headless:=true \
  record:=true \
  output_dir:=config/lerobot/datasets/raw/panda_pick_green \
  randomize:=true \
  watchdog_timeout:=2.0 &

python3 scripts/publish_dummy_heartbeat.py --rate 50 &
ros2 service call /safety/reset std_srvs/srv/Trigger "{}"

ros2 run synth_data_gen batch_generator \
  --ros-args \
  -p target_object_name:=object_green_sphere \
  -p language_instruction:="pick up the green sphere and place it in the left bin" \
  -p validation_mode:=place \
  -p episodes:=100

python3 scripts/validate_dataset.py config/lerobot/datasets/raw/panda_pick_green --min-frames 5
```

### 4.3 循环采集三类任务

不传 `target_object_name` 时，`batch_generator` 会按红方块、蓝圆柱、绿球循环采集：

```bash
ros2 run synth_data_gen batch_generator --ros-args \
  -p episodes:=300 \
  -p validation_mode:=place \
  -p max_attempts_per_episode:=5 \
  -p lift_success_delta:=0.02 \
  -p bin_xy_tolerance:=0.14

python3 scripts/validate_dataset.py config/lerobot/datasets/raw/panda_sorting --min-frames 5
```

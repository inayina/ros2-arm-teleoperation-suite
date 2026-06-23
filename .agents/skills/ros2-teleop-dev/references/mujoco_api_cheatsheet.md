# MuJoCo v3 API 速查（本项目常用）

## 模型加载

```python
import mujoco

model = mujoco.MjModel.from_xml_path("config/models/franka_panda.xml")
data  = mujoco.MjData(model)
```

## 关键数据字段

| 字段 | 含义 | 形状 |
|---|---|---|
| `data.qpos` | 关节位置（广义坐标） | `(nq,)` |
| `data.qvel` | 关节速度 | `(nv,)` |
| `data.ctrl` | actuator 控制输入（力矩/位置目标） | `(nu,)` |
| `data.qfrc_applied` | 外加广义力（直接施加到关节） | `(nv,)` |
| `data.xpos` | 所有 body 的世界坐标位置 | `(nbody, 3)` |
| `data.xquat` | 所有 body 的世界坐标四元数 | `(nbody, 4)` |
| `data.contact` | 接触信息数组 | — |

## 施加力矩（两种方式）

```python
# 方式1：通过 actuator ctrl（推荐，需 XML 中定义 actuator）
data.ctrl[joint_idx] = torque_value

# 方式2：直接施加广义力
data.qfrc_applied[joint_idx] = torque_value
```

## 传感器读取

```python
# 读取命名传感器（需在 XML <sensor> 块定义）
force  = data.sensor("ee_force_sensor").data.copy()   # (3,) N
torque = data.sensor("ee_torque_sensor").data.copy()  # (3,) N·m
```

## franka_panda.xml 传感器块（手动添加）

```xml
<worldbody>
  <!-- ... Panda body tree ... -->
  <site name="attachment_site" pos="0 0 0" size="0.01"
        body="hand"/>  <!-- 末端执行器 site -->
</worldbody>

<sensor>
  <force  name="ee_force_sensor"  site="attachment_site"/>
  <torque name="ee_torque_sensor" site="attachment_site"/>
</sensor>
```

## Viewer（非阻塞）

```python
import mujoco.viewer

# launch_passive：viewer 在子线程渲染，主线程保持控制权
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()

# 无头模式（无 viewer，适合 CI / headless server）
while rclpy.ok():
    mujoco.mj_step(model, data)
```

## 正运动学：末端位置查询

```python
# 获取 body 名为 "hand" 的世界坐标
body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hand")
ee_pos  = data.xpos[body_id].copy()    # (3,) xyz
ee_quat = data.xquat[body_id].copy()   # (4,) wxyz → 注意 MuJoCo 是 wxyz 顺序
```

## 关节名 → 索引映射（Franka Panda）

```python
JOINT_NAMES = [f"panda_joint{i}" for i in range(1, 8)]  # 1~7

def get_joint_idx(model, joint_name: str) -> int:
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
```

## 常见问题

| 问题 | 原因 | 解决 |
|---|---|---|
| `KeyError: sensor name not found` | XML 中未定义传感器 | 在 `<sensor>` 块添加 force/torque |
| 末端力矩为零 | `site` 位置不对 | 确认 site 绑定在夹爪 body 上 |
| viewer 弹出后立即崩溃 | 无显示器（headless） | 加 `MUJOCO_GL=osmesa` 或用 `--no-render` |
| 仿真发散（关节位置飞出） | 力矩过大或步长太长 | 检查 ctrl 单位，减小 K 矩阵 |

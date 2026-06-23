# SPEC M3: 笛卡尔阻抗控制器（C++ 核心）

**分支**：`feat/impedance-controller`  
**里程碑**：M3  
**预计用时**：Week 3-4（8 天）  
**负责模块**：`src/impedance_controller/`

---

## 1. 目标

用 C++17 实现**笛卡尔空间六维阻抗控制器**，作为系统控制核心：

1. 接收遥操作目标末端位姿（`/master_pose`）
2. 结合当前关节状态和末端接触力，计算关节力矩指令
3. 发布 `/joint_torque_cmd` 驱动 CAN Bridge → MuJoCo

---

## 2. 技术选型与理由

### 2.1 语言：C++17

| 方案 | 优点 | 缺点 | 结论 |
|---|---|---|---|
| **C++17** ✅ | 控制频率 ≥ 1kHz；直接体现 C++ 嵌入式开发能力 | 开发周期长 | **选用** |
| Python | 开发快 | GIL 限制并发；控制频率难保证 ≥ 500Hz | 否 |

**选用理由**：深兰岗位 JD 明确要求 C++ 机器人控制经验，用 C++ 实现阻抗控制器是核心展示点。

### 2.2 运动学库：KDL（Orocos Kinematics and Dynamics Library）

| 方案 | 优点 | 缺点 | 结论 |
|---|---|---|---|
| **KDL** ✅ | ROS2 生态原生支持；`orocos_kdl` apt 可装；API 成熟 | IK 求解器精度一般 | **选用** |
| Pinocchio | 精度高；支持 6D 雅可比 | 安装复杂；需 conda-forge | 备选（M6 可升级） |
| 自实现 DH 参数 | 无依赖 | 维护成本高；易出 bug | 否 |
| MoveIt2 | 功能完整 | 过度依赖；启动慢 | 否 |

**选用理由**：`ros-jazzy-kdl-parser` 一行 apt 可装，与 ROS2 话题无缝集成。`ChainIkSolverVel_pinv` 的速度级 IK 用于笛卡尔阻抗控制已足够精确。

### 2.3 线性代数：Eigen3

| 方案 | 缺点 | 结论 |
|---|---|---|
| **Eigen3** ✅ | 无（ROS2/KDL 标准配套） | **选用** |
| OpenBLAS | 配置复杂 | 否 |

### 2.4 并发模型：`MultiThreadedExecutor` + 两个 CallbackGroup

```cpp
// 控制回路：MutuallyExclusive（串行，保证力矩计算原子性）
cb_group_control_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

// 传感器回调：Reentrant（并发，force + joint_states 互不阻塞）
cb_group_sensor_  = create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);
```

**选用理由**：单线程 Executor 会让力传感器回调（100Hz）和控制计算（500Hz）互相阻塞，导致控制频率实际只有 ~50Hz。双 CallbackGroup + 多线程 Executor 是 ROS2 实时控制的标准方案。

---

## 3. 控制律详解

### 3.1 笛卡尔阻抗控制律

```
F_cmd = K * (x_d - x) + D * (ẋ_d - ẋ)
τ_cmd = J^T * F_cmd + τ_gravity
```

| 符号 | 含义 | 默认值 |
|---|---|---|
| `x_d` | 目标末端位姿（来自 `/master_pose`，7D：pos + quat） | — |
| `x` | 当前末端位姿（由 KDL 正运动学计算） | — |
| `K` | 刚度矩阵（6×6 对角） | `diag(500,500,500, 50,50,50)` [N/m, N·m/rad] |
| `D` | 阻尼矩阵（6×6 对角，按临界阻尼设计 D=2√K） | `diag(44.7,44.7,44.7, 14.1,14.1,14.1)` |
| `J` | 末端雅可比矩阵（由 KDL 实时计算） | — |
| `τ_gravity` | 重力补偿项（KDL ChainDynParam） | — |

### 3.2 位置误差计算（含旋转）

```cpp
// 平移误差（3D）
Eigen::Vector3d pos_error = x_d.head<3>() - x.head<3>();

// 旋转误差：四元数差 → 轴角（角速度方向）
Eigen::Quaterniond q_d(x_d[6], x_d[3], x_d[4], x_d[5]);
Eigen::Quaterniond q  (x [6], x [3], x [4], x [5]);
Eigen::Quaterniond q_err = q_d * q.inverse();
Eigen::AngleAxisd  aa_err(q_err);
Eigen::Vector3d    rot_error = aa_err.axis() * aa_err.angle();

// 合并为 6D 误差
Eigen::Matrix<double,6,1> error;
error.head<3>() = pos_error;
error.tail<3>() = rot_error;
```

### 3.3 接触力自适应刚度（柔顺模式）

```cpp
void update_stiffness(const geometry_msgs::msg::WrenchStamped & ft) {
    double fz = ft.wrench.force.z;
    if (std::abs(fz) > contact_threshold_) {  // 默认 5N
        // 接触方向（z 轴）刚度降低到正常值的 20%
        K_(2, 2) = K_nominal_(2, 2) * 0.2;
    } else {
        K_(2, 2) = K_nominal_(2, 2);
    }
}
```

---

## 4. 接口定义

### 4.1 订阅

| Topic | 类型 | CallbackGroup | 说明 |
|---|---|---|---|
| `/master_pose` | `geometry_msgs/PoseStamped` | control | 目标末端位姿 |
| `/joint_states` | `sensor_msgs/JointState` | sensor | 当前关节状态（来自 MuJoCo） |
| `/ft_sensor` | `geometry_msgs/WrenchStamped` | sensor | 末端接触力 |

### 4.2 发布

| Topic | 类型 | 频率 | 说明 |
|---|---|---|---|
| `/joint_torque_cmd` | `sensor_msgs/JointState` | 500Hz | 关节力矩指令（`effort` 字段） |

### 4.3 可配置参数（`controller_params.yaml`）

```yaml
impedance_controller:
  ros__parameters:
    stiffness: [500.0, 500.0, 500.0, 50.0, 50.0, 50.0]   # K 对角元素
    damping:   [44.7,  44.7,  44.7,  14.1, 14.1, 14.1]   # D 对角元素
    contact_threshold: 5.0       # N，超过此值切换柔顺模式
    control_freq: 500.0          # Hz，控制循环目标频率
    urdf_path: "config/franka_panda.urdf"
```

---

## 5. 文件清单

```
src/impedance_controller/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── impedance_controller/
│       ├── impedance_controller_node.hpp
│       └── panda_kinematics.hpp       ← KDL 封装
└── src/
    ├── impedance_controller_node.cpp
    └── panda_kinematics.cpp

config/
├── controller_params.yaml
└── franka_panda.urdf                  ← 从 ROS2 Franka 包导出

tests/
├── test_impedance_controller.cpp      ← GTest 单元测试
└── CMakeLists.txt
```

---

## 6. 验收标准

### 必须通过（阻塞合并）

| # | 验收项 | 验证方法 | 指标 |
|---|---|---|---|
| AC-1 | `colcon build --packages-select impedance_controller` 零 warning 通过 | CI / 本地构建 | 0 warnings |
| AC-2 | 控制节点启动，以 500Hz 发布 `/joint_torque_cmd`（`ros2 topic hz` 测量） | `ros2 topic hz` | 475–525Hz |
| AC-3 | 给定静止目标位姿，末端位置跟踪误差 < 2mm（在 MuJoCo 仿真中测量） | 正运动学计算 + `ros2 topic echo` | < 2mm |
| AC-4 | 末端受到 > 5N 外力时（通过 `/ft_sensor` 模拟），刚度 K(2,2) 自动降低（日志输出确认） | `ros2 run ... --ros-args --log-level DEBUG` | 日志出现 "compliant mode" |
| AC-5 | `tests/test_impedance_controller.cpp` GTest 全通过，包含：力矩计算正确性、边界值（奇异位形附近）、CallbackGroup 并发安全 | `colcon test` | 0 failures |
| AC-6 | 与 M2（MuJoCo 节点）集成运行 30 秒，无死锁、无力矩发散（`effort` 始终 < 100 N·m） | 联合启动 + 监控 | 稳定 |

### 加分项

- [ ] 控制频率在 MuJoCo 1kHz 步进下实测达到 ≥ 500Hz
- [ ] KDL IK 奇异位形检测：行列式 < 阈值时输出警告，不发布力矩
- [ ] 可通过 ROS2 服务 `/set_stiffness` 动态调整 K/D 矩阵（无需重启）

---

## 7. 面试话术关键点

> "阻抗控制器里有两个值得重点说的设计决策：  
>
> 第一，**旋转误差的表示方式**：用轴角（axis-angle）而不是欧拉角，避免了万向节死锁（Gimbal Lock）问题，这在末端做大幅旋转时有实际意义。  
>
> 第二，**并发架构**：传感器回调（力矩、关节状态）用 Reentrant CallbackGroup，控制计算用 MutuallyExclusive CallbackGroup，配合 MultiThreadedExecutor。这样 100Hz 的传感器更新不会被 500Hz 的控制计算阻塞，是 ROS2 实时控制的标准做法。"

# SPEC V2-M3: 笛卡尔阻抗控制器（ros2_control 插件）

**分支**：`feat/v2-impedance-controller`  
**依赖**：M2（`feat/v2-canopen-fieldbus` 已合入 main）  
**核心目标**：将 V1 的「独立阻抗控制器节点」改写为 `ros2_control` 控制器插件（`controller_interface::ControllerInterface`），在 controller_manager 的 1kHz 实时循环内执行笛卡尔阻抗控制律，并支持与 `joint_trajectory_controller` 热切换。  
**预计工作量**：8~10 天

> V1 对照：V1 中阻抗控制器是独立 ROS2 节点，通过 Topic 订阅/发布，控制频率受 DDS 调度影响。V2 改为 `controller_interface` 插件，`update()` 在 controller_manager 的 RT 线程中被调用，保证 1kHz 确定性执行。

---

## 1. 目标

1. 实现 `cartesian_impedance_controller` C++ 插件，经 `pluginlib` 注册到 controller_manager
2. `command_interfaces`：`<joint>/effort`（写力矩）；`state_interfaces`：`<joint>/position`、`<joint>/velocity`
3. 订阅 `/joint_target`（来自 M4 MoveIt Servo），计算笛卡尔阻抗控制律
4. 订阅 `/ft_sensor`（来自 MuJoCo），实现接触自适应刚度
5. 与 `joint_trajectory_controller` 可热切换（`ros2 control switch_controllers`）
6. GTest 覆盖控制律核心函数

---

## 2. 包清单（M3 引入/修改）

```
src/
├── teleop_controllers/             ← [NEW] L3 阻抗控制器（C++ ros2_control 插件）
│   ├── include/teleop_controllers/
│   │   ├── cartesian_impedance_controller.hpp   # 控制器主头文件
│   │   ├── impedance_math.hpp                    # 数学工具（解析式 FK、Jacobian、误差）
│   │   └── visibility_control.h                  # 跨平台符号导出宏（待添加）
│   ├── src/
│   │   ├── cartesian_impedance_controller.cpp   # 控制器实现
│   │   └── impedance_math.cpp                    # Eigen 解析式实现（见 §5.6）
│   ├── controllers_plugin.xml      # pluginlib 导出声明
│   ├── config/
│   │   └── impedance_params.yaml   # 刚度 K、阻尼 D、力矩限幅
│   ├── test/
│   │   ├── test_impedance_math.cpp       # GTest：FK/Jacobian 数值精度
│   │   └── test_impedance_controller.cpp # GTest：控制律正确性
│   ├── CMakeLists.txt
│   └── package.xml
│
└── teleop_bringup/
    └── config/
        └── controllers.yaml   # [MODIFY] 添加 cartesian_impedance_controller 和 jtc 条目
```

---

## 3. 控制律

### 3.1 笛卡尔阻抗控制律

$$
\boldsymbol{\tau} = \boldsymbol{J}^{\top} \bigl[ \boldsymbol{K}(\boldsymbol{x}_d - \boldsymbol{x}) + \boldsymbol{D}(\dot{\boldsymbol{x}}_d - \dot{\boldsymbol{x}}) \bigr] + \boldsymbol{g}(\boldsymbol{q})
$$

- $\boldsymbol{x}_d$：期望末端位姿（来自 `/joint_target` 经正运动学）
- $\boldsymbol{x}$：当前末端位姿（由 `state_interfaces` 关节角经 FK 计算）
- $\boldsymbol{K} \in \mathbb{R}^{6\times6}$：刚度矩阵（对角，translational/rotational 分离）
- $\boldsymbol{D} \in \mathbb{R}^{6\times6}$：阻尼矩阵（临界阻尼：$D = 2\sqrt{K}$）
- $\boldsymbol{J}$：雅可比矩阵（用 KDL 在线计算）
- $\boldsymbol{g}(\boldsymbol{q})$：重力补偿（MuJoCo `qfrc_bias` 真值，或 KDL 计算）

### 3.2 接触自适应刚度

```cpp
// 当末端接触力 > threshold 时，降低接触方向刚度（柔顺）
void adapt_stiffness(const Wrench & ft) {
    double fn = ft.force.norm();
    if (fn > contact_threshold_) {
        // 沿接触法向降刚度：K_contact = K_free * alpha
        double alpha = std::max(0.1, 1.0 - (fn - contact_threshold_) / stiffness_scale_);
        K_cartesian_.block<3,3>(0,0) *= alpha;   // 平动刚度
    }
}
```

### 3.3 方位误差（四元数）

```cpp
// 位置误差：Δx = x_d - x（直接相减）
Eigen::Vector3d delta_pos = x_desired.translation() - x_current.translation();

// 方位误差：使用四元数差值，避免万向锁
Eigen::Quaterniond q_d(x_desired.rotation());
Eigen::Quaterniond q_c(x_current.rotation());
Eigen::Quaterniond q_err = q_c.inverse() * q_d;
// 转为轴角误差（在当前坐标系下）
Eigen::AngleAxisd aa_err(q_err);
Eigen::Vector3d delta_rot = x_current.rotation() * (aa_err.angle() * aa_err.axis());

Eigen::VectorXd cart_error(6);
cart_error << delta_pos, delta_rot;
```

---

## 4. 接口定义

### 4.1 控制器订阅 Topic

| Topic | 类型 | 频率 | 说明 |
|---|---|---|---|
| `/joint_target` | `trajectory_msgs/JointTrajectory` | 125 Hz | 来自 MoveIt Servo（M4）；M3 阶段可用键盘直接发 |
| `/ft_sensor` | `geometry_msgs/WrenchStamped` | 100 Hz | 来自 MuJoCo，接触力触发柔顺 |
| `/safety/estop` | `std_msgs/Bool` | event | E-Stop：收到 `true` 后立即归零力矩 |

### 4.2 控制器发布 Topic

| Topic | 类型 | 频率 | 说明 |
|---|---|---|---|
| `/cartesian_impedance_controller/status` | `control_msgs/JointControllerState` | 100 Hz | 跟踪误差监控 |

### 4.3 `impedance_params.yaml`

```yaml
cartesian_impedance_controller:
  ros__parameters:
    joints:
      - panda_joint1
      - panda_joint2
      - panda_joint3
      - panda_joint4
      - panda_joint5
      - panda_joint6
      - panda_joint7

    # 笛卡尔刚度（[Tx Ty Tz Rx Ry Rz]，单位 N/m 和 N·m/rad）
    cartesian_stiffness: [200.0, 200.0, 200.0, 10.0, 10.0, 10.0]

    # 阻尼（临界阻尼 = 2*sqrt(K)，可超调时减小）
    cartesian_damping: [28.3, 28.3, 28.3, 6.3, 6.3, 6.3]

    # 接触柔顺
    contact_threshold_n: 5.0     # 接触力阈值（N），超过此值降刚度
    stiffness_scale: 50.0        # 刚度衰减系数

    # 安全限幅
    max_torque_nm: [87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0]  # Panda 各关节
    max_cartesian_error_m: 0.1   # 超过此误差截断（防发散）
```

### 4.4 控制器热切换命令

```bash
# 激活阻抗控制器（停用 jtc）
ros2 control switch_controllers \
  --deactivate joint_trajectory_controller \
  --activate cartesian_impedance_controller \
  --strict

# 切回轨迹控制器（停用阻抗）
ros2 control switch_controllers \
  --deactivate cartesian_impedance_controller \
  --activate joint_trajectory_controller \
  --strict
```

---

## 5. 关键实现细节

### 5.1 控制器生命周期（`controller_interface` 规范）

```cpp
class CartesianImpedanceController
    : public controller_interface::ControllerInterface
{
public:
  // 生命周期回调（按 ROS2 controller lifecycle 顺序调用）
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::CallbackReturn
  on_init() override;  // 参数声明、KDL 树加载

  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;  // 订阅建立、参数读取

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override;  // 记录初始位姿

  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;  // 归零力矩

  controller_interface::return_type
  update(const rclcpp::Time &, const rclcpp::Duration &) override;  // 1kHz 控制律
};
```

### 5.2 `update()` 核心流程（1kHz，< 1ms 执行时间要求）

```cpp
controller_interface::return_type
CartesianImpedanceController::update(
    const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // 1. 读当前关节角/速度（来自 state_interfaces，零拷贝）
  for (size_t i = 0; i < 7; ++i) {
    q_[i]  = state_interfaces_[i * 2    ].get_value();  // position
    dq_[i] = state_interfaces_[i * 2 + 1].get_value();  // velocity
  }

  // 2. E-Stop 检查（最高优先级）
  if (estop_active_.load()) {
    set_zero_torque();
    return controller_interface::return_type::OK;
  }

  // 3. 正运动学：q → x_current（KDL）
  KDL::Frame x_current;
  fk_solver_->JntToCart(q_kdl_, x_current);

  // 4. 获取期望位姿（来自 /joint_target，经 FK 转换）
  KDL::Frame x_desired;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    x_desired = x_desired_;
  }

  // 5. 计算笛卡尔误差（位置 + 方位四元数误差）
  Eigen::VectorXd cart_error = compute_cartesian_error(x_current, x_desired);

  // 6. 接触力自适应刚度
  Wrench ft_current;
  {
    std::lock_guard<std::mutex> lock(ft_mutex_);
    ft_current = ft_current_;
  }
  adapt_stiffness(ft_current);

  // 7. 雅可比矩阵（KDL）
  KDL::Jacobian J(7);
  jac_solver_->JntToJac(q_kdl_, J);

  // 8. 阻抗控制律：τ = J^T * [K*Δx + D*Δẋ] + g(q)
  Eigen::VectorXd tau = J.data.transpose() * (K_ * cart_error + D_ * cart_vel_error)
                        + gravity_compensation();

  // 9. 力矩限幅 + 写 command_interfaces
  for (size_t i = 0; i < 7; ++i) {
    tau[i] = std::clamp(tau[i], -max_torque_[i], max_torque_[i]);
    command_interfaces_[i].set_value(tau[i]);
  }

  return controller_interface::return_type::OK;
}
```

### 5.3 `pluginlib` 导出（`controllers_plugin.xml`）

```xml
<library path="teleop_controllers">
  <class
    name="teleop_controllers/CartesianImpedanceController"
    type="teleop_controllers::CartesianImpedanceController"
    base_class_type="controller_interface::ControllerInterface">
    <description>
      Cartesian impedance controller for Franka Panda.
      Implements τ = Jᵀ[K(x_d - x) + D(ẋ_d - ẋ)] + g(q).
    </description>
  </class>
</library>
```

### 5.4 CallbackGroup 设计（防止传感器回调阻塞控制律）

```cpp
// on_configure() 中
cb_group_sensor_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);

auto ft_options = rclcpp::SubscriptionOptions();
ft_options.callback_group = cb_group_sensor_;

// /ft_sensor 使用 Reentrant（允许并发），不阻塞 update()
ft_sub_ = node_->create_subscription<WrenchStamped>(
    "/ft_sensor", rclcpp::SensorDataQoS(),
    [this](WrenchStamped::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(ft_mutex_);
      ft_current_ = msg->wrench;
    }, ft_options);
```

### 5.5 `controllers.yaml` 新增条目（实际文件名）

```yaml
cartesian_impedance_controller:
  type: teleop_controllers/CartesianImpedanceController

joint_trajectory_controller:
  type: joint_trajectory_controller/JointTrajectoryController
  ros__parameters:
    joints:
      - panda_joint1
      # ... panda_joint2 ~ 7
    command_interfaces: [effort]
    state_interfaces: [position, velocity]
```

### 5.6 设计决策：Eigen 解析式 FK/Jacobian（不使用 KDL）

> **决策日期**：2026-06-24（M3 实施阶段）

原 SPEC 草稿提及用 KDL 计算 Jacobian。实际实现**改为 Eigen 手写 Panda 解析式 FK 和几何 Jacobian**。

**原因**：

| 因素 | KDL 方案 | Eigen 解析式方案（已采用） |
|---|---|---|
| 依赖 | `ros-jazzy-orocos-kdl`（额外 apt 包） | 无额外依赖（Eigen 已是 ros2_control 基础依赖） |
| 性能 | 运行时 URDF 树遍历 | 编译时已知 DH 参数，无分支 |
| 可测试性 | 需 KDL 运行时环境 | 纯 Eigen，GTest 直接链接无 ROS |
| 精度 | 数值与解析式相同 | 可与数值微分对比验证 |
| 维护 | KDL API 变化有风险 | Panda DH 参数固定，不会变化 |

**实现文件**：
- `include/teleop_controllers/impedance_math.hpp`：FK/Jacobian/误差函数声明
- `src/impedance_math.cpp`：修正 DH 参数（Franka Emika Technical Spec Table 4），实现解析式几何 Jacobian
- `test/test_impedance_math.cpp`：零位 FK 参考值验证 + 数值微分 Jacobian 对比（误差 < 1e-4 rad）

**重力补偿（M3 阶段）**：`g(q) = 0`。MuJoCo 在物理引擎层面已补偿重力，M3 不需额外重力模型。M4 阶段如接入真实硬件可引入 KDL 重力项或 Pinocchio。

---

## 6. 验收标准

### 必须通过（阻塞合并至 main）

| # | 验收项 | 验证命令 |
|---|---|---|
| AC-1 | `cartesian_impedance_controller` 经 pluginlib 找到并被 controller_manager 加载 | `ros2 control list_controllers` 显示 `active` |
| AC-2 | `command_interface` = `effort`；`state_interface` = `position`/`velocity` | `ros2 control list_hardware_interfaces` |
| AC-3 | 给定 `/joint_target`，末端位姿跟踪误差 < 2mm（在 MuJoCo viewer 中确认） | `ros2 topic echo /cartesian_impedance_controller/status` |
| AC-4 | 接触力 > 5N 时刚度自动降低（柔顺），Panda 末端可被推开 | 在 MuJoCo viewer 中施加外力 |
| AC-5 | `update()` 在 1kHz 下稳定执行，`ros2 topic hz /joint_states` ≥ 950 Hz | `ros2 topic hz /joint_states` |
| AC-6 | `ros2 control switch_controllers` 热切换到 `joint_trajectory_controller` 再切回，无崩溃 | 按验收命令执行 |
| AC-7 | `test_impedance_controller.cpp` GTest 通过 | `colcon test --packages-select teleop_controllers` |
| AC-8 | E-Stop 收到后力矩立即归零（< 1ms） | 发布 `/safety/estop: true` 后 `candump vcan0` 观察 RPDO 归零 |

### 加分项

- [ ] 关节奇异点附近（Jacobian 行列式 < 阈值）自动降刚度/减速
- [ ] 参数可运行时热修改（`ros2 param set /cartesian_impedance_controller cartesian_stiffness ...`）
- [ ] Valgrind 无内存泄漏（控制律部分）

---

## 7. 常用调试命令

```bash
# 构建阻抗控制器
colcon build --packages-select teleop_controllers
source install/setup.bash

# 验证 pluginlib 发现
ros2 run pluginlib_tutorials_interfaces pluginlib_list | grep CartesianImpedance

# 控制器状态
ros2 control list_controllers
ros2 control list_hardware_interfaces

# 热切换（impedance ↔ jtc）
ros2 control switch_controllers \
  --deactivate cartesian_impedance_controller \
  --activate joint_trajectory_controller --strict

# 测试跟踪（直接发 /joint_target，M4 前临时）
ros2 topic pub /joint_target trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [panda_joint1,...], points: [{positions: [0,0,0,0,0,0,0], time_from_start: {sec: 1}}]}"

# 跟踪误差监控
ros2 topic echo /cartesian_impedance_controller/status

# 运行 GTest
colcon test --packages-select teleop_controllers
colcon test-result --verbose
```

---

## 8. 关键风险与应对

| 风险 | 应对 |
|---|---|
| ~~KDL Jacobian 在 1kHz 内计算超时~~ | **已解决**：改用 Eigen 解析式 Jacobian（见 §5.6），无 KDL 依赖，运行时零树遍历开销 |
| 方位误差四元数翻转（q 和 -q 等价） | `cartesian_error()` 中检测 `q_cur.dot(q_des) < 0` 时翻转 q_des 符号，强制最短路径 |
| 刚度过大导致 Panda 振荡 | 先用小刚度（K=[50,50,50,5,5,5]）验证稳定性，再逐步增大 |
| pluginlib 找不到插件（ament_index 问题） | 确认 `package.xml` 中 `<export><build_type>` 和 `controllers_plugin.xml` 路径正确 |

---

*本文件为 V2-M3 细化 SPEC；架构基线见 [`ARCHITECTURE_V2.md`](./ARCHITECTURE_V2.md) §6.3，里程碑总览见 [`ROADMAP.md`](./ROADMAP.md)。*

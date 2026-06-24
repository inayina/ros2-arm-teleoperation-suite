# SPEC V2-M5: 安全层 + E-Stop 闭环

**分支**：`feat/v2-safety-layer`  
**依赖**：M4（`feat/v2-motion-layer` 已合入 main）  
**核心目标**：实现工业级安全监督节点 `safety_monitor`（C++），包含 5 个子监视器、可锁存 E-Stop、DS402 Quick Stop 闭环联动，通过 GTest 逐项验证，并在 rqt_robot_monitor 中可视化诊断。  
**预计工作量**：7~9 天

> V1 对照：V1 无独立安全层，Teleop 指令直达控制器。V2 的 `safety_monitor` 是数据流上的**强制串联节点**——所有 Teleop 指令必须经过安全层，全部检查通过才能输出 `/safe_master_pose`，否则拒绝并保持安全位姿。M4 阶段的临时「直通」在本里程碑被真实安全层替换。

---

## 1. 目标

1. 实现 `safety_monitor` C++ 节点，含 5 个子监视器（JointLimit / Workspace / Velocity / CommWatchdog / EStopManager）
2. 替换 M4 中 `teleop_input_node` 的「临时直通」，激活真实安全过滤路径
3. E-Stop → DS402 Quick Stop 闭环联动（`/safety/estop → canopen_hw_interface`）
4. `/safety/trigger_estop` / `/safety/reset` 服务实现
5. GTest 覆盖 5 个监视器逐项验证
6. `diagnostic_aggregator` + rqt_robot_monitor 可视化

---

## 2. 包清单（M5 引入/修改）

```
src/
├── safety_monitor/                 ← [NEW] L1 安全层（C++）
│   ├── include/safety_monitor/
│   │   ├── safety_monitor_node.hpp          # 主节点头文件
│   │   ├── joint_limit_monitor.hpp          # 子监视器：关节限位
│   │   ├── workspace_limit_monitor.hpp      # 子监视器：工作空间限位
│   │   ├── velocity_limit_monitor.hpp       # 子监视器：速度限位
│   │   ├── comm_watchdog.hpp                # 子监视器：通信看门狗
│   │   └── estop_manager.hpp                # E-Stop 管理（latch + 复位）
│   ├── src/
│   │   ├── safety_monitor_node.cpp
│   │   ├── joint_limit_monitor.cpp
│   │   ├── workspace_limit_monitor.cpp
│   │   ├── velocity_limit_monitor.cpp
│   │   ├── comm_watchdog.cpp
│   │   └── estop_manager.cpp
│   ├── config/
│   │   └── safety_limits.yaml      # 关节软限位、工作空间包络、速度限幅、超时阈值
│   ├── test/
│   │   └── test_safety_monitor.cpp  # GTest：5 个监视器逐项
│   ├── CMakeLists.txt
│   └── package.xml
│
└── teleop_bringup/
    └── launch/
        └── safety.launch.py        # [NEW] safety_monitor + diagnostic_aggregator
```

---

## 3. 接口定义

### 3.1 `safety_monitor` 订阅话题

| Topic | 类型 | QoS | 用途 |
|---|---|---|---|
| `/teleop/cmd_pose` | `geometry_msgs/PoseStamped` | Best Effort | 待检查的遥操作指令输入 |
| `/teleop/heartbeat` | `std_msgs/Header` | Reliable | CommWatchdog 活跃性检测 |
| `/joint_states` | `sensor_msgs/JointState` | Best Effort | 当前关节角用于限位检查 |

### 3.2 `safety_monitor` 发布话题

| Topic | 类型 | 频率 | QoS | 说明 |
|---|---|---|---|---|
| `/safe_master_pose` | `geometry_msgs/PoseStamped` | 100 Hz | Reliable | 仅全部检查通过时发布 |
| `/safety/estop` | `std_msgs/Bool` | event | Reliable + **Transient Local** | E-Stop 状态（latch，订阅方自动获得最新值） |
| `/safety/status` | `teleop_interfaces/SafetyStatus` | 50 Hz | Reliable | 详细安全状态（供 Recorder 和 rqt） |
| `/safety/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 10 Hz | Reliable | 5 个监视器诊断条目 |

### 3.3 服务接口

| 服务 | 类型 | 说明 |
|---|---|---|
| `/safety/trigger_estop` | `std_srvs/Trigger` | 手动触发急停（外部调用，如 GUI 急停按钮） |
| `/safety/reset` | `std_srvs/Trigger` | 急停复位（需先排除故障；EStop latch 解除） |

### 3.4 `safety_limits.yaml`

```yaml
safety_monitor:
  ros__parameters:
    # JointLimitMonitor：Panda 软限位（rad）+ 安全裕度
    joint_limits:
      panda_joint1: { lower: -2.7, upper:  2.7, margin: 0.1 }
      panda_joint2: { lower: -1.7, upper:  1.7, margin: 0.1 }
      panda_joint3: { lower: -2.8, upper:  2.8, margin: 0.1 }
      panda_joint4: { lower: -3.0, upper: -0.1, margin: 0.1 }
      panda_joint5: { lower: -2.8, upper:  2.8, margin: 0.1 }
      panda_joint6: { lower: -0.1, upper:  3.7, margin: 0.1 }
      panda_joint7: { lower: -2.8, upper:  2.8, margin: 0.1 }

    # WorkspaceLimitMonitor：末端 Cartesian 包络（m）
    workspace:
      type: box   # box / cylinder（可选）
      x_min: -0.8
      x_max:  0.8
      y_min: -0.8
      y_max:  0.8
      z_min:  0.05   # 高于桌面
      z_max:  1.2

    # VelocityLimitMonitor
    velocity:
      max_joint_velocity_radps: 2.0   # 关节速度上限
      max_cartesian_velocity_mps: 0.5  # 末端线速度上限（钳位阈值）
      estop_cartesian_velocity_mps: 1.5  # 超过此值触发 E-Stop

    # CommWatchdog
    watchdog:
      heartbeat_timeout_ms: 100   # 心跳超时阈值（触发 E-Stop）
      joint_states_timeout_ms: 200  # 关节状态超时阈值

    # 诊断发布频率
    diagnostics_rate_hz: 10
    status_rate_hz: 50
```

---

## 4. 安全监视器架构

```
                  ┌────────────────────────── safety_monitor ────────────────────────────┐
/teleop/cmd_pose  │  ┌──────────────────┐  ┌──────────────────┐  ┌───────────────────┐  │
──────────────────▶  │ JointLimit       │  │ Workspace        │  │ Velocity          │  │──▶ /safe_master_pose
/joint_states     │  │ Monitor          │  │ LimitMonitor     │  │ LimitMonitor      │  │   (仅全部 PASS)
──────────────────▶  │ (软限位 + margin)│  │ (box/cylinder包络)│  │ (钳位/E-Stop)    │  │
                  │  └────────┬─────────┘  └────────┬─────────┘  └─────────┬─────────┘  │
                  │           └───────────────────────┼───────────────────────┘           │──▶ /safety/estop
                  │  ┌──────────────────┐   ┌─────────▼──────────┐                        │──▶ /safety/status
/teleop/heartbeat │  │ CommWatchdog     │──▶│ EStopManager       │                        │──▶ /safety/diagnostics
──────────────────▶  │ (100ms 超时)     │   │ (latch + 复位逻辑) │◀── /safety/trigger_estop│
                  │  └──────────────────┘   └────────────────────┘◀── /safety/reset        │
                  └───────────────────────────────────────────────────────────────────────┘
```

---

## 5. 关键实现细节

### 5.1 `safety_monitor_node.cpp` 主循环（250 Hz MultiThreaded Executor）

```cpp
class SafetyMonitorNode : public rclcpp::Node {
public:
  SafetyMonitorNode() : Node("safety_monitor") {
    // 子监视器初始化
    joint_limit_   = std::make_unique<JointLimitMonitor>(this);
    workspace_     = std::make_unique<WorkspaceLimitMonitor>(this);
    velocity_      = std::make_unique<VelocityLimitMonitor>(this);
    comm_watchdog_ = std::make_unique<CommWatchdog>(this);
    estop_mgr_     = std::make_unique<EStopManager>(this);

    // 订阅
    cmd_sub_ = create_subscription<PoseStamped>(
        "/teleop/cmd_pose", rclcpp::SensorDataQoS(),
        std::bind(&SafetyMonitorNode::on_cmd_pose, this, _1));

    joint_sub_ = create_subscription<JointState>(
        "/joint_states", rclcpp::SensorDataQoS(),
        std::bind(&SafetyMonitorNode::on_joint_states, this, _1));

    hb_sub_ = create_subscription<Header>(
        "/teleop/heartbeat", rclcpp::QoS(10).reliable(),
        std::bind(&SafetyMonitorNode::on_heartbeat, this, _1));

    // 发布
    safe_pub_   = create_publisher<PoseStamped>("/safe_master_pose", rclcpp::QoS(10).reliable());
    estop_pub_  = create_publisher<Bool>("/safety/estop",
        rclcpp::QoS(10).reliable().transient_local());   // Transient Local！

    // 服务
    reset_srv_ = create_service<Trigger>(
        "/safety/reset",
        std::bind(&SafetyMonitorNode::on_reset, this, _1, _2));
  }

  void on_cmd_pose(const PoseStamped::SharedPtr msg) {
    // 1. E-Stop 检查（最高优先级）
    if (estop_mgr_->is_active()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "E-Stop active, rejecting command");
      return;
    }

    // 2. 逐项监视器检查
    bool pass = true;
    pass &= joint_limit_->check(last_joint_states_);
    pass &= workspace_->check(msg->pose);
    pass &= velocity_->check(msg->pose, last_safe_pose_, msg->header.stamp);

    if (pass) {
      last_safe_pose_ = msg->pose;
      safe_pub_->publish(*msg);
    } else {
      // 拒绝：重发上一安全位姿（保持当前位置）
      PoseStamped safe_msg = *msg;
      safe_msg.pose = last_safe_pose_;
      safe_pub_->publish(safe_msg);
    }
  }
};
```

### 5.2 `CommWatchdog`（100ms 超时 → E-Stop）

```cpp
class CommWatchdog {
public:
  void on_heartbeat(const Header & hb) {
    last_hb_time_ = rclcpp::Time(hb.stamp);
  }

  bool check(const rclcpp::Time & now) {
    auto dt_ms = (now - last_hb_time_).nanoseconds() / 1e6;
    if (dt_ms > timeout_ms_) {
      RCLCPP_ERROR(logger_, "Heartbeat timeout: %.1f ms > %d ms", dt_ms, timeout_ms_);
      return false;  // 触发 E-Stop
    }
    return true;
  }

private:
  rclcpp::Time last_hb_time_;
  int timeout_ms_{100};
};
```

### 5.3 `EStopManager`（Latch + Quick Stop 联动）

```cpp
class EStopManager {
public:
  void trigger(const std::string & reason) {
    if (!estop_active_.load()) {
      RCLCPP_FATAL(logger_, "E-STOP TRIGGERED: %s", reason.c_str());
      estop_active_.store(true);

      // 发布 E-Stop（Transient Local，确保 canopen_hw_interface 收到）
      auto msg = std_msgs::msg::Bool();
      msg.data = true;
      estop_pub_->publish(msg);

      // 记录到诊断
      diagnostics_.add("EStop", diagnostic_msgs::msg::DiagnosticStatus::ERROR, reason);
    }
  }

  void reset(const std::string & /*operator_id*/) {
    if (estop_active_.load()) {
      // 安全确认后复位
      estop_active_.store(false);
      auto msg = std_msgs::msg::Bool();
      msg.data = false;
      estop_pub_->publish(msg);
      RCLCPP_INFO(logger_, "E-Stop reset by operator");
    }
  }

  bool is_active() const { return estop_active_.load(); }

private:
  std::atomic<bool> estop_active_{false};
};
```

### 5.4 `canopen_hw_interface` E-Stop 响应

```cpp
// canopen_system.cpp 中订阅 /safety/estop
estop_sub_ = node_->create_subscription<Bool>(
    "/safety/estop",
    rclcpp::QoS(10).reliable().transient_local(),   // 匹配 Transient Local
    [this](Bool::SharedPtr msg) {
      if (msg->data) {
        // 发送 DS402 Quick Stop controlword（0x0002）到所有驱动器
        for (int id = 1; id <= 7; ++id) {
          send_sdo_write(id, 0x6040, 0x0002);   // Quick Stop
        }
        RCLCPP_WARN(logger_, "DS402 Quick Stop issued to all drives");
      }
    });
```

### 5.5 GTest 测试设计（`test_safety_monitor.cpp`）

```cpp
// 测试 1：JointLimitMonitor — 越限指令被拒
TEST(JointLimitMonitor, RejectsOutOfRange) {
  JointLimitMonitor monitor;
  monitor.set_limits("panda_joint1", -2.7, 2.7, 0.1);
  JointState js;
  js.name = {"panda_joint1"};
  js.position = {3.0};   // 超出 2.7 + 0.1
  EXPECT_FALSE(monitor.check(js));
}

// 测试 2：CommWatchdog — 100ms 超时触发
TEST(CommWatchdog, TriggersOnTimeout) {
  CommWatchdog wd;
  wd.set_timeout(100);
  Header hb;
  hb.stamp.sec = 0;
  wd.on_heartbeat(hb);
  rclcpp::Time now(200'000'000LL);   // 200ms 后
  EXPECT_FALSE(wd.check(now));
}

// 测试 3：WorkspaceLimitMonitor — 指令钳位到边界
TEST(WorkspaceLimitMonitor, ClampsToBox) {
  WorkspaceLimitMonitor wsm;
  wsm.set_box({-0.8, 0.8}, {-0.8, 0.8}, {0.05, 1.2});
  Pose pose;
  pose.position.x = 1.5;   // 超出 0.8
  EXPECT_FALSE(wsm.check(pose));
}

// 测试 4：VelocityLimitMonitor — 速度超限触发 E-Stop
TEST(VelocityLimitMonitor, EstopOnHighVelocity) { /* ... */ }

// 测试 5：EStopManager — latch + 复位
TEST(EStopManager, LatchAndReset) {
  EStopManager mgr;
  mgr.trigger("test");
  EXPECT_TRUE(mgr.is_active());
  mgr.reset("operator");
  EXPECT_FALSE(mgr.is_active());
}
```

---

## 6. 验收标准

### 必须通过（阻塞合并至 main）

| # | 验收项 | 验证方法 |
|---|---|---|
| AC-1 | `test_safety_monitor.cpp` GTest 5 个监视器逐项通过 | `colcon test --packages-select safety_monitor` |
| AC-2 | 越限关节指令被拒，`/safe_master_pose` 保持上一安全位姿 | `ros2 topic pub /teleop/cmd_pose` 发越限指令 + `ros2 topic echo /safe_master_pose` |
| AC-3 | 工作空间越界指令被拒 | 发 x=1.5m 指令，Panda 不移动 |
| AC-4 | 心跳超时 100ms → `/safety/estop: true` → `candump vcan0` 抓到 DS402 Quick Stop SDO 帧 | 停止 `teleop_input_node` 后等待 > 100ms |
| AC-5 | `/safety/reset` 服务调用后 E-Stop 解除，Panda 可再次接受指令 | `ros2 service call /safety/reset std_srvs/srv/Trigger {}` |
| AC-6 | `/safety/trigger_estop` 服务可手动触发 E-Stop | `ros2 service call /safety/trigger_estop std_srvs/srv/Trigger {}` |
| AC-7 | `/safety/diagnostics` 在 rqt_robot_monitor 中可视化（5 个条目，正常时全绿） | `ros2 run rqt_robot_monitor rqt_robot_monitor` |
| AC-8 | E-Stop 后 `virtual_servo_driver` 进入 `Quick Stop Active`，`/servo_drive/status` 更新 | `ros2 topic echo /servo_drive/status` |

### 加分项

- [ ] `/safety/status` 在 rqt 中绘制时序图（看门狗/E-Stop 事件时间轴）
- [ ] 速度钳位（轻度越速钳位而非立即 E-Stop）实现
- [ ] E-Stop 历史日志写入文件（用于事故回溯）

---

## 7. 常用调试命令

```bash
# 构建安全层
colcon build --packages-select safety_monitor
colcon test --packages-select safety_monitor
colcon test-result --verbose
source install/setup.bash

# 启动安全层（含 diagnostic_aggregator）
ros2 launch teleop_bringup safety.launch.py

# 查看安全状态
ros2 topic echo /safety/status
ros2 topic echo /safety/estop

# 手动触发/复位 E-Stop
ros2 service call /safety/trigger_estop std_srvs/srv/Trigger "{}"
ros2 service call /safety/reset std_srvs/srv/Trigger "{}"

# 模拟心跳超时（停掉 teleop_input 后等 >100ms）
ros2 run teleop_input teleop_input_node &
sleep 0.2
kill %1   # 停掉 teleop_input，触发看门狗
ros2 topic echo /safety/estop --once

# rqt 诊断可视化
ros2 run rqt_robot_monitor rqt_robot_monitor

# 全链路（含安全层）启动
ros2 launch teleop_bringup full_system.launch.py
```

---

## 8. 关键风险与应对

| 风险 | 应对 |
|---|---|
| E-Stop Transient Local QoS 与 canopen_hw_interface 订阅不匹配导致未收到 | 两端均设置 `Reliable + Transient Local`，启动后用 `ros2 topic info --verbose /safety/estop` 确认 |
| 安全检查延迟影响 `/safe_master_pose` 发布频率 | 监视器用无锁原子变量缓存最新状态，on_cmd_pose 回调中同步检查，不引入额外等待 |
| GTest 中 rclcpp 初始化问题 | 单测直接测监视器纯逻辑（不依赖 Node），避免 rclcpp init/shutdown 复杂性 |
| `quick stop` SDO 下发时 canopen_hw_interface RT 循环阻塞 | SDO 写在独立线程，不阻塞 `write()` 实时路径 |

---

*本文件为 V2-M5 细化 SPEC；架构基线见 [`ARCHITECTURE_V2.md`](./ARCHITECTURE_V2.md) §6.1，里程碑总览见 [`ROADMAP.md`](./ROADMAP.md)。*

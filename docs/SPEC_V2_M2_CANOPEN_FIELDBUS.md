# SPEC V2-M2: CANopen DS402 现场总线 + 虚拟伺服驱动器

**分支**：`feat/v2-canopen-fieldbus`  
**依赖**：M1（`feat/v2-control-skeleton` 已合入 main）  
**核心目标**：将 M1 的「直连 sim」模式升级为真实 CANopen DS402 总线链路，引入 `virtual_servo_driver` 仿真 7 轴伺服驱动器，建立「控制器 → CAN 总线 → 驱动器 → 物理引擎」的完整现场总线链路。  
**预计工作量**：6~8 天

> V1 对照：V1 中 `can_bridge` 只是 ROS2 ↔ CAN 帧的简单桥接，CANopen 状态机不完整。V2 中 DS402 状态机、PDO/SDO/NMT/EMCY 全部实现，`virtual_servo_driver` 严格模拟工业伺服驱动器行为。

---

## 1. 目标

1. 建立 `vcan0` 虚拟 CAN 总线（`setup_vcan.sh`）
2. 实现 `virtual_servo_driver`（Python，每关节独立进程）：DS402 状态机、PDO 收发、SDO 对象字典、故障注入
3. 升级 `canopen_hw_interface`：M2 模式下走真实 SocketCAN，write→RPDO、read←TPDO
4. 通过 `forward_command_controller` 经 CAN 驱动 Panda 完整运动
5. 故障注入测试：过流/超速 → EMCY 帧 + `Fault` 态

---

## 2. 包清单（M2 引入/修改）

```
src/
├── virtual_servo_driver/           ← [NEW] L4 DS402 伺服驱动器仿真（Python）
│   ├── virtual_servo_driver/
│   │   ├── __init__.py
│   │   ├── driver_node.py          # ROS2 节点入口，每关节一个实例
│   │   ├── ds402_state_machine.py  # DS402 完整状态机（NMT + 控制字/状态字）
│   │   ├── pdo_codec.py            # RPDO/TPDO 编解码（力矩/位置/速度）
│   │   ├── sdo_server.py           # CANopen 对象字典（OD）+ SDO 服务器
│   │   └── current_loop.py         # 一阶力矩/电流环 + 故障注入
│   ├── config/
│   │   └── servo_config.yaml       # 每轴配置（node_id、限幅、故障阈值）
│   ├── package.xml
│   └── setup.py
│
├── canopen_hw_interface/           ← [MODIFY] 升级为真实 CAN 模式
│   ├── src/
│   │   └── canopen_system.cpp      # write(): RPDO; read(): TPDO; on_activate(): SDO 配置 + NMT
│   └── config/
│       └── can_config.yaml         # sim_direct: false, interface: vcan0
│
├── gripper_driver/                 ← [NEW] RS485 Modbus 夹爪驱动（Python）
│   └── gripper_driver/
│       └── gripper_modbus_node.py
│
└── teleop_bringup/
    └── launch/
        └── fieldbus.launch.py      # [NEW] vcan0 setup + virtual_servo_driver ×7

scripts/
└── setup_vcan.sh                   # [MODIFY] 完整版（含 SYNC 帧定时器）

tests/
├── test_ds402_state_machine.py     # [NEW] DS402 状态机单测
└── test_pdo_codec.py               # [NEW] PDO 编解码单测
```

---

## 3. CANopen DS402 协议详解

### 3.1 CAN 帧 ID 分配（7 关节，node_id = 1~7）

| 帧类型 | CAN ID | 方向 | 触发 | 说明 |
|---|---|---|---|---|
| **RPDO1**（力矩指令） | `0x200 + node_id` | CM → Driver | SYNC | 目标力矩（DS402 `0x6071`） |
| **TPDO1**（位置/速度反馈） | `0x180 + node_id` | Driver → CM | SYNC | 实际位置 `0x6064` + 速度 `0x606C` |
| **TPDO2**（状态/力矩反馈） | `0x280 + node_id` | Driver → CM | SYNC | 状态字 `0x6041` + 实际力矩 `0x6077` |
| **SDO Request** | `0x600 + node_id` | CM → Driver | 按需 | 对象字典读写 |
| **SDO Response** | `0x580 + node_id` | Driver → CM | 按需 | SDO 响应 |
| **NMT** | `0x000` | CM → Driver | 按需 | `0x01 node_id`=启动；`0x02`=停止；`0x80`=预操作 |
| **SYNC** | `0x080` | CM 广播 | 1kHz | 触发 PDO 同步传输 |
| **EMCY** | `0x080 + node_id` | Driver → CM | 故障 | 急停/故障码广播 |
| **Heartbeat** | `0x700 + node_id` | Driver → CM | 1 Hz | 驱动器在线检测 |

### 3.2 DS402 状态机（`ds402_state_machine.py`）

```
                ┌─────────────────────────────────┐
                │   Not Ready to Switch On          │
                │   (上电初始化)                     │
                └──────────────┬──────────────────┘
                               │ 自动
                               ▼
                ┌─────────────────────────────────┐
                │   Switch On Disabled              │ ← NMT Operational
                │   Statusword bit[6]=1             │
                └──────────────┬──────────────────┘
                               │ Controlword=0x0006 (Shutdown)
                               ▼
                ┌─────────────────────────────────┐
                │   Ready to Switch On              │
                │   Statusword bit[0]=1             │
                └──────────────┬──────────────────┘
                               │ Controlword=0x0007 (Switch On)
                               ▼
                ┌─────────────────────────────────┐
                │   Switched On                     │
                │   Statusword bit[1]=1             │
                └──────────────┬──────────────────┘
                               │ Controlword=0x000F (Enable Operation)
                               ▼
                ┌─────────────────────────────────┐
    ┌──────────▶│   Operation Enabled               │◀──── 正常运行状态
    │           │   Statusword bit[2]=1             │
    │           └──────────────┬──────────────────┘
    │                          │ Quick Stop (Controlword bit[2]=0)
    │                          ▼
    │           ┌─────────────────────────────────┐
    │           │   Quick Stop Active               │ ← E-Stop 触发后进入
    │           │   Statusword bit[5]=1             │
    │           └──────────────┬──────────────────┘
    │                          │ Fault Reset
    │                          ▼
    │           ┌─────────────────────────────────┐
    └───────────│   Fault                           │ ← 过流/超速/EMCY
                │   Statusword bit[3]=1             │
                └─────────────────────────────────┘
```

### 3.3 PDO 帧格式（`pdo_codec.py`）

```python
import struct

# 常量
TORQUE_SCALE     = 0.001    # N·m / bit（int16，范围 ±32.767 N·m）
POSITION_SCALE   = 1.0 / 4096 * 2 * 3.14159265  # rad / count（int32，4096 cnt/rev）
VELOCITY_SCALE   = 0.001    # rad/s / bit

# RPDO1：力矩指令（CM → Driver），8 bytes
# Byte 0-1: int16 目标力矩 [0.001 N·m/bit]
# Byte 2-7: 保留
def pack_torque_rpdo(torque_nm: float) -> bytes:
    raw = int(torque_nm / TORQUE_SCALE)
    raw = max(-32768, min(32767, raw))
    return struct.pack("<h6x", raw)

# TPDO1：位置 + 速度反馈（Driver → CM），8 bytes
# Byte 0-3: int32 实际位置 [counts]
# Byte 4-5: int16 实际速度 [0.001 rad/s/bit]
# Byte 6-7: 保留
def pack_position_tpdo(pos_rad: float, vel_radps: float) -> bytes:
    pos_cnt = int(pos_rad / POSITION_SCALE)
    vel_raw = int(vel_radps / VELOCITY_SCALE)
    vel_raw = max(-32768, min(32767, vel_raw))
    return struct.pack("<ih2x", pos_cnt, vel_raw)

def unpack_position_tpdo(data: bytes) -> tuple[float, float]:
    pos_cnt, vel_raw = struct.unpack("<ih2x", data)
    return pos_cnt * POSITION_SCALE, vel_raw * VELOCITY_SCALE

# TPDO2：状态字 + 实际力矩（Driver → CM），8 bytes
# Byte 0-1: uint16 状态字（DS402 Statusword）
# Byte 2-3: int16 实际力矩 [0.001 N·m/bit]
# Byte 4-7: 保留
def pack_status_tpdo(statusword: int, torque_nm: float) -> bytes:
    torque_raw = int(torque_nm / TORQUE_SCALE)
    torque_raw = max(-32768, min(32767, torque_raw))
    return struct.pack("<Hh4x", statusword, torque_raw)
```

### 3.4 SDO 对象字典（关键条目）

| 索引 | 子索引 | 名称 | 类型 | 说明 |
|---|---|---|---|---|
| `0x6060` | 0 | Modes of Operation | INT8 | `3`=Profile Velocity, `4`=Profile Torque |
| `0x6071` | 0 | Target Torque | INT16 | 目标力矩（RPDO1 映射） |
| `0x6072` | 0 | Max Torque | UINT16 | 力矩限幅 |
| `0x6064` | 0 | Position Actual Value | INT32 | 实际位置（TPDO1 映射） |
| `0x606C` | 0 | Velocity Actual Value | INT32 | 实际速度 |
| `0x6077` | 0 | Torque Actual Value | INT16 | 实际力矩（TPDO2 映射） |
| `0x6041` | 0 | Statusword | UINT16 | DS402 状态字（TPDO2 映射） |
| `0x6040` | 0 | Controlword | UINT16 | DS402 控制字（SDO 写） |
| `0x1017` | 0 | Heartbeat Producer Time | UINT16 | 心跳周期（ms），`1000`=1 Hz |

---

## 4. `virtual_servo_driver` 核心实现

### 4.1 `driver_node.py` 骨架

```python
class VirtualServoDriver(rclpy.node.Node):
    """每个关节对应一个实例，node_id = 1~7"""

    def __init__(self, node_id: int):
        super().__init__(f"virtual_servo_driver_{node_id}")
        self.node_id = node_id

        # 子模块
        self.ds402   = DS402StateMachine(node_id)
        self.codec   = PdoCodec()
        self.sdo_srv = SdoServer(node_id)
        self.cur_loop = CurrentLoop(node_id)

        # SocketCAN
        self.bus = can.interface.Bus(channel="vcan0", bustype="socketcan")
        self.notifier = can.Notifier(self.bus, [self._on_can_message])

        # 与 MuJoCo 的 /sim/* 接口
        self.create_subscription(JointState, "/sim/encoder_state",
                                 self._on_encoder, qos_profile_sensor_data)
        self._effort_pub = self.create_publisher(
            Float64MultiArray, "/sim/joint_effort_cmd", qos_profile_sensor_data)

        # SYNC 定时器（1kHz，由 canopen_hw_interface 广播 SYNC 帧触发）
        # 也可用本地定时器兜底
        self.create_timer(0.001, self._on_sync)

    def _on_can_message(self, msg: can.Message):
        cob_id = msg.arbitration_id
        # RPDO1：力矩指令
        if cob_id == 0x200 + self.node_id:
            target_torque = self.codec.unpack_torque_rpdo(msg.data)
            self.cur_loop.set_target(target_torque)
        # SDO Request
        elif cob_id == 0x600 + self.node_id:
            response = self.sdo_srv.handle(msg.data)
            self._send_can(0x580 + self.node_id, response)
        # NMT
        elif cob_id == 0x000:
            self.ds402.handle_nmt(msg.data)
        # SYNC
        elif cob_id == 0x080:
            self._on_sync()

    def _on_sync(self):
        """SYNC 触发：执行电流环，发 TPDO"""
        if self.ds402.state == "OperationEnabled":
            effort = self.cur_loop.compute(self._actual_pos, self._actual_vel)
        elif self.ds402.state in ("QuickStopActive", "Fault"):
            effort = self.cur_loop.ramp_to_zero()
        else:
            effort = 0.0

        # 发布力矩到 MuJoCo
        msg = Float64MultiArray()
        msg.data = [0.0] * 7
        msg.data[self.node_id - 1] = effort
        self._effort_pub.publish(msg)

        # 发送 TPDO1（位置 + 速度）
        tpdo1 = self.codec.pack_position_tpdo(self._actual_pos, self._actual_vel)
        self._send_can(0x180 + self.node_id, tpdo1)

        # 发送 TPDO2（状态字 + 实际力矩）
        tpdo2 = self.codec.pack_status_tpdo(self.ds402.statusword, effort)
        self._send_can(0x280 + self.node_id, tpdo2)

    def inject_fault(self, fault_code: int = 0x3210):
        """故障注入（过流 0x3210 / 超速 0x8480 / 跟随误差 0x8611）"""
        self.ds402.trigger_fault()
        emcy = struct.pack("<HBB5s", fault_code, 0x00, 0x00, b"\x00" * 5)
        self._send_can(0x080 + self.node_id, emcy)
```

### 4.2 `current_loop.py`（一阶力矩环）

```python
class CurrentLoop:
    """一阶低通力矩环，模拟真实驱动器的电流环带宽"""
    def __init__(self, node_id: int, bandwidth_hz: float = 500.0):
        self.alpha  = 1.0 - math.exp(-2 * math.pi * bandwidth_hz * 0.001)
        self._actual_torque = 0.0
        self._target_torque = 0.0
        # 故障阈值（可通过 SDO 0x6072 设置）
        self.max_torque = 87.0   # N·m（Panda joint1 最大值）
        self.overcurrent_thresh = 90.0

    def compute(self, pos_rad: float, vel_radps: float) -> float:
        # 一阶环路滤波
        self._actual_torque += self.alpha * (self._target_torque - self._actual_torque)
        # 限幅
        self._actual_torque = max(-self.max_torque, min(self.max_torque, self._actual_torque))
        # 过流检测（故障注入条件）
        if abs(self._actual_torque) > self.overcurrent_thresh:
            raise OvercurrentFault(self._actual_torque)
        return self._actual_torque

    def ramp_to_zero(self, ramp_rate: float = 500.0) -> float:
        """Quick Stop 斜坡归零（DS402 Quick Stop Option Code = 2）"""
        delta = ramp_rate * 0.001  # 每 1ms 减少量
        if abs(self._actual_torque) < delta:
            self._actual_torque = 0.0
        else:
            self._actual_torque -= math.copysign(delta, self._actual_torque)
        return self._actual_torque
```

### 4.3 `canopen_hw_interface` M2 升级（`canopen_system.cpp`）

```cpp
// write(): 力矩命令 → RPDO1
hardware_interface::return_type CanopenSystem::write(
    const rclcpp::Time &, const rclcpp::Duration &)
{
  if (sim_direct_) {
    // M1 逻辑（保留兜底）
    publish_effort_to_sim();
  } else {
    // M2: encode → RPDO1 → vcan0
    for (size_t i = 0; i < 7; ++i) {
      auto frame = encode_torque_rpdo(i + 1, hw_commands_effort_[i]);
      can_send(frame);  // SocketCAN write()
    }
    send_sync_frame();   // 广播 SYNC 触发 TPDO
  }
  return hardware_interface::return_type::OK;
}

// read(): TPDO → state_interfaces
hardware_interface::return_type CanopenSystem::read(
    const rclcpp::Time &, const rclcpp::Duration &)
{
  if (sim_direct_) {
    read_encoder_from_sim();
  } else {
    // M2: 从 TPDO 缓存读取最新状态
    std::lock_guard<std::mutex> lock(tpdo_mutex_);
    for (size_t i = 0; i < 7; ++i) {
      hw_states_position_[i] = tpdo_position_[i];
      hw_states_velocity_[i] = tpdo_velocity_[i];
      hw_states_effort_[i]   = tpdo_torque_[i];
    }
  }
  return hardware_interface::return_type::OK;
}
```

---

## 5. 接口定义

### 5.1 新增 ROS2 话题

| Topic | 类型 | 发布者 | 订阅者 | 说明 |
|---|---|---|---|---|
| `/servo_drive/status` | `teleop_interfaces/DriveStatus` | `virtual_servo_driver` | recorder, rqt | 所有驱动器状态汇总，50 Hz |

### 5.2 Gripper RS485（`gripper_driver`）

| Topic | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/teleop/gripper_cmd` | `std_msgs/Float64` | 订阅 | 0.0=全闭，1.0=全开 |
| `/gripper/state` | `std_msgs/Float64` | 发布 | 当前开合度反馈 |

```python
# Modbus 寄存器映射（pymodbus TCP 仿真）
GRIPPER_TARGET_REG = 0x0040   # Holding: 目标开合度（0~1000 → 0%~100%）
GRIPPER_STATE_REG  = 0x0041   # Holding: 实际开合度反馈
GRIPPER_ERROR_REG  = 0x0042   # Holding: 错误状态（0=正常）
```

---

## 6. 验收标准

### 必须通过（阻塞合并至 main）

| # | 验收项 | 验证命令 |
|---|---|---|
| AC-1 | `setup_vcan.sh` 运行成功，`vcan0` UP | `ip link show vcan0` |
| AC-2 | `candump vcan0` 能抓到 7 路 TPDO1（`0x181~0x187`）和 TPDO2（`0x281~0x287`）周期帧 | `candump vcan0 \| head -30` |
| AC-3 | `candump vcan0` 能抓到 RPDO1（`0x201~0x207`）周期帧 | `candump vcan0 \| grep -E "20[1-7]"` |
| AC-4 | DS402 状态机从 `Switch On Disabled` 走到 `Operation Enabled` | `ros2 topic echo /servo_drive/status --once` |
| AC-5 | `forward_command_controller` 经 CAN 驱动 Panda 关节 1 旋转 | `ros2 topic pub /forward_command_controller/commands ...` + 视觉确认 |
| AC-6 | 故障注入（`inject_fault()` 调用）→ `candump` 抓到 EMCY 帧（`0x081~0x087`） | Python 调用 + `candump` |
| AC-7 | 故障注入后 `/servo_drive/status` 显示对应驱动器进入 `Fault` 态 | `ros2 topic echo /servo_drive/status` |
| AC-8 | `test_ds402_state_machine.py` pytest 全通过 | `pytest tests/test_ds402_state_machine.py -v` |
| AC-9 | `test_pdo_codec.py` pytest 全通过（含边界值：最大力矩、溢出） | `pytest tests/test_pdo_codec.py -v` |

### 加分项

- [ ] `candump` 帧率稳定在 1kHz ± 5%（7 轴 × 2 TPDO = 14kHz 峰值）
- [ ] SDO 读写测试：通过 SDO 修改 `0x6072`（MaxTorque），驱动器实时生效
- [ ] Heartbeat 超时检测：停掉一路 `virtual_servo_driver`，`canopen_hw_interface` 报警

---

## 7. 视觉证明与采集产物

M2 的 README 可见证明图已补到 `media/m2/m2_canopen_fieldbus_proof.svg`，运行证据 PNG 也已补齐，覆盖本里程碑的正常现场总线路径和故障注入路径：

```text
canopen_system(use_sim:=false)
  → vcan0 RPDO/SYNC
  → virtual_servo_driver ×7 (DS402 Operation Enabled)
  → /sim/joint_effort_cmd
  → mujoco_sim
  → /sim/encoder_state
  → vcan0 TPDO
  → /joint_states

inject_fault()
  → EMCY 0x081~0x087
  → /servo_drive/status = Fault
```

运行证据产物如下，采集要求见 [`MEDIA_CAPTURE_PLAN.md`](./MEDIA_CAPTURE_PLAN.md)：

| 文件 | 证明内容 | 对应验收 |
|---|---|---|
| `media/m2/candump_pdo.png` | `candump vcan0` 显示 RPDO/TPDO 周期帧 | AC-2/AC-3 |
| `media/m2/ds402_state_machine.png` | DS402 状态从 `Switch On Disabled` 进入 `Operation Enabled` | AC-4 |
| `media/m2/emcy_fault_injection.png` | `/servo_drive/status` 已锁存 `Fault` 状态；当前补图未主动调用新的 `inject_fault` 服务 | AC-6/AC-7 |

---

## 8. 常用调试命令

```bash
# 建立 vcan0
bash scripts/setup_vcan.sh
ip link show vcan0

# 监听所有 CAN 帧（带时间戳）
candump -t a vcan0

# 监听特定节点（关节 1，node_id=1）
candump vcan0 181:7FF  # TPDO1 of joint1
candump vcan0 281:7FF  # TPDO2 of joint1

# 手动发送 SYNC 帧（触发 TPDO）
cansend vcan0 080#00

# 手动发 RPDO1（关节 1，力矩 1.0 N·m = 1000 counts）
cansend vcan0 201#E8030000000000000

# 手动发 NMT 启动（所有节点）
cansend vcan0 000#0100

# 发送 DS402 控制字（Shutdown → Ready to Switch On）
cansend vcan0 601#2B4060000600000   # SDO Write 0x6040 = 0x0006

# 启动 M2 fieldbus launch
ros2 launch teleop_bringup fieldbus.launch.py

# 检查驱动器状态
ros2 topic echo /servo_drive/status --once

# PDO 单测
pytest tests/test_pdo_codec.py -v
pytest tests/test_ds402_state_machine.py -v
```

---

## 9. 关键风险与应对

| 风险 | 应对 |
|---|---|
| 7 轴 PDO 1kHz 帧率下 vcan0 带宽不足 | 先单轴验证，再扩 7 轴；vcan0 带宽远大于实体 CAN |
| SocketCAN `read()` 阻塞影响 ros2_control RT 循环 | 使用独立线程 + 无锁环形缓冲区缓存 TPDO；read() 只从缓存取值 |
| DS402 状态机转换条件复杂，调试困难 | 单测覆盖所有状态转换路径；加详细 rclcpp 日志 |
| `virtual_servo_driver` 7 个进程资源占用 | Python multiprocessing 或单进程多线程（asyncio event loop per joint） |

---

*本文件为 V2-M2 细化 SPEC；架构基线见 [`ARCHITECTURE_V2.md`](./ARCHITECTURE_V2.md) §6.4，里程碑总览见 [`ROADMAP.md`](./ROADMAP.md)。*

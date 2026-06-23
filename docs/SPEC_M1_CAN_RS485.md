# SPEC M1: CAN + RS485 通信层

**分支**：`feat/can-rs485-layer`  
**里程碑**：M1  
**预计用时**：Week 1（5 天）  
**负责模块**：`src/can_bridge/`

---

## 1. 目标

在无实体硬件的条件下，用 Linux 虚拟 CAN（vcan0）和 pymodbus TCP 仿真，完整实现：

1. **CAN 通信层**：ROS2 ↔ CANopen DS402 PDO 帧的双向桥接
2. **RS485 通信层**：Modbus RTU 协议夹爪仿真节点

---

## 2. 技术选型与理由

### 2.1 CAN 方案：Linux SocketCAN + python-can

| 方案 | 优点 | 缺点 | 结论 |
|---|---|---|---|
| **SocketCAN + python-can** ✅ | Linux 内核原生支持；vcan/can0 无缝切换；工业标准 | 需 root 初始化 vcan | **选用** |
| Socket 自己实现 CAN 帧 | 无依赖 | 需手写 PF_CAN 协议，维护成本高 | 否 |
| PEAK PCAN USB SDK | 硬件驱动齐全 | 无硬件无法测试 | 否 |

**选用理由**：`python-can` 对 socketcan 接口封装良好，`cantools` 提供 DBC 文件解析能力，两者合用是 ROS2 + CANopen 的工业常见组合。切换 `vcan0 → can0` 仅需改一行 YAML 配置。

### 2.2 RS485 方案：pymodbus ModbusTcpServer（纯软件仿真）

| 方案 | 优点 | 缺点 | 结论 |
|---|---|---|---|
| **pymodbus TCP Server** ✅ | 纯软件、零硬件；API 清晰 | 非真实串口 RTU，帧格式略有差异 | **选用** |
| socat 虚拟串口 + pyserial | 更接近真实 RTU | 配置复杂，CI 不友好 | 备选 |
| MinimalModbus | 轻量 | 只有 Client 端，无 Server 仿真 | 否 |

**选用理由**：面试展示的是协议理解深度（寄存器级读写、功能码 0x03/0x10），而非串口驱动细节。pymodbus Server 能完整演示读写寄存器逻辑，且在 CI 中无需 root 权限。

---

## 3. 接口定义

### 3.1 CAN Bridge 节点

**订阅**：

| Topic | 类型 | 频率 | 说明 |
|---|---|---|---|
| `/joint_torque_cmd` | `sensor_msgs/JointState` | 500Hz | 来自阻抗控制器，`effort` 字段为力矩 |

**发布**：

| Topic | 类型 | 频率 | 说明 |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | 100Hz | 来自 CAN 编码器反馈（或 MuJoCo 直通） |

**CAN 帧格式（CANopen DS402 PDO1）**：

```
发送方向（PC → 驱动器）：
  CAN ID: 0x200 + node_id  (node_id = 关节编号 0~6)
  Data[0:2]: int16, 目标力矩, 单位 0.001 N·m/bit
  Data[2:8]: reserved (0x00)

接收方向（驱动器 → PC）：
  CAN ID: 0x180 + node_id
  Data[0:4]: int32, 实际位置, 单位 encoder counts (1 rev = 4096 counts)
  Data[4:8]: reserved
```

### 3.2 RS485 / Modbus 节点

**订阅**：

| Topic | 类型 | 说明 |
|---|---|---|
| `/gripper_cmd` | `std_msgs/Float32` | 0.0 = 全闭，1.0 = 全开 |

**发布**：

| Topic | 类型 | 说明 |
|---|---|---|
| `/gripper_state` | `std_msgs/Float32` | 当前夹爪开合度反馈 |

**Modbus 寄存器映射**：

```
Write  0x0040 (Holding Register): 目标开合度 (0–1000 → 0–100%)
Read   0x0041 (Holding Register): 实际开合度反馈
Read   0x0042 (Holding Register): 错误状态 (0=正常)
```

---

## 4. 文件清单

```
src/can_bridge/
├── package.xml
├── setup.py
└── can_bridge/
    ├── __init__.py
    ├── can_bridge_node.py       ← CAN ↔ ROS2 桥接主节点
    ├── pdo_codec.py             ← PDO 帧编解码（单元测试覆盖）
    └── rs485_modbus_node.py     ← Modbus RTU 仿真节点

config/
└── can_config.yaml              ← interface: vcan0 / can0 切换点

scripts/
└── setup_vcan.sh                ← vcan0 一键初始化

tests/
└── test_can_bridge.py           ← PDO 编解码 + 话题收发
```

---

## 5. 关键实现细节

### 5.1 vcan0 初始化

```bash
#!/bin/bash
# scripts/setup_vcan.sh
set -e
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan 2>/dev/null || true
sudo ip link set up vcan0
echo "[OK] vcan0 ready"
cansend vcan0 123#DEADBEEF  # 自检帧
```

### 5.2 PDO 编解码（`pdo_codec.py`）

```python
import struct

ENCODER_COUNTS_PER_REV = 4096
TORQUE_SCALE = 0.001  # N·m per bit

def pack_torque_pdo(torque_nm: float) -> bytes:
    """将力矩值编码为 CANopen TPDO1 格式（8 bytes）"""
    raw = int(torque_nm / TORQUE_SCALE)
    raw = max(-32768, min(32767, raw))  # int16 clamp
    return struct.pack("<h6x", raw)     # little-endian int16 + 6 bytes padding

def unpack_encoder_pdo(data: bytes) -> float:
    """将 RPDO1 解码为关节角（radians）"""
    counts = struct.unpack("<i4x", data)[0]  # int32 + 4 bytes padding
    return counts / ENCODER_COUNTS_PER_REV * 2 * 3.14159265
```

### 5.3 接口切换（`can_config.yaml`）

```yaml
can:
  interface: vcan0      # 切换实体硬件只改这一行 → can0
  bitrate: 1000000      # 1Mbps（CANopen DS402 典型值）
  timeout: 0.01         # 接收超时 10ms

modbus:
  host: "127.0.0.1"
  port: 5020
  unit_id: 1
```

---

## 6. 验收标准

### 必须通过（阻塞合并）

| # | 验收项 | 验证方法 |
|---|---|---|
| AC-1 | `setup_vcan.sh` 运行成功，`ip link show vcan0` 显示 UP | shell 命令 |
| AC-2 | `can_bridge_node.py` 启动后，向 `/joint_torque_cmd` 发布一个 7 关节 JointState，`candump vcan0` 能抓到 7 帧 CAN ID `0x200~0x206` | `ros2 topic pub` + `candump` |
| AC-3 | 用 `cansend vcan0 181#0010000000000000` 模拟编码器反馈，`/joint_states` 能收到对应关节角变化 | `ros2 topic echo` |
| AC-4 | `rs485_modbus_node.py` 启动后，向 `/gripper_cmd` 发布 `0.5`，寄存器 `0x0040` 变为 500 | pymodbus client 直读 |
| AC-5 | `tests/test_can_bridge.py` pytest 全通过 | `pytest tests/test_can_bridge.py -v` |
| AC-6 | `config/can_config.yaml` 中 `interface` 改为 `can0` 后，代码无需修改即可重启（即便 can0 不存在，报错应为 "interface not found"，而非 KeyError） | 手动测试 |

### 加分项（不阻塞合并）

- [ ] `candump` 输出帧率稳定在 500Hz ± 5%
- [ ] Modbus 节点异常断线后能自动重连
- [ ] PDO 编解码通过 fuzzing 测试（随机输入不 crash）

---

## 7. 面试话术关键点

> "CAN 通信层的工程价值在于**零硬件可测试**：通过 vcan0 虚拟总线，所有协议层逻辑在 CI 中就能跑，切换实体 can0 时一行配置都不用改。PDO 帧的编解码我专门抽成独立模块 pdo_codec.py，单测覆盖边界值（力矩截断、编码器溢出），这是嵌入式通信层的标准做法。"

# CANopen DS402 + KDL 快速参考

## CANopen DS402 PDO 帧格式

### 本项目使用的 PDO 映射

```
TPDO1（PC → 驱动器，发送力矩指令）
  CAN ID : 0x200 + node_id   (node_id = 关节编号 0~6)
  Byte 0-1: int16 LE,  目标力矩, 1 bit = 0.001 N·m
  Byte 2-7: 0x00 (padding)

RPDO1（驱动器 → PC，接收编码器反馈）
  CAN ID : 0x180 + node_id
  Byte 0-3: int32 LE,  实际位置, 1 bit = 1 encoder count (4096 counts/rev)
  Byte 4-7: 0x00 (padding)
```

### 快速编解码

```python
import struct

# 力矩 → CAN 帧（N·m → PDO bytes）
def pack_torque(torque_nm: float) -> bytes:
    raw = int(torque_nm / 0.001)
    raw = max(-32768, min(32767, raw))  # int16 clamp
    return struct.pack("<h6x", raw)

# CAN 帧 → 关节角（PDO bytes → radians）
def unpack_position(data: bytes) -> float:
    counts = struct.unpack("<i4x", data)[0]
    return counts / 4096 * 6.28318530  # 2π

# 关节编号 → CAN ID
def torque_can_id(joint_idx: int) -> int:
    return 0x200 + joint_idx   # 0x200 ~ 0x206

def feedback_can_id(joint_idx: int) -> int:
    return 0x180 + joint_idx   # 0x180 ~ 0x186
```

### python-can 收发示例

```python
import can

# 创建总线（vcan0 或 can0）
bus = can.interface.Bus(channel="vcan0", interface="socketcan")

# 发送力矩帧
msg = can.Message(
    arbitration_id=0x200,          # 关节 0
    data=pack_torque(5.0),         # 5 N·m
    is_extended_id=False
)
bus.send(msg)

# 接收编码器反馈（阻塞，timeout=0.01s）
msg = bus.recv(timeout=0.01)
if msg and (msg.arbitration_id & 0xF80) == 0x180:
    joint_idx = msg.arbitration_id & 0x07F
    position  = unpack_position(bytes(msg.data))
```

---

## KDL（Kinematics and Dynamics Library）

### 安装

```bash
sudo apt install ros-jazzy-kdl-parser ros-jazzy-robot-state-publisher liborocos-kdl-dev
```

### CMakeLists.txt 依赖

```cmake
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Eigen3 REQUIRED)

target_link_libraries(${PROJECT_NAME}
  ${orocos_kdl_LIBRARIES}
  ${kdl_parser_LIBRARIES}
  Eigen3::Eigen
)
```

### 从 URDF 构建 KDL Chain（C++）

```cpp
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

// 从 URDF 字符串加载
KDL::Tree kdl_tree;
kdl_parser::treeFromString(urdf_string, kdl_tree);

// 提取 Franka Panda 运动链
KDL::Chain chain;
kdl_tree.getChain("panda_link0", "panda_link8", chain);

// 创建求解器
auto fk_solver  = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain);
auto jac_solver = std::make_unique<KDL::ChainJntToJacSolver>(chain);
auto ik_solver  = std::make_unique<KDL::ChainIkSolverVel_pinv>(chain);
auto dyn_solver = std::make_unique<KDL::ChainIdSolver_RNE>(chain, KDL::Vector(0, 0, -9.81));
```

### 计算雅可比矩阵

```cpp
KDL::JntArray q(7);        // 当前关节角
for (int i = 0; i < 7; i++) q(i) = joint_positions[i];

KDL::Jacobian J(7);
jac_solver->JntToJac(q, J);
// J.data 是 Eigen::MatrixXd(6, 7)，可直接与 Eigen 运算
```

### 计算重力补偿力矩

```cpp
KDL::JntArray q(7), qdot(7), qdotdot(7), tau_gravity(7);
KDL::Wrenches ext_forces(chain.getNrOfSegments());

// qdot, qdotdot 置零（只算重力项）
SetToZero(qdot); SetToZero(qdotdot);
dyn_solver->CartToJnt(q, qdot, qdotdot, ext_forces, tau_gravity);
```

### 阻抗控制律（完整 C++ 实现草图）

```cpp
// 计算末端位姿误差（6D）
Eigen::Matrix<double,6,1> compute_error(
    const Eigen::Vector7d& q_current,
    const geometry_msgs::msg::Pose& target)
{
    KDL::JntArray q(7);
    for (int i=0; i<7; i++) q(i) = q_current(i);

    KDL::Frame ee_frame;
    fk_solver_->JntToCart(q, ee_frame);

    // 平移误差
    Eigen::Vector3d pos_err(
        target.position.x - ee_frame.p.x(),
        target.position.y - ee_frame.p.y(),
        target.position.z - ee_frame.p.z());

    // 旋转误差（轴角）
    Eigen::Quaterniond q_d(target.orientation.w, target.orientation.x,
                            target.orientation.y, target.orientation.z);
    Eigen::Quaterniond q_cur;
    ee_frame.M.GetQuaternion(q_cur.x(), q_cur.y(), q_cur.z(), q_cur.w());
    Eigen::Quaterniond q_err = q_d * q_cur.inverse();
    Eigen::AngleAxisd aa(q_err);
    Eigen::Vector3d rot_err = aa.axis() * aa.angle();

    Eigen::Matrix<double,6,1> error;
    error.head<3>() = pos_err;
    error.tail<3>() = rot_err;
    return error;
}

// 计算力矩指令
Eigen::VectorXd compute_torque(const Eigen::Matrix<double,6,1>& error,
                                const Eigen::Matrix<double,6,1>& error_dot)
{
    KDL::Jacobian J_kdl(7);
    jac_solver_->JntToJac(q_, J_kdl);
    Eigen::MatrixXd J = J_kdl.data;                    // (6×7)
    Eigen::Matrix<double,6,1> F = K_ * error + D_ * error_dot;
    return J.transpose() * F + tau_gravity_;            // (7×1)
}
```

---

## 常见问题

| 问题 | 原因 | 解决 |
|---|---|---|
| `getChain` 返回 false | 链名错误 | Panda 用 `panda_link0` → `panda_link8` |
| Jacobian 全零 | 关节角初始化为 0（奇异位形） | 用 home position 初始化 |
| 重力补偿力矩数量级不对 | 重力向量方向错 | KDL 用 `Vector(0, 0, -9.81)`，注意符号 |
| IK 不收敛 | 目标超出工作空间 | 检查 `x_d` 是否在 Panda 可达范围内 |

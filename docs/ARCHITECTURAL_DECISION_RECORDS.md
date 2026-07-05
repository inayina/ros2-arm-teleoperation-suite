# Architectural Decision Records (ADRs)

本文档记录了 `ros2-arm-teleoperation-suite` 在系统集成与硬件仿真层面的关键技术选型决策，主要面向**具身数据采集**与**工业级伺服通信**场景。

---

## ADR 01：物理仿真引擎选型（为什么选择 MuJoCo 而非 Gazebo）

### 1. 上下文与痛点 (Context)
具身智能数据采集（LeRobot 链路）对仿真器提出了与传统导航（Nav2）完全不同的要求：
- **接触动力学稳定性**：操纵任务（如 pick-and-lift）涉及频繁的“臂-指-物体”接触力突变，要求仿真器在极小时间步长下对摩擦力、穿透和抓取建立（`grasp_established`）有极其精确且稳定的解算。
- **多模态传感器仿真**：系统需要输出触觉图像（类似 GelSight 的光度学触觉传感器模拟）和多摄像头深度图。
- **高并发数据生成**：策略训练（ACT/Diffusion）需要数以千计的 episode。仿真器必须足够轻量，支持在 Docker 容器中多进程 headless 快速步进。

### 2. 方案对比 (Comparison)

| 评估维度 | Gazebo (Classic/Ignition) | MuJoCo (Multi-Joint dynamics with Collocation) |
|---|---|---|
| **接触力解算器** | 采用 ODE/Bullet 解算器，易出现“抓取滑落”、“物体高频抖动跳飞”等接触奇异性。 | 采用凸优化接触模型，支持软接触动力学，抓取力与接触滑移极其稳定、真实。 |
| **触觉仿真支持** | 无原生触觉图像仿真，需编写复杂的第三方渲染插件。 | 原生支持高质量深度/法向贴图获取，易于模拟 GelSight 触觉相机的光度学成像。 |
| **运行效率与并发** | 架构较重（基于 Ogre 渲染和复杂的 DDS 插件），多进程无头运行开销极大。 | 纯 C 编写，内存极小，支持高并发无头运行（多线程加速达数千 FPS），适合强化学习与数据合成。 |
| **ROS 2 集成度** | 原生 `gazebo_ros_pkgs` 支持，开箱即用。 | 需要自行编写桥接节点，开发接口成本稍高。 |

### 3. 决策 (Decision)
**选择 MuJoCo 作为核心物理仿真引擎。**  
由于项目聚焦于**操作臂触觉抓取与多模态数据合成（LeRobot 录制）**，Gazebo 在接触力解算上的抖动缺陷和较重的并发架构无法满足需求。我们选择自研 `mujoco_sim` ROS 2 节点，用较小的接口开发成本换取了极其稳定的抓取动力学和高效的数据生成性能。

---

## ADR 02：工业现场总线选型（为什么选择 CANopen 而非 EtherCAT）

### 1. 上下文与痛点 (Context)
为了让作品集呈现真实的“工业级”开发水准，我们需要仿真电机的控制环路（状态机机能、状态字转换）：
- **嵌入式易开发性**：实机 HAL 层需支持低成本嵌入式节点（如 ESP32/STM32 组成的电机测试 bench），以便在 L3 控制层无缝切换实机。
- **协议标准化**：必须遵循标准的操作器驱动规范，支持 NMT（网络管理）、PDO/SDO（过程/服务数据对象）和 Quick Stop 故障响应。
- **容器与虚拟化友好**：必须能在标准 Linux 虚拟机和 Docker 容器中跑通总线级仿真，无需复杂的内核实时补丁（RT-preempt）或专有硬件网卡。

### 2. 方案对比 (Comparison)

| 评估维度 | EtherCAT | CANopen (SocketCAN / DS402) |
|---|---|---|
| **硬件与芯片成本** | 必须依赖专用的从站控制器芯片（ESC，如 Beckhoff ET1100），嵌入式开发门槛和成本高。 | 几乎所有 MCU（ESP32/STM32）都内置 CAN 物理层控制器，无需专有芯片，外设成本极低。 |
| **内核依赖性** | Linux 下通常依赖 IgH EtherCAT Master，需要特定的网络驱动和实时内核（RT Patch）支持。 | Linux 内核原生集成 SocketCAN（Socket 套接字接口），无需编译内核或安装专有驱动。 |
| **容器化与虚拟化** | 很难在 Docker 容器中虚拟化从站设备，无法进行纯软件的总线环路集成测试。 | 原生支持 `vcan0`（虚拟 CAN 接口），可在 Docker 容器中直接回环读写，完美支持 CI 自动化测试。 |
| **标准协议支持** | CoE (CANopen over EtherCAT) 支持 DS402。 | 原生支持 CANopen DS402 驱动器规范，状态机定义与 CoE 完全一致。 |

### 3. 决策 (Decision)
**选择 CANopen (DS402 Profile over SocketCAN/vcan0) 作为系统总线方案。**  
EtherCAT 虽然在工业界带宽极高（100Mbps），但其对实时内核的强依赖和无法在标准 Docker 容器中虚拟化运行的限制，破坏了我们“一键 CI/CD 验证”与“低成本嵌入式 HAL 对齐”的原则。  
通过选择 CANopen，我们利用 SocketCAN 的 `vcan0` 实现了在 Docker 容器中跑通包含 7 个虚拟伺服驱动器（`virtual_servo_driver`）的完整 CAN 总线闭环，同时保留了与 EtherCAT CoE 完全一致的 DS402 状态机控制逻辑。

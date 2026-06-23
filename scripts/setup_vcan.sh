#!/bin/bash
# scripts/setup_vcan.sh
# 一键配置 Linux 虚拟 CAN 总线（开发/测试用）
# 切换实体硬件时：修改 config/can_config.yaml 中 interface: vcan0 -> can0

set -e

echo "[setup_vcan] 加载 vcan 内核模块..."
sudo modprobe vcan

echo "[setup_vcan] 创建 vcan0 虚拟接口..."
if ip link show vcan0 > /dev/null 2>&1; then
    echo "[setup_vcan] vcan0 已存在，跳过创建"
else
    sudo ip link add dev vcan0 type vcan
fi

echo "[setup_vcan] 启动 vcan0..."
sudo ip link set up vcan0

echo "[setup_vcan] ✅ vcan0 就绪"
echo "[setup_vcan] 验证命令: candump vcan0"
ip link show vcan0

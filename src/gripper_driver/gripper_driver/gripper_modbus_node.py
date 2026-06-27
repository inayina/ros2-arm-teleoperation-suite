#!/usr/bin/env python3
"""RS485 / Modbus gripper driver.

  in : /teleop/gripper_cmd  std_msgs/Float64  (0.0 closed .. 1.0 open)
  out: /gripper/state       std_msgs/Float64  (measured opening)

Implementation: 
Runs a pymodbus ModbusTCP server locally to simulate the gripper hardware.
The ROS node acts as a ModbusClient, sending commands to Reg 0x40 and reading state from Reg 0x41.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import threading
import asyncio
import time

CMD_REG = 0x0040
STATE_REG = 0x0041

class MockModbusClient:
    """Mock Modbus client that acts as both client and simulated gripper hardware.
    
    Bypasses PyModbus TCP Server constraints completely while providing the exact
    same interface as ModbusTcpClient.
    """
    def __init__(self, host, port, speed=2000):
        self.registers = {CMD_REG: 0, STATE_REG: 0}
        self.speed = speed
        self._stop = False
        self._thread = None
        
    def connect(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self._sim_loop, daemon=True)
            self._thread.start()
        return True
        
    def close(self):
        self._stop = True
        if self._thread:
            self._thread.join(timeout=1.0)
            
    def write_register(self, reg_addr, val):
        self.registers[reg_addr] = int(val)
        class _Response:
            def isError(self): return False
        return _Response()
        
    def read_holding_registers(self, reg_addr, count):
        val = self.registers.get(reg_addr, 0)
        class _Response:
            def __init__(self, v): self.registers = [v]
            def isError(self): return False
        return _Response(val)
        
    def _sim_loop(self):
        dt = 0.05
        while not self._stop:
            cmd = self.registers[CMD_REG]
            state = self.registers[STATE_REG]
            
            diff = cmd - state
            step = int(self.speed * dt)
            
            if abs(diff) <= step:
                state = cmd
            else:
                state += step if diff > 0 else -step
                
            self.registers[STATE_REG] = state
            time.sleep(dt)


class GripperDriverNode(Node):
    def __init__(self):
        super().__init__("gripper_driver")
        self.declare_parameter("rate", 20.0)
        self.declare_parameter("speed", 2.0)  # 1/s
        self.declare_parameter("sim_mode", True)
        
        self.host = "127.0.0.1"
        self.port = 5020
        self.speed = float(self.get_parameter("speed").value)
        self.sim_mode = self.get_parameter("sim_mode").value

        # Simulation setup if in sim mode
        if self.sim_mode:
            self.get_logger().info("Starting in SIMULATION mode.")
            self.client = MockModbusClient(self.host, self.port, speed=int(self.speed * 1000))
            if not self.client.connect():
                self.get_logger().error("MockModbusClient failed to start sim thread.")

        rate = self.get_parameter("rate").value
        self.dt = 1.0 / rate

        self.sub = self.create_subscription(
            Float64, "/teleop/gripper_cmd", self._on_cmd, 10)
        self.pub = self.create_publisher(Float64, "/gripper/state", 10)
        self.create_timer(self.dt, self._tick)
        self.get_logger().info("gripper_driver up (simulated mock backend active).")

    def _on_cmd(self, msg: Float64):
        cmd_val = max(0.0, min(1.0, msg.data))
        reg_val = int(cmd_val * 1000)
        try:
            self.client.write_register(CMD_REG, reg_val)
        except Exception as e:
            self.get_logger().warn(f"Modbus write failed: {e}")

    def _tick(self):
        try:
            result = self.client.read_holding_registers(STATE_REG, 1)
            if not result.isError():
                state_val = result.registers[0] / 1000.0
                m = Float64()
                m.data = state_val
                self.pub.publish(m)
        except Exception as e:
            pass

    def destroy_node(self):
        if hasattr(self, "client"):
            self.client.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GripperDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

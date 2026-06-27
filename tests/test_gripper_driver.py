import time
import pytest

from gripper_driver.gripper_modbus_node import MockModbusClient, CMD_REG, STATE_REG

def test_modbus_sim_server_client():
    # MockModbusClient is both a simulated driver and client
    client = MockModbusClient(host="127.0.0.1", port=5025, speed=10000)
    assert client.connect()
    
    try:
        # Write to command register: set command target to 800 (equivalent to 0.8 open)
        write_res = client.write_register(CMD_REG, 800)
        assert not write_res.isError()
        
        # Read starting state, should be 0
        read_res = client.read_holding_registers(STATE_REG, 1)
        assert not read_res.isError()
        
        # Give simulation loop a small amount of time to step and move the state
        # Server speed is 10000 counts/sec. 10000 * 0.05 = 500 counts per 50ms step.
        # It should reach 800 very quickly.
        time.sleep(0.2)
        
        read_res2 = client.read_holding_registers(STATE_REG, 1)
        assert not read_res2.isError()
        # Verify the gripper state has updated towards the target
        assert read_res2.registers[0] > 0
        assert read_res2.registers[0] <= 800
        
    finally:
        client.close()

from virtual_servo_driver.ds402_state_machine import DS402StateMachine, State


def test_power_on_sequence():
    sm = DS402StateMachine()
    assert sm.state == State.SWITCH_ON_DISABLED
    sm.apply_controlword(0x06)
    assert sm.state == State.READY_TO_SWITCH_ON
    sm.apply_controlword(0x07)
    assert sm.state == State.SWITCHED_ON
    sm.apply_controlword(0x0F)
    assert sm.state == State.OPERATION_ENABLED
    assert sm.torque_enabled()
    assert sm.statusword == 0x0027


def test_quick_stop():
    sm = DS402StateMachine()
    for cw in (0x06, 0x07, 0x0F):
        sm.apply_controlword(cw)
    sm.apply_controlword(0x02)
    assert sm.state == State.QUICK_STOP_ACTIVE
    assert not sm.torque_enabled()


def test_fault_and_reset():
    sm = DS402StateMachine()
    for cw in (0x06, 0x07, 0x0F):
        sm.apply_controlword(cw)
    sm.fault(0x8400)
    assert sm.state == State.FAULT
    assert sm.fault_code == 0x8400
    sm.apply_controlword(0x00)
    sm.apply_controlword(0x80)
    assert sm.state == State.SWITCH_ON_DISABLED
    assert sm.fault_code == 0

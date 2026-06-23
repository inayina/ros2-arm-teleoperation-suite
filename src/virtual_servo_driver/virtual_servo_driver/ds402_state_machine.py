"""CiA 402 (DS402) drive finite state machine.

Implements the standard transitions driven by the controlword (0x6040) and
reports the statusword (0x6041). Kept pure-Python and dependency-free so it can
be unit-tested without ROS or a CAN bus.
"""
from enum import IntEnum


class State(IntEnum):
    NOT_READY_TO_SWITCH_ON = 0
    SWITCH_ON_DISABLED = 1
    READY_TO_SWITCH_ON = 2
    SWITCHED_ON = 3
    OPERATION_ENABLED = 4
    QUICK_STOP_ACTIVE = 5
    FAULT_REACTION_ACTIVE = 6
    FAULT = 7


# Controlword command bits (0x6040)
CW_SHUTDOWN = 0x06          # -> Ready to Switch On
CW_SWITCH_ON = 0x07         # -> Switched On
CW_ENABLE_OPERATION = 0x0F  # -> Operation Enabled
CW_DISABLE_VOLTAGE = 0x00   # -> Switch On Disabled
CW_QUICK_STOP = 0x02        # -> Quick Stop Active
CW_FAULT_RESET = 0x80       # rising edge clears Fault


# Statusword (0x6041) masks for the canonical states
_SW = {
    State.NOT_READY_TO_SWITCH_ON: 0x0000,
    State.SWITCH_ON_DISABLED: 0x0040,
    State.READY_TO_SWITCH_ON: 0x0021,
    State.SWITCHED_ON: 0x0023,
    State.OPERATION_ENABLED: 0x0027,
    State.QUICK_STOP_ACTIVE: 0x0007,
    State.FAULT_REACTION_ACTIVE: 0x000F,
    State.FAULT: 0x0008,
}


class DS402StateMachine:
    def __init__(self):
        self.state = State.SWITCH_ON_DISABLED
        self.fault_code = 0
        self._last_cw = 0

    @property
    def statusword(self) -> int:
        return _SW[self.state]

    def torque_enabled(self) -> bool:
        return self.state == State.OPERATION_ENABLED

    def fault(self, code: int):
        """Raise a fault (e.g. from EMCY conditions)."""
        self.fault_code = code
        self.state = State.FAULT

    def quick_stop(self):
        if self.state == State.OPERATION_ENABLED:
            self.state = State.QUICK_STOP_ACTIVE

    def apply_controlword(self, cw: int) -> State:
        cmd = cw & 0x8F
        rising_fault_reset = bool(cw & CW_FAULT_RESET) and not (self._last_cw & CW_FAULT_RESET)
        self._last_cw = cw

        if self.state == State.FAULT:
            if rising_fault_reset:
                self.fault_code = 0
                self.state = State.SWITCH_ON_DISABLED
            return self.state

        if (cmd & 0x82) == CW_QUICK_STOP and self.state == State.OPERATION_ENABLED:
            self.state = State.QUICK_STOP_ACTIVE
            return self.state

        if self.state == State.SWITCH_ON_DISABLED:
            if (cmd & 0x06) == CW_SHUTDOWN:
                self.state = State.READY_TO_SWITCH_ON
        elif self.state == State.READY_TO_SWITCH_ON:
            if (cmd & 0x0F) == CW_SWITCH_ON or (cmd & 0x0F) == CW_ENABLE_OPERATION:
                self.state = State.SWITCHED_ON
                if (cmd & 0x0F) == CW_ENABLE_OPERATION:
                    self.state = State.OPERATION_ENABLED
            elif (cmd & 0x07) == CW_DISABLE_VOLTAGE:
                self.state = State.SWITCH_ON_DISABLED
        elif self.state == State.SWITCHED_ON:
            if (cmd & 0x0F) == CW_ENABLE_OPERATION:
                self.state = State.OPERATION_ENABLED
            elif (cmd & 0x06) == CW_SHUTDOWN:
                self.state = State.READY_TO_SWITCH_ON
        elif self.state == State.OPERATION_ENABLED:
            if (cmd & 0x0F) == CW_SWITCH_ON:
                self.state = State.SWITCHED_ON
            elif (cmd & 0x06) == CW_SHUTDOWN:
                self.state = State.READY_TO_SWITCH_ON
        elif self.state == State.QUICK_STOP_ACTIVE:
            if (cmd & 0x07) == CW_DISABLE_VOLTAGE:
                self.state = State.SWITCH_ON_DISABLED

        return self.state

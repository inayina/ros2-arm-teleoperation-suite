"""First-order torque/current loop with simple protection + fault injection."""


class CurrentLoop:
    def __init__(self, bandwidth_hz: float = 2000.0, max_torque: float = 87.0,
                 max_velocity: float = 3.0):
        self.tau_cmd = 0.0
        self.tau_out = 0.0
        self.bw = bandwidth_hz
        self.max_torque = max_torque
        self.max_velocity = max_velocity
        self._inject_fault = 0  # EMCY code to raise on next update, 0 = none

    def set_target(self, tau_nm: float):
        self.tau_cmd = max(-self.max_torque, min(self.max_torque, tau_nm))

    def inject_fault(self, emcy_code: int):
        self._inject_fault = emcy_code

    def update(self, dt: float, enabled: bool, velocity: float):
        """Return (torque_out, fault_code). fault_code != 0 => trip."""
        # Protection checks
        if abs(velocity) > self.max_velocity:
            return 0.0, 0x8400  # DS402 velocity fault
        if self._inject_fault:
            code, self._inject_fault = self._inject_fault, 0
            return 0.0, code

        target = self.tau_cmd if enabled else 0.0
        alpha = min(1.0, self.bw * dt)
        self.tau_out += alpha * (target - self.tau_out)
        self.tau_out = max(-self.max_torque, min(self.max_torque, self.tau_out))
        if abs(self.tau_out) > self.max_torque * 1.05:
            return 0.0, 0x3210
        return self.tau_out, 0

    def ramp_to_zero(self, dt: float = 0.001, ramp_rate: float = 500.0) -> float:
        """Quick Stop ramp torque to zero."""
        delta = ramp_rate * dt
        if abs(self.tau_out) < delta:
            self.tau_out = 0.0
        else:
            self.tau_out -= delta if self.tau_out > 0 else -delta
        self.tau_cmd = 0.0
        return self.tau_out

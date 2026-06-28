from typing import Tuple, Optional

from teleop_input.driver_base import TeleopDriverBase
from teleop_input.keyboard_reader import KeyboardReader


class KeyboardDriver(TeleopDriverBase):
    """Keyboard input driver using stdin TTY."""

    def __init__(self, position_step_m: float, rotation_step_rad: float):
        self.position_step_m = position_step_m
        self.rotation_step_rad = rotation_step_rad
        self.keyboard = KeyboardReader()
        self.gripper_open = False

    def initialize(self) -> bool:
        return self.keyboard.enabled

    def get_pose_delta(self) -> Tuple[list, list]:
        pos_delta = [0.0, 0.0, 0.0]
        rpy_delta = [0.0, 0.0, 0.0]

        key = self.keyboard.get_key()
        if not key:
            return pos_delta, rpy_delta

        self._last_key = key
        key_lower = key.lower()

        linear = {
            "w": (0, self.position_step_m),
            "s": (0, -self.position_step_m),
            "a": (1, self.position_step_m),
            "d": (1, -self.position_step_m),
            "q": (2, self.position_step_m),
            "e": (2, -self.position_step_m),
        }
        angular = {
            "i": (0, self.rotation_step_rad),
            "k": (0, -self.rotation_step_rad),
            "j": (1, self.rotation_step_rad),
            "l": (1, -self.rotation_step_rad),
            "u": (2, self.rotation_step_rad),
            "o": (2, -self.rotation_step_rad),
        }

        if key_lower in linear:
            axis, delta = linear[key_lower]
            pos_delta[axis] += delta
        elif key_lower in angular:
            axis, delta = angular[key_lower]
            rpy_delta[axis] += delta

        return pos_delta, rpy_delta

    def get_gripper_cmd(self) -> Optional[float]:
        key = getattr(self, "_last_key", None)
        if key and key.lower() == "g":
            self._last_key = None  # Consume key
            self.gripper_open = not self.gripper_open
            return 1.0 if self.gripper_open else 0.0
        return None

    def get_record_trigger(self) -> Optional[str]:
        key = getattr(self, "_last_key", None)
        if key and key.lower() in ("r", "t"):
            self._last_key = None  # Consume key
            return "start" if key.lower() == "r" else "stop"
        return None

    def get_reset_trigger(self) -> bool:
        key = getattr(self, "_last_key", None)
        if key == " ":
            self._last_key = None  # Consume key
            return True
        return False

    def close(self) -> None:
        if hasattr(self, "keyboard"):
            self.keyboard.close()

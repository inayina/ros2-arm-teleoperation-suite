from typing import Tuple, Optional
from teleop_input.driver_base import TeleopDriverBase


class SpaceMouseDriver(TeleopDriverBase):
    """Stub driver for 3Dconnexion SpaceMouse.
    
    To implement:
      1. Install spacenavd: sudo apt install spacenavd
      2. sudo systemctl start spacenavd
      3. Use pyspacenav to read 6-DOF input events.
    """

    def __init__(self, position_step_m: float, rotation_step_rad: float):
        self.position_step_m = position_step_m
        self.rotation_step_rad = rotation_step_rad

    def initialize(self) -> bool:
        # Stub implementation always fails initialization 
        # so it's not used by mistake without the real dependency.
        return False

    def get_pose_delta(self) -> Tuple[list, list]:
        # Return empty deltas
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]

    def get_gripper_cmd(self) -> Optional[float]:
        return None

    def get_record_trigger(self) -> Optional[str]:
        return None

    def get_reset_trigger(self) -> bool:
        return False

    def close(self) -> None:
        pass

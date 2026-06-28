from abc import ABC, abstractmethod
from typing import Tuple, Optional


class TeleopDriverBase(ABC):
    """Base class for teleoperation input drivers."""

    @abstractmethod
    def initialize(self) -> bool:
        """Initialize the device.
        
        Returns:
            bool: True if successful, False otherwise.
        """
        pass

    @abstractmethod
    def get_pose_delta(self) -> Tuple[list, list]:
        """Get the pose delta for the current cycle.
        
        Returns:
            Tuple[list, list]: (position_delta_xyz, rpy_delta)
        """
        pass

    @abstractmethod
    def get_gripper_cmd(self) -> Optional[float]:
        """Get the gripper command.
        
        Returns:
            Optional[float]: 0.0 (closed) to 1.0 (open) or None if no change.
        """
        pass

    @abstractmethod
    def get_record_trigger(self) -> Optional[str]:
        """Get the record trigger command.
        
        Returns:
            Optional[str]: "start", "stop", or None.
        """
        pass

    @abstractmethod
    def get_reset_trigger(self) -> bool:
        """Get the reset trigger (e.g. to home the pose or reset scene).

        Returns:
            bool: True if reset is triggered, False otherwise.
        """
        pass

    @abstractmethod
    def close(self) -> None:
        """Release resources."""
        pass

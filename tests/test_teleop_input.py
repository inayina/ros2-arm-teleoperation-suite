import unittest
from unittest.mock import MagicMock, patch
from teleop_input.keyboard_driver import KeyboardDriver
from teleop_input.gamepad_driver import GamepadDriver

class TestTeleopInput(unittest.TestCase):
    def test_keyboard_driver(self):
        # Mock KeyboardReader to return specific keys
        with patch('teleop_input.keyboard_driver.KeyboardReader') as mock_reader_cls:
            mock_reader = MagicMock()
            mock_reader_cls.return_value = mock_reader
            mock_reader.enabled = True
            
            driver = KeyboardDriver(position_step_m=0.01, rotation_step_rad=0.1)
            assert driver.initialize() is True

            # Test no key pressed
            mock_reader.get_key.return_value = None
            pos, rpy = driver.get_pose_delta()
            assert pos == [0.0, 0.0, 0.0]
            assert rpy == [0.0, 0.0, 0.0]

            # Test X-forward key 'w'
            mock_reader.get_key.return_value = 'w'
            pos, rpy = driver.get_pose_delta()
            assert pos == [0.01, 0.0, 0.0]
            assert rpy == [0.0, 0.0, 0.0]

            # Test Gripper Toggle 'g'
            mock_reader.get_key.return_value = 'g'
            driver.get_pose_delta()  # will set self._last_key
            assert driver.get_gripper_cmd() == 1.0  # Toggles to open
            assert driver.get_gripper_cmd() is None  # consumed

            # Test Record Trigger 'r' (start) and 't' (stop)
            mock_reader.get_key.return_value = 'r'
            driver.get_pose_delta()
            assert driver.get_record_trigger() == 'start'
            
            mock_reader.get_key.return_value = 't'
            driver.get_pose_delta()
            assert driver.get_record_trigger() == 'stop'

            # Test Reset Trigger ' '
            mock_reader.get_key.return_value = ' '
            driver.get_pose_delta()
            assert driver.get_reset_trigger() is True

    def test_gamepad_driver_translation_mode(self):
        driver = GamepadDriver(position_step_m=0.01, rotation_step_rad=0.1)
        
        # Mode Toggle released: ABS_Z = 0.0 (Translation Mode)
        driver.axes["ABS_Z"] = 0.0
        driver.axes["ABS_X"] = 0.5  # Left Stick Y-axis translation
        driver.axes["ABS_Y"] = -0.5 # Left Stick X-axis translation
        driver.axes["ABS_RY"] = 0.25 # Right Stick Z-axis translation
        driver.axes["ABS_RX"] = 1.0  # Should be ignored in Translation Mode
        
        pos, rpy = driver.get_pose_delta()
        
        # pos_delta[0] = -ABS_Y * step = -(-0.5) * 0.01 = 0.005
        # pos_delta[1] = -ABS_X * step = -(0.5) * 0.01 = -0.005
        # pos_delta[2] = -ABS_RY * step = -(0.25) * 0.01 = -0.0025
        assert abs(pos[0] - 0.005) < 1e-6
        assert abs(pos[1] - (-0.005)) < 1e-6
        assert abs(pos[2] - (-0.0025)) < 1e-6
        assert rpy == [0.0, 0.0, 0.0]

    def test_gamepad_driver_rotation_mode(self):
        driver = GamepadDriver(position_step_m=0.01, rotation_step_rad=0.1)
        
        # Mode Toggle pressed: ABS_Z = 0.8 (Rotation Mode)
        driver.axes["ABS_Z"] = 0.8
        driver.axes["ABS_X"] = 0.5  # Left Stick: Roll
        driver.axes["ABS_Y"] = -0.5 # Left Stick: Pitch
        driver.axes["ABS_RX"] = 0.25 # Right Stick: Yaw
        driver.axes["ABS_RY"] = 1.0  # Should be ignored in Rotation Mode
        
        pos, rpy = driver.get_pose_delta()
        
        # rpy_delta[0] = -ABS_X * step = -(0.5) * 0.1 = -0.05 (Roll)
        # rpy_delta[1] = -ABS_Y * step = -(-0.5) * 0.1 = 0.05 (Pitch)
        # rpy_delta[2] = -ABS_RX * step = -(0.25) * 0.1 = -0.025 (Yaw)
        assert pos == [0.0, 0.0, 0.0]
        assert abs(rpy[0] - (-0.05)) < 1e-6
        assert abs(rpy[1] - 0.05) < 1e-6
        assert abs(rpy[2] - (-0.025)) < 1e-6

    def test_gamepad_driver_buttons(self):
        driver = GamepadDriver(position_step_m=0.01, rotation_step_rad=0.1)

        # Test explicit gripper command logic (A=open, B=close, X=toggle)
        assert driver.get_gripper_cmd() is None
        driver._gripper_cmd = 1.0
        assert driver.get_gripper_cmd() == 1.0
        assert driver.get_gripper_cmd() is None

        driver._gripper_cmd = 0.0
        assert driver.get_gripper_cmd() == 0.0
        assert driver.get_gripper_cmd() is None

        # Test Reset trigger logic
        assert driver.get_reset_trigger() is False
        driver._reset_flag = True
        assert driver.get_reset_trigger() is True
        assert driver.get_reset_trigger() is False

        # Test Record triggers
        assert driver.get_record_trigger() is None
        driver._record_cmd = "start"
        assert driver.get_record_trigger() == "start"
        assert driver.get_record_trigger() is None

    def test_gamepad_driver_axis_range_normalization(self):
        driver = GamepadDriver(position_step_m=0.01, rotation_step_rad=0.1)

        driver.axis_ranges["ABS_X"] = (0, 255, 8)
        assert driver._normalize_axis("ABS_X", 128) == 0.0
        assert driver._normalize_axis("ABS_X", 255) > 0.9
        assert driver._normalize_axis("ABS_X", 0) < -0.9

        driver.axis_ranges["ABS_RY"] = (-32768, 32767, 0)
        assert driver._normalize_axis("ABS_RY", 0) == 0.0
        assert driver._normalize_axis("ABS_RY", 32767) > 0.9
        assert driver._normalize_axis("ABS_RY", -32768) < -0.9

        driver.axis_ranges["ABS_Z"] = (0, 1023, 0)
        assert driver._normalize_axis("ABS_Z", 0) == 0.0
        assert driver._normalize_axis("ABS_Z", 1023) == 1.0

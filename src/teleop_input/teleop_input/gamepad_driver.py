import fcntl
import os
import struct
import threading
import time
from typing import Optional, Tuple

from teleop_input.driver_base import TeleopDriverBase

try:
    import inputs
    _HAS_INPUTS = True
except ImportError:
    _HAS_INPUTS = False

ABS_CODES = {
    'ABS_X': 0x00,
    'ABS_Y': 0x01,
    'ABS_Z': 0x02,
    'ABS_RX': 0x03,
    'ABS_RY': 0x04,
    'ABS_RZ': 0x05,
    'ABS_HAT0X': 0x10,
    'ABS_HAT0Y': 0x11,
}
TRIGGER_AXES = {'ABS_Z', 'ABS_RZ'}


class GamepadDriver(TeleopDriverBase):
    """Gamepad input driver using inputs library (Xbox/PS controllers)."""

    def __init__(self, position_step_m: float, rotation_step_rad: float):
        self.position_step_m = position_step_m
        self.rotation_step_rad = rotation_step_rad

        self._running = False
        self._thread = None
        self.last_error = ''

        self.axes = {
            'ABS_X': 0.0,
            'ABS_Y': 0.0,
            'ABS_RX': 0.0,
            'ABS_RY': 0.0,
            'ABS_Z': 0.0,
            'ABS_RZ': 0.0,
            'ABS_HAT0X': 0.0,
            'ABS_HAT0Y': 0.0,
        }
        self.buttons = {
            'BTN_SOUTH': 0,  # A / Cross
            'BTN_NORTH': 0,  # Y / Triangle
            'BTN_EAST': 0,   # B / Circle
            'BTN_WEST': 0,   # X / Square
            'BTN_TR': 0,     # R1
            'BTN_TL': 0,     # L1
        }

        self.axis_ranges = {}
        self._gripper_cmd = None
        self._gripper_open = True
        self._record_cmd = None
        self._reset_flag = False

    def _find_and_register_gamepad(self) -> bool:
        """Scan /sys/class/input/event* for gamepads not detected by inputs."""
        import glob
        import os
        for path in sorted(glob.glob('/sys/class/input/event*')):
            name_file = os.path.join(path, 'device', 'name')
            if os.path.exists(name_file):
                try:
                    with open(name_file, 'r') as f:
                        device_name = f.read().strip().lower()
                    if any(
                        x in device_name
                        for x in ('xbox', 'controller', 'gamepad', 'joystick')
                    ):
                        event_num = os.path.basename(path).replace('event', '')
                        real_dev_path = f'/dev/input/event{event_num}'
                        fake_path = f'/dev/input/by-id/usb-xbox{event_num}-event-joystick'
                        gp = inputs.GamePad(inputs.devices, fake_path, real_dev_path)
                        inputs.devices.gamepads.append(gp)
                        inputs.devices._update_all_devices()
                        return True
                except OSError as exc:
                    self.last_error = f'cannot inspect {name_file}: {exc}'
                except Exception as exc:
                    self.last_error = f'gamepad auto-detection failed: {exc}'
        return False

    def initialize(self) -> bool:
        if not _HAS_INPUTS:
            self.last_error = (
                "Python package 'inputs' is not installed in the ROS 2 "
                'runtime environment.'
            )
            return False
        try:
            pads = inputs.devices.gamepads
            if not pads:
                self._find_and_register_gamepad()
                pads = inputs.devices.gamepads
            if not pads:
                self.last_error = (
                    'no gamepad device found; check USB/Bluetooth connection '
                    'and /dev/input permissions'
                )
                return False
        except Exception as e:
            self.last_error = f'gamepad initialization exception: {e}'
            return False

        self._load_axis_ranges(pads)
        self.last_error = f'detected {len(pads)} gamepad device(s)'
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        return True

    def _load_axis_ranges(self, pads) -> None:
        """Read evdev ABS ranges so sticks normalize correctly across drivers."""
        self.axis_ranges = {}
        ev_iocgabs_base = (2 << 30) | (24 << 16) | (ord('E') << 8) | 0x40
        for pad in pads:
            try:
                path = pad.get_char_device_path()
            except Exception:
                continue
            try:
                fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
            except OSError as exc:
                self.last_error = f'cannot inspect gamepad axis ranges at {path}: {exc}'
                continue
            try:
                for code, abs_code in ABS_CODES.items():
                    buf = bytearray(24)
                    try:
                        fcntl.ioctl(fd, ev_iocgabs_base + abs_code, buf, True)
                    except OSError:
                        continue
                    value, minimum, maximum, _fuzz, flat, _resolution = struct.unpack('iiiiii', buf)
                    if minimum < maximum:
                        self.axis_ranges[code] = (minimum, maximum, flat)
                        if code in self.axes:
                            self.axes[code] = self._normalize_axis(code, value)
                if self.axis_ranges:
                    return
            finally:
                os.close(fd)

    def _normalize_axis(self, code: str, value: int) -> float:
        if code.startswith('ABS_HAT'):
            return float(value)

        axis_range = self.axis_ranges.get(code)
        if axis_range is not None:
            minimum, maximum, flat = axis_range
            span = float(maximum - minimum)
            if span <= 0.0:
                return 0.0
            if code in TRIGGER_AXES:
                val = (float(value) - minimum) / span
                return max(0.0, min(1.0, val))

            center = (maximum + minimum) / 2.0
            half_span = max(center - minimum, maximum - center, 1.0)
            val = (float(value) - center) / half_span
            deadzone = max(0.15, float(flat) / half_span if flat > 0 else 0.0)
            if abs(val) < deadzone:
                return 0.0
            return max(-1.0, min(1.0, val))

        if code in TRIGGER_AXES:
            if value <= 255:
                return max(0.0, min(1.0, value / 255.0))
            if value <= 1023:
                return max(0.0, min(1.0, value / 1023.0))
            return max(0.0, min(1.0, value / 65535.0))

        if value < 0:
            val = value / 32768.0
        elif value > 32767:
            val = (value - 32768.0) / 32767.0
        else:
            val = value / 32767.0
        if abs(val) < 0.15:
            return 0.0
        return max(-1.0, min(1.0, val))

    def _read_loop(self):
        while self._running:
            try:
                events = inputs.get_gamepad()
                for event in events:
                    if event.ev_type == 'Absolute':
                        if event.code in self.axes:
                            self.axes[event.code] = self._normalize_axis(event.code, event.state)
                    elif event.ev_type == 'Key':
                        if event.code in self.buttons:
                            self.buttons[event.code] = event.state
                            if event.state == 1:
                                if event.code == 'BTN_SOUTH':
                                    self._gripper_cmd = 1.0
                                    self._gripper_open = True
                                elif event.code == 'BTN_EAST':
                                    self._gripper_cmd = 0.0
                                    self._gripper_open = False
                                elif event.code == 'BTN_WEST':
                                    self._gripper_open = not self._gripper_open
                                    self._gripper_cmd = 1.0 if self._gripper_open else 0.0
                                elif event.code == 'BTN_NORTH':
                                    self._reset_flag = True
                                elif event.code == 'BTN_TR':
                                    self._record_cmd = 'start'
                                elif event.code == 'BTN_TL':
                                    self._record_cmd = 'stop'
            except Exception as exc:
                self.last_error = f'gamepad read loop error: {exc}'
                time.sleep(0.1)

    def get_pose_delta(self) -> Tuple[list, list]:
        pos_delta = [0.0, 0.0, 0.0]
        rpy_delta = [0.0, 0.0, 0.0]

        # Check if LT trigger is pressed to toggle Rotation Mode
        is_rotation_mode = self.axes.get('ABS_Z', 0.0) > 0.5

        if is_rotation_mode:
            # Rotation Mode:
            # Left Stick: Roll (X) and Pitch (Y)
            rpy_delta[0] = -self.axes['ABS_X'] * self.rotation_step_rad
            rpy_delta[1] = -self.axes['ABS_Y'] * self.rotation_step_rad
            # Right Stick: Yaw (Z)
            rpy_delta[2] = -self.axes['ABS_RX'] * self.rotation_step_rad
        else:
            # Translation Mode:
            # Left Stick: Forward/Back (X) and Left/Right (Y)
            pos_delta[0] = -self.axes['ABS_Y'] * self.position_step_m
            pos_delta[1] = -self.axes['ABS_X'] * self.position_step_m
            # Right Stick: Up/Down (Z)
            pos_delta[2] = -self.axes['ABS_RY'] * self.position_step_m

        return pos_delta, rpy_delta

    def get_gripper_cmd(self) -> Optional[float]:
        cmd = self._gripper_cmd
        self._gripper_cmd = None
        return cmd

    def get_record_trigger(self) -> Optional[str]:
        cmd = self._record_cmd
        self._record_cmd = None
        return cmd

    def get_reset_trigger(self) -> bool:
        res = self._reset_flag
        self._reset_flag = False
        return res

    def close(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=0.2)

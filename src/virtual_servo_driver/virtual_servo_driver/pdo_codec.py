"""CANopen DS402 PDO encode/decode helpers (pure Python, unit-testable).

RPDO1 (master -> drive): target torque (DS402 0x6071, int16).
TPDO1 (drive -> master): actual position (0x6064, int32) + velocity (0x606C, int16).
TPDO2 (drive -> master): statusword (0x6041, uint16) + actual torque (0x6077, int16).
"""
import math
import os
import cantools
from ament_index_python.packages import get_package_share_directory

TORQUE_SCALE = 0.001          # N*m per bit
VELOCITY_SCALE = 0.001        # rad/s per bit
COUNTS_PER_REV = 131072       # 17-bit encoder
_TWO_PI = 2.0 * math.pi

# Load the DBC database
_dbc_path = os.path.join(get_package_share_directory('virtual_servo_driver'), 'config', 'ds402.dbc')
DB = cantools.database.load_file(_dbc_path)

def _clamp_i16(v: int) -> int:
    return max(-32768, min(32767, v))

def _clamp_i32(v: int) -> int:
    return max(-2147483648, min(2147483647, v))

def rad_to_counts(rad: float) -> int:
    return _clamp_i32(int(round(rad / _TWO_PI * COUNTS_PER_REV)))

def counts_to_rad(counts: int) -> float:
    return counts / COUNTS_PER_REV * _TWO_PI

def pack_rpdo_torque(torque_nm: float) -> bytes:
    """Encode target torque as an 8-byte RPDO1."""
    raw = _clamp_i16(int(round(torque_nm / TORQUE_SCALE)))
    msg = DB.get_message_by_name("RPDO1_TargetTorque")
    return msg.encode({"TargetTorque": raw})

def unpack_rpdo_torque(data: bytes) -> float:
    msg = DB.get_message_by_name("RPDO1_TargetTorque")
    decoded = msg.decode(data)
    return decoded["TargetTorque"] * TORQUE_SCALE

def pack_tpdo1_position(pos_rad: float, vel_rad_s: float) -> bytes:
    """Encode actual position + velocity into TPDO1."""
    pos = rad_to_counts(pos_rad)
    vel = _clamp_i16(int(round(vel_rad_s / VELOCITY_SCALE)))
    msg = DB.get_message_by_name("TPDO1_PositionVelocity")
    return msg.encode({"ActualPosition": pos, "ActualVelocity": vel})

def unpack_tpdo1_position(data: bytes) -> tuple[float, float]:
    msg = DB.get_message_by_name("TPDO1_PositionVelocity")
    decoded = msg.decode(data)
    return counts_to_rad(decoded["ActualPosition"]), decoded["ActualVelocity"] * VELOCITY_SCALE

def pack_tpdo2_status(statusword: int, torque_nm: float) -> bytes:
    """Encode statusword + actual torque into TPDO2."""
    tau = _clamp_i16(int(round(torque_nm / TORQUE_SCALE)))
    msg = DB.get_message_by_name("TPDO2_StatusTorque")
    return msg.encode({"StatusWord": statusword, "ActualTorque": tau})

def unpack_tpdo2_status(data: bytes) -> tuple[int, float]:
    msg = DB.get_message_by_name("TPDO2_StatusTorque")
    decoded = msg.decode(data)
    return decoded["StatusWord"], decoded["ActualTorque"] * TORQUE_SCALE

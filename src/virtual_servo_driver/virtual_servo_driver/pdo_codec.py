"""CANopen DS402 PDO encode/decode helpers (pure Python, unit-testable).

RPDO1 (master -> drive): target torque (DS402 0x6071, int16).
TPDO1 (drive -> master): actual position (0x6064, int32) + velocity (0x606C, int16).
TPDO2 (drive -> master): statusword (0x6041, uint16) + actual torque (0x6077, int16).
"""
import math
import struct

TORQUE_SCALE = 0.001          # N*m per bit
VELOCITY_SCALE = 0.001        # rad/s per bit
COUNTS_PER_REV = 131072       # 17-bit encoder
_TWO_PI = 2.0 * math.pi


def _clamp_i16(v: int) -> int:
    return max(-32768, min(32767, v))


def _clamp_i32(v: int) -> int:
    return max(-2147483648, min(2147483647, v))


def rad_to_counts(rad: float) -> int:
    return _clamp_i32(int(round(rad / _TWO_PI * COUNTS_PER_REV)))


def counts_to_rad(counts: int) -> float:
    return counts / COUNTS_PER_REV * _TWO_PI


def pack_rpdo_torque(torque_nm: float) -> bytes:
    """Encode target torque as an 8-byte RPDO1 (int16 + padding)."""
    raw = _clamp_i16(int(round(torque_nm / TORQUE_SCALE)))
    return struct.pack("<h6x", raw)


def unpack_rpdo_torque(data: bytes) -> float:
    raw = struct.unpack("<h6x", data[:8])[0]
    return raw * TORQUE_SCALE


def pack_tpdo1_position(pos_rad: float, vel_rad_s: float) -> bytes:
    """Encode actual position + velocity into TPDO1."""
    pos = rad_to_counts(pos_rad)
    vel = _clamp_i16(int(round(vel_rad_s / VELOCITY_SCALE)))
    return struct.pack("<ih2x", pos, vel)


def unpack_tpdo1_position(data: bytes) -> tuple[float, float]:
    pos, vel = struct.unpack("<ih2x", data[:8])
    return counts_to_rad(pos), vel * VELOCITY_SCALE


def pack_tpdo2_status(statusword: int, torque_nm: float) -> bytes:
    """Encode statusword + actual torque into TPDO2."""
    tau = _clamp_i16(int(round(torque_nm / TORQUE_SCALE)))
    return struct.pack("<Hh4x", statusword & 0xFFFF, tau)


def unpack_tpdo2_status(data: bytes) -> tuple[int, float]:
    sw, tau = struct.unpack("<Hh4x", data[:8])
    return sw, tau * TORQUE_SCALE

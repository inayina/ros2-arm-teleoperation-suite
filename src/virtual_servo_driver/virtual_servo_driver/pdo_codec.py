"""CANopen DS402 PDO encode/decode helpers (pure Python, unit-testable).

RPDO (master -> drive): target torque (DS402 0x6071, int16, per-mille of rated).
TPDO (drive -> master): actual position (0x6064, int32 counts),
                        actual velocity (0x606C, int32),
                        actual torque (0x6077, int16).
"""
import math
import struct

TORQUE_SCALE = 0.001          # N*m per bit
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
    """Encode target torque as an 8-byte RPDO (int16 + padding)."""
    raw = _clamp_i16(int(round(torque_nm / TORQUE_SCALE)))
    return struct.pack("<h6x", raw)


def unpack_rpdo_torque(data: bytes) -> float:
    raw = struct.unpack("<h6x", data[:8])[0]
    return raw * TORQUE_SCALE


def pack_tpdo_feedback(pos_rad: float, vel_rad_s: float, torque_nm: float) -> bytes:
    """Encode actual position(int32) + torque(int16) into an 8-byte TPDO."""
    pos = rad_to_counts(pos_rad)
    tau = _clamp_i16(int(round(torque_nm / TORQUE_SCALE)))
    vel = _clamp_i16(int(round(vel_rad_s * 1000.0)))  # milli-rad/s
    return struct.pack("<ihh", pos, vel, tau)


def unpack_tpdo_feedback(data: bytes):
    pos, vel, tau = struct.unpack("<ihh", data[:8])
    return counts_to_rad(pos), vel / 1000.0, tau * TORQUE_SCALE

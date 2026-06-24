import math

from virtual_servo_driver import pdo_codec


def test_rpdo_torque_roundtrip():
    for nm in (-5.0, 0.0, 1.234, 30.0):
        data = pdo_codec.pack_rpdo_torque(nm)
        assert len(data) == 8
        back = pdo_codec.unpack_rpdo_torque(data)
        assert abs(back - nm) < pdo_codec.TORQUE_SCALE * 2


def test_rpdo_torque_max_boundary():
    max_nm = 32767 * pdo_codec.TORQUE_SCALE
    data = pdo_codec.pack_rpdo_torque(max_nm + 10.0)
    assert pdo_codec.unpack_rpdo_torque(data) == max_nm


def test_tpdo1_roundtrip():
    pos, vel = 1.5707, 0.5
    data = pdo_codec.pack_tpdo1_position(pos, vel)
    assert len(data) == 8
    p, v = pdo_codec.unpack_tpdo1_position(data)
    assert abs(p - pos) < 1e-3
    assert abs(v - vel) < 1e-2


def test_tpdo2_roundtrip():
    sw, tau = 0x0027, 2.0
    data = pdo_codec.pack_tpdo2_status(sw, tau)
    assert len(data) == 8
    s, t = pdo_codec.unpack_tpdo2_status(data)
    assert s == sw
    assert abs(t - tau) < pdo_codec.TORQUE_SCALE * 2


def test_counts_roundtrip():
    counts = pdo_codec.rad_to_counts(math.pi)
    assert abs(pdo_codec.counts_to_rad(counts) - math.pi) < 1e-3

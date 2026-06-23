import math

from virtual_servo_driver import pdo_codec


def test_rpdo_torque_roundtrip():
    for nm in (-5.0, 0.0, 1.234, 30.0):
        data = pdo_codec.pack_rpdo_torque(nm)
        assert len(data) == 8
        back = pdo_codec.unpack_rpdo_torque(data)
        assert abs(back - nm) < pdo_codec.TORQUE_SCALE * 2


def test_tpdo_feedback_roundtrip():
    pos, vel, tau = 1.5707, 0.5, 2.0
    data = pdo_codec.pack_tpdo_feedback(pos, vel, tau)
    assert len(data) == 8
    p, v, t = pdo_codec.unpack_tpdo_feedback(data)
    assert abs(p - pos) < 1e-3
    assert abs(v - vel) < 1e-2
    assert abs(t - tau) < pdo_codec.TORQUE_SCALE * 2


def test_counts_roundtrip():
    counts = pdo_codec.rad_to_counts(math.pi)
    assert abs(pdo_codec.counts_to_rad(counts) - math.pi) < 1e-3

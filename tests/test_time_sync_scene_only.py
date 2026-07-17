from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import Image, JointState

from lerobot_recorder.time_sync import MultiModalSync


class FakeLogger:
    def warn(self, _text):
        pass


class FakeNode:
    def create_subscription(self, *_args, **_kwargs):
        return object()

    def get_clock(self):
        return SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=1_000_000_000))

    def get_logger(self):
        return FakeLogger()


def stamp(message, seconds: int = 1):
    message.header.stamp.sec = seconds
    message.header.stamp.nanosec = 0
    return message


def test_scene_only_sync_does_not_wait_for_wrist():
    emitted = []
    sync = MultiModalSync(
        FakeNode(),
        lambda *messages: emitted.append(messages),
        slop=0.2,
        include_images=True,
        visual_keys=("color",),
    )
    sync._update("joint", stamp(JointState()))
    sync._update("ee", stamp(PoseStamped()))
    sync._update("ft", stamp(WrenchStamped()))
    sync._update("object", stamp(PoseStamped()))
    color = stamp(Image())
    sync._update("color", color)

    assert len(emitted) == 1
    assert emitted[0][4] is None
    assert sync.diagnostics_snapshot()["enabled_visual_streams"] == ["scene"]

    sync._update("color", color)
    assert len(emitted) == 1
    assert sync.reject_counts["reused"] == 1

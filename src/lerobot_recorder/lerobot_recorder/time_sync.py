"""Multi-modal ROS message synchronization for LeRobot recording."""

from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState


class MultiModalSync:
    """Camera-driven latest-sample synchronizer.

    ApproximateTimeSynchronizer is brittle here because joint state, MuJoCo
    truth, and four camera bridge nodes stamp from separate ROS nodes. Batch
    collection needs a frame whenever the scene camera ticks and every required
    modality has a recent sample.
    """

    def __init__(self, node, callback, queue_size: int = 30, slop: float = 0.05,
                 include_images: bool = True):
        del queue_size  # latest-cache sync is O(1); keep the public constructor stable.
        self.node = node
        self.callback = callback
        self.slop = max(0.0, float(slop))
        self.latest = {}
        self.last_emitted_stamp = {}
        self.include_images = include_images
        self._last_warn_s = 0.0
        self._last_trigger_stamp = 0.0
        self._subscriptions = [
            node.create_subscription(
                JointState,
                "/joint_states",
                lambda msg: self._update("joint", msg),
                qos_profile_sensor_data,
            ),
            node.create_subscription(
                PoseStamped,
                "/ee_pose",
                lambda msg: self._update("ee", msg),
                qos_profile_sensor_data,
            ),
            node.create_subscription(
                WrenchStamped,
                "/ft_sensor",
                lambda msg: self._update("ft", msg),
                qos_profile_sensor_data,
            ),
            node.create_subscription(
                Image,
                "/camera/color/image_raw",
                lambda msg: self._update("color", msg),
                qos_profile_sensor_data,
            ),
            node.create_subscription(
                Image,
                "/camera/wrist/color/image_raw",
                lambda msg: self._update("wrist", msg),
                qos_profile_sensor_data,
            ),
            node.create_subscription(
                PoseStamped,
                "/sim/object_pose",
                lambda msg: self._update("object", msg),
                qos_profile_sensor_data,
            ),
        ]

    def _update(self, key: str, msg):
        self.latest[key] = msg
        if key == ("color" if self.include_images else "joint"):
            if not self.include_images:
                stamp = self._stamp_sec(msg)
                if stamp - self._last_trigger_stamp < 0.0099:
                    return
                self._last_trigger_stamp = stamp
            self._try_emit(msg)

    def _try_emit(self, color_msg: Image):
        required = [
            "joint",
            "ee",
            "ft",
            "object",
        ]
        if self.include_images:
            required.extend(["color", "wrist"])
        required = tuple(required)
        missing = [key for key in required if key not in self.latest]
        if missing:
            self._warn_throttled(f"waiting for recorder modalities: {', '.join(missing)}")
            return

        stale = self._stale_keys(color_msg, required)
        if stale:
            self._warn_throttled(
                f"recorder modalities outside sync_slop={self.slop:.3f}s: {', '.join(stale)}"
            )
            return

        reused = [
            key for key in required
            if self._stamp_sec(self.latest[key]) <= self.last_emitted_stamp.get(key, -1.0)
        ]
        if reused:
            self._warn_throttled(f"waiting for unique recorder samples: {', '.join(reused)}")
            return

        self.callback(
            self.latest["joint"],
            self.latest["ee"],
            self.latest["ft"],
            self.latest.get("color"),
            self.latest.get("wrist"),
            self.latest["object"],
        )
        for key in required:
            self.last_emitted_stamp[key] = self._stamp_sec(self.latest[key])

    def _stale_keys(self, anchor_msg, keys: tuple[str, ...]) -> list[str]:
        if self.slop <= 0.0:
            return []
        anchor = self._stamp_sec(anchor_msg)
        stale = []
        for key in keys:
            delta = abs(self._stamp_sec(self.latest[key]) - anchor)
            if delta > self.slop:
                stale.append(f"{key}({delta:.3f}s)")
        return stale

    @staticmethod
    def _stamp_sec(msg) -> float:
        stamp = msg.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _warn_throttled(self, text: str):
        now = self.node.get_clock().now().nanoseconds * 1e-9
        if now - self._last_warn_s < 5.0:
            return
        self._last_warn_s = now
        self.node.get_logger().warn(text)

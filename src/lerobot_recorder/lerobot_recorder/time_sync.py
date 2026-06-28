"""Multi-modal ROS message synchronization for LeRobot recording."""

import message_filters
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState


class MultiModalSync:
    """Approximate synchronization around the 30 Hz camera cadence."""

    def __init__(self, node, callback, queue_size: int = 30, slop: float = 0.05):
        self.joint_sub = message_filters.Subscriber(
            node, JointState, "/joint_states", qos_profile=qos_profile_sensor_data
        )
        self.ee_sub = message_filters.Subscriber(
            node, PoseStamped, "/ee_pose", qos_profile=qos_profile_sensor_data
        )
        self.ft_sub = message_filters.Subscriber(
            node, WrenchStamped, "/ft_sensor", qos_profile=qos_profile_sensor_data
        )
        self.color_sub = message_filters.Subscriber(
            node, Image, "/camera/color/image_raw", qos_profile=qos_profile_sensor_data
        )
        self.depth_sub = message_filters.Subscriber(
            node, Image, "/camera/depth/image_raw", qos_profile=qos_profile_sensor_data
        )
        self.obj_sub = message_filters.Subscriber(
            node, PoseStamped, "/sim/object_pose", qos_profile=qos_profile_sensor_data
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [
                self.joint_sub,
                self.ee_sub,
                self.ft_sub,
                self.color_sub,
                self.depth_sub,
                self.obj_sub,
            ],
            queue_size=queue_size,
            slop=slop,
        )
        self.sync.registerCallback(callback)

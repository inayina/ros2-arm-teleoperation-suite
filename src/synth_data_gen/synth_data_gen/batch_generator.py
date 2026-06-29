#!/usr/bin/env python3
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String, Float64
from std_srvs.srv import Trigger
import tf2_ros

class BatchGenerator(Node):
    def __init__(self):
        super().__init__('batch_generator')
        self.declare_parameter('episodes', 10)
        self.declare_parameter('seed', 42)
        self.declare_parameter('hover_duration', 3.0)
        self.declare_parameter('descend_duration', 2.5)
        self.declare_parameter('grasp_pause', 1.0)
        self.declare_parameter('lift_duration', 2.0)
        self.declare_parameter('hover_height', 0.45)
        self.declare_parameter('pick_height_offset', 0.05)
        self.declare_parameter('reset_timeout', 5.0)
        
        self.episodes = self.get_parameter('episodes').value
        self.seed = self.get_parameter('seed').value
        self.hover_duration = float(self.get_parameter('hover_duration').value)
        self.descend_duration = float(self.get_parameter('descend_duration').value)
        self.grasp_pause = float(self.get_parameter('grasp_pause').value)
        self.lift_duration = float(self.get_parameter('lift_duration').value)
        self.hover_height = float(self.get_parameter('hover_height').value)
        self.pick_height_offset = float(self.get_parameter('pick_height_offset').value)
        self.reset_timeout = float(self.get_parameter('reset_timeout').value)
        
        self.initial_pose = None
        self.latest_object_pose = None
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.cli_reset = self.create_client(Trigger, '/sim/reset_scene')
        self.pub_rec = self.create_publisher(String, '/teleop/record_trigger', 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/teleop/cmd_pose', 10)
        self.pub_grip = self.create_publisher(Float64, '/teleop/gripper_cmd', 10)
        self.sub_object = self.create_subscription(
            PoseStamped, '/sim/object_pose', self._on_object_pose, 10)
        
        # Start run_batch thread
        self.get_logger().info(f'Batch generator ready. Running {self.episodes} episodes.')
        import threading
        threading.Thread(target=self.run_batch).start()

    def run_batch(self):
        # Wait for tf to become available
        while True:
            try:
                trans = self.tf_buffer.lookup_transform('panda_link0', 'panda_ee', rclpy.time.Time())
                p = Pose()
                p.position.x = trans.transform.translation.x
                p.position.y = trans.transform.translation.y
                p.position.z = trans.transform.translation.z
                p.orientation = trans.transform.rotation
                self.initial_pose = p
                break
            except Exception as e:
                self.get_logger().info(f'Wait for tf... {e}')
                time.sleep(0.5)

        for i in range(self.episodes):
            self.get_logger().info(f'--- Starting Episode {i+1}/{self.episodes} ---')
            
            # 1. Reset Scene
            self._reset_scene(timeout=self.reset_timeout)
            
            # Let the scene settle
            time.sleep(1.0)
            object_pose = self._wait_for_object_pose(timeout=3.0)
            
            # Update initial pose after reset
            try:
                trans = self.tf_buffer.lookup_transform('panda_link0', 'panda_ee', rclpy.time.Time())
                self.initial_pose.position.x = trans.transform.translation.x
                self.initial_pose.position.y = trans.transform.translation.y
                self.initial_pose.position.z = trans.transform.translation.z
                self.initial_pose.orientation = trans.transform.rotation
            except Exception:
                pass
            
            # 2. Start Recording
            self.pub_rec.publish(String(data='start'))
            time.sleep(0.5)

            # 3. Execute "Pick" Motion
            start_p = [self.initial_pose.position.x, self.initial_pose.position.y, self.initial_pose.position.z]
            start_q = [self.initial_pose.orientation.w, self.initial_pose.orientation.x, self.initial_pose.orientation.y, self.initial_pose.orientation.z]
            
            down_q = [0.0, 1.0, 0.0, 0.0]  # W, X, Y, Z
            
            # 1. Move to hover pose and orient straight down over the randomized object.
            object_x = object_pose.pose.position.x if object_pose else 0.4
            object_y = object_pose.pose.position.y if object_pose else 0.0
            object_z = object_pose.pose.position.z if object_pose else 0.05
            hover_p = [object_x, object_y, max(0.35, object_z + self.hover_height)]
            self._move_arm_smooth(
                start_p, hover_p, duration=self.hover_duration,
                start_ori=start_q, end_ori=down_q
            )
            
            # 2. Move down to object
            pick_p = [object_x, object_y, max(0.04, object_z + self.pick_height_offset)]
            self._move_arm_smooth(
                hover_p, pick_p, duration=self.descend_duration,
                start_ori=down_q, end_ori=down_q
            )
            
            # 3. Close gripper
            self.pub_grip.publish(Float64(data=0.0))
            time.sleep(self.grasp_pause)
            
            # 4. Move up
            self._move_arm_smooth(
                pick_p, hover_p, duration=self.lift_duration,
                start_ori=down_q, end_ori=down_q
            )
            
            # 4. Stop Recording
            self.pub_rec.publish(String(data='stop'))
            time.sleep(1.0)
            
            self.get_logger().info(f'--- Finished Episode {i+1}/{self.episodes} ---')
            
        self.get_logger().info('Batch generation completed successfully.')
        import os
        os._exit(0)

    def _reset_scene(self, timeout=5.0):
        if not self.cli_reset.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn('/sim/reset_scene unavailable; continuing without scene reset.')
            return False

        future = self.cli_reset.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if future.done():
                try:
                    result = future.result()
                    if result and result.success:
                        self.get_logger().info(f'/sim/reset_scene: {result.message}')
                    elif result:
                        self.get_logger().warn(f'/sim/reset_scene failed: {result.message}')
                    return bool(result and result.success)
                except Exception as exc:
                    self.get_logger().warn(f'/sim/reset_scene call failed: {exc}')
                    return False
            time.sleep(0.02)

        self.get_logger().warn('/sim/reset_scene timed out; continuing with current scene.')
        return False

    def _on_object_pose(self, msg):
        self.latest_object_pose = msg

    def _wait_for_object_pose(self, timeout=3.0):
        deadline = time.monotonic() + timeout
        last_pose = self.latest_object_pose
        while time.monotonic() < deadline:
            if self.latest_object_pose is not None and self.latest_object_pose is not last_pose:
                return self.latest_object_pose
            if self.latest_object_pose is not None and last_pose is None:
                return self.latest_object_pose
            time.sleep(0.05)
        if self.latest_object_pose is None:
            self.get_logger().warn('No /sim/object_pose received; falling back to nominal pick pose.')
        return self.latest_object_pose

    def _move_arm_smooth(self, start_pos, end_pos, duration=1.0, start_ori=None, end_ori=None):
        steps = int(duration * 100)
        dt = duration / steps
        
        if start_ori is None:
            start_ori = [1.0, 0.0, 0.0, 0.0]
        if end_ori is None:
            end_ori = start_ori
            
        # Ensure shortest path on quaternion sphere
        dot = sum(start_ori[j] * end_ori[j] for j in range(4))
        if dot < 0.0:
            end_ori = [-x for x in end_ori]
            
        for i in range(steps):
            alpha = (i + 1) / float(steps)
            pos = [start_pos[j] * (1.0 - alpha) + end_pos[j] * alpha for j in range(3)]
            
            qw = start_ori[0] * (1.0 - alpha) + end_ori[0] * alpha
            qx = start_ori[1] * (1.0 - alpha) + end_ori[1] * alpha
            qy = start_ori[2] * (1.0 - alpha) + end_ori[2] * alpha
            qz = start_ori[3] * (1.0 - alpha) + end_ori[3] * alpha
            mag = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
            if mag < 1e-6:
                mag = 1.0
            ori = [qw/mag, qx/mag, qy/mag, qz/mag]
            
            self._move_arm(pos, ori)
            time.sleep(dt)

    def _move_arm(self, pos, ori=None):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "panda_link0"
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        if ori is not None:
            msg.pose.orientation.w = float(ori[0])
            msg.pose.orientation.x = float(ori[1])
            msg.pose.orientation.y = float(ori[2])
            msg.pose.orientation.z = float(ori[3])
        elif self.initial_pose:
            msg.pose.orientation = self.initial_pose.orientation
        else:
            msg.pose.orientation.x = 1.0
            msg.pose.orientation.w = 0.0
        self.pub_pose.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BatchGenerator()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass

if __name__ == '__main__':
    main()

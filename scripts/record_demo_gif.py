#!/usr/bin/env python3
import argparse
import os
import time

import imageio
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class GifRecorder(Node):
    def __init__(self, filename, seconds=18.0, fps=15, topic='/camera/color/image_raw'):
        super().__init__('gif_recorder')
        self.filename = filename
        self.seconds = seconds
        self.fps = fps
        self.max_frames = max(1, int(seconds * fps))
        self.frames = []
        self.recording = False
        self.start_time = None
        self.last_frame_time = None
        self.frame_period = 1.0 / max(1, fps)
        
        from std_msgs.msg import String
        self.sub_trig = self.create_subscription(
            String,
            '/teleop/record_trigger',
            self.trigger_callback,
            10
        )
        
        from rclpy.qos import qos_profile_sensor_data
        self.sub = self.create_subscription(
            Image,
            topic,
            self.image_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info(
            f'Waiting for trigger to record {seconds:.1f}s at {fps} fps '
            f'from {topic} into {filename}...'
        )

    def trigger_callback(self, msg):
        if msg.data == 'start' and not self.recording:
            self.recording = True
            self.frames = []
            now = time.monotonic()
            self.start_time = now
            self.last_frame_time = None
            self.get_logger().info('Recording started! Will save on stop or max frames.')
        elif msg.data == 'stop' and self.recording:
            self.recording = False
            self.save_gif()

    def image_callback(self, msg):
        if not self.recording:
            return

        now = time.monotonic()
        if self.start_time is None:
            self.start_time = now

        if now - self.start_time >= self.seconds or len(self.frames) >= self.max_frames:
            self.recording = False
            self.save_gif()
            return

        if self.last_frame_time is not None and now - self.last_frame_time < self.frame_period:
            return

        # Convert RGB8 msg to numpy array
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        self.frames.append(arr)
        self.last_frame_time = now
        if len(self.frames) % 10 == 0:
            self.get_logger().info(f'Recorded {len(self.frames)}/{self.max_frames} frames...')
                
    def save_gif(self):
        if not self.frames:
            self.get_logger().warn('No frames recorded! Shutting down anyway.')
            rclpy.shutdown()
            return
        self.get_logger().info('Saving GIF...')
        os.makedirs(os.path.dirname(self.filename) or '.', exist_ok=True)
        imageio.mimsave(self.filename, self.frames, fps=self.fps)
        self.get_logger().info(f'Saved {self.filename}')
        rclpy.shutdown()

def main():
    parser = argparse.ArgumentParser(description='Record /camera/color/image_raw to a GIF.')
    parser.add_argument('filename', nargs='?', default='media/demo.gif')
    parser.add_argument('--seconds', type=float, default=18.0)
    parser.add_argument('--fps', type=int, default=15)
    parser.add_argument('--topic', default='/camera/color/image_raw')
    args = parser.parse_args()

    rclpy.init()
    node = GifRecorder(
        args.filename,
        seconds=args.seconds,
        fps=args.fps,
        topic=args.topic,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass

if __name__ == '__main__':
    main()

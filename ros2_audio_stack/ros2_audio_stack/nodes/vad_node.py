#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import webrtcvad
import numpy as np
import struct


# ROS2 node that detects speech segments using WebRTC VAD and publishes them to 'audio/speech'.
class VADNode(Node):

    def __init__(self):
        super().__init__('vad_node')

        # Node parameters
        self.declare_parameter('aggressiveness', 2)
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('frame_ms', 30)

        self.aggr = self.get_parameter('aggressiveness').value
        self.vad = webrtcvad.Vad(self.aggr)
        self.sr = self.get_parameter('sample_rate').value
        self.frame_ms = self.get_parameter('frame_ms').value
        self.frame_size = int(self.sr * self.frame_ms / 1000)

        # ROS subscriptions and publishers
        self.sub = self.create_subscription(Int16MultiArray, 'audio/raw', self.cb_audio, 10)
        self.pub = self.create_publisher(Int16MultiArray, 'audio/speech', 10)

        # Internal buffer for accumulating speech frames
        self.speech_active = False
        self.buffer = bytearray()
        self.max_buffer = self.sr * 5 * 2  # max 5 seconds (×2 for 16-bit)

        self.get_logger().info(
            f'VADNode ready: sr={self.sr}, frame_ms={self.frame_ms}, aggressiveness={self.aggr}'
        )

    def cb_audio(self, msg: Int16MultiArray):
        # Process incoming raw audio and detect speech frames
        try:
            pcm = np.array(msg.data, dtype=np.int16)
            total_len = len(pcm)
            if total_len < self.frame_size:
                return

            for start in range(0, total_len - self.frame_size + 1, self.frame_size):
                frame = pcm[start:start + self.frame_size]
                frame_bytes = struct.pack('<' + 'h' * len(frame), *frame)

                try:
                    is_speech = self.vad.is_speech(frame_bytes, self.sr)
                except Exception as e:
                    self.get_logger().warn(f'Skipping empty frame: {e}')
                    continue

                if is_speech:
                    if not self.speech_active:
                        self.speech_active = True
                    self.buffer.extend(frame_bytes)
                    if len(self.buffer) > self.max_buffer:
                        self.publish_segment()
                else:
                    if self.speech_active:
                        self.publish_segment()
                        self.speech_active = False

        except Exception as e:
            self.get_logger().error(f'Unexpected error in cb_audio: {e}')

    def publish_segment(self):
        # Publish accumulated speech segment and clear buffer
        if not self.buffer:
            return
        try:
            data = np.frombuffer(bytes(self.buffer), dtype=np.int16)
            msg = Int16MultiArray()
            msg.data = data.tolist()
            self.pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error while publishing segment: {e}')
        finally:
            self.buffer.clear()

    def destroy_node(self):
        # Graceful shutdown
        self.get_logger().info('VADNode stopping...')
        super().destroy_node()


def main(args=None):
    # Initialize ROS2 node and spin
    rclpy.init(args=args)
    node = VADNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Process stopped by the user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import sys
sys.path.insert(0, '/usr/local/lib/python3.12/dist-packages')
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray  # Message type for sending audio samples
import soundfile as sf
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory

# ROS2 node that plays back a WAV file and publishes audio chunks to 'audio/raw'.
class AudioPlayerNode(Node):

    def __init__(self):
        super().__init__('audio_player')

        # Publisher for raw audio data
        self.pub = self.create_publisher(Int16MultiArray, 'audio/raw', 10)

        # Load WAV file from package share directory
        pkg_share = get_package_share_directory('ros2_audio_stack')
        wav_path = os.path.join(pkg_share, 'testfile_fixed.wav')
        self.data, self.sr = sf.read(wav_path, dtype='int16')

        # If stereo, take only the first channel
        if self.data.ndim > 1:
            self.data = self.data[:, 0]

        # Chunking parameters
        self.idx = 0
        self.chunk = int(self.sr * 0.03)  # 30 ms per chunk
        self.timer = self.create_timer(0.03, self.publish_chunk)  # Publish periodically

        self.get_logger().info(
            f"AudioPlayer started. Loaded '{wav_path}' ({len(self.data)} samples @ {self.sr} Hz)"
        )

    def publish_chunk(self):
        # Loop back to start if end of audio is reached
        if self.idx + self.chunk >= len(self.data):
            self.idx = 0

        # Prepare chunk and publish as Int16 list
        chunk = self.data[self.idx:self.idx + self.chunk].astype(np.int16)
        msg = Int16MultiArray()
        msg.data = chunk.tolist()
        self.pub.publish(msg)

        self.idx += self.chunk


def main(args=None):
    rclpy.init(args=args)
    node = AudioPlayerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Process stopped by the user.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
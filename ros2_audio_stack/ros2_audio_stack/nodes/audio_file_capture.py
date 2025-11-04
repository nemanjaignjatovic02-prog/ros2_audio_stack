#!/usr/bin/env python3
import sys
sys.path.insert(0, '/usr/local/lib/python3.12/dist-packages')
import soundfile as sf
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np

 # ROS2 node that captures audio from a file and publishes raw bytes to 'audio/raw'.
class AudioFileCaptureNode(Node):
    def __init__(self):
        super().__init__('audio_file_capture')
        self.pub = self.create_publisher(Int16MultiArray, 'audio/raw', 10)

        pkg_share = get_package_share_directory('ros2_audio_stack')
        wav_path = os.path.join(pkg_share, 'testfile_fixed.wav')

        # Load the audio file and save it to self.audio_data
        data, sr = sf.read(wav_path, dtype='int16')
        if data.ndim > 1:
            data = data[:, 0]  # take first channel if stereo
        self.audio_data = data
        self.sr = sr
        self.get_logger().info(f"Loaded '{wav_path}', sr={self.sr}, samples={len(self.audio_data)}")

        self.chunk_size = int(self.sr * 0.03)  # 30 ms
        self.offset = 0

        self.timer = self.create_timer(0.03, self.publish_chunk)

    def publish_chunk(self):
        if self.offset >= len(self.audio_data):
            self.offset = 0 # restart from beginning
            self.get_logger().info('End of audio file, restarting.')
        
        end = self.offset + self.chunk_size
        chunk = self.audio_data[self.offset:end]

        msg = Int16MultiArray()
        msg.data = chunk.tolist()  

        self.pub.publish(msg)
        self.offset += self.chunk_size

def main():
    rclpy.init()
    node = AudioFileCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Process stopped by the user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

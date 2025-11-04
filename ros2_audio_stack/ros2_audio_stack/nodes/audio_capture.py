#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
import sounddevice as sd
import numpy as np
import argparse

 # ROS2 node that captures audio from the microphone and publishes raw bytes to 'audio/raw'.
class AudioCaptureNode(Node):

    def __init__(self, sample_rate=16000, chunk_ms=30):
        super().__init__('audio_capture')
        self.sr = sample_rate
        self.chunk_ms = chunk_ms
        self.chunk_samples = int(self.sr * (self.chunk_ms / 1000.0))

        # Create publisher for raw audio data
        self.pub = self.create_publisher(ByteMultiArray, 'audio/raw', 10)

        # Open input audio stream and start capturing
        self.stream = sd.InputStream(
            samplerate=self.sr,
            channels=1,
            dtype='int16',
            blocksize=self.chunk_samples,
            callback=self.callback
        )
        self.stream.start()

    def callback(self, indata, frames, time_info, status):
        # Convert audio chunk to bytes and publish
        if status:
            self.get_logger().warn(str(status))
        arr = indata.reshape(-1)
        msg = ByteMultiArray()
        msg.data = list(arr.tobytes())
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument('--sample-rate', type=int, default=16000)
    parsed, _ = parser.parse_known_args()

    # Start the ROS2 node and handle clean shutdown
    node = AudioCaptureNode(sample_rate=parsed.sample_rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stream.stop()
    node.stream.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
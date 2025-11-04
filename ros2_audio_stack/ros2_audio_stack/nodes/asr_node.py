#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray
import json
import os
import numpy as np

try:
    from vosk import Model, KaldiRecognizer
except Exception:
    Model = None
    KaldiRecognizer = None

# ROS2 node that receives audio frames, runs Vosk speech recognition, and outputs text transcripts.
class ASRNode(Node):
    def __init__(self):
        super().__init__('asr_node')

        # Parameters - change as needed
        self.declare_parameter('vosk_model_path', '~/vosk-models/vosk-model-small-en-us-0.15')
        self.declare_parameter('sample_rate', 16000)

        model_path = os.path.expanduser(self.get_parameter('vosk_model_path').value)
        self.sr = self.get_parameter('sample_rate').value

        # Check Vosk installation and model path
        if Model is None:
            self.get_logger().error(
                'Vosk model not installed. Install using: pip install vosk'
            )
            raise RuntimeError('Vosk not available')

        if not os.path.isdir(model_path):
            self.get_logger().error(f'Vosk model path doesn\'t exist: {model_path}')
            raise RuntimeError('Model not found')

        self.model = Model(model_path)
        self.rec = KaldiRecognizer(self.model, self.sr)

        # ROS2 communication
        self.sub = self.create_subscription(Int16MultiArray, 'audio/speech', self.cb_frame, 10)
        self.pub = self.create_publisher(String, 'speech/transcript', 10)

        # Buffer for accumulating audio frames 
        self.speech_buffer = bytearray()
        self.segment_max_len = self.sr * 10  # max 10 sekundi
        self.get_logger().info(f'ASRNode running, model={model_path}, sample_rate={self.sr}')

    def cb_frame(self, msg):
        if not msg.data:
            return

        frame_bytes = np.array(msg.data, dtype=np.int16).tobytes()

        # Send current frame to the Vosk recognizer
        if self.rec.AcceptWaveform(bytes(frame_bytes)):
            # If a complete segment is recognized
            result = json.loads(self.rec.Result())
            text = result.get('text', '').strip()
            if text:
                out = String()
                out.data = json.dumps({'text': text, 'partial': False})
                self.get_logger().info(f'[ASR] Završeni segment: "{text}"')
                self.pub.publish(out)

            # Reset buffer and recognizer for next segment
            self.speech_buffer.clear()
            self.rec.Reset()
        else:
            # Speech is still ongoing, accumulate frames
            self.speech_buffer.extend(frame_bytes)

            # In case buffer exceeds max length, force recognition 
            if len(self.speech_buffer) >= self.segment_max_len:
                self.get_logger().warn('Buffer predugačak, forsiram prepoznavanje...')
                if self.rec.AcceptWaveform(bytes(self.speech_buffer)):
                    result = json.loads(self.rec.Result())
                    text = result.get('text', '').strip()
                    if text:
                        out = String()
                        out.data = json.dumps({'text': text, 'partial': False})
                        self.get_logger().info(f'[ASR] Forcirani rezultat: "{text}"')
                        self.pub.publish(out)
                # Reset buffer and recognizer
                self.speech_buffer.clear()
                self.rec.Reset()

    def destroy_node(self):
        self.get_logger().info('ASRNode stopping...')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ASRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Process stopped by the user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

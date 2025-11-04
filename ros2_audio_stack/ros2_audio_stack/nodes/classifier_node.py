#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

# ROS2 node that classifies speech transcripts into commands and basic emotions.
class ClassifierNode(Node):

    def __init__(self):
        super().__init__('classifier_node')

        # Subscribe to speech transcript messages
        self.sub = self.create_subscription(String, 'speech/transcript', self.cb_transcript, 10)
        # Publisher for classification results
        self.pub = self.create_publisher(String, 'speech/classification', 10)

    def cb_transcript(self, msg):
        # Parse transcript text from JSON or plain string
        try:
            info = json.loads(msg.data)
            text = info.get('text', '').lower()
        except Exception:
            text = msg.data.lower()

        # Default label
        label = 'unknown'

        # Simple keyword-based command detection
        if any(k in text for k in ['start','go','run']):
            label = 'command:start'
        elif any(k in text for k in ['stop','halt','pause']):
            label = 'command:stop'
        elif any(k in text for k in ['left','right','forward','back']):
            label = 'command:direction'

        # Naive emotion detection
        emotion = 'neutral'
        if any(k in text for k in ['happy','great','awesome']):
            emotion = 'happy'
        elif any(k in text for k in ['sad','unhappy','depress']):
            emotion = 'sad'

        # Publish classification result as JSON
        out = String()
        out.data = json.dumps({'label': label, 'emotion': emotion, 'text': text})
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ClassifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Process stopped by the user.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
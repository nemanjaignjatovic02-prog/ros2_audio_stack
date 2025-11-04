#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sqlite3
import json
import os
from ament_index_python.packages import get_package_share_directory

# ROS2 node that records transcripts and classification results into an SQLite database.
class RecorderNode(Node):

    def __init__(self):
        super().__init__('recorder_node')

        # Database path inside the package share directory
        pkg_share = get_package_share_directory('ros2_audio_stack')
        self.db_path = os.path.join(pkg_share, 'data', 'speech_records.db')

        # Connect to SQLite database and initialize tables
        self.conn = sqlite3.connect(self.db_path)
        self._init_db()

        # Subscribe to transcripts and classification messages
        self.sub_t = self.create_subscription(String, 'speech/transcript', self.cb_transcript, 10)
        self.sub_c = self.create_subscription(String, 'speech/classification', self.cb_class, 10)

        self.get_logger().info(f'Recorder writing to {self.db_path}')

    def _init_db(self):
        # Create tables if they do not exist
        cur = self.conn.cursor()
        cur.execute('''CREATE TABLE IF NOT EXISTS transcripts (
            id INTEGER PRIMARY KEY,
            ts DATETIME DEFAULT CURRENT_TIMESTAMP,
            text TEXT
        )''')
        cur.execute('''CREATE TABLE IF NOT EXISTS classifications (
            id INTEGER PRIMARY KEY,
            ts DATETIME DEFAULT CURRENT_TIMESTAMP,
            label TEXT,
            emotion TEXT,
            text TEXT
        )''')
        self.conn.commit()

    def cb_transcript(self, msg):
        # Save incoming transcript to database
        try:
            info = json.loads(msg.data)
            text = info.get('text', '')
        except Exception:
            text = msg.data
        cur = self.conn.cursor()
        cur.execute('INSERT INTO transcripts (text) VALUES (?)', (text,))
        self.conn.commit()

    def cb_class(self, msg):
        # Save incoming classification result to database
        info = json.loads(msg.data)
        label = info.get('label')
        emotion = info.get('emotion')
        text = info.get('text')
        cur = self.conn.cursor()
        cur.execute('INSERT INTO classifications (label, emotion, text) VALUES (?,?,?)',
                    (label, emotion, text))
        self.conn.commit()

    def destroy_node(self):
        # Close database connection on shutdown
        self.conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Process stopped by the user.')
    node.destroy_node()
    rclpy.shutdown()
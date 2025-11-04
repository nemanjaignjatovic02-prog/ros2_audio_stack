from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ros2_audio_stack'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    install_requires=['setuptools', 'rclpy', 'numpy', 'soundfile', 'webrtcvad'],
    zip_safe=True,
    maintainer='nemanja',
    maintainer_email='nemanja@example.com',
    description='ROS2 Audio Stack',
    license='Apache License 2.0',
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/data', glob('data/*')),
        ('share/' + package_name, ['testfile_fixed.wav']),
    ],
    entry_points={
        'console_scripts': [
            'audio_file_capture = ros2_audio_stack.nodes.audio_file_capture:main',
            'audio_player_node = ros2_audio_stack.nodes.audio_player_node:main',
            'vad_node = ros2_audio_stack.nodes.vad_node:main',
            'classifier_node = ros2_audio_stack.nodes.classifier_node:main',
            'recorder_node = ros2_audio_stack.nodes.recorder_node:main',
            'asr_node = ros2_audio_stack.nodes.asr_node:main',
        ],
    },
)

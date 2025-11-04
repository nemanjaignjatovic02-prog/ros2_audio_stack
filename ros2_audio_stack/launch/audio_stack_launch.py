from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='ros2_audio_stack',
            executable='audio_file_capture',
            output='screen',
            parameters=[{'input_file': '/ros2_ws/src/ros2_audio_stack/testfile.wav'}],
        ),
        Node(
            package='ros2_audio_stack',
            executable='audio_player_node',
            output='screen',
        ),
        Node(
            package='ros2_audio_stack',
            executable='asr_node',
            output='screen',
        ),
        Node(
            package='ros2_audio_stack',
            executable='vad_node',
            output='screen',
            parameters=[{'sample_rate': 16000, 'frame_ms': 30, 'aggressiveness': 2}],
        ),
        Node(
            package='ros2_audio_stack',
            executable='classifier_node',
            output='screen',
        ),
        Node(
            package='ros2_audio_stack',
            executable='recorder_node',
            output='screen',
        ),
    ])
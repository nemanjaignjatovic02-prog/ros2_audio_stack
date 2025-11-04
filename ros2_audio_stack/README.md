This package provides a small ROS2 Rolling audio pipeline:

## Package contents
- audio_capture: reads from microphone (sounddevice) and publishes raw audio frames on `audio/raw`.
- audio_file_capture: alternative to audio_capture, used BY DEFAULT. Reads from file (default is the provided testfile_fixed.wav) and publishes raw audio frames on `audio/raw`.
- vad_node: uses `webrtcvad` to detect speech frames, publishes speech frames on `audio/speech`.
- asr_node: uses `vosk` offline ASR to transcribe speech frames and publishes JSON transcripts on `speech/transcript`.
- classifier_node: simple keyword-based command and emotion classifier publishing on `speech/classification`.
- recorder_node: saves transcripts and classifications to a local SQLite DB.

## Project structure
```
ros2_audio_stack/
├── launch/
│   └── audio_stack_launch.py
├── src/
│   └── ros2_audio_stack/
│       ├── audio_capture_node.py
│       ├── vad_node.py
│       ├── asr_node.py
│       ├── classifier_node.py
│       ├── recorder_node.py
│       └── audio_player_node.py
├── data/
│   └── speech_records.db
├── README.md
└── setup.py / package.xml
```
## Notes
- Your python version must be 3.12 or newer!
- You must install required dependencies!
- Download a Vosk model (command included in the setup section) and pass its path to the ASR node in the launch file or via parameter `--vosk-model-path`.
- This is a pragmatic, *minimal* stack. Speaker diarization and robust emotion recognition require heavier models and are not included, but you can extend the `vad_node` to buffer segments per speaker or add `pyannote` for diarization.

## Setup
1. Create and setup a virtual environment:
`cd ~/ros2_ws \
python3 -m venv .venv_audio \
source .venv_audio/bin/activate`

2. Install/upgrade pip:
`pip install --upgrade pip`

3. Install Python dependencies:
`pip install rclpy numpy sounddevice webrtcvad vosk soundfile`

4. Make sure ROS2 environment is sourced after activating the venv:
`source /opt/ros/rolling/setup.bash`

## Run
1. Put the package in `~/ros2_ws/src/ros2_audio_stack`
2. `colcon build --packages-select ros2_audio_stack`
3. `source install/setup.bash`
4. `ros2 launch ros2_audio_stack audio_stack_launch.py` (edit model path in launch or provide CLI args)

## Extending the Stack
	•	Add new classifiers or emotion recognition using ML models.
	•	Replace classifier_node with neural-network-based models for better command detection.
	•	Extend vad_node with speaker buffers for diarization.
	•	Integrate with GUI or robotics applications via ROS2 topics.

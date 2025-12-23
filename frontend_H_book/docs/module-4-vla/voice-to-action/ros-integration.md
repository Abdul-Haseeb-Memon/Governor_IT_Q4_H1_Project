---
sidebar_position: 4
title: 'Integration with ROS 2 Systems'
---

# Integration with ROS 2 Systems

This guide covers integrating voice command processing with ROS 2 systems. You'll learn how to create ROS 2 nodes that receive voice commands and translate them into robot actions.

## Overview

Integrating voice command processing with ROS 2 involves creating nodes that can:
1. Receive voice commands from the Whisper processing system
2. Convert structured commands to appropriate ROS 2 messages
3. Publish commands to robot control topics
4. Handle feedback and status updates

## Voice Command ROS Node

Here's a complete example of a ROS 2 node that handles voice commands:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import json
from typing import Dict, Any

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')

        # Create publisher for robot movement commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create subscriber for voice commands (structured data)
        self.voice_sub = self.create_subscription(
            String,
            'voice_commands',
            self.voice_command_callback,
            10
        )

        # Create publisher for voice feedback
        self.feedback_pub = self.create_publisher(String, 'voice_feedback', 10)

        self.get_logger().info('Voice Command Node initialized')

    def voice_command_callback(self, msg):
        """
        Process incoming voice command
        """
        try:
            # Parse the structured command (JSON format)
            command_data = json.loads(msg.data)
            command_type = command_data.get('command_type', 'unknown')

            self.get_logger().info(f'Received voice command: {command_type}')

            if command_type == 'navigation':
                self.handle_navigation_command(command_data)
            elif command_type == 'manipulation':
                self.handle_manipulation_command(command_data)
            elif command_type == 'arm_control':
                self.handle_arm_control_command(command_data)
            else:
                self.publish_feedback(f"Unknown command type: {command_type}")

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in voice command')
            self.publish_feedback('Error: Invalid command format')
        except Exception as e:
            self.get_logger().error(f'Error processing voice command: {e}')
            self.publish_feedback(f'Error processing command: {str(e)}')

    def handle_navigation_command(self, command_data: Dict[str, Any]):
        """
        Handle navigation commands (move, turn, etc.)
        """
        entities = command_data.get('entities', {})
        direction = entities.get('direction')
        distance = entities.get('distance')

        if direction:
            twist_msg = Twist()

            if direction in ['forward', 'go']:
                twist_msg.linear.x = 0.5  # m/s
            elif direction == 'backward':
                twist_msg.linear.x = -0.5
            elif direction == 'left':
                twist_msg.angular.z = 0.5  # rad/s
            elif direction == 'right':
                twist_msg.angular.z = -0.5

            # If distance is specified, calculate duration
            if distance:
                duration = float(distance) / 0.5  # assuming 0.5 m/s speed
                self.execute_movement_for_duration(twist_msg, duration)
            else:
                # Publish continuous movement
                self.cmd_vel_pub.publish(twist_msg)
                self.publish_feedback(f"Moving {direction}")

    def handle_manipulation_command(self, command_data: Dict[str, Any]):
        """
        Handle manipulation commands (pick up, put down, etc.)
        """
        entities = command_data.get('entities', {})
        object_name = entities.get('object')

        if object_name:
            self.publish_feedback(f"Attempting to manipulate {object_name}")
            # Add manipulation logic here
        else:
            self.publish_feedback("No object specified for manipulation")

    def handle_arm_control_command(self, command_data: Dict[str, Any]):
        """
        Handle arm control commands (raise, lower, etc.)
        """
        entities = command_data.get('entities', {})
        direction = entities.get('direction', 'unknown')

        self.publish_feedback(f"Controlling arm: {direction}")
        # Add arm control logic here

    def execute_movement_for_duration(self, twist_msg: Twist, duration: float):
        """
        Execute movement command for specified duration
        """
        import time

        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < duration:
            self.cmd_vel_pub.publish(twiss_msg)
            time.sleep(0.1)  # 10 Hz

        # Stop the robot after movement
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

    def publish_feedback(self, message: str):
        """
        Publish feedback message
        """
        feedback_msg = String()
        feedback_msg.data = message
        self.feedback_pub.publish(feedback_msg)
        self.get_logger().info(f'Feedback: {message}')


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Voice Processing Node

You'll also need a separate node to handle the voice processing and command structuring:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI
import os
import tempfile
import wave
import pyaudio
import json
import threading

class VoiceProcessingNode(Node):
    def __init__(self):
        super().__init__('voice_processing_node')

        # Initialize OpenAI client
        self.client = OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))

        # Create publisher for structured voice commands
        self.command_pub = self.create_publisher(String, 'voice_commands', 10)

        # Create subscriber for raw audio or transcription requests
        self.audio_sub = self.create_subscription(
            String,
            'raw_audio_path',
            self.audio_path_callback,
            10
        )

        # Create subscriber for voice feedback
        self.feedback_sub = self.create_subscription(
            String,
            'voice_feedback',
            self.feedback_callback,
            10
        )

        # Audio recording parameters
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 44100

        self.get_logger().info('Voice Processing Node initialized')

    def audio_path_callback(self, msg):
        """
        Process audio file path and transcribe
        """
        audio_path = msg.data
        self.get_logger().info(f'Processing audio file: {audio_path}')

        try:
            # Transcribe the audio
            transcription = self.transcribe_audio(audio_path)
            self.get_logger().info(f'Transcription: {transcription}')

            # Structure the command
            structured_cmd = self.structure_command(transcription)

            # Publish structured command
            cmd_msg = String()
            cmd_msg.data = json.dumps(structured_cmd)
            self.command_pub.publish(cmd_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing audio: {e}')

    def transcribe_audio(self, audio_file_path):
        """
        Transcribe audio using OpenAI Whisper
        """
        with open(audio_file_path, "rb") as audio_file:
            transcription = self.client.audio.transcriptions.create(
                model="whisper-1",
                file=audio_file
            )
        return transcription.text

    def structure_command(self, transcription):
        """
        Structure the transcription into a command object
        """
        # Import the command processing logic from structured-inputs guide
        from voice_command_parser import parse_voice_command, validate_command

        # Parse the voice command
        parsed_cmd = parse_voice_command(transcription)

        # Validate the command
        is_valid, message = validate_command(parsed_cmd)

        if not is_valid:
            # Handle invalid command
            return {
                'command_type': 'invalid',
                'original_text': transcription,
                'error': message
            }

        # Return structured command
        return {
            'command_type': parsed_cmd.command_type,
            'original_text': transcription,
            'distance': parsed_cmd.distance,
            'direction': parsed_cmd.direction,
            'object_name': parsed_cmd.object_name,
            'arm_position': parsed_cmd.arm_position,
            'confidence': 0.9  # You might calculate actual confidence
        }

    def feedback_callback(self, msg):
        """
        Handle feedback from voice command execution
        """
        self.get_logger().info(f'Voice feedback: {msg.data}')

    def record_and_process_voice(self):
        """
        Record voice from microphone and process it
        """
        # Record audio
        audio_file = self.record_audio(duration=5)

        try:
            # Transcribe using Whisper
            transcription = self.transcribe_audio(audio_file)

            # Structure the command
            structured_cmd = self.structure_command(transcription)

            # Publish structured command
            cmd_msg = String()
            cmd_msg.data = json.dumps(structured_cmd)
            self.command_pub.publish(cmd_msg)

        except Exception as e:
            self.get_logger().error(f'Error in voice recording: {e}')
        finally:
            # Clean up temporary file
            if os.path.exists(audio_file):
                os.unlink(audio_file)

    def record_audio(self, duration=5):
        """
        Record audio from microphone for specified duration
        """
        p = pyaudio.PyAudio()

        stream = p.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        self.get_logger().info("Recording...")
        frames = []

        for i in range(0, int(self.rate / self.chunk * duration)):
            data = stream.read(self.chunk)
            frames.append(data)

        self.get_logger().info("Finished recording")

        stream.stop_stream()
        stream.close()
        p.terminate()

        # Save to temporary file
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
            wf = wave.open(temp_file.name, 'wb')
            wf.setnchannels(self.channels)
            wf.setsampwidth(p.get_sample_size(self.format))
            wf.setframerate(self.rate)
            wf.writeframes(b''.join(frames))
            wf.close()

        return temp_file.name


def main(args=None):
    rclpy.init(args=args)
    node = VoiceProcessingNode()

    try:
        # Optionally start continuous recording in a separate thread
        # recording_thread = threading.Thread(target=node.continuous_recording)
        # recording_thread.daemon = True
        # recording_thread.start()

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Launch File

Create a launch file to start both nodes together:

```xml
<!-- voice_command_system.launch.py -->
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='voice_command_package',
            executable='voice_processing_node',
            name='voice_processing_node',
            parameters=[],
            output='screen'
        ),
        Node(
            package='voice_command_package',
            executable='voice_command_node',
            name='voice_command_node',
            parameters=[],
            output='screen'
        )
    ])
```

## Service Integration

You can also implement voice commands as ROS 2 services:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_srvs.srv import Trigger
from voice_command_interfaces.srv import ProcessVoiceCommand
import json

class VoiceCommandService(Node):
    def __init__(self):
        super().__init__('voice_command_service')

        # Create service for processing voice commands
        self.srv = self.create_service(
            ProcessVoiceCommand,
            'process_voice_command',
            self.process_voice_command_callback
        )

        # Create publisher for robot commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Voice Command Service initialized')

    def process_voice_command_callback(self, request, response):
        """
        Process voice command service call
        """
        try:
            # Parse the voice command
            transcription = request.transcription
            structured_cmd = self.structure_command(transcription)

            # Execute the command
            success = self.execute_command(structured_cmd)

            response.success = success
            response.message = f"Command processed: {structured_cmd.get('command_type', 'unknown')}"

        except Exception as e:
            response.success = False
            response.message = f"Error processing command: {str(e)}"

        return response

    def structure_command(self, transcription):
        """
        Structure the transcription (same as in processing node)
        """
        # Implementation from structured-inputs guide
        pass

    def execute_command(self, structured_cmd):
        """
        Execute the structured command
        """
        command_type = structured_cmd.get('command_type')
        # Execute based on command type
        return True  # Return success status

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Configuration and Parameters

Create a configuration file for voice command settings:

```yaml
# config/voice_commands.yaml
voice_command_node:
  ros__parameters:
    whisper_api_key: "your-api-key-here"  # Use parameter server or environment variable
    confidence_threshold: 0.7
    max_recording_duration: 10.0
    command_timeout: 5.0
    enable_feedback: true
    feedback_topic: "voice_feedback"
```

## Best Practices

- **Security**: Never hardcode API keys; use environment variables or parameter servers
- **Error Handling**: Implement robust error handling for network and API failures
- **Performance**: Consider the latency of Whisper API calls in real-time applications
- **Resource Management**: Properly manage audio recording and file resources
- **Modularity**: Separate voice processing from command execution for flexibility

## Next Steps

With voice command integration working, you can now combine it with other VLA components to create more sophisticated robot behaviors. The next chapter covers Language-Driven Planning with LLMs.
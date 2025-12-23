---
sidebar_position: 2
title: 'OpenAI Whisper Integration'
---

# OpenAI Whisper Integration

This guide covers implementing speech recognition using OpenAI Whisper API for humanoid robot voice control. You'll learn how to set up Whisper integration, process audio input, and convert speech to text that can be used for robot commands.

## Prerequisites

- Basic understanding of ROS 2 concepts (covered in Module 1)
- Python programming knowledge
- OpenAI API key for Whisper service

## Setting Up OpenAI Whisper

First, install the required Python packages for Whisper integration:

```bash
pip install openai
pip install pyaudio  # For audio input
pip install SpeechRecognition  # Alternative speech recognition library
```

## Basic Whisper Implementation

Here's a basic implementation for voice recognition using OpenAI Whisper:

```python
import openai
import os
from openai import OpenAI

# Initialize OpenAI client
client = OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))

def transcribe_audio(audio_file_path):
    """
    Transcribe audio file using OpenAI Whisper API
    """
    with open(audio_file_path, "rb") as audio_file:
        transcription = client.audio.transcriptions.create(
            model="whisper-1",
            file=audio_file
        )
    return transcription.text

# Example usage
audio_path = "path/to/your/audio/file.wav"
command = transcribe_audio(audio_path)
print(f"Recognized command: {command}")
```

## Real-time Audio Processing

For real-time voice command processing, you can capture audio from a microphone:

```python
import pyaudio
import wave
import tempfile
import os
from openai import OpenAI

class VoiceCommandProcessor:
    def __init__(self, api_key):
        self.client = OpenAI(api_key=api_key)
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 44100

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

        print("Recording...")
        frames = []

        for i in range(0, int(self.rate / self.chunk * duration)):
            data = stream.read(self.chunk)
            frames.append(data)

        print("Finished recording")

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

    def process_voice_command(self, duration=5):
        """
        Record and process voice command
        """
        # Record audio
        audio_file = self.record_audio(duration)

        try:
            # Transcribe using Whisper
            with open(audio_file, "rb") as f:
                transcription = self.client.audio.transcriptions.create(
                    model="whisper-1",
                    file=f
                )

            # Clean up temporary file
            os.unlink(audio_file)

            return transcription.text
        except Exception as e:
            print(f"Error processing voice command: {e}")
            os.unlink(audio_file)
            return None

# Example usage
processor = VoiceCommandProcessor(os.environ.get("OPENAI_API_KEY"))
command = processor.process_voice_command(duration=3)
if command:
    print(f"Recognized command: {command}")
```

## Voice Command Preprocessing

Before sending voice commands to your robot, you may want to preprocess them to extract specific robot commands:

```python
def preprocess_voice_command(transcription):
    """
    Preprocess voice transcription to extract robot commands
    """
    # Convert to lowercase for easier processing
    command = transcription.lower().strip()

    # Define known commands
    known_commands = {
        'move forward': 'MOVE_FORWARD',
        'move backward': 'MOVE_BACKWARD',
        'turn left': 'TURN_LEFT',
        'turn right': 'TURN_RIGHT',
        'stop': 'STOP',
        'raise arm': 'RAISE_ARM',
        'lower arm': 'LOWER_ARM',
        'pick up': 'PICK_UP',
        'put down': 'PUT_DOWN'
    }

    # Find matching command
    for phrase, command_type in known_commands.items():
        if phrase in command:
            return command_type, command

    # If no exact match, return as custom command
    return 'CUSTOM_COMMAND', command

# Example usage
raw_command = "Please move forward two meters"
command_type, processed_command = preprocess_voice_command(raw_command)
print(f"Command type: {command_type}, Command: {processed_command}")
```

## Best Practices

- **Audio Quality**: Ensure good audio quality by using noise reduction and proper microphone placement
- **Error Handling**: Always handle API errors and network issues gracefully
- **Rate Limiting**: Be aware of OpenAI API rate limits and implement appropriate delays
- **Privacy**: Consider privacy implications when processing voice data

## Next Steps

Once you have Whisper integration working, you can move on to converting the recognized text into structured robot commands in the next section.
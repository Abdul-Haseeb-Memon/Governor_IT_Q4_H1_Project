---
sidebar_position: 3
title: 'Converting Voice to Structured Data'
---

# Converting Voice to Structured Data

This guide covers how to convert voice commands into structured data that can be processed by ROS 2 systems. You'll learn how to parse natural language commands and convert them into structured robot actions.

## Overview

Voice commands from OpenAI Whisper are typically natural language text. To control robots, we need to convert this text into structured data that can be processed by ROS 2 nodes. This involves:

1. Parsing the natural language command
2. Extracting relevant parameters (direction, distance, object names, etc.)
3. Converting to structured data formats compatible with ROS messages

## Command Parsing

Here's an example of how to parse common robot commands from voice input:

```python
import re
from dataclasses import dataclass
from typing import Optional

@dataclass
class RobotCommand:
    command_type: str
    distance: Optional[float] = None
    direction: Optional[str] = None
    object_name: Optional[str] = None
    arm_position: Optional[str] = None

def parse_voice_command(voice_text: str) -> RobotCommand:
    """
    Parse voice command text and return structured RobotCommand
    """
    # Convert to lowercase for easier processing
    text = voice_text.lower().strip()

    # Define command patterns
    patterns = {
        r'move forward.*?(\d+(?:\.\d+)?) meters?': ('MOVE', 'forward', r'\d+(?:\.\d+)?'),
        r'move backward.*?(\d+(?:\.\d+)?) meters?': ('MOVE', 'backward', r'\d+(?:\.\d+)?'),
        r'move (forward|backward|left|right)': ('MOVE', r'(forward|backward|left|right)', None),
        r'go (forward|backward|left|right)': ('MOVE', r'(forward|backward|left|right)', None),
        r'turn (left|right)': ('TURN', r'(left|right)', None),
        r'rotate (left|right)': ('TURN', r'(left|right)', None),
        r'pick up (.+)': ('PICK_UP', None, r'(.+)'),
        r'grab (.+)': ('PICK_UP', None, r'(.+)'),
        r'put down (.+)': ('PUT_DOWN', None, r'(.+)'),
        r'drop (.+)': ('PUT_DOWN', None, r'(.+)'),
        r'raise arm': ('ARM_CONTROL', 'raise', None),
        r'lift arm': ('ARM_CONTROL', 'raise', None),
        r'lower arm': ('ARM_CONTROL', 'lower', None),
        r'put arm down': ('ARM_CONTROL', 'lower', None),
    }

    for pattern, (cmd_type, direction_pattern, object_pattern) in patterns.items():
        match = re.search(pattern, text)
        if match:
            groups = match.groups()

            if cmd_type == 'MOVE':
                if len(groups) > 0 and re.match(r'\d+(?:\.\d+)?', groups[0]):
                    # Distance was captured
                    distance = float(groups[0])
                    direction = direction_pattern if isinstance(direction_pattern, str) else None
                    return RobotCommand(command_type=cmd_type, distance=distance, direction=direction)
                else:
                    # Only direction was captured
                    direction = groups[0] if groups else direction_pattern
                    return RobotCommand(command_type=cmd_type, direction=direction)

            elif cmd_type in ['PICK_UP', 'PUT_DOWN']:
                object_name = groups[0] if groups else None
                return RobotCommand(command_type=cmd_type, object_name=object_name)

            elif cmd_type == 'TURN':
                direction = groups[0] if groups else None
                return RobotCommand(command_type=cmd_type, direction=direction)

            elif cmd_type == 'ARM_CONTROL':
                arm_position = groups[0] if groups else None
                return RobotCommand(command_type=cmd_type, arm_position=arm_position)

    # If no pattern matches, return as CUSTOM command
    return RobotCommand(command_type='CUSTOM', object_name=text)

# Example usage
commands = [
    "Move forward 2 meters",
    "Turn left",
    "Pick up the red ball",
    "Raise arm",
    "Go backward 1.5 meters"
]

for cmd in commands:
    parsed = parse_voice_command(cmd)
    print(f"Original: {cmd}")
    print(f"Parsed: {parsed}")
    print("---")
```

## Natural Language Processing Pipeline

For more sophisticated command parsing, you can implement a multi-stage NLP pipeline:

```python
import json
from typing import Dict, Any

class VoiceCommandProcessor:
    def __init__(self):
        # Define entity extraction rules
        self.entity_rules = {
            'distance': [r'(\d+(?:\.\d+)?)\s*(meters?|m|feet|ft)', r'(\d+(?:\.\d+)?)\s*(cm|centimeters?)'],
            'direction': [r'\b(forward|backward|back|left|right|up|down)\b'],
            'object': [r'(\w+)\s+(ball|box|cup|object|item|thing)'],
            'action': [r'\b(move|go|turn|rotate|pick|grab|put|drop|raise|lift|lower)\b']
        }

        # Define command templates
        self.command_templates = {
            'navigation': ['move', 'go', 'forward', 'backward', 'turn', 'rotate'],
            'manipulation': ['pick', 'grab', 'put', 'drop'],
            'arm_control': ['raise', 'lift', 'lower']
        }

    def extract_entities(self, text: str) -> Dict[str, Any]:
        """
        Extract entities from text using regex patterns
        """
        entities = {}

        for entity_type, patterns in self.entity_rules.items():
            for pattern in patterns:
                matches = re.findall(pattern, text, re.IGNORECASE)
                if matches:
                    entities[entity_type] = matches[0] if isinstance(matches[0], str) else matches[0][0]
                    break

        return entities

    def classify_command_type(self, text: str) -> str:
        """
        Classify the command type based on keywords
        """
        text_lower = text.lower()

        for cmd_type, keywords in self.command_templates.items():
            for keyword in keywords:
                if keyword in text_lower:
                    return cmd_type

        return 'general'

    def process_voice_command(self, voice_text: str) -> Dict[str, Any]:
        """
        Process voice command and return structured data
        """
        entities = self.extract_entities(voice_text)
        command_type = self.classify_command_type(voice_text)

        # Build structured command
        structured_command = {
            'command_type': command_type,
            'original_text': voice_text,
            'entities': entities,
            'confidence': 0.9 if entities else 0.5  # Simple confidence measure
        }

        return structured_command

# Example usage
processor = VoiceCommandProcessor()

voice_commands = [
    "Move forward 2 meters",
    "Turn left and pick up the red cup",
    "Raise the robot's left arm",
    "Go backward 1.5 meters and stop"
]

for cmd in voice_commands:
    result = processor.process_voice_command(cmd)
    print(f"Command: {cmd}")
    print(f"Structured: {json.dumps(result, indent=2)}")
    print("---")
```

## Integration with ROS 2 Messages

Once you have structured data, you can convert it to ROS 2 messages:

```python
# Assuming you have generated ROS messages
# from std_msgs.msg import String
# from geometry_msgs.msg import Twist
# from custom_msgs.msg import RobotCommand as RobotCommandMsg

def create_ros_command(robot_cmd: RobotCommand):
    """
    Convert RobotCommand to ROS message
    """
    if robot_cmd.command_type == 'MOVE':
        # Create Twist message for movement
        twist_msg = Twist()

        if robot_cmd.direction == 'forward':
            twist_msg.linear.x = 0.5  # m/s
        elif robot_cmd.direction == 'backward':
            twist_msg.linear.x = -0.5
        elif robot_cmd.direction == 'left':
            twist_msg.angular.z = 0.5
        elif robot_cmd.direction == 'right':
            twist_msg.angular.z = -0.5

        if robot_cmd.distance:
            # Duration would depend on distance and speed
            duration = robot_cmd.distance / 0.5  # assuming 0.5 m/s speed

        return twist_msg

    elif robot_cmd.command_type in ['PICK_UP', 'PUT_DOWN']:
        # Create custom message for manipulation
        cmd_msg = RobotCommandMsg()
        cmd_msg.command = robot_cmd.command_type
        cmd_msg.object_name = robot_cmd.object_name or ""
        return cmd_msg

    # Add more command types as needed
    return None

# Example usage
parsed_cmd = parse_voice_command("Move forward 2 meters")
ros_msg = create_ros_command(parsed_cmd)
print(f"Generated ROS message: {ros_msg}")
```

## Error Handling and Validation

Always validate and handle errors in voice command processing:

```python
def validate_command(robot_cmd: RobotCommand) -> tuple[bool, str]:
    """
    Validate the parsed robot command
    """
    if robot_cmd.command_type == 'MOVE':
        if robot_cmd.distance and robot_cmd.distance > 10.0:
            return False, "Distance too large (max 10 meters)"
        if robot_cmd.direction not in ['forward', 'backward', 'left', 'right']:
            return False, f"Invalid direction: {robot_cmd.direction}"

    elif robot_cmd.command_type in ['PICK_UP', 'PUT_DOWN']:
        if not robot_cmd.object_name:
            return False, "Object name required for manipulation commands"

    return True, "Valid command"

# Example usage
parsed_cmd = parse_voice_command("Move forward 15 meters")  # Too far
is_valid, message = validate_command(parsed_cmd)
print(f"Command valid: {is_valid}, Message: {message}")
```

## Best Practices

- **Context Awareness**: Consider the robot's current state when interpreting commands
- **Ambiguity Resolution**: Implement strategies to handle ambiguous commands
- **Fallback Mechanisms**: Provide fallbacks when parsing fails
- **User Feedback**: Provide clear feedback about command interpretation
- **Safety Checks**: Always validate commands for safety before execution

## Next Steps

After converting voice commands to structured data, you need to integrate them with your ROS 2 system. The next section covers ROS integration.
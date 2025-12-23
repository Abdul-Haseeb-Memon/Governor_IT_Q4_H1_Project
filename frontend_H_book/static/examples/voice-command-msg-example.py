"""
Sample ROS message definition for voice commands.

This file demonstrates how voice commands can be structured as ROS messages.
In a real implementation, these would be .msg files in a custom ROS package.
"""

class VoiceCommand:
    """
    A sample voice command message structure.

    This is a Python representation of what a ROS .msg file would define.
    In practice, you would create a custom_msgs package with a .msg file.

    Fields:
    - command_type: string (e.g., 'MOVE', 'PICK_UP', 'PUT_DOWN', etc.)
    - original_text: string (the raw transcribed text)
    - distance: float (distance parameter for movement commands)
    - direction: string (direction parameter)
    - object_name: string (name of object for manipulation)
    - confidence: float (confidence score from voice processing)
    """

    def __init__(self):
        self.command_type = ""
        self.original_text = ""
        self.distance = 0.0
        self.direction = ""
        self.object_name = ""
        self.confidence = 0.0

    def to_dict(self):
        """Convert to dictionary for JSON serialization."""
        return {
            'command_type': self.command_type,
            'original_text': self.original_text,
            'distance': self.distance,
            'direction': self.direction,
            'object_name': self.object_name,
            'confidence': self.confidence
        }

    @classmethod
    def from_dict(cls, data):
        """Create instance from dictionary."""
        cmd = cls()
        cmd.command_type = data.get('command_type', '')
        cmd.original_text = data.get('original_text', '')
        cmd.distance = data.get('distance', 0.0)
        cmd.direction = data.get('direction', '')
        cmd.object_name = data.get('object_name', '')
        cmd.confidence = data.get('confidence', 0.0)
        return cmd


class VoiceFeedback:
    """
    A sample voice feedback message structure.

    Fields:
    - status: string (e.g., 'received', 'processing', 'executed', 'error')
    - message: string (feedback message)
    - command_id: string (ID of the command being processed)
    """

    def __init__(self):
        self.status = ""
        self.message = ""
        self.command_id = ""

    def to_dict(self):
        """Convert to dictionary for JSON serialization."""
        return {
            'status': self.status,
            'message': self.message,
            'command_id': self.command_id
        }

    @classmethod
    def from_dict(cls, data):
        """Create instance from dictionary."""
        feedback = cls()
        feedback.status = data.get('status', '')
        feedback.message = data.get('message', '')
        feedback.command_id = data.get('command_id', '')
        return feedback


# Example usage
if __name__ == "__main__":
    # Create a sample voice command
    cmd = VoiceCommand()
    cmd.command_type = "MOVE"
    cmd.original_text = "move forward 2 meters"
    cmd.distance = 2.0
    cmd.direction = "forward"
    cmd.confidence = 0.95

    print("Sample Voice Command:")
    print(cmd.to_dict())

    # Create a sample feedback message
    feedback = VoiceFeedback()
    feedback.status = "executed"
    feedback.message = "Successfully moved forward 2 meters"
    feedback.command_id = "cmd_001"

    print("\nSample Voice Feedback:")
    print(feedback.to_dict())
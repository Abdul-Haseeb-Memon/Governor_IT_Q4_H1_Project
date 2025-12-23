---
sidebar_position: 2
title: 'High-Level VLA Workflow'
---

# High-Level VLA Workflow

This guide covers the complete Vision-Language-Action (VLA) workflow that connects voice commands to robot actions through language planning and visual perception. You'll learn how to implement an end-to-end system that processes natural language instructions and executes complex robotic behaviors.

## Overview

The VLA workflow consists of four main components working in sequence:
1. **Voice Processing**: Converting spoken commands to text
2. **Language Planning**: Translating natural language goals into action sequences
3. **Vision Processing**: Perceiving and understanding the environment
4. **Action Execution**: Executing planned actions on the robot

## Complete VLA Architecture

Here's the complete architecture of the VLA system:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
import json
import asyncio
from typing import Dict, Any, List, Optional
import threading
import queue

class VLAMasterNode(Node):
    """
    Master node that orchestrates the complete VLA workflow.
    """
    def __init__(self):
        super().__init__('vla_master')

        # Publishers
        self.vla_status_pub = self.create_publisher(String, 'vla_system_status', 10)
        self.vla_command_pub = self.create_publisher(String, 'vla_commands', 10)

        # Subscribers
        self.voice_command_sub = self.create_subscription(
            String,
            'voice_commands',
            self.voice_command_callback,
            10
        )

        self.planning_result_sub = self.create_subscription(
            String,
            'planning_results',
            self.planning_result_callback,
            10
        )

        self.vision_result_sub = self.create_subscription(
            String,
            'integrated_perception',
            self.vision_result_callback,
            10
        )

        self.action_status_sub = self.create_subscription(
            String,
            'action_execution_status',
            self.action_status_callback,
            10
        )

        # Internal state
        self.current_state = 'IDLE'
        self.pending_command = None
        self.planning_queue = queue.Queue()
        self.execution_queue = queue.Queue()
        self.system_status = {
            'voice_processor': 'ready',
            'planner': 'ready',
            'vision_system': 'ready',
            'action_executor': 'ready'
        }

        # Timer for system monitoring
        self.monitor_timer = self.create_timer(1.0, self.monitor_system_status)

        self.get_logger().info('VLA Master Node initialized')

    def voice_command_callback(self, msg: String):
        """
        Handle incoming voice commands and start VLA workflow.
        """
        try:
            command_data = json.loads(msg.data)
            self.get_logger().info(f'Received voice command: {command_data.get("original_text", "unknown")}')

            # Update state
            self.current_state = 'PROCESSING_VOICE'
            self.pending_command = command_data

            # Publish command to planning system
            planning_msg = String()
            planning_msg.data = msg.data
            self.vla_command_pub.publish(planning_msg)

            # Update status
            self.publish_system_status('voice_processed', command_data)

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in voice command')
        except Exception as e:
            self.get_logger().error(f'Error processing voice command: {e}')

    def planning_result_callback(self, msg: String):
        """
        Handle planning results and coordinate with vision system.
        """
        try:
            plan_data = json.loads(msg.data)
            self.get_logger().info(f'Received planning result with {len(plan_data.get("actions", []))} actions')

            # Check if we need vision input for the plan
            if self.needs_vision_input(plan_data):
                self.current_state = 'WAITING_FOR_VISION'
                self.request_vision_assistance(plan_data)
            else:
                # Execute plan directly
                self.execute_plan(plan_data)

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in planning result')
        except Exception as e:
            self.get_logger().error(f'Error processing planning result: {e}')

    def vision_result_callback(self, msg: String):
        """
        Handle vision results and continue execution if needed.
        """
        try:
            vision_data = json.loads(msg.data)
            self.get_logger().info('Received vision result')

            if self.current_state == 'WAITING_FOR_VISION':
                # Process vision data and continue execution
                self.process_vision_for_execution(vision_data)

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in vision result')
        except Exception as e:
            self.get_logger().error(f'Error processing vision result: {e}')

    def action_status_callback(self, msg: String):
        """
        Handle action execution status updates.
        """
        try:
            status_data = json.loads(msg.data)
            action_status = status_data.get('status', 'unknown')
            self.get_logger().info(f'Action status: {action_status}')

            if action_status == 'completed':
                self.current_state = 'IDLE'
                self.pending_command = None
                self.publish_system_status('execution_completed', status_data)
            elif action_status == 'failed':
                self.current_state = 'ERROR'
                self.publish_system_status('execution_failed', status_data)

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in action status')
        except Exception as e:
            self.get_logger().error(f'Error processing action status: {e}')

    def needs_vision_input(self, plan_data: Dict[str, Any]) -> bool:
        """
        Determine if the plan needs vision system input.
        """
        actions = plan_data.get('actions', [])

        # Check if any action requires object detection or navigation to specific objects
        vision_required_actions = [
            'GRASP_OBJECT', 'NAVIGATE_TO_OBJECT', 'DETECT_OBJECT',
            'APPROACH_OBJECT', 'PLACE_ON_SURFACE', 'FIND_PERSON'
        ]

        for action in actions:
            if action.get('action_type') in vision_required_actions:
                return True

        return False

    def request_vision_assistance(self, plan_data: Dict[str, Any]):
        """
        Request vision system to provide necessary information for plan execution.
        """
        # Create vision request based on plan requirements
        vision_request = {
            'request_type': 'OBJECT_LOCALIZATION',
            'target_objects': self.extract_target_objects(plan_data),
            'timestamp': self.get_clock().now().nanoseconds
        }

        # Publish vision request
        vision_msg = String()
        vision_msg.data = json.dumps(vision_request)
        # In a real system, this would go to the vision system
        # self.vision_request_pub.publish(vision_msg)

        self.get_logger().info(f'Requested vision assistance for objects: {vision_request["target_objects"]}')

    def extract_target_objects(self, plan_data: Dict[str, Any]) -> List[str]:
        """
        Extract target objects from the plan that require vision assistance.
        """
        target_objects = []
        actions = plan_data.get('actions', [])

        for action in actions:
            obj_class = action.get('parameters', {}).get('object_class')
            if obj_class:
                target_objects.append(obj_class)

        return list(set(target_objects))  # Remove duplicates

    def execute_plan(self, plan_data: Dict[str, Any]):
        """
        Execute the planned actions.
        """
        self.current_state = 'EXECUTING_PLAN'

        # Publish execution command
        execution_msg = String()
        execution_msg.data = json.dumps(plan_data)
        # In a real system, this would go to the action execution system
        # self.action_execution_pub.publish(execution_msg)

        self.get_logger().info(f'Started executing plan with {len(plan_data.get("actions", []))} actions')

    def process_vision_for_execution(self, vision_data: Dict[str, Any]):
        """
        Process vision results and continue plan execution.
        """
        # Update plan with vision information
        updated_plan = self.update_plan_with_vision_info(
            self.pending_command.get('plan', {}),
            vision_data
        )

        # Execute updated plan
        self.execute_plan(updated_plan)

        self.current_state = 'EXECUTING_PLAN'

    def update_plan_with_vision_info(self, plan: Dict[str, Any], vision_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Update the plan with information from vision system.
        """
        updated_plan = plan.copy()

        # Update action parameters with object positions from vision
        for action in updated_plan.get('actions', []):
            if action.get('action_type') in ['NAVIGATE_TO_OBJECT', 'GRASP_OBJECT']:
                target_obj = action.get('parameters', {}).get('object_class')
                if target_obj:
                    # Find object position in vision data
                    obj_position = self.find_object_position(vision_data, target_obj)
                    if obj_position:
                        action['parameters']['position_3d'] = obj_position

        return updated_plan

    def find_object_position(self, vision_data: Dict[str, Any], obj_class: str) -> Optional[Dict[str, float]]:
        """
        Find the 3D position of an object in vision data.
        """
        for obj in vision_data.get('components', {}).get('scene_description', {}).get('prominent_objects', []):
            if obj.get('class') == obj_class:
                return obj.get('position')

        return None

    def monitor_system_status(self):
        """
        Monitor the status of all VLA components.
        """
        # In a real system, this would check actual component status
        # For this example, we'll just publish current status
        self.publish_system_status('monitor_update', {'state': self.current_state})

    def publish_system_status(self, status_type: str, details: Dict[str, Any]):
        """
        Publish system status update.
        """
        status_msg = String()
        status_msg.data = json.dumps({
            'status_type': status_type,
            'current_state': self.current_state,
            'system_status': self.system_status,
            'details': details,
            'timestamp': self.get_clock().now().nanoseconds
        })
        self.vla_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VLAMasterNode()

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

## VLA Workflow Components

### Voice Processing Component

The voice processing component handles speech recognition and command interpretation:

```python
class VoiceProcessingComponent:
    """
    Component responsible for voice processing in VLA system.
    """
    def __init__(self, api_key: str):
        from openai import OpenAI
        self.client = OpenAI(api_key=api_key)

    def process_voice_command(self, audio_file_path: str) -> Dict[str, Any]:
        """
        Process audio file and return structured command.
        """
        with open(audio_file_path, "rb") as audio_file:
            transcription = self.client.audio.transcriptions.create(
                model="whisper-1",
                file=audio_file
            )

        # Structure the command
        structured_command = self.structure_command(transcription.text)
        return structured_command

    def structure_command(self, text: str) -> Dict[str, Any]:
        """
        Convert transcribed text to structured command.
        """
        # This would use NLP to extract intent and parameters
        return {
            'original_text': text,
            'structured_command': self.parse_intent(text),
            'confidence': 0.9
        }

    def parse_intent(self, text: str) -> Dict[str, Any]:
        """
        Parse the intent from the voice command.
        """
        # Simple intent parsing (in reality, this would be more sophisticated)
        text_lower = text.lower()

        if any(word in text_lower for word in ['go to', 'navigate', 'move to']):
            return {
                'intent': 'NAVIGATION',
                'target_location': self.extract_location(text)
            }
        elif any(word in text_lower for word in ['pick up', 'grasp', 'get', 'bring']):
            return {
                'intent': 'MANIPULATION',
                'target_object': self.extract_object(text)
            }
        elif any(word in text_lower for word in ['find', 'locate', 'show']):
            return {
                'intent': 'DETECTION',
                'target_object': self.extract_object(text)
            }
        else:
            return {
                'intent': 'GENERAL',
                'text': text
            }

    def extract_location(self, text: str) -> str:
        """
        Extract location from text (simplified).
        """
        # In a real system, this would use more sophisticated NLP
        locations = ['kitchen', 'living room', 'bedroom', 'office', 'dining room']
        for loc in locations:
            if loc in text.lower():
                return loc
        return 'unknown'

    def extract_object(self, text: str) -> str:
        """
        Extract object from text (simplified).
        """
        # In a real system, this would use more sophisticated NLP
        objects = ['cup', 'bottle', 'book', 'phone', 'laptop', 'ball', 'box']
        for obj in objects:
            if obj in text.lower():
                return obj
        return 'unknown'
```

### Language Planning Component

The language planning component translates goals into action sequences:

```python
class LanguagePlanningComponent:
    """
    Component responsible for language planning in VLA system.
    """
    def __init__(self, api_key: str):
        from openai import OpenAI
        self.client = OpenAI(api_key=api_key)

    def plan_actions(self, goal: str, context: Dict[str, Any] = None) -> Dict[str, Any]:
        """
        Plan actions for a given goal using LLM.
        """
        prompt = self.create_planning_prompt(goal, context)

        response = self.client.chat.completions.create(
            model="gpt-4-turbo",
            messages=[
                {"role": "system", "content": self.get_system_prompt()},
                {"role": "user", "content": prompt}
            ],
            temperature=0.1,
            max_tokens=1000
        )

        plan_json = response.choices[0].message.content.strip()

        # Clean up response
        if plan_json.startswith("```json"):
            plan_json = plan_json[7:]
        if "```" in plan_json:
            plan_json = plan_json.split("```")[0]

        return json.loads(plan_json)

    def create_planning_prompt(self, goal: str, context: Dict[str, Any]) -> str:
        """
        Create prompt for action planning.
        """
        context_str = json.dumps(context or {}, indent=2) if context else "No context provided"

        return f"""
Goal: {goal}

Environmental Context:
{context_str}

Create a detailed action plan to achieve this goal. The plan should be a JSON object with:
- goal: The original goal
- actions: Array of actions, each with:
  - action_type: Type of action (NAVIGATE, GRASP, DETECT, SPEAK, etc.)
  - parameters: Object with action-specific parameters
  - description: Human-readable description

Consider the environmental context when creating the plan.
Return only the JSON plan with no additional text.
"""

    def get_system_prompt(self) -> str:
        """
        Get system prompt for planning.
        """
        return """
You are a robot task planner. Create detailed, executable action plans that consider environmental context and robot capabilities. Return your response as valid JSON only.
"""
```

### Vision Processing Component

The vision processing component handles object detection and scene understanding:

```python
class VisionProcessingComponent:
    """
    Component responsible for vision processing in VLA system.
    """
    def __init__(self):
        from ultralytics import YOLO
        self.model = YOLO("yolov8n.pt")
        self.class_names = self.model.names

    def process_image(self, image_path: str) -> Dict[str, Any]:
        """
        Process image and return object detections.
        """
        results = self.model(image_path, conf=0.5)

        detections = []
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    confidence = float(box.conf[0])
                    class_id = int(box.cls[0])
                    class_name = self.class_names[class_id]

                    detection = {
                        'class_id': class_id,
                        'class_name': class_name,
                        'confidence': confidence,
                        'bbox': {
                            'x1': int(x1),
                            'y1': int(y1),
                            'x2': int(x2),
                            'y2': int(y2),
                            'center_x': int((x1 + x2) / 2),
                            'center_y': int((y1 + y2) / 2)
                        }
                    }
                    detections.append(detection)

        return {
            'detections': detections,
            'image_path': image_path,
            'timestamp': time.time()
        }

    def find_object_by_class(self, detections: List[Dict[str, Any]], class_name: str) -> List[Dict[str, Any]]:
        """
        Find all detections of a specific class.
        """
        return [det for det in detections if det['class_name'] == class_name]

    def get_object_position(self, detection: Dict[str, Any]) -> Dict[str, float]:
        """
        Get 2D position of object (for now, will be extended to 3D).
        """
        return {
            'x': detection['bbox']['center_x'],
            'y': detection['bbox']['center_y']
        }
```

## VLA System Integration Pattern

The integration pattern that connects all components:

```python
class VLAIntegrationManager:
    """
    Manages the integration between all VLA components.
    """
    def __init__(self, openai_api_key: str):
        self.voice_processor = VoiceProcessingComponent(openai_api_key)
        self.language_planner = LanguagePlanningComponent(openai_api_key)
        self.vision_processor = VisionProcessingComponent()

        # Queues for inter-component communication
        self.voice_to_plan_queue = queue.Queue()
        self.plan_to_vision_queue = queue.Queue()
        self.vision_to_exec_queue = queue.Queue()

    def process_complete_vla_workflow(self, audio_file_path: str, image_path: str = None) -> Dict[str, Any]:
        """
        Execute complete VLA workflow from voice to action.
        """
        # Step 1: Process voice command
        voice_result = self.voice_processor.process_voice_command(audio_file_path)
        self.get_logger().info(f"Voice processing result: {voice_result['structured_command']}")

        # Step 2: Plan actions based on voice command
        goal = voice_result['structured_command']
        context = {}

        # If image is provided, add vision context
        if image_path:
            vision_result = self.vision_processor.process_image(image_path)
            context['vision_data'] = vision_result
            context['available_objects'] = [det['class_name'] for det in vision_result['detections']]

        plan_result = self.language_planner.plan_actions(
            goal['structured_command']['text'] if goal['intent'] == 'GENERAL' else str(goal),
            context
        )
        self.get_logger().info(f"Planning result: {len(plan_result['actions'])} actions planned")

        # Step 3: Execute plan (in a real system, this would go to action execution)
        execution_result = self.execute_plan(plan_result)

        return {
            'voice_input': voice_result,
            'planning_output': plan_result,
            'vision_context': context if image_path else None,
            'execution_result': execution_result,
            'workflow_complete': True
        }

    def execute_plan(self, plan: Dict[str, Any]) -> Dict[str, Any]:
        """
        Execute the planned actions (simulated).
        """
        # In a real system, this would interface with robot action execution
        # For simulation, we'll just return success for all actions
        executed_actions = []

        for action in plan.get('actions', []):
            executed_action = action.copy()
            executed_action['status'] = 'completed'
            executed_action['execution_time'] = time.time()
            executed_actions.append(executed_action)

        return {
            'actions_executed': len(executed_actions),
            'actions': executed_actions,
            'status': 'completed'
        }

    def get_logger(self):
        """
        Simple logger for demonstration.
        """
        import logging
        return logging.getLogger(__name__)
```

## VLA Workflow Best Practices

### Error Handling and Recovery

Implement comprehensive error handling across all VLA components:

```python
class VLARobustWorkflow:
    """
    Robust VLA workflow with error handling and recovery.
    """
    def __init__(self, openai_api_key: str):
        self.integration_manager = VLAIntegrationManager(openai_api_key)
        self.max_retries = 3
        self.retry_delay = 2.0

    async def execute_with_retry(self, audio_file_path: str, image_path: str = None) -> Dict[str, Any]:
        """
        Execute VLA workflow with retry mechanism.
        """
        for attempt in range(self.max_retries):
            try:
                result = self.integration_manager.process_complete_vla_workflow(
                    audio_file_path, image_path
                )

                if result.get('execution_result', {}).get('status') == 'completed':
                    return result
                else:
                    self.get_logger().warning(f"Attempt {attempt + 1} failed, retrying...")

            except Exception as e:
                self.get_logger().error(f"Attempt {attempt + 1} failed with error: {e}")

                if attempt == self.max_retries - 1:  # Last attempt
                    return {
                        'workflow_complete': False,
                        'error': str(e),
                        'attempts': attempt + 1
                    }

            # Wait before retry
            await asyncio.sleep(self.retry_delay)

        return {
            'workflow_complete': False,
            'error': 'Max retries exceeded',
            'attempts': self.max_retries
        }

    def get_logger(self):
        """
        Simple logger for demonstration.
        """
        import logging
        return logging.getLogger(__name__)
```

## Performance Optimization

Optimize the VLA workflow for real-time performance:

```python
class OptimizedVLAWorkflow:
    """
    Optimized VLA workflow for real-time performance.
    """
    def __init__(self, openai_api_key: str):
        self.voice_processor = VoiceProcessingComponent(openai_api_key)
        self.language_planner = LanguagePlanningComponent(openai_api_key)
        self.vision_processor = VisionProcessingComponent()

        # Caching for frequently used operations
        self.command_cache = {}
        self.object_cache = {}

        # Threading for parallel processing
        self.executor = ThreadPoolExecutor(max_workers=4)

    def process_workflow_optimized(self, audio_file_path: str, image_path: str = None) -> Dict[str, Any]:
        """
        Process VLA workflow with optimizations.
        """
        # Parallel processing of voice and vision
        voice_future = self.executor.submit(
            self.voice_processor.process_voice_command, audio_file_path
        )

        vision_future = None
        if image_path:
            vision_future = self.executor.submit(
                self.vision_processor.process_image, image_path
            )

        # Get results
        voice_result = voice_future.result()
        vision_result = vision_future.result() if vision_future else None

        # Plan actions with context
        goal = voice_result['structured_command']
        context = {}
        if vision_result:
            context['vision_data'] = vision_result
            context['available_objects'] = [det['class_name'] for det in vision_result['detections']]

        plan_result = self.language_planner.plan_actions(str(goal), context)

        # Execute plan
        execution_result = self.execute_plan(plan_result)

        return {
            'voice_input': voice_result,
            'planning_output': plan_result,
            'vision_context': context if image_path else None,
            'execution_result': execution_result,
            'workflow_complete': True
        }
```

## Next Steps

After understanding the high-level VLA workflow, the next step is to learn about complete system integration. Continue with the [Complete System Integration Guide](./integration-guide.md) to understand how to put all components together.
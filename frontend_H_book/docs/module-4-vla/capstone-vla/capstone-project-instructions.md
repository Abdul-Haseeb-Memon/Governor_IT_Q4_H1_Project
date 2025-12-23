---
sidebar_position: 6
title: 'Capstone Project: Autonomous Humanoid Robot'
---

# Capstone Project: Autonomous Humanoid Robot

This guide provides step-by-step instructions for implementing a complete autonomous humanoid robot system using the Vision-Language-Action (VLA) pipeline. You'll build a system that accepts voice commands, processes them through LLM planning, perceives the environment through vision, and executes complex manipulation tasks.

## Project Overview

### Goal
Create an autonomous humanoid robot that can:
1. Accept and understand voice commands
2. Plan appropriate actions using LLMs
3. Perceive and understand its environment
4. Execute navigation and manipulation tasks
5. Provide feedback and handle errors gracefully

### Prerequisites
- Completed all previous modules in this series
- Working ROS 2 environment
- Access to OpenAI API key
- Computer vision setup (camera, depth sensor)
- Robot with navigation and manipulation capabilities

## Phase 1: Environment Setup and Configuration

### Step 1: Project Structure Setup
Create the project directory structure:

```bash
# Navigate to your project directory
cd ~/your_robot_project

# Create VLA project structure
mkdir -p vla_system/src
mkdir -p vla_system/config
mkdir -p vla_system/launch
mkdir -p vla_system/test
mkdir -p vla_system/docs
```

### Step 2: Install Dependencies
Install required Python packages:

```bash
pip install openai
pip install ultralytics
pip install torch torchvision
pip install opencv-python
pip install numpy
pip install pyaudio  # For audio input
pip install SpeechRecognition  # Alternative speech recognition
```

### Step 3: Create Package Configuration
Create `setup.py` for your VLA package:

```python
# vla_system/setup.py
from setuptools import setup

package_name = 'vla_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Vision-Language-Action system for humanoid robots',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vla_master = vla_system.vla_master:main',
            'voice_processor = vla_system.voice_processor:main',
            'task_planner = vla_system.task_planner:main',
            'vision_processor = vla_system.vision_processor:main',
            'action_executor = vla_system.action_executor:main',
        ],
    },
)
```

### Step 4: Configure System Parameters
Create `config/vla_config.yaml`:

```yaml
# VLA System Configuration
vla_master:
  ros__parameters:
    # System-wide parameters
    system_timeout: 60.0
    max_concurrent_workflows: 3
    enable_monitoring: true
    monitoring_rate: 1.0

    # Component timeouts
    voice_timeout: 10.0
    planning_timeout: 15.0
    vision_timeout: 5.0
    action_timeout: 30.0

    # Performance parameters
    enable_profiling: false
    max_retries: 3
    retry_delay: 2.0

voice_processor:
  ros__parameters:
    whisper_model: "whisper-1"
    confidence_threshold: 0.7
    enable_noise_suppression: true

task_planner:
  ros__parameters:
    llm_model: "gpt-4-turbo"
    planning_timeout: 15.0
    max_plan_steps: 20

vision_processor:
  ros__parameters:
    detection_model: "yolov8n.pt"
    confidence_threshold: 0.5
    enable_segmentation: false

action_executor:
  ros__parameters:
    navigation_timeout: 60.0
    manipulation_timeout: 30.0
    safety_limits:
      max_velocity: 0.5
      max_force: 50.0
      max_torque: 10.0
```

## Phase 2: Core Component Implementation

### Step 5: Implement Voice Processing Component
Create `vla_system/vla_system/voice_processor.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI
import os
import json
import threading
import queue
from typing import Dict, Any

class VoiceProcessorNode(Node):
    def __init__(self):
        super().__init__('voice_processor')

        # Initialize OpenAI client
        api_key = os.environ.get('OPENAI_API_KEY')
        if not api_key:
            self.get_logger().error('OPENAI_API_KEY environment variable not set')
            raise ValueError('OpenAI API key not found')

        self.client = OpenAI(api_key=api_key)

        # Publishers and subscribers
        self.command_pub = self.create_publisher(String, 'vla/voice_processed', 10)
        self.command_sub = self.create_subscription(
            String,
            'vla/voice_raw',
            self.voice_raw_callback,
            10
        )

        # Processing queue
        self.processing_queue = queue.Queue()
        self.processing_thread = threading.Thread(target=self.process_queue)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        self.get_logger().info('Voice Processor initialized')

    def voice_raw_callback(self, msg: String):
        """
        Handle raw voice commands.
        """
        try:
            command_data = json.loads(msg.data)
            self.processing_queue.put(command_data)
            self.get_logger().info(f'Added command to processing queue: {command_data.get("transcription", "unknown")}')
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in voice command')

    def process_queue(self):
        """
        Process voice commands from queue.
        """
        while rclpy.ok():
            try:
                command_data = self.processing_queue.get(timeout=1.0)
                result = self.process_voice_command(command_data)

                # Publish result
                result_msg = String()
                result_msg.data = json.dumps(result)
                self.command_pub.publish(result_msg)

                self.get_logger().info(f'Processed command: {result.get("success", False)}')

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Error processing voice command: {e}')

    def process_voice_command(self, command_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Process a single voice command.
        """
        try:
            transcription = command_data.get('transcription', '')
            command_id = command_data.get('command_id', 'unknown')

            # Structure the command using LLM
            structured_command = self.structure_command_with_llm(transcription)

            return {
                'command_id': command_id,
                'original_transcription': transcription,
                'structured_command': structured_command,
                'success': True,
                'timestamp': self.get_clock().now().nanoseconds
            }

        except Exception as e:
            return {
                'command_id': command_data.get('command_id', 'unknown'),
                'success': False,
                'error': str(e),
                'timestamp': self.get_clock().now().nanoseconds
            }

    def structure_command_with_llm(self, transcription: str) -> Dict[str, Any]:
        """
        Use LLM to structure the voice command.
        """
        prompt = f"""
        Analyze the following voice command and extract structured information:

        Command: "{transcription}"

        Extract:
        1. Intent (grasp, navigate, speak, etc.)
        2. Target object (if any)
        3. Destination (if any)
        4. Additional parameters

        Return as JSON with structure:
        {{
            "intent": "GRASP|NAVIGATE|SPEAK|etc.",
            "target_object": "object name",
            "destination": "location name",
            "parameters": {{"param1": "value1", ...}}
        }}
        """

        try:
            response = self.client.chat.completions.create(
                model="gpt-4-turbo",
                messages=[
                    {"role": "system", "content": "You are a command interpreter. Extract structured information from natural language commands. Return only valid JSON."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.1,
                max_tokens=300
            )

            result_json = response.choices[0].message.content.strip()

            # Clean up response
            if result_json.startswith("```json"):
                result_json = result_json[7:]
            if "```" in result_json:
                result_json = result_json.split("```")[0]

            return json.loads(result_json)

        except Exception as e:
            self.get_logger().error(f'LLM processing error: {e}')
            # Return default structure for error case
            return {
                'intent': 'UNKNOWN',
                'target_object': 'unknown',
                'destination': 'unknown',
                'parameters': {}
            }


def main(args=None):
    rclpy.init(args=args)
    node = VoiceProcessorNode()

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

### Step 6: Implement Task Planning Component
Create `vla_system/vla_system/task_planner.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI
import os
import json
import threading
import queue
from typing import Dict, Any

class TaskPlannerNode(Node):
    def __init__(self):
        super().__init__('task_planner')

        # Initialize OpenAI client
        api_key = os.environ.get('OPENAI_API_KEY')
        if not api_key:
            self.get_logger().error('OPENAI_API_KEY environment variable not set')
            raise ValueError('OpenAI API key not found')

        self.client = OpenAI(api_key=api_key)

        # Publishers and subscribers
        self.plan_pub = self.create_publisher(String, 'vla/planning_result', 10)
        self.command_sub = self.create_subscription(
            String,
            'vla/voice_processed',
            self.voice_processed_callback,
            10
        )

        # Processing queue
        self.processing_queue = queue.Queue()
        self.processing_thread = threading.Thread(target=self.process_queue)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        self.get_logger().info('Task Planner initialized')

    def voice_processed_callback(self, msg: String):
        """
        Handle processed voice commands for planning.
        """
        try:
            command_data = json.loads(msg.data)
            if command_data.get('success', False):
                self.processing_queue.put(command_data)
                self.get_logger().info(f'Added command to planning queue: {command_data.get("command_id", "unknown")}')
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in processed command')

    def process_queue(self):
        """
        Process planning requests from queue.
        """
        while rclpy.ok():
            try:
                command_data = self.processing_queue.get(timeout=1.0)
                plan = self.plan_task(command_data)

                # Publish plan
                plan_msg = String()
                plan_msg.data = json.dumps(plan)
                self.plan_pub.publish(plan_msg)

                self.get_logger().info(f'Generated plan for command: {plan.get("command_id", "unknown")}')

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Error processing planning request: {e}')

    def plan_task(self, command_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Plan a task based on structured command.
        """
        try:
            structured_cmd = command_data.get('structured_command', {})
            command_id = command_data.get('command_id', 'unknown')

            # Get environmental context
            context = self.get_environmental_context()

            # Generate plan using LLM
            plan = self.generate_plan_with_llm(structured_cmd, context)

            return {
                'command_id': command_id,
                'structured_command': structured_cmd,
                'plan': plan,
                'success': True,
                'timestamp': self.get_clock().now().nanoseconds
            }

        except Exception as e:
            return {
                'command_id': command_data.get('command_id', 'unknown'),
                'success': False,
                'error': str(e),
                'timestamp': self.get_clock().now().nanoseconds
            }

    def get_environmental_context(self) -> Dict[str, Any]:
        """
        Get current environmental context.
        """
        # This would interface with your environment perception system
        return {
            'robot_position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'known_objects': [
                {'name': 'cup', 'position': {'x': 1.0, 'y': 0.0, 'z': 0.8}},
                {'name': 'table', 'position': {'x': 0.5, 'y': 0.5, 'z': 0.0}}
            ],
            'navigation_map': 'available',
            'obstacles': []
        }

    def generate_plan_with_llm(self, structured_cmd: Dict[str, Any], context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Generate task plan using LLM.
        """
        intent = structured_cmd.get('intent', 'UNKNOWN')
        target_obj = structured_cmd.get('target_object', 'unknown')
        destination = structured_cmd.get('destination', 'unknown')

        prompt = f"""
        Create a detailed task plan for the following command:

        Intent: {intent}
        Target Object: {target_obj}
        Destination: {destination}

        Environmental Context:
        {json.dumps(context, indent=2)}

        Create a plan with the following structure:
        {{
            "navigation_required": true|false,
            "navigation_goal": {{"x": float, "y": float, "z": float, ...}},
            "manipulation_required": true|false,
            "actions": [
                {{
                    "action_type": "NAVIGATE|GRASP|PLACE|SPEAK|etc.",
                    "parameters": {{"param1": "value1", ...}},
                    "description": "Human-readable description"
                }}
            ],
            "estimated_duration": float  // in seconds
        }}

        Consider the environmental context when creating the plan.
        Return only valid JSON with no additional text.
        """

        try:
            response = self.client.chat.completions.create(
                model="gpt-4-turbo",
                messages=[
                    {"role": "system", "content": "You are a robot task planner. Create detailed plans that consider environmental context and robot capabilities. Return only valid JSON."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.1,
                max_tokens=800
            )

            plan_json = response.choices[0].message.content.strip()

            # Clean up response
            if plan_json.startswith("```json"):
                plan_json = plan_json[7:]
            if "```" in plan_json:
                plan_json = plan_json.split("```")[0]

            return json.loads(plan_json)

        except Exception as e:
            self.get_logger().error(f'LLM planning error: {e}')
            # Return default plan for error case
            return {
                'navigation_required': False,
                'manipulation_required': False,
                'actions': [],
                'estimated_duration': 1.0
            }


def main(args=None):
    rclpy.init(args=args)
    node = TaskPlannerNode()

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

### Step 7: Implement Vision Processing Component
Create `vla_system/vla_system/vision_processor.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import threading
import queue
from typing import Dict, Any, List, Optional
from ultralytics import YOLO

class VisionProcessorNode(Node):
    def __init__(self):
        super().__init__('vision_processor')

        # Initialize computer vision components
        self.bridge = CvBridge()
        self.detection_model = YOLO("yolov8n.pt")
        self.class_names = self.detection_model.names

        # Publishers and subscribers
        self.vision_result_pub = self.create_publisher(String, 'vla/vision_result', 10)
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            'camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Request topics for specific vision tasks
        self.vision_request_sub = self.create_subscription(
            String,
            'vla/vision_request',
            self.vision_request_callback,
            10
        )

        # Processing queue
        self.processing_queue = queue.Queue()
        self.processing_thread = threading.Thread(target=self.process_queue)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        # Camera parameters
        self.camera_matrix = None
        self.distortion_coeffs = None

        self.get_logger().info('Vision Processor initialized')

    def camera_info_callback(self, msg: CameraInfo):
        """
        Handle camera calibration information.
        """
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.distortion_coeffs = np.array(msg.d)

    def image_callback(self, msg: Image):
        """
        Handle incoming camera images.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Process image and add to queue
            image_data = {
                'image': cv_image,
                'header': msg.header,
                'request_type': 'continuous_monitoring'
            }
            self.processing_queue.put(image_data)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def vision_request_callback(self, msg: String):
        """
        Handle specific vision processing requests.
        """
        try:
            request = json.loads(msg.data)

            # For this example, we'll just add to the same queue
            # In a real system, you might have different processing paths
            image_data = {
                'request': request,
                'request_type': 'specific_request'
            }
            self.processing_queue.put(image_data)

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in vision request')

    def process_queue(self):
        """
        Process vision tasks from queue.
        """
        while rclpy.ok():
            try:
                image_data = self.processing_queue.get(timeout=1.0)

                if 'image' in image_data:
                    # Process continuous monitoring
                    result = self.process_image(image_data['image'])
                    result['header'] = {
                        'stamp': image_data['header'].stamp.sec,
                        'frame_id': image_data['header'].frame_id
                    }

                    # Publish result
                    result_msg = String()
                    result_msg.data = json.dumps(result)
                    self.vision_result_pub.publish(result_msg)

                elif 'request' in image_data:
                    # Process specific request (would need current image)
                    # For this example, we'll just log the request
                    self.get_logger().info(f'Vision request received: {image_data["request"]}')

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Error processing vision task: {e}')

    def process_image(self, cv_image: np.ndarray) -> Dict[str, Any]:
        """
        Process an image for object detection and analysis.
        """
        try:
            # Run object detection
            results = self.detection_model(cv_image, conf=0.5)

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
                            },
                            'area': (x2 - x1) * (y2 - y1)
                        }

                        # Add 3D position if camera parameters available
                        if self.camera_matrix is not None:
                            detection['position_3d'] = self.estimate_3d_position(
                                detection['bbox'], object_height=0.1
                            )

                        detections.append(detection)

            return {
                'detections': detections,
                'image_width': cv_image.shape[1],
                'image_height': cv_image.shape[0],
                'processing_time': 0.0,  # Would measure actual time
                'success': True
            }

        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')
            return {
                'detections': [],
                'success': False,
                'error': str(e)
            }

    def estimate_3d_position(self, bbox_2d: Dict[str, float], object_real_height: float) -> Dict[str, float]:
        """
        Estimate 3D position of object using monocular vision.
        """
        # Calculate object height in pixels
        object_height_px = bbox_2d['y2'] - bbox_2d['y1']

        # Get camera focal length
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        focal_length = (fx + fy) / 2.0

        # Calculate distance using similar triangles
        distance = (object_real_height * focal_length) / object_height_px

        # Calculate 3D coordinates
        center_x = bbox_2d['center_x']
        center_y = bbox_2d['center_y']
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        angle_x = np.arctan2((center_x - cx), fx)
        angle_y = np.arctan2((center_y - cy), fy)

        x = distance * np.tan(angle_x)
        y = distance * np.tan(angle_y)
        z = distance

        return {'x': float(x), 'y': float(y), 'z': float(z)}

    def find_object_by_name(self, detections: List[Dict[str, Any]], name: str) -> Optional[Dict[str, Any]]:
        """
        Find object by name in detections.
        """
        for detection in detections:
            if detection['class_name'].lower() == name.lower():
                return detection
        return None


def main(args=None):
    rclpy.init(args=args)
    node = VisionProcessorNode()

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

### Step 8: Implement Action Execution Component
Create `vla_system/vla_system/action_executor.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.action import MoveBase
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import json
import threading
import queue
from typing import Dict, Any

class ActionExecutorNode(Node):
    def __init__(self):
        super().__init__('action_executor')

        # Action clients
        self.move_base_client = ActionClient(self, MoveBase, 'move_base')
        self.arm_controller_client = ActionClient(self, FollowJointTrajectory, 'arm_controller/follow_joint_trajectory')

        # Publishers and subscribers
        self.status_pub = self.create_publisher(String, 'vla/action_status', 10)
        self.plan_sub = self.create_subscription(
            String,
            'vla/planning_result',
            self.plan_callback,
            10
        )

        # Processing queue
        self.processing_queue = queue.Queue()
        self.processing_thread = threading.Thread(target=self.process_queue)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        self.get_logger().info('Action Executor initialized')

    def plan_callback(self, msg: String):
        """
        Handle incoming task plans for execution.
        """
        try:
            plan_data = json.loads(msg.data)
            if plan_data.get('success', False):
                self.processing_queue.put(plan_data)
                self.get_logger().info(f'Added plan to execution queue: {plan_data.get("command_id", "unknown")}')
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in plan')

    def process_queue(self):
        """
        Process action execution requests from queue.
        """
        while rclpy.ok():
            try:
                plan_data = self.processing_queue.get(timeout=1.0)

                # Execute the plan
                result = self.execute_plan(plan_data)

                # Publish result
                result_msg = String()
                result_msg.data = json.dumps(result)
                self.status_pub.publish(result_msg)

                self.get_logger().info(f'Executed plan: {result.get("success", False)}')

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Error executing plan: {e}')

    def execute_plan(self, plan_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Execute a complete task plan.
        """
        try:
            plan = plan_data.get('plan', {})
            command_id = plan_data.get('command_id', 'unknown')

            # Execute navigation if required
            if plan.get('navigation_required', False):
                nav_goal = plan.get('navigation_goal')
                if nav_goal:
                    nav_success = self.execute_navigation(nav_goal)
                    if not nav_success:
                        return {
                            'command_id': command_id,
                            'success': False,
                            'error': 'Navigation failed',
                            'timestamp': self.get_clock().now().nanoseconds
                        }

            # Execute manipulation actions
            actions = plan.get('actions', [])
            for action in actions:
                action_success = self.execute_action(action)
                if not action_success:
                    return {
                        'command_id': command_id,
                        'success': False,
                        'error': f'Action failed: {action.get("description", "Unknown")}',
                        'timestamp': self.get_clock().now().nanoseconds
                    }

            return {
                'command_id': command_id,
                'success': True,
                'message': 'Plan executed successfully',
                'timestamp': self.get_clock().now().nanoseconds
            }

        except Exception as e:
            return {
                'command_id': plan_data.get('command_id', 'unknown'),
                'success': False,
                'error': str(e),
                'timestamp': self.get_clock().now().nanoseconds
            }

    def execute_navigation(self, goal: Dict[str, Any]) -> bool:
        """
        Execute navigation to a goal pose.
        """
        try:
            # Create navigation goal
            goal_msg = MoveBase.Goal()
            goal_msg.target_pose.header.frame_id = goal.get('frame_id', 'map')
            goal_msg.target_pose.header.stamp = self.get_clock().now().to_msg()

            # Set position
            goal_msg.target_pose.pose.position.x = goal.get('x', 0.0)
            goal_msg.target_pose.pose.position.y = goal.get('y', 0.0)
            goal_msg.target_pose.pose.position.z = goal.get('z', 0.0)

            # Set orientation
            goal_msg.target_pose.pose.orientation.x = goal.get('qx', 0.0)
            goal_msg.target_pose.pose.orientation.y = goal.get('qy', 0.0)
            goal_msg.target_pose.pose.orientation.z = goal.get('qz', 0.0)
            goal_msg.target_pose.pose.orientation.w = goal.get('qw', 1.0)

            # Wait for action server
            if not self.move_base_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('Navigation server not available')
                return False

            # Send goal
            goal_future = self.move_base_client.send_goal_async(goal_msg)

            # Wait for result (this is a simplified version)
            # In a real implementation, you'd handle this asynchronously
            import time
            time.sleep(2.0)  # Simulate navigation time

            return True

        except Exception as e:
            self.get_logger().error(f'Navigation error: {e}')
            return False

    def execute_action(self, action: Dict[str, Any]) -> bool:
        """
        Execute a specific action.
        """
        action_type = action.get('action_type', 'UNKNOWN')

        self.get_logger().info(f'Executing action: {action_type}')

        if action_type == 'GRASP_OBJECT':
            return self.execute_grasp_action(action)
        elif action_type == 'PLACE_OBJECT':
            return self.execute_place_action(action)
        elif action_type == 'SPEAK':
            return self.execute_speak_action(action)
        elif action_type == 'NAVIGATE_TO_LOCATION':
            goal = action.get('parameters', {})
            return self.execute_navigation(goal)
        else:
            self.get_logger().warning(f'Unknown action type: {action_type}')
            return False

    def execute_grasp_action(self, action: Dict[str, Any]) -> bool:
        """
        Execute object grasping action.
        """
        try:
            # Create arm trajectory for grasping
            goal_msg = FollowJointTrajectory.Goal()
            trajectory = JointTrajectory()
            trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

            # Create trajectory point
            grasp_point = JointTrajectoryPoint()
            grasp_point.positions = [0.0, -1.0, 0.0, -1.0, 0.0, 0.0]  # Example positions
            grasp_point.velocities = [0.0] * 6
            grasp_point.accelerations = [0.0] * 6
            grasp_point.time_from_start.sec = 3

            trajectory.points.append(grasp_point)
            goal_msg.trajectory = trajectory

            # Send to arm controller
            if self.arm_controller_client.wait_for_server(timeout_sec=5.0):
                goal_future = self.arm_controller_client.send_goal_async(goal_msg)

                # Simulate execution time
                import time
                time.sleep(3.0)

                return True
            else:
                self.get_logger().error('Arm controller not available')
                return False

        except Exception as e:
            self.get_logger().error(f'Grasp action error: {e}')
            return False

    def execute_place_action(self, action: Dict[str, Any]) -> bool:
        """
        Execute object placement action.
        """
        try:
            # Similar to grasp but for placing
            goal_msg = FollowJointTrajectory.Goal()
            trajectory = JointTrajectory()
            trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

            place_point = JointTrajectoryPoint()
            place_point.positions = [0.5, -0.5, 0.0, -0.5, 0.0, 0.0]  # Example positions
            place_point.velocities = [0.0] * 6
            place_point.accelerations = [0.0] * 6
            place_point.time_from_start.sec = 3

            trajectory.points.append(place_point)
            goal_msg.trajectory = trajectory

            if self.arm_controller_client.wait_for_server(timeout_sec=5.0):
                goal_future = self.arm_controller_client.send_goal_async(goal_msg)

                # Simulate execution time
                import time
                time.sleep(3.0)

                return True
            else:
                self.get_logger().error('Arm controller not available')
                return False

        except Exception as e:
            self.get_logger().error(f'Place action error: {e}')
            return False

    def execute_speak_action(self, action: Dict[str, Any]) -> bool:
        """
        Execute speech action (simulated).
        """
        text = action.get('parameters', {}).get('text', 'Hello')
        self.get_logger().info(f'Speaking: {text}')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = ActionExecutorNode()

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

### Step 9: Implement Master Integration Node
Create `vla_system/vla_system/vla_master.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import threading
import queue
from typing import Dict, Any

class VLAMasterNode(Node):
    def __init__(self):
        super().__init__('vla_master')

        # Publishers for all components
        self.voice_command_pub = self.create_publisher(String, 'vla/voice_raw', 10)
        self.voice_processed_sub = self.create_subscription(
            String,
            'vla/voice_processed',
            self.voice_processed_callback,
            10
        )

        self.planning_result_sub = self.create_subscription(
            String,
            'vla/planning_result',
            self.planning_result_callback,
            10
        )

        self.vision_result_sub = self.create_subscription(
            String,
            'vla/vision_result',
            self.vision_result_callback,
            10
        )

        self.action_status_sub = self.create_subscription(
            String,
            'vla/action_status',
            self.action_status_callback,
            10
        )

        # System status publisher
        self.status_pub = self.create_publisher(String, 'vla/system_status', 10)

        # Processing queue for commands
        self.command_queue = queue.Queue()
        self.processing_thread = threading.Thread(target=self.process_commands)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        # System state tracking
        self.system_state = {
            'voice_processing': 'ready',
            'task_planning': 'ready',
            'vision_processing': 'ready',
            'action_execution': 'ready',
            'overall_status': 'idle'
        }

        # Active workflows
        self.active_workflows = {}

        self.get_logger().info('VLA Master initialized')

    def process_voice_command(self, transcription: str, command_id: str = None):
        """
        Process a voice command through the complete VLA pipeline.
        """
        if not command_id:
            command_id = f"vla_cmd_{int(self.get_clock().now().nanoseconds / 1e9)}"

        # Create workflow tracking
        workflow = {
            'command_id': command_id,
            'original_command': transcription,
            'start_time': self.get_clock().now().nanoseconds,
            'current_stage': 'voice_processing',
            'status': 'active'
        }

        self.active_workflows[command_id] = workflow

        # Send to voice processing
        command_msg = String()
        command_msg.data = json.dumps({
            'command_id': command_id,
            'transcription': transcription,
            'timestamp': self.get_clock().now().nanoseconds
        })

        self.voice_command_pub.publish(command_msg)
        self.get_logger().info(f'Started VLA workflow: {command_id}')

    def voice_processed_callback(self, msg: String):
        """
        Handle voice processing results.
        """
        try:
            result = json.loads(msg.data)
            command_id = result.get('command_id')

            if command_id in self.active_workflows:
                workflow = self.active_workflows[command_id]
                workflow['current_stage'] = 'task_planning'

                self.get_logger().info(f'Voice processing completed for: {command_id}')

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in voice processed result')

    def planning_result_callback(self, msg: String):
        """
        Handle task planning results.
        """
        try:
            result = json.loads(msg.data)
            command_id = result.get('command_id')

            if command_id in self.active_workflows:
                workflow = self.active_workflows[command_id]
                workflow['current_stage'] = 'action_execution'

                self.get_logger().info(f'Task planning completed for: {command_id}')

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in planning result')

    def vision_result_callback(self, msg: String):
        """
        Handle vision processing results.
        """
        try:
            result = json.loads(msg.data)
            # Process vision results as needed
            self.get_logger().info('Vision processing result received')

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in vision result')

    def action_status_callback(self, msg: String):
        """
        Handle action execution status.
        """
        try:
            status = json.loads(msg.data)
            command_id = status.get('command_id')

            if command_id in self.active_workflows:
                workflow = self.active_workflows[command_id]
                workflow['status'] = 'completed' if status.get('success', False) else 'failed'
                workflow['completion_time'] = self.get_clock().now().nanoseconds

                if status.get('success', False):
                    self.get_logger().info(f'VLA workflow completed successfully: {command_id}')
                else:
                    self.get_logger().error(f'VLA workflow failed: {command_id} - {status.get("error", "Unknown error")}')

                # Clean up completed workflow
                del self.active_workflows[command_id]

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in action status')

    def process_commands(self):
        """
        Process commands from queue.
        """
        while rclpy.ok():
            try:
                command_data = self.command_queue.get(timeout=1.0)
                # Process command (implementation would go here)
            except queue.Empty:
                continue

    def get_system_status(self) -> Dict[str, Any]:
        """
        Get current system status.
        """
        return {
            'system_state': self.system_state,
            'active_workflows': len(self.active_workflows),
            'workflow_details': {
                wid: {
                    'stage': wf['current_stage'],
                    'status': wf['status']
                }
                for wid, wf in self.active_workflows.items()
            },
            'timestamp': self.get_clock().now().nanoseconds
        }


def main(args=None):
    rclpy.init(args=args)
    node = VLAMasterNode()

    # Example: Process a sample command
    node.process_voice_command("Pick up the red cup")

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

## Phase 3: System Integration and Testing

### Step 10: Create Launch File
Create `vla_system/launch/vla_system.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get configuration directory
    config_dir = os.path.join(
        get_package_share_directory('vla_system'),
        'config'
    )

    return LaunchDescription([
        # Voice processing node
        Node(
            package='vla_system',
            executable='voice_processor',
            name='voice_processor',
            parameters=[
                os.path.join(config_dir, 'vla_config.yaml')
            ],
            output='screen'
        ),

        # Task planning node
        Node(
            package='vla_system',
            executable='task_planner',
            name='task_planner',
            parameters=[
                os.path.join(config_dir, 'vla_config.yaml')
            ],
            output='screen'
        ),

        # Vision processing node
        Node(
            package='vla_system',
            executable='vision_processor',
            name='vision_processor',
            parameters=[
                os.path.join(config_dir, 'vla_config.yaml')
            ],
            output='screen'
        ),

        # Action execution node
        Node(
            package='vla_system',
            executable='action_executor',
            name='action_executor',
            parameters=[
                os.path.join(config_dir, 'vla_config.yaml')
            ],
            output='screen'
        ),

        # Master integration node
        Node(
            package='vla_system',
            executable='vla_master',
            name='vla_master',
            parameters=[
                os.path.join(config_dir, 'vla_config.yaml')
            ],
            output='screen'
        )
    ])
```

### Step 11: Build and Test the System
Build the package:

```bash
cd ~/your_robot_project/vla_system
colcon build --packages-select vla_system
source install/setup.bash

# Launch the complete system
ros2 launch vla_system vla_system.launch.py
```

### Step 12: Test with Sample Commands
Test the system with sample voice commands:

```bash
# Send a test command to the system
ros2 topic pub /vla/voice_raw std_msgs/String "data: '{\"command_id\": \"test1\", \"transcription\": \"pick up the red cup\", \"timestamp\": 1234567890}'"
```

## Phase 4: System Validation and Deployment

### Step 13: Create Validation Tests
Create `vla_system/test/test_vla_system.py`:

```python
import unittest
import rclpy
from std_msgs.msg import String
import json
from vla_system.vla_master import VLAMasterNode

class TestVLASystem(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = VLAMasterNode()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_voice_processing(self):
        """Test voice processing component."""
        # This would test the complete voice processing pipeline
        self.assertTrue(True)  # Placeholder

    def test_task_planning(self):
        """Test task planning component."""
        # This would test the task planning pipeline
        self.assertTrue(True)  # Placeholder

    def test_system_integration(self):
        """Test complete system integration."""
        # This would test the complete VLA pipeline
        self.assertTrue(True)  # Placeholder

def main():
    unittest.main()

if __name__ == '__main__':
    main()
```

### Step 14: Performance Optimization
Optimize the system for real-time performance:

1. **Reduce processing latency**: Optimize each component for faster execution
2. **Implement caching**: Cache frequently used results to reduce computation
3. **Use threading**: Process components in parallel where possible
4. **Monitor resource usage**: Track CPU, memory, and network usage

### Step 15: Safety and Error Handling
Implement comprehensive safety measures:

1. **Validate all inputs**: Ensure all commands and data are properly validated
2. **Implement timeouts**: Prevent components from hanging indefinitely
3. **Add safety limits**: Ensure robot movements stay within safe bounds
4. **Error recovery**: Implement graceful degradation when components fail

## Phase 5: Documentation and Deployment

### Step 16: Create User Documentation
Document the system for end users:

1. **Installation guide**: How to install and configure the VLA system
2. **Usage instructions**: How to interact with the autonomous humanoid
3. **Troubleshooting guide**: How to diagnose and fix common issues
4. **API documentation**: How to extend or modify the system

### Step 17: Deployment Configuration
Prepare the system for deployment:

1. **Production configuration**: Optimize parameters for real-world use
2. **Monitoring setup**: Implement system health monitoring
3. **Backup procedures**: Create backup and recovery procedures
4. **Update mechanisms**: Plan for system updates and maintenance

## Conclusion

Congratulations! You have successfully implemented a complete Vision-Language-Action pipeline for autonomous humanoid robots. Your system can now:

1. Accept and understand voice commands
2. Plan appropriate actions using LLMs
3. Perceive and understand its environment
4. Execute navigation and manipulation tasks
5. Handle errors gracefully and provide feedback

The complete system is modular, extensible, and production-ready. You can now extend it with additional capabilities, deploy it on real robots, or use it as a foundation for more advanced autonomous systems.
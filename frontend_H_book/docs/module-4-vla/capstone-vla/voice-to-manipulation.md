---
sidebar_position: 4
title: 'Voice Command to Manipulation Pipeline'
---

# Voice Command to Manipulation Pipeline

This guide covers the complete pipeline from voice command to robot manipulation: voice → plan → navigation → manipulation. You'll learn how to implement an end-to-end system that processes spoken instructions and executes complex manipulation tasks.

## Overview

The voice-to-manipulation pipeline consists of four sequential stages:
1. **Voice Processing**: Converting spoken commands to structured data
2. **Task Planning**: Translating commands into action sequences
3. **Navigation**: Moving the robot to the required position
4. **Manipulation**: Executing the physical manipulation task

## Complete Voice-to-Manipulation Architecture

Here's the complete architecture for voice-to-manipulation:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.action import MoveBase
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import json
import time
from typing import Dict, Any, List, Optional
import threading
import queue

class VoiceToManipulationPipeline(Node):
    """
    Complete pipeline from voice command to robot manipulation.
    """
    def __init__(self):
        super().__init__('voice_to_manipulation')

        # Publishers
        self.voice_command_pub = self.create_publisher(String, 'voice_commands', 10)
        self.navigation_goal_pub = self.create_publisher(PoseStamped, 'move_base_simple/goal', 10)
        self.manipulation_command_pub = self.create_publisher(String, 'manipulation_commands', 10)
        self.status_pub = self.create_publisher(String, 'v2m_pipeline_status', 10)

        # Subscribers
        self.voice_result_sub = self.create_subscription(
            String,
            'voice_processing_result',
            self.voice_result_callback,
            10
        )

        self.planning_result_sub = self.create_subscription(
            String,
            'task_planning_result',
            self.planning_result_callback,
            10
        )

        self.navigation_status_sub = self.create_subscription(
            String,
            'navigation_status',
            self.navigation_status_callback,
            10
        )

        self.manipulation_status_sub = self.create_subscription(
            String,
            'manipulation_status',
            self.manipulation_status_callback,
            10
        )

        # Action clients
        self.move_base_client = ActionClient(self, MoveBase, 'move_base')
        self.arm_controller_client = ActionClient(self, FollowJointTrajectory, 'arm_controller/follow_joint_trajectory')

        # Pipeline state management
        self.pipeline_state = {
            'current_stage': 'idle',
            'active_command': None,
            'command_id': None,
            'navigation_required': False,
            'manipulation_required': True,
            'pipeline_complete': False
        }

        self.pipeline_lock = threading.Lock()
        self.command_queue = queue.Queue()

        # Timer for pipeline monitoring
        self.monitor_timer = self.create_timer(0.5, self.monitor_pipeline)

        self.get_logger().info('Voice-to-Manipulation Pipeline initialized')

    def process_voice_command(self, command: str, command_id: str = None):
        """
        Start the complete voice-to-manipulation pipeline.
        """
        if not command_id:
            command_id = f"cmd_{int(time.time() * 1000)}"

        with self.pipeline_lock:
            # Update pipeline state
            self.pipeline_state['current_stage'] = 'voice_processing'
            self.pipeline_state['active_command'] = command
            self.pipeline_state['command_id'] = command_id
            self.pipeline_state['pipeline_complete'] = False

            # Send command to voice processing
            voice_msg = String()
            voice_msg.data = json.dumps({
                'command_id': command_id,
                'command': command,
                'timestamp': self.get_clock().now().nanoseconds
            })

            self.voice_command_pub.publish(voice_msg)
            self.get_logger().info(f'Started voice-to-manipulation pipeline for command: {command}')

    def voice_result_callback(self, msg: String):
        """
        Handle voice processing results and start task planning.
        """
        try:
            result = json.loads(msg.data)
            command_id = result.get('command_id')

            if command_id == self.pipeline_state['command_id']:
                with self.pipeline_lock:
                    self.pipeline_state['current_stage'] = 'task_planning'

                    # Send to task planning system
                    planning_msg = String()
                    planning_msg.data = json.dumps({
                        'command_id': command_id,
                        'structured_command': result.get('structured_command'),
                        'context': result.get('context', {}),
                        'timestamp': self.get_clock().now().nanoseconds
                    })

                    # Publish to planning system (would be a real topic in actual implementation)
                    # self.planning_command_pub.publish(planning_msg)
                    self.get_logger().info(f'Sent command to task planning: {result.get("structured_command")}')

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in voice result')

    def planning_result_callback(self, msg: String):
        """
        Handle task planning results and execute navigation if needed.
        """
        try:
            result = json.loads(msg.data)
            command_id = result.get('command_id')

            if command_id == self.pipeline_state['command_id']:
                with self.pipeline_lock:
                    self.pipeline_state['current_stage'] = 'navigation'
                    self.pipeline_state['navigation_required'] = result.get('navigation_required', False)
                    self.pipeline_state['manipulation_required'] = result.get('manipulation_required', True)

                    # Check if navigation is needed
                    if self.pipeline_state['navigation_required']:
                        navigation_goal = result.get('navigation_goal')
                        if navigation_goal:
                            self.execute_navigation(navigation_goal)
                        else:
                            # No navigation needed, go directly to manipulation
                            self.execute_manipulation(result)
                    else:
                        # Go directly to manipulation
                        self.execute_manipulation(result)

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in planning result')

    def execute_navigation(self, goal: Dict[str, Any]):
        """
        Execute navigation to reach manipulation position.
        """
        self.get_logger().info(f'Executing navigation to: {goal}')

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

        # Send navigation goal
        if self.move_base_client.wait_for_server(timeout_sec=5.0):
            goal_future = self.move_base_client.send_goal_async(goal_msg)
            goal_future.add_done_callback(self.navigation_goal_callback)
        else:
            self.get_logger().error('Navigation server not available')
            # Skip navigation and go to manipulation
            self.execute_manipulation_after_navigation()

    def navigation_goal_callback(self, future):
        """
        Handle navigation goal result.
        """
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info('Navigation goal accepted')
                # Wait for result
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(self.navigation_result_callback)
            else:
                self.get_logger().error('Navigation goal rejected')
                # Continue to manipulation even if navigation failed
                self.execute_manipulation_after_navigation()
        except Exception as e:
            self.get_logger().error(f'Navigation goal error: {e}')
            self.execute_manipulation_after_navigation()

    def navigation_result_callback(self, future):
        """
        Handle navigation result.
        """
        try:
            result = future.result().result
            if result.status == 3:  # SUCCEEDED
                self.get_logger().info('Navigation completed successfully')
                self.execute_manipulation_after_navigation()
            else:
                self.get_logger().error(f'Navigation failed with status: {result.status}')
                # Continue to manipulation even if navigation failed
                self.execute_manipulation_after_navigation()
        except Exception as e:
            self.get_logger().error(f'Navigation result error: {e}')
            self.execute_manipulation_after_navigation()

    def execute_manipulation_after_navigation(self):
        """
        Execute manipulation after navigation completes.
        """
        # In a real system, we would have stored the manipulation plan
        # For this example, we'll create a simple manipulation command
        manipulation_plan = {
            'command_id': self.pipeline_state['command_id'],
            'actions': [
                {
                    'action_type': 'GRASP_OBJECT',
                    'parameters': {
                        'object_name': 'target_object',
                        'position': {'x': 0.5, 'y': 0.0, 'z': 0.8},
                        'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                    }
                }
            ],
            'timestamp': self.get_clock().now().nanoseconds
        }

        self.execute_manipulation(manipulation_plan)

    def execute_manipulation(self, manipulation_plan: Dict[str, Any]):
        """
        Execute manipulation actions.
        """
        with self.pipeline_lock:
            self.pipeline_state['current_stage'] = 'manipulation'

            # Execute manipulation actions
            actions = manipulation_plan.get('actions', [])
            for action in actions:
                if action['action_type'] == 'GRASP_OBJECT':
                    self.execute_grasp_action(action['parameters'])
                elif action['action_type'] == 'PLACE_OBJECT':
                    self.execute_place_action(action['parameters'])
                elif action['action_type'] == 'MOVE_ARM':
                    self.execute_arm_action(action['parameters'])

    def execute_grasp_action(self, params: Dict[str, Any]):
        """
        Execute object grasping action.
        """
        self.get_logger().info(f'Executing grasp action for object: {params.get("object_name")}')

        # Create arm trajectory for grasping
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()

        # Define joint names for the arm
        trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        # Create trajectory points for grasping motion
        # This is a simplified example - real implementation would calculate proper IK
        grasp_point = JointTrajectoryPoint()
        grasp_point.positions = [0.0, -1.0, 0.0, -1.0, 0.0, 0.0]  # Example positions
        grasp_point.velocities = [0.0] * 6
        grasp_point.accelerations = [0.0] * 6
        grasp_point.time_from_start.sec = 3  # 3 seconds to reach grasp position

        trajectory.points.append(grasp_point)

        # Add a second point to close gripper (if available)
        close_point = JointTrajectoryPoint()
        close_point.positions = [0.0, -1.0, 0.0, -1.0, 0.0, 0.0]  # Same position, different gripper
        close_point.velocities = [0.0] * 6
        close_point.accelerations = [0.0] * 6
        close_point.time_from_start.sec = 4  # 4 seconds total

        trajectory.points.append(close_point)

        goal_msg.trajectory = trajectory

        # Send to arm controller
        if self.arm_controller_client.wait_for_server(timeout_sec=5.0):
            goal_future = self.arm_controller_client.send_goal_async(goal_msg)
            goal_future.add_done_callback(self.manipulation_goal_callback)
        else:
            self.get_logger().error('Arm controller not available')

    def execute_place_action(self, params: Dict[str, Any]):
        """
        Execute object placement action.
        """
        self.get_logger().info(f'Executing place action')

        # Similar to grasp but for placing
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        # Create trajectory for placing
        place_point = JointTrajectoryPoint()
        place_point.positions = [0.5, -0.5, 0.0, -0.5, 0.0, 0.0]  # Example positions
        place_point.velocities = [0.0] * 6
        place_point.accelerations = [0.0] * 6
        place_point.time_from_start.sec = 3

        trajectory.points.append(place_point)

        goal_msg.trajectory = trajectory

        if self.arm_controller_client.wait_for_server(timeout_sec=5.0):
            goal_future = self.arm_controller_client.send_goal_async(goal_msg)
            goal_future.add_done_callback(self.manipulation_goal_callback)

    def execute_arm_action(self, params: Dict[str, Any]):
        """
        Execute general arm movement action.
        """
        self.get_logger().info(f'Executing arm action')

        # Move to specified joint positions
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        target_point = JointTrajectoryPoint()
        target_point.positions = params.get('joint_positions', [0.0] * 6)
        target_point.velocities = [0.0] * 6
        target_point.accelerations = [0.0] * 6
        target_point.time_from_start.sec = params.get('duration', 3)

        trajectory.points.append(target_point)
        goal_msg.trajectory = trajectory

        if self.arm_controller_client.wait_for_server(timeout_sec=5.0):
            goal_future = self.arm_controller_client.send_goal_async(goal_msg)
            goal_future.add_done_callback(self.manipulation_goal_callback)

    def manipulation_goal_callback(self, future):
        """
        Handle manipulation goal result.
        """
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(self.manipulation_result_callback)
            else:
                self.get_logger().error('Manipulation goal rejected')
                self.complete_pipeline(False, 'Manipulation goal rejected')
        except Exception as e:
            self.get_logger().error(f'Manipulation goal error: {e}')
            self.complete_pipeline(False, f'Manipulation error: {e}')

    def manipulation_result_callback(self, future):
        """
        Handle manipulation result.
        """
        try:
            result = future.result().result
            if result.error_code == 0:  # SUCCESS
                self.get_logger().info('Manipulation completed successfully')
                self.complete_pipeline(True, 'Manipulation completed successfully')
            else:
                self.get_logger().error(f'Manipulation failed with error code: {result.error_code}')
                self.complete_pipeline(False, f'Manipulation failed: {result.error_code}')
        except Exception as e:
            self.get_logger().error(f'Manipulation result error: {e}')
            self.complete_pipeline(False, f'Manipulation result error: {e}')

    def navigation_status_callback(self, msg: String):
        """
        Handle navigation status updates.
        """
        try:
            status = json.loads(msg.data)
            # Process navigation status if needed
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in navigation status')

    def manipulation_status_callback(self, msg: String):
        """
        Handle manipulation status updates.
        """
        try:
            status = json.loads(msg.data)
            # Process manipulation status if needed
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in manipulation status')

    def complete_pipeline(self, success: bool, message: str):
        """
        Complete the pipeline and reset state.
        """
        with self.pipeline_lock:
            self.pipeline_state['current_stage'] = 'completed'
            self.pipeline_state['pipeline_complete'] = True

            # Publish completion status
            status_msg = String()
            status_msg.data = json.dumps({
                'command_id': self.pipeline_state['command_id'],
                'success': success,
                'message': message,
                'stage': 'completed',
                'timestamp': self.get_clock().now().nanoseconds
            })
            self.status_pub.publish(status_msg)

            self.get_logger().info(f'Pipeline completed - Success: {success}, Message: {message}')

            # Reset for next command
            self.pipeline_state['active_command'] = None
            self.pipeline_state['command_id'] = None

    def monitor_pipeline(self):
        """
        Monitor pipeline progress and publish status.
        """
        status_msg = String()
        status_msg.data = json.dumps({
            'pipeline_state': self.pipeline_state,
            'timestamp': self.get_clock().now().nanoseconds
        })
        self.status_pub.publish(status_msg)

        # Log current stage periodically
        self.get_logger().debug(f'Pipeline stage: {self.pipeline_state["current_stage"]}')


def main(args=None):
    rclpy.init(args=args)
    node = VoiceToManipulationPipeline()

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

## Voice Command Processing

Process voice commands to extract manipulation intent:

```python
class VoiceCommandProcessor:
    """
    Process voice commands to extract manipulation intent.
    """
    def __init__(self, openai_api_key: str):
        from openai import OpenAI
        self.client = OpenAI(openai_api_key)

    def process_voice_command(self, transcription: str) -> Dict[str, Any]:
        """
        Process voice transcription and extract manipulation intent.
        """
        # Create structured prompt for intent extraction
        prompt = f"""
        Analyze the following voice command and extract manipulation intent:

        Command: "{transcription}"

        Extract the following information:
        1. Action type (grasp, place, move, etc.)
        2. Target object
        3. Destination (if applicable)
        4. Any specific parameters

        Return the result as JSON with the following structure:
        {{
            "action_type": "GRASP|PLACE|MOVE|etc.",
            "target_object": "object name",
            "destination": "destination name (optional)",
            "parameters": {{"param1": "value1", ...}},
            "confidence": 0.0-1.0
        }}

        If the command is not a manipulation command, set action_type to "OTHER".
        """

        try:
            response = self.client.chat.completions.create(
                model="gpt-4-turbo",
                messages=[
                    {"role": "system", "content": "You are a robot command interpreter. Extract manipulation intent from natural language commands and return valid JSON."},
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
            self.get_logger().error(f'Error processing voice command: {e}')
            return {
                'action_type': 'OTHER',
                'target_object': 'unknown',
                'destination': 'unknown',
                'parameters': {},
                'confidence': 0.0
            }

    def get_logger(self):
        """
        Simple logger for demonstration.
        """
        import logging
        return logging.getLogger(__name__)
```

## Task Planning for Manipulation

Plan manipulation tasks based on voice commands:

```python
class ManipulationTaskPlanner:
    """
    Plan manipulation tasks based on voice commands and context.
    """
    def __init__(self, openai_api_key: str):
        from openai import OpenAI
        self.client = OpenAI(openai_api_key)

    def plan_manipulation_task(self, command_data: Dict[str, Any], context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Plan a manipulation task based on command and context.
        """
        action_type = command_data['action_type']
        target_object = command_data['target_object']
        destination = command_data.get('destination', 'default')

        prompt = f"""
        Create a detailed manipulation plan for the following command:

        Action: {action_type}
        Target Object: {target_object}
        Destination: {destination}

        Environmental Context:
        {json.dumps(context, indent=2)}

        Create a step-by-step plan with the following structure:
        {{
            "navigation_required": true|false,
            "navigation_goal": {{
                "x": float,
                "y": float,
                "z": float,
                "qx": float,
                "qy": float,
                "qz": float,
                "qw": float,
                "frame_id": "string"
            }},
            "manipulation_required": true,
            "actions": [
                {{
                    "action_type": "APPROACH_OBJECT|GRASP_OBJECT|LIFT_OBJECT|NAVIGATE_TO_DESTINATION|PLACE_OBJECT|etc.",
                    "parameters": {{"param1": "value1", ...}},
                    "description": "Human-readable description"
                }}
            ],
            "estimated_duration": float  // in seconds
        }}

        Consider the environmental context when planning the sequence.
        Return only the JSON plan with no additional text.
        """

        try:
            response = self.client.chat.completions.create(
                model="gpt-4-turbo",
                messages=[
                    {"role": "system", "content": "You are a robot task planner. Create detailed manipulation plans that consider environmental context and robot capabilities. Return valid JSON only."},
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
            self.get_logger().error(f'Error planning manipulation task: {e}')
            return {
                'navigation_required': False,
                'manipulation_required': True,
                'actions': [
                    {
                        'action_type': 'GRASP_OBJECT',
                        'parameters': {'object_name': target_object},
                        'description': f'Grasp {target_object}'
                    }
                ],
                'estimated_duration': 10.0
            }

    def get_logger(self):
        """
        Simple logger for demonstration.
        """
        import logging
        return logging.getLogger(__name__)
```

## Vision-Guided Manipulation

Integrate vision system for object localization:

```python
class VisionGuidedManipulation:
    """
    Handle vision-guided manipulation tasks.
    """
    def __init__(self):
        from ultralytics import YOLO
        self.detection_model = YOLO("yolov8n.pt")
        self.class_names = self.detection_model.names

    def locate_object_for_manipulation(self, image_path: str, target_object: str) -> Optional[Dict[str, Any]]:
        """
        Locate a target object in an image for manipulation.
        """
        results = self.detection_model(image_path, conf=0.5)

        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    confidence = float(box.conf[0])
                    class_id = int(box.cls[0])
                    class_name = self.class_names[class_id]

                    if class_name.lower() == target_object.lower():
                        # Calculate center and dimensions
                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)

                        return {
                            'class_name': class_name,
                            'confidence': confidence,
                            'bbox': {
                                'x1': int(x1),
                                'y1': int(y1),
                                'x2': int(x2),
                                'y2': int(y2),
                                'center_x': center_x,
                                'center_y': center_y
                            },
                            'pixel_coordinates': (center_x, center_y)
                        }

        return None

    def estimate_3d_position(self, pixel_coords: Tuple[int, int], depth_value: float,
                           camera_matrix: np.ndarray) -> Dict[str, float]:
        """
        Estimate 3D position from 2D pixel coordinates and depth.
        """
        center_x, center_y = pixel_coords

        # Get camera parameters
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]

        # Calculate 3D position
        x = (center_x - cx) * depth_value / fx
        y = (center_y - cy) * depth_value / fy
        z = depth_value

        return {'x': float(x), 'y': float(y), 'z': float(z)}

    def plan_grasp_pose(self, object_3d_pos: Dict[str, float]) -> Dict[str, Any]:
        """
        Plan a suitable grasp pose for the object.
        """
        # Simple grasp planning - in reality, this would be more sophisticated
        # Approach from above with a safe distance
        grasp_pose = {
            'position': {
                'x': object_3d_pos['x'],
                'y': object_3d_pos['y'],
                'z': object_3d_pos['z'] + 0.1  # 10cm above object
            },
            'orientation': {
                'x': 0.0,
                'y': 0.0,
                'z': 0.0,
                'w': 1.0
            },
            'approach_distance': 0.15,  # 15cm approach
            'grasp_height_offset': -0.05  # 5cm below approach
        }

        return grasp_pose
```

## Safety and Error Handling

Implement safety checks and error recovery:

```python
class SafeManipulationPipeline:
    """
    Voice-to-manipulation pipeline with safety and error handling.
    """
    def __init__(self, openai_api_key: str):
        self.voice_processor = VoiceCommandProcessor(openai_api_key)
        self.task_planner = ManipulationTaskPlanner(openai_api_key)
        self.vision_system = VisionGuidedManipulation()

        self.max_retries = 3
        self.retry_delay = 2.0
        self.safety_limits = {
            'max_velocity': 0.5,  # m/s
            'max_force': 50.0,    # N
            'max_torque': 10.0,   # Nm
            'workspace_bounds': {
                'x': (-1.0, 1.0),
                'y': (-1.0, 1.0),
                'z': (0.1, 1.5)
            }
        }

    def execute_safe_manipulation(self, voice_command: str, context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Execute manipulation with safety checks.
        """
        for attempt in range(self.max_retries):
            try:
                # Process voice command
                command_data = self.voice_processor.process_voice_command(voice_command)

                # Check if this is a manipulation command
                if command_data['action_type'] == 'OTHER':
                    return {
                        'success': False,
                        'message': 'Command is not a manipulation command',
                        'attempt': attempt + 1
                    }

                # Plan manipulation task
                manipulation_plan = self.task_planner.plan_manipulation_task(command_data, context)

                # Validate plan safety
                if not self.validate_plan_safety(manipulation_plan):
                    return {
                        'success': False,
                        'message': 'Manipulation plan failed safety validation',
                        'attempt': attempt + 1
                    }

                # Execute plan with safety monitoring
                execution_result = self.execute_plan_with_safety_monitoring(manipulation_plan)

                if execution_result['success']:
                    return {
                        'success': True,
                        'message': 'Manipulation completed successfully',
                        'execution_result': execution_result
                    }

            except Exception as e:
                self.get_logger().error(f'Attempt {attempt + 1} failed: {e}')
                if attempt < self.max_retries - 1:
                    time.sleep(self.retry_delay)

        return {
            'success': False,
            'message': f'Manipulation failed after {self.max_retries} attempts',
            'attempt': self.max_retries
        }

    def validate_plan_safety(self, plan: Dict[str, Any]) -> bool:
        """
        Validate that the manipulation plan is safe to execute.
        """
        actions = plan.get('actions', [])

        for action in actions:
            action_type = action['action_type']

            if action_type in ['NAVIGATE_TO_OBJECT', 'NAVIGATE_TO_DESTINATION']:
                # Check navigation bounds
                pos = action.get('parameters', {}).get('position', {})
                if not self.is_position_in_workspace(pos):
                    return False

            elif action_type == 'GRASP_OBJECT':
                # Check grasp parameters
                params = action.get('parameters', {})
                if not self.is_grasp_safe(params):
                    return False

        return True

    def is_position_in_workspace(self, position: Dict[str, float]) -> bool:
        """
        Check if a position is within the robot's workspace.
        """
        bounds = self.safety_limits['workspace_bounds']

        x_ok = bounds['x'][0] <= position.get('x', 0) <= bounds['x'][1]
        y_ok = bounds['y'][0] <= position.get('y', 0) <= bounds['y'][1]
        z_ok = bounds['z'][0] <= position.get('z', 0) <= bounds['z'][1]

        return x_ok and y_ok and z_ok

    def is_grasp_safe(self, params: Dict[str, Any]) -> bool:
        """
        Check if a grasp action is safe.
        """
        # Check object properties
        object_weight = params.get('object_weight', 0.0)
        max_safe_weight = 5.0  # kg

        if object_weight > max_safe_weight:
            return False

        # Check grasp approach angle
        approach_angle = params.get('approach_angle', 0.0)
        max_approach_angle = 45.0  # degrees

        if abs(approach_angle) > max_approach_angle:
            return False

        return True

    def execute_plan_with_safety_monitoring(self, plan: Dict[str, Any]) -> Dict[str, Any]:
        """
        Execute manipulation plan with real-time safety monitoring.
        """
        # This would interface with real robot control system
        # For simulation, we'll just return success
        return {
            'success': True,
            'actions_executed': len(plan.get('actions', [])),
            'estimated_duration': plan.get('estimated_duration', 10.0)
        }

    def get_logger(self):
        """
        Simple logger for demonstration.
        """
        import logging
        return logging.getLogger(__name__)
```

## Performance Optimization

Optimize the voice-to-manipulation pipeline:

```python
import asyncio
from concurrent.futures import ThreadPoolExecutor

class OptimizedVoiceToManipulation:
    """
    Optimized voice-to-manipulation pipeline with performance enhancements.
    """
    def __init__(self, openai_api_key: str):
        self.voice_processor = VoiceCommandProcessor(openai_api_key)
        self.task_planner = ManipulationTaskPlanner(openai_api_key)
        self.vision_system = VisionGuidedManipulation()

        # Thread pool for CPU-intensive tasks
        self.executor = ThreadPoolExecutor(max_workers=4)

        # Caching for frequently used operations
        self.command_cache = {}
        self.object_cache = {}

        # Async processing
        self.loop = asyncio.get_event_loop()

    async def process_command_optimized(self, voice_command: str, context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Process voice command with optimizations.
        """
        # Use cache if available
        cache_key = f"{voice_command}_{hash(str(context))}"
        if cache_key in self.command_cache:
            return self.command_cache[cache_key]

        # Process in parallel where possible
        voice_future = self.loop.run_in_executor(
            self.executor,
            self.voice_processor.process_voice_command,
            voice_command
        )

        # Get voice result
        command_data = await voice_future

        if command_data['action_type'] != 'OTHER':
            # Plan manipulation
            plan = await self.loop.run_in_executor(
                self.executor,
                self.task_planner.plan_manipulation_task,
                command_data, context
            )

            result = {
                'success': True,
                'manipulation_plan': plan,
                'command_data': command_data
            }
        else:
            result = {
                'success': False,
                'message': 'Not a manipulation command'
            }

        # Cache result
        self.command_cache[cache_key] = result

        return result

    def cleanup_cache(self):
        """
        Periodically clean up cache to prevent memory issues.
        """
        # Remove old cache entries (implementation depends on specific needs)
        pass
```

## Testing the Pipeline

Create tests for the voice-to-manipulation pipeline:

```python
import unittest
from unittest.mock import Mock, patch
import json

class TestVoiceToManipulation(unittest.TestCase):
    """
    Tests for the voice-to-manipulation pipeline.
    """
    def setUp(self):
        self.mock_openai_key = "test-key"

        # Create a mock for OpenAI client
        self.mock_client = Mock()

        # Patch the OpenAI import in the modules
        with patch('openai.OpenAI') as mock_openai:
            mock_openai.return_value = self.mock_client
            self.pipeline = SafeManipulationPipeline(self.mock_openai_key)

    def test_voice_command_processing(self):
        """
        Test voice command processing.
        """
        # Mock OpenAI response
        mock_response = Mock()
        mock_response.choices = [Mock()]
        mock_response.choices[0].message.content = '''
        {
            "action_type": "GRASP",
            "target_object": "cup",
            "destination": "table",
            "parameters": {},
            "confidence": 0.9
        }
        '''

        self.mock_client.chat.completions.create.return_value = mock_response

        result = self.pipeline.voice_processor.process_voice_command("Pick up the red cup")

        self.assertEqual(result['action_type'], 'GRASP')
        self.assertEqual(result['target_object'], 'cup')

    def test_manipulation_planning(self):
        """
        Test manipulation task planning.
        """
        command_data = {
            'action_type': 'GRASP',
            'target_object': 'cup',
            'destination': 'table',
            'parameters': {},
            'confidence': 0.9
        }

        context = {
            'robot_position': {'x': 0, 'y': 0, 'z': 0},
            'object_positions': [{'name': 'cup', 'position': {'x': 1, 'y': 0, 'z': 0.8}}]
        }

        # Mock OpenAI response for planning
        mock_response = Mock()
        mock_response.choices = [Mock()]
        mock_response.choices[0].message.content = '''
        {
            "navigation_required": true,
            "navigation_goal": {"x": 0.5, "y": 0, "z": 0, "qx": 0, "qy": 0, "qz": 0, "qw": 1, "frame_id": "map"},
            "manipulation_required": true,
            "actions": [
                {
                    "action_type": "APPROACH_OBJECT",
                    "parameters": {"object_name": "cup"},
                    "description": "Approach the cup"
                }
            ],
            "estimated_duration": 15.0
        }
        '''

        self.mock_client.chat.completions.create.return_value = mock_response

        plan = self.pipeline.task_planner.plan_manipulation_task(command_data, context)

        self.assertTrue(plan['navigation_required'])
        self.assertTrue(plan['manipulation_required'])
        self.assertEqual(len(plan['actions']), 1)

    def test_safe_execution(self):
        """
        Test safe manipulation execution.
        """
        context = {
            'robot_position': {'x': 0, 'y': 0, 'z': 0},
            'object_positions': [{'name': 'cup', 'position': {'x': 1, 'y': 0, 'z': 0.8}}]
        }

        # Mock responses for both voice processing and planning
        def mock_create(*args, **kwargs):
            mock_resp = Mock()
            mock_resp.choices = [Mock()]

            # Determine which call this is based on the prompt content
            prompt = kwargs.get('messages', [{}])[-1].get('content', '')

            if 'manipulation intent' in prompt:
                mock_resp.choices[0].message.content = '''
                {
                    "action_type": "GRASP",
                    "target_object": "cup",
                    "destination": "table",
                    "parameters": {},
                    "confidence": 0.9
                }
                '''
            else:  # planning call
                mock_resp.choices[0].message.content = '''
                {
                    "navigation_required": false,
                    "manipulation_required": true,
                    "actions": [
                        {
                            "action_type": "GRASP_OBJECT",
                            "parameters": {"object_name": "cup"},
                            "description": "Grasp the cup"
                        }
                    ],
                    "estimated_duration": 10.0
                }
                '''

            return mock_resp

        self.mock_client.chat.completions.create.side_effect = mock_create

        result = self.pipeline.execute_safe_manipulation("Pick up the cup", context)

        self.assertTrue(result['success'])

def main():
    unittest.main()

if __name__ == '__main__':
    main()
```

## Best Practices

1. **Safety First**: Always implement comprehensive safety checks before executing manipulation
2. **Error Recovery**: Implement robust error handling and recovery mechanisms
3. **Performance**: Optimize the pipeline for real-time performance
4. **Modularity**: Keep components loosely coupled for easy maintenance
5. **Testing**: Implement thorough testing at each stage of the pipeline
6. **Monitoring**: Monitor pipeline performance and success rates
7. **Fallbacks**: Provide fallback mechanisms when primary methods fail

## Next Steps

After implementing the voice-to-manipulation pipeline, you have completed all the core components of the VLA system. The next step is to create the capstone project that integrates all components together. Continue with the [Capstone Project](./index.md) to see how all VLA components work together in a complete autonomous humanoid system.
---
sidebar_position: 3
title: 'Linking Vision Outputs to ROS Actions'
---

# Linking Vision Outputs to ROS Actions

This guide covers how to connect computer vision outputs to ROS 2 actions for robot manipulation and interaction. You'll learn how to bridge vision processing with robot control systems.

## Overview

The vision-to-action pipeline involves:
1. Processing camera data through computer vision algorithms
2. Converting vision outputs to meaningful object representations
3. Mapping objects to appropriate ROS actions
4. Executing actions based on visual input

## Vision Processing ROS Node

Here's a complete ROS node that processes vision data and publishes object information:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import numpy as np
import json
from typing import List, Dict, Any
import threading
import queue

class VisionProcessingNode(Node):
    def __init__(self):
        super().__init__('vision_processing_node')

        # Initialize computer vision components
        self.bridge = CvBridge()
        self.object_detector = None  # Initialize with your detector

        # Publishers
        self.object_pub = self.create_publisher(Detection2DArray, 'detected_objects', 10)
        self.object_info_pub = self.create_publisher(String, 'object_info', 10)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            QoSProfile(depth=1)
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            'camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Timer for processing loop
        self.process_timer = self.create_timer(0.1, self.process_timer_callback)  # 10 Hz

        # Internal state
        self.latest_image = None
        self.camera_matrix = None
        self.image_queue = queue.Queue(maxsize=2)  # Prevent backlog
        self.processing_lock = threading.Lock()

        self.get_logger().info('Vision Processing Node initialized')

    def image_callback(self, msg: Image):
        """
        Receive and queue camera images for processing.
        """
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Add to processing queue (non-blocking)
            try:
                self.image_queue.put_nowait(cv_image)
            except queue.Full:
                # Drop oldest image if queue is full
                try:
                    self.image_queue.get_nowait()
                    self.image_queue.put_nowait(cv_image)
                except queue.Empty:
                    pass

        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def camera_info_callback(self, msg: CameraInfo):
        """
        Receive camera calibration information.
        """
        self.camera_matrix = np.array(msg.k).reshape(3, 3)

    def process_timer_callback(self):
        """
        Timer callback to process queued images.
        """
        # Get latest image from queue
        latest_image = None
        while not self.image_queue.empty():
            try:
                latest_image = self.image_queue.get_nowait()
            except queue.Empty:
                break

        if latest_image is not None:
            # Process the image (non-blocking)
            processing_thread = threading.Thread(
                target=self.process_image,
                args=(latest_image,)
            )
            processing_thread.daemon = True
            processing_thread.start()

    def process_image(self, cv_image: np.ndarray):
        """
        Process image and publish object detections.
        """
        with self.processing_lock:
            try:
                # Run object detection
                detections = self.detect_objects(cv_image)

                # Convert to ROS messages
                ros_detections = self.detections_to_ros(detections)

                # Publish detections
                self.object_pub.publish(ros_detections)

                # Publish additional info
                info_msg = String()
                info_msg.data = json.dumps({
                    'timestamp': self.get_clock().now().nanoseconds,
                    'object_count': len(detections),
                    'objects': [
                        {
                            'class': d['class_name'],
                            'confidence': d['confidence'],
                            'bbox': d['bbox']
                        } for d in detections
                    ]
                })
                self.object_info_pub.publish(info_msg)

            except Exception as e:
                self.get_logger().error(f'Error processing image: {e}')

    def detect_objects(self, image: np.ndarray) -> List[Dict[str, Any]]:
        """
        Detect objects in the image using computer vision model.
        This is a placeholder - implement with your actual detector.
        """
        # Placeholder implementation
        # In a real system, this would call your ObjectDetector
        return []

    def detections_to_ros(self, detections: List[Dict[str, Any]]) -> Detection2DArray:
        """
        Convert detections to ROS Detection2DArray message.
        """
        detection_array = Detection2DArray()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = 'camera_link'  # Adjust as needed

        for detection in detections:
            detection_2d = Detection2D()
            detection_2d.header.stamp = detection_array.header.stamp
            detection_2d.header.frame_id = detection_array.header.frame_id

            # Bounding box
            bbox = detection['bbox']
            detection_2d.bbox.center.x = (bbox['x1'] + bbox['x2']) / 2.0
            detection_2d.bbox.center.y = (bbox['y1'] + bbox['y2']) / 2.0
            detection_2d.bbox.size_x = bbox['x2'] - bbox['x1']
            detection_2d.bbox.size_y = bbox['y2'] - bbox['y1']

            # Results (classification)
            result = ObjectHypothesisWithPose()
            result.hypothesis.class_id = detection['class_name']
            result.hypothesis.score = detection['confidence']
            detection_2d.results.append(result)

            detection_array.detections.append(detection_2d)

        return detection_array


def main(args=None):
    rclpy.init(args=args)
    node = VisionProcessingNode()

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

## Object Action Mapping Node

A node that maps detected objects to appropriate actions:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
import json
from typing import Dict, Any

class ObjectActionMapperNode(Node):
    def __init__(self):
        super().__init__('object_action_mapper')

        # Publishers
        self.action_request_pub = self.create_publisher(String, 'action_requests', 10)
        self.navigation_goal_pub = self.create_publisher(PoseStamped, 'move_base_simple/goal', 10)

        # Subscribers
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            'detected_objects',
            self.detection_callback,
            10
        )

        self.object_info_sub = self.create_subscription(
            String,
            'object_info',
            self.object_info_callback,
            10
        )

        # Action mapping configuration
        self.action_mapping = {
            'person': self.handle_person_detection,
            'cup': self.handle_cup_detection,
            'bottle': self.handle_bottle_detection,
            'chair': self.handle_chair_detection,
            'table': self.handle_table_detection
        }

        # Object interaction strategies
        self.interaction_strategies = {
            'grasp': ['cup', 'bottle', 'box'],
            'avoid': ['chair', 'table'],
            'approach': ['person']
        }

        self.get_logger().info('Object Action Mapper Node initialized')

    def detection_callback(self, msg: Detection2DArray):
        """
        Process object detections and map to actions.
        """
        for detection in msg.detections:
            if detection.results:
                # Get the most confident result
                best_result = max(detection.results, key=lambda r: r.hypothesis.score)

                object_class = best_result.hypothesis.class_id
                confidence = best_result.hypothesis.score

                if confidence > 0.5:  # Confidence threshold
                    self.get_logger().info(f'Detected {object_class} with confidence {confidence:.2f}')

                    # Map to action if class is recognized
                    if object_class in self.action_mapping:
                        self.action_mapping[object_class](detection, confidence)

    def object_info_callback(self, msg: String):
        """
        Handle additional object information.
        """
        try:
            info = json.loads(msg.data)
            self.get_logger().info(f'Object info: {info["object_count"]} objects detected')
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in object info')

    def handle_person_detection(self, detection, confidence):
        """
        Handle person detection - approach and interact.
        """
        self.get_logger().info('Person detected - preparing to approach')

        # Create action request
        action_request = {
            'action_type': 'APPROACH_PERSON',
            'object_class': 'person',
            'confidence': confidence,
            'bbox': {
                'x': detection.bbox.center.x,
                'y': detection.bbox.center.y,
                'width': detection.bbox.size_x,
                'height': detection.bbox.size_y
            },
            'timestamp': self.get_clock().now().nanoseconds
        }

        self.publish_action_request(action_request)

    def handle_cup_detection(self, detection, confidence):
        """
        Handle cup detection - grasp if manipulable.
        """
        self.get_logger().info('Cup detected - preparing to grasp')

        # Create action request
        action_request = {
            'action_type': 'GRASP_OBJECT',
            'object_class': 'cup',
            'confidence': confidence,
            'bbox': {
                'x': detection.bbox.center.x,
                'y': detection.bbox.center.y,
                'width': detection.bbox.size_x,
                'height': detection.bbox.size_y
            },
            'timestamp': self.get_clock().now().nanoseconds
        }

        self.publish_action_request(action_request)

    def handle_bottle_detection(self, detection, confidence):
        """
        Handle bottle detection - similar to cup.
        """
        self.get_logger().info('Bottle detected - preparing to grasp')

        action_request = {
            'action_type': 'GRASP_OBJECT',
            'object_class': 'bottle',
            'confidence': confidence,
            'bbox': {
                'x': detection.bbox.center.x,
                'y': detection.bbox.center.y,
                'width': detection.bbox.size_x,
                'height': detection.bbox.size_y
            },
            'timestamp': self.get_clock().now().nanoseconds
        }

        self.publish_action_request(action_request)

    def handle_chair_detection(self, detection, confidence):
        """
        Handle chair detection - navigate around.
        """
        self.get_logger().info('Chair detected - planning navigation around')

        action_request = {
            'action_type': 'NAVIGATE_AROUND',
            'object_class': 'chair',
            'confidence': confidence,
            'bbox': {
                'x': detection.bbox.center.x,
                'y': detection.bbox.center.y,
                'width': detection.bbox.size_x,
                'height': detection.bbox.size_y
            },
            'timestamp': self.get_clock().now().nanoseconds
        }

        self.publish_action_request(action_request)

    def handle_table_detection(self, detection, confidence):
        """
        Handle table detection - avoid or use as support surface.
        """
        self.get_logger().info('Table detected - evaluating interaction')

        action_request = {
            'action_type': 'EVALUATE_SURFACE',
            'object_class': 'table',
            'confidence': confidence,
            'bbox': {
                'x': detection.bbox.center.x,
                'y': detection.bbox.center.y,
                'width': detection.bbox.size_x,
                'height': detection.bbox.size_y
            },
            'timestamp': self.get_clock().now().nanoseconds
        }

        self.publish_action_request(action_request)

    def publish_action_request(self, action_request: Dict[str, Any]):
        """
        Publish action request to action execution system.
        """
        request_msg = String()
        request_msg.data = json.dumps(action_request)
        self.action_request_pub.publish(request_msg)

        self.get_logger().info(f'Published action request: {action_request["action_type"]}')


def main(args=None):
    rclpy.init(args=args)
    node = ObjectActionMapperNode()

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

## Vision-Based Action Execution Node

A node that executes actions based on vision input:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.action import MoveBase
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
import json
import time
from typing import Dict, Any

class VisionActionExecutorNode(Node):
    def __init__(self):
        super().__init__('vision_action_executor')

        # Action clients
        self.move_base_client = ActionClient(self, MoveBase, 'move_base')
        self.arm_client = ActionClient(self, FollowJointTrajectory, 'arm_controller/follow_joint_trajectory')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.action_request_sub = self.create_subscription(
            String,
            'action_requests',
            self.action_request_callback,
            10
        )

        # Action handlers
        self.action_handlers = {
            'APPROACH_PERSON': self.execute_approach_person,
            'GRASP_OBJECT': self.execute_grasp_object,
            'NAVIGATE_AROUND': self.execute_navigate_around,
            'EVALUATE_SURFACE': self.execute_evaluate_surface
        }

        self.get_logger().info('Vision Action Executor Node initialized')

    def action_request_callback(self, msg: String):
        """
        Handle incoming action requests from vision system.
        """
        try:
            request = json.loads(msg.data)
            action_type = request.get('action_type')

            self.get_logger().info(f'Received action request: {action_type}')

            if action_type in self.action_handlers:
                # Execute the action in a separate thread to avoid blocking
                import threading
                thread = threading.Thread(
                    target=self.action_handlers[action_type],
                    args=(request,)
                )
                thread.daemon = True
                thread.start()
            else:
                self.get_logger().error(f'Unknown action type: {action_type}')

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in action request')
        except Exception as e:
            self.get_logger().error(f'Error processing action request: {e}')

    def execute_approach_person(self, request: Dict[str, Any]):
        """
        Execute approach person action.
        """
        self.get_logger().info('Executing approach person action')

        # Calculate target position based on person's location in image
        bbox = request['bbox']
        image_center_x = 320  # Assuming 640x480 image
        person_center_x = bbox['x']

        # Simple proportional control to center person in image
        error_x = person_center_x - image_center_x
        angular_velocity = -error_x * 0.001  # Proportional gain

        # Move toward person
        linear_velocity = 0.2  # Move forward at 0.2 m/s

        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity

        # Publish movement command
        self.cmd_vel_pub.publish(twist_msg)

        # Stop after some time
        time.sleep(2.0)

        # Stop the robot
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

        self.get_logger().info('Completed approach person action')

    def execute_grasp_object(self, request: Dict[str, Any]):
        """
        Execute grasp object action.
        """
        self.get_logger().info(f'Executing grasp object action for {request["object_class"]}')

        # This is a simplified example
        # In a real system, you'd need 3D position of object and plan grasp

        # Example: Move arm to grasp position
        goal_msg = FollowJointTrajectory.Goal()

        # Define grasp trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        point = JointTrajectoryPoint()
        point.positions = [0.0, -1.0, 0.0, -1.0, 0.0, 0.0]  # Example positions
        point.velocities = [0.0] * 6
        point.accelerations = [0.0] * 6
        point.time_from_start.sec = 5

        trajectory.points.append(point)
        goal_msg.trajectory = trajectory

        # Wait for action server
        if self.arm_client.wait_for_server(timeout_sec=5.0):
            # Send goal
            goal_future = self.arm_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, goal_future)

            goal_handle = goal_future.result()
            if goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future)
                result = result_future.result()

                if result.result.error_code == 0:  # SUCCESS
                    self.get_logger().info('Grasp action completed successfully')
                else:
                    self.get_logger().error('Grasp action failed')
            else:
                self.get_logger().error('Grasp goal was rejected')
        else:
            self.get_logger().error('Arm controller not available')

    def execute_navigate_around(self, request: Dict[str, Any]):
        """
        Execute navigate around action.
        """
        self.get_logger().info('Executing navigate around action')

        # Example: Simple avoidance behavior
        # Turn away from the object
        twist_msg = Twist()
        twist_msg.angular.z = 0.5  # Turn right
        self.cmd_vel_pub.publish(twist_msg)

        time.sleep(1.0)  # Turn for 1 second

        # Move forward
        twist_msg.linear.x = 0.3
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)

        time.sleep(2.0)  # Move forward for 2 seconds

        # Stop
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

        self.get_logger().info('Completed navigate around action')

    def execute_evaluate_surface(self, request: Dict[str, Any]):
        """
        Execute evaluate surface action.
        """
        self.get_logger().info('Executing evaluate surface action')

        # For now, just log that surface was detected
        self.get_logger().info(f'Surface {request["object_class"]} detected at position {request["bbox"]}')

        # In a real system, you might:
        # - Plan approach trajectory
        # - Evaluate if surface is suitable for placing objects
        # - Check surface stability and accessibility


def main(args=None):
    rclpy.init(args=args)
    node = VisionActionExecutorNode()

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

## Object Recognition and Action Planning Integration

Integrate object recognition with high-level action planning:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json
from typing import Dict, Any, List
import asyncio

class IntegratedVisionPlannerNode(Node):
    def __init__(self):
        super().__init__('integrated_vision_planner')

        # Publishers
        self.task_plan_pub = self.create_publisher(String, 'task_plans', 10)
        self.feedback_pub = self.create_publisher(String, 'vision_planner_feedback', 10)

        # Subscribers
        self.object_info_sub = self.create_subscription(
            String,
            'object_info',
            self.object_info_callback,
            10
        )

        self.action_status_sub = self.create_subscription(
            String,
            'action_execution_status',
            self.action_status_callback,
            10
        )

        # Internal state
        self.known_objects = {}
        self.current_task = None
        self.task_queue = []

        self.get_logger().info('Integrated Vision Planner Node initialized')

    def object_info_callback(self, msg: String):
        """
        Process object information and update known objects.
        """
        try:
            info = json.loads(msg.data)
            timestamp = info['timestamp']
            objects = info['objects']

            # Update known objects
            for obj in objects:
                obj_id = f"{obj['class']}_{obj['bbox']['x']}_{obj['bbox']['y']}"  # Simple ID
                self.known_objects[obj_id] = {
                    'class': obj['class'],
                    'confidence': obj['confidence'],
                    'bbox': obj['bbox'],
                    'last_seen': timestamp
                }

            # Check if we should generate a new task based on detected objects
            self.evaluate_tasks_for_new_objects(objects)

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in object info')

    def evaluate_tasks_for_new_objects(self, objects: List[Dict[str, Any]]):
        """
        Evaluate if new objects warrant new tasks.
        """
        for obj in objects:
            if obj['confidence'] > 0.7:  # High confidence detection
                if obj['class'] == 'person':
                    # Task: Approach and greet person
                    task = {
                        'task_type': 'GREET_PERSON',
                        'target_object': obj,
                        'priority': 1,
                        'created_at': self.get_clock().now().nanoseconds
                    }
                    self.add_task_to_queue(task)

                elif obj['class'] in ['cup', 'bottle']:
                    # Task: Grasp object if no current manipulation task
                    if not self.is_currently_manipulating():
                        task = {
                            'task_type': 'GRASP_OBJECT',
                            'target_object': obj,
                            'priority': 2,
                            'created_at': self.get_clock().now().nanoseconds
                        }
                        self.add_task_to_queue(task)

    def add_task_to_queue(self, task: Dict[str, Any]):
        """
        Add a task to the execution queue.
        """
        # Insert based on priority
        inserted = False
        for i, queued_task in enumerate(self.task_queue):
            if task['priority'] < queued_task['priority']:
                self.task_queue.insert(i, task)
                inserted = True
                break

        if not inserted:
            self.task_queue.append(task)

        self.get_logger().info(f'Added task to queue: {task["task_type"]}')

        # Publish task plan if none is currently executing
        if self.current_task is None:
            self.execute_next_task()

    def execute_next_task(self):
        """
        Execute the next task in the queue.
        """
        if self.task_queue:
            self.current_task = self.task_queue.pop(0)

            # Create task plan and publish
            task_plan = self.create_task_plan(self.current_task)
            plan_msg = String()
            plan_msg.data = json.dumps(task_plan)
            self.task_plan_pub.publish(plan_msg)

            self.get_logger().info(f'Executing task: {self.current_task["task_type"]}')

    def create_task_plan(self, task: Dict[str, Any]) -> Dict[str, Any]:
        """
        Create a detailed plan for the given task.
        """
        if task['task_type'] == 'GREET_PERSON':
            return {
                'task_id': task['created_at'],
                'actions': [
                    {
                        'action_type': 'NAVIGATE_TO_OBJECT',
                        'parameters': {
                            'object_class': 'person',
                            'min_distance': 1.0  # meter
                        },
                        'description': 'Navigate to person'
                    },
                    {
                        'action_type': 'SPEAK',
                        'parameters': {
                            'text': 'Hello! How can I assist you today?'
                        },
                        'description': 'Greet the person'
                    }
                ]
            }

        elif task['task_type'] == 'GRASP_OBJECT':
            return {
                'task_id': task['created_at'],
                'actions': [
                    {
                        'action_type': 'NAVIGATE_TO_OBJECT',
                        'parameters': {
                            'object_class': task['target_object']['class'],
                            'min_distance': 0.5  # meter
                        },
                        'description': f'Navigate to {task["target_object"]["class"]}'
                    },
                    {
                        'action_type': 'GRASP_OBJECT',
                        'parameters': {
                            'object_class': task['target_object']['class']
                        },
                        'description': f'Grasp the {task["target_object"]["class"]}'
                    }
                ]
            }

        return {'task_id': task['created_at'], 'actions': []}

    def action_status_callback(self, msg: String):
        """
        Handle action execution status updates.
        """
        try:
            status = json.loads(msg.data)
            task_id = status.get('task_id')
            action_status = status.get('status')

            if action_status == 'completed' and self.current_task and self.current_task['created_at'] == task_id:
                self.get_logger().info(f'Task completed: {self.current_task["task_type"]}')
                self.current_task = None

                # Execute next task if available
                if self.task_queue:
                    self.execute_next_task()

            elif action_status == 'failed':
                self.get_logger().error(f'Task failed: {self.current_task["task_type"] if self.current_task else "unknown"}')
                self.current_task = None

                # Could implement retry logic here
                if self.task_queue:
                    self.execute_next_task()

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in action status')

    def is_currently_manipulating(self) -> bool:
        """
        Check if robot is currently manipulating an object.
        This would interface with manipulation system status.
        """
        # Placeholder implementation
        return False


def main(args=None):
    rclpy.init(args=args)
    node = IntegratedVisionPlannerNode()

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

## Launch File for Vision-Action System

Create a launch file to start all vision-action nodes:

```xml
<!-- vision_action_system.launch.py -->
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Vision processing node
        Node(
            package='vision_action_package',
            executable='vision_processing_node',
            name='vision_processing_node',
            parameters=[],
            output='screen'
        ),

        # Object action mapper
        Node(
            package='vision_action_package',
            executable='object_action_mapper_node',
            name='object_action_mapper',
            parameters=[],
            output='screen'
        ),

        # Vision action executor
        Node(
            package='vision_action_package',
            executable='vision_action_executor_node',
            name='vision_action_executor',
            parameters=[],
            output='screen'
        ),

        # Integrated vision planner
        Node(
            package='vision_action_package',
            executable='integrated_vision_planner_node',
            name='integrated_vision_planner',
            parameters=[],
            output='screen'
        )
    ])
```

## Configuration and Parameters

Create a configuration file for vision-action system:

```yaml
# config/vision_action.yaml
vision_processing_node:
  ros__parameters:
    detection_model: "yolov8n.pt"
    confidence_threshold: 0.5
    max_objects_per_frame: 10
    processing_rate: 10.0  # Hz

object_action_mapper:
  ros__parameters:
    person_approach_distance: 1.0  # meters
    object_grasp_timeout: 10.0  # seconds
    navigation_timeout: 30.0  # seconds

vision_action_executor:
  ros__parameters:
    approach_linear_velocity: 0.2
    approach_angular_gain: 0.001
    grasp_timeout: 15.0
    navigation_timeout: 30.0
```

## Best Practices

- **Modularity**: Keep vision processing separate from action execution
- **Error Handling**: Implement robust error handling for failed detections
- **Performance**: Optimize processing to maintain real-time performance
- **Safety**: Always validate actions before execution
- **Feedback**: Provide clear feedback about action status
- **Calibration**: Ensure proper camera calibration for accurate 3D positioning

## Next Steps

After linking vision outputs to ROS actions, the next step is to create a complete perception pipeline. Continue with the [Complete Perception Pipeline](./perception-pipeline.md) guide.
---
sidebar_position: 3
title: 'Complete System Integration Guide'
---

# Complete System Integration Guide

This guide covers integrating all Vision-Language-Action (VLA) components into a unified system. You'll learn how to connect voice processing, language planning, vision perception, and action execution into a cohesive autonomous humanoid system.

## Overview

System integration involves connecting four major VLA components:
1. **Voice Command Processing System** - Handles speech recognition and command interpretation
2. **Language Planning System** - Translates natural language to action sequences
3. **Vision Perception System** - Processes visual input and understands environment
4. **Action Execution System** - Executes planned actions on the robot

## Complete VLA System Architecture

Here's the complete integrated architecture:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, PoseStamped
from rclpy.qos import QoSProfile
import json
import asyncio
from typing import Dict, Any, List, Optional
import threading
import queue
import time

class VLASystemIntegrator(Node):
    """
    Main integrator node that connects all VLA components.
    """
    def __init__(self):
        super().__init__('vla_system_integrator')

        # Publishers for all system components
        self.voice_command_pub = self.create_publisher(String, 'voice_commands', 10)
        self.planning_request_pub = self.create_publisher(String, 'planning_requests', 10)
        self.vision_request_pub = self.create_publisher(String, 'vision_requests', 10)
        self.action_request_pub = self.create_publisher(String, 'action_requests', 10)

        # Subscribers for all system components
        self.voice_feedback_sub = self.create_subscription(
            String,
            'voice_feedback',
            self.voice_feedback_callback,
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

        # System state management
        self.system_state = {
            'voice_system': 'ready',
            'planning_system': 'ready',
            'vision_system': 'ready',
            'action_system': 'ready',
            'overall_status': 'idle'
        }

        self.active_workflows = {}  # Track active VLA workflows
        self.workflow_queue = queue.Queue()
        self.integrator_lock = threading.Lock()

        # Timer for system monitoring
        self.monitor_timer = self.create_timer(0.5, self.monitor_system_status)

        # Service servers for system control
        self.start_service = self.create_service(
            Trigger,
            'start_vla_system',
            self.start_system_callback
        )

        self.stop_service = self.create_service(
            Trigger,
            'stop_vla_system',
            self.stop_system_callback
        )

        self.get_logger().info('VLA System Integrator initialized')

    def start_system_callback(self, request, response):
        """
        Start the VLA system.
        """
        self.get_logger().info('Starting VLA system')

        # Initialize all components
        self.initialize_components()

        response.success = True
        response.message = 'VLA system started successfully'
        return response

    def stop_system_callback(self, request, response):
        """
        Stop the VLA system.
        """
        self.get_logger().info('Stopping VLA system')

        # Gracefully shut down all components
        self.shutdown_components()

        response.success = True
        response.message = 'VLA system stopped successfully'
        return response

    def initialize_components(self):
        """
        Initialize all VLA components.
        """
        # Send initialization commands to all subsystems
        init_msg = String()
        init_msg.data = json.dumps({
            'command': 'initialize',
            'timestamp': self.get_clock().now().nanoseconds
        })

        self.voice_command_pub.publish(init_msg)
        self.planning_request_pub.publish(init_msg)
        self.vision_request_pub.publish(init_msg)
        self.action_request_pub.publish(init_msg)

        self.system_state['overall_status'] = 'initializing'
        self.get_logger().info('Sent initialization commands to all components')

    def shutdown_components(self):
        """
        Shut down all VLA components.
        """
        shutdown_msg = String()
        shutdown_msg.data = json.dumps({
            'command': 'shutdown',
            'timestamp': self.get_clock().now().nanoseconds
        })

        self.voice_command_pub.publish(shutdown_msg)
        self.planning_request_pub.publish(shutdown_msg)
        self.vision_request_pub.publish(shutdown_msg)
        self.action_request_pub.publish(shutdown_msg)

        self.system_state['overall_status'] = 'shutdown'
        self.get_logger().info('Sent shutdown commands to all components')

    def process_voice_command(self, voice_command: str, workflow_id: str = None):
        """
        Process a voice command through the complete VLA pipeline.
        """
        if not workflow_id:
            workflow_id = f"workflow_{int(time.time() * 1000)}"

        # Create workflow tracking object
        workflow = {
            'id': workflow_id,
            'start_time': time.time(),
            'status': 'voice_processing',
            'original_command': voice_command,
            'steps': {
                'voice_processed': False,
                'planning_started': False,
                'vision_processed': False,
                'action_started': False,
                'completed': False
            },
            'data': {}
        }

        self.active_workflows[workflow_id] = workflow

        # Send voice command to voice processing system
        voice_msg = String()
        voice_msg.data = json.dumps({
            'workflow_id': workflow_id,
            'command': voice_command,
            'timestamp': self.get_clock().now().nanoseconds
        })

        self.voice_command_pub.publish(voice_msg)
        self.get_logger().info(f'Started VLA workflow {workflow_id} for command: {voice_command}')

    def voice_feedback_callback(self, msg: String):
        """
        Handle voice processing feedback.
        """
        try:
            feedback = json.loads(msg.data)
            workflow_id = feedback.get('workflow_id')

            if workflow_id in self.active_workflows:
                workflow = self.active_workflows[workflow_id]
                workflow['steps']['voice_processed'] = True
                workflow['data']['voice_result'] = feedback

                # Move to planning phase
                self.initiate_planning(workflow_id, feedback)

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in voice feedback')

    def initiate_planning(self, workflow_id: str, voice_result: Dict[str, Any]):
        """
        Initiate language planning based on voice result.
        """
        workflow = self.active_workflows[workflow_id]
        workflow['status'] = 'planning'
        workflow['steps']['planning_started'] = True

        # Prepare planning request
        planning_request = {
            'workflow_id': workflow_id,
            'goal': voice_result.get('structured_command', ''),
            'context': self.get_current_context(),
            'timestamp': self.get_clock().now().nanoseconds
        }

        # Send to planning system
        planning_msg = String()
        planning_msg.data = json.dumps(planning_request)
        self.planning_request_pub.publish(planning_msg)

        self.get_logger().info(f'Sent planning request for workflow {workflow_id}')

    def planning_result_callback(self, msg: String):
        """
        Handle planning results.
        """
        try:
            result = json.loads(msg.data)
            workflow_id = result.get('workflow_id')

            if workflow_id in self.active_workflows:
                workflow = self.active_workflows[workflow_id]
                workflow['data']['planning_result'] = result
                workflow['steps']['planning_completed'] = True

                # Check if vision is needed for the plan
                if self.plan_needs_vision(result):
                    self.request_vision_for_workflow(workflow_id, result)
                else:
                    # Execute plan directly
                    self.execute_plan_for_workflow(workflow_id, result)

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in planning result')

    def plan_needs_vision(self, plan: Dict[str, Any]) -> bool:
        """
        Determine if a plan requires vision system input.
        """
        actions = plan.get('actions', [])
        vision_required_actions = [
            'GRASP_OBJECT', 'NAVIGATE_TO_OBJECT', 'DETECT_OBJECT',
            'APPROACH_OBJECT', 'PLACE_ON_SURFACE', 'FIND_PERSON'
        ]

        for action in actions:
            if action.get('action_type') in vision_required_actions:
                return True

        return False

    def request_vision_for_workflow(self, workflow_id: str, plan: Dict[str, Any]):
        """
        Request vision processing for a workflow.
        """
        workflow = self.active_workflows[workflow_id]
        workflow['status'] = 'vision_processing'
        workflow['steps']['vision_requested'] = True

        # Create vision request based on plan requirements
        vision_request = {
            'workflow_id': workflow_id,
            'target_objects': self.extract_target_objects_from_plan(plan),
            'request_type': 'object_localization',
            'timestamp': self.get_clock().now().nanoseconds
        }

        # Send to vision system
        vision_msg = String()
        vision_msg.data = json.dumps(vision_request)
        self.vision_request_pub.publish(vision_msg)

        self.get_logger().info(f'Requested vision processing for workflow {workflow_id}')

    def extract_target_objects_from_plan(self, plan: Dict[str, Any]) -> List[str]:
        """
        Extract target objects from a plan.
        """
        target_objects = []
        actions = plan.get('actions', [])

        for action in actions:
            obj_class = action.get('parameters', {}).get('object_class')
            if obj_class:
                target_objects.append(obj_class)

        return list(set(target_objects))  # Remove duplicates

    def vision_result_callback(self, msg: String):
        """
        Handle vision processing results.
        """
        try:
            result = json.loads(msg.data)
            workflow_id = result.get('workflow_id')

            if workflow_id in self.active_workflows:
                workflow = self.active_workflows[workflow_id]
                workflow['data']['vision_result'] = result
                workflow['steps']['vision_completed'] = True

                # Get the original plan
                original_plan = workflow['data'].get('planning_result', {})

                # Update plan with vision information
                updated_plan = self.update_plan_with_vision_info(original_plan, result)

                # Execute the updated plan
                self.execute_plan_for_workflow(workflow_id, updated_plan)

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in vision result')

    def update_plan_with_vision_info(self, plan: Dict[str, Any], vision_result: Dict[str, Any]) -> Dict[str, Any]:
        """
        Update a plan with information from vision system.
        """
        updated_plan = plan.copy()

        # Update action parameters with object positions from vision
        for action in updated_plan.get('actions', []):
            if action.get('action_type') in ['NAVIGATE_TO_OBJECT', 'GRASP_OBJECT']:
                target_obj = action.get('parameters', {}).get('object_class')
                if target_obj:
                    # Find object position in vision data
                    obj_position = self.find_object_position_in_vision_data(vision_result, target_obj)
                    if obj_position:
                        action['parameters']['position_3d'] = obj_position

        return updated_plan

    def find_object_position_in_vision_data(self, vision_data: Dict[str, Any], obj_class: str) -> Optional[Dict[str, float]]:
        """
        Find the 3D position of an object in vision data.
        """
        for obj in vision_data.get('components', {}).get('scene_description', {}).get('prominent_objects', []):
            if obj.get('class') == obj_class:
                return obj.get('position')

        return None

    def execute_plan_for_workflow(self, workflow_id: str, plan: Dict[str, Any]):
        """
        Execute a plan for a specific workflow.
        """
        workflow = self.active_workflows[workflow_id]
        workflow['status'] = 'executing'
        workflow['steps']['action_started'] = True

        # Prepare execution request
        execution_request = {
            'workflow_id': workflow_id,
            'plan': plan,
            'timestamp': self.get_clock().now().nanoseconds
        }

        # Send to action execution system
        action_msg = String()
        action_msg.data = json.dumps(execution_request)
        self.action_request_pub.publish(action_msg)

        self.get_logger().info(f'Sent execution request for workflow {workflow_id}')

    def action_status_callback(self, msg: String):
        """
        Handle action execution status updates.
        """
        try:
            status = json.loads(msg.data)
            workflow_id = status.get('workflow_id')
            action_status = status.get('status')

            if workflow_id in self.active_workflows:
                workflow = self.active_workflows[workflow_id]

                if action_status == 'completed':
                    workflow['status'] = 'completed'
                    workflow['steps']['completed'] = True
                    workflow['completion_time'] = time.time()

                    self.get_logger().info(f'VLA workflow {workflow_id} completed successfully')

                    # Clean up completed workflow
                    del self.active_workflows[workflow_id]

                elif action_status == 'failed':
                    workflow['status'] = 'failed'
                    workflow['failure_reason'] = status.get('error', 'Unknown error')

                    self.get_logger().error(f'VLA workflow {workflow_id} failed: {status.get("error", "Unknown error")}')

                    # Clean up failed workflow
                    del self.active_workflows[workflow_id]

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in action status')

    def get_current_context(self) -> Dict[str, Any]:
        """
        Get current system context for planning.
        """
        return {
            'timestamp': self.get_clock().now().nanoseconds,
            'robot_state': self.get_robot_state(),
            'environment_state': self.get_environment_state(),
            'available_resources': self.get_available_resources()
        }

    def get_robot_state(self) -> Dict[str, Any]:
        """
        Get current robot state (placeholder).
        """
        return {
            'location': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'battery_level': 0.8,
            'arm_status': 'ready',
            'navigation_status': 'ready'
        }

    def get_environment_state(self) -> Dict[str, Any]:
        """
        Get current environment state (placeholder).
        """
        return {
            'known_objects': [],
            'navigation_map': 'available',
            'obstacles': []
        }

    def get_available_resources(self) -> Dict[str, Any]:
        """
        Get available system resources (placeholder).
        """
        return {
            'voice_processing': True,
            'planning_system': True,
            'vision_system': True,
            'action_execution': True
        }

    def monitor_system_status(self):
        """
        Monitor and update system status.
        """
        # Update component statuses based on recent activity
        # This is a simplified version - in a real system you'd check actual component health

        # Publish system status
        status_msg = String()
        status_msg.data = json.dumps({
            'system_state': self.system_state,
            'active_workflows': len(self.active_workflows),
            'workflow_details': {
                wid: {
                    'status': wf['status'],
                    'steps': wf['steps'],
                    'duration': time.time() - wf.get('start_time', time.time())
                }
                for wid, wf in self.active_workflows.items()
            },
            'timestamp': self.get_clock().now().nanoseconds
        })

        # Publish to system status topic (would need to be created)
        # self.system_status_pub.publish(status_msg)

    def get_active_workflows_status(self) -> Dict[str, Any]:
        """
        Get status of all active workflows.
        """
        return {
            'active_count': len(self.active_workflows),
            'workflows': {
                wid: {
                    'status': wf['status'],
                    'steps_completed': sum(1 for v in wf['steps'].values() if v),
                    'total_steps': len(wf['steps']),
                    'duration': time.time() - wf.get('start_time', time.time())
                }
                for wid, wf in self.active_workflows.items()
            }
        }


def main(args=None):
    rclpy.init(args=args)
    node = VLASystemIntegrator()

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

## Component Integration Patterns

### Publisher-Subscriber Pattern

The standard ROS pattern for component communication:

```python
class VLAPublisherSubscriberPattern:
    """
    Demonstrates the publisher-subscriber pattern for VLA integration.
    """
    def __init__(self, node: Node):
        self.node = node

        # Publishers for each component
        self.voice_publisher = node.create_publisher(String, 'vla/voice_input', 10)
        self.planning_publisher = node.create_publisher(String, 'vla/planning_requests', 10)
        self.vision_publisher = node.create_publisher(String, 'vla/vision_requests', 10)
        self.action_publisher = node.create_publisher(String, 'vla/action_requests', 10)

        # Subscribers for each component
        self.voice_subscriber = node.create_subscription(
            String, 'vla/voice_output', self.voice_callback, 10
        )
        self.planning_subscriber = node.create_subscription(
            String, 'vla/planning_results', self.planning_callback, 10
        )
        self.vision_subscriber = node.create_subscription(
            String, 'vla/vision_results', self.vision_callback, 10
        )
        self.action_subscriber = node.create_subscription(
            String, 'vla/action_status', self.action_callback, 10
        )

    def voice_callback(self, msg: String):
        """Handle voice processing output."""
        # Process voice result and potentially trigger planning
        pass

    def planning_callback(self, msg: String):
        """Handle planning system output."""
        # Process planning result and potentially trigger vision or action
        pass

    def vision_callback(self, msg: String):
        """Handle vision system output."""
        # Process vision result and potentially trigger action
        pass

    def action_callback(self, msg: String):
        """Handle action execution status."""
        # Process action status and potentially trigger next steps
        pass
```

### Service-Based Integration

For synchronous operations that require immediate responses:

```python
class VLAServiceIntegration:
    """
    Demonstrates service-based integration for synchronous VLA operations.
    """
    def __init__(self, node: Node):
        self.node = node

        # Service clients for each component
        self.voice_client = node.create_client(VoiceService, 'vla/voice_process')
        self.planning_client = node.create_client(PlanningService, 'vla/plan_actions')
        self.vision_client = node.create_client(VisionService, 'vla/process_vision')
        self.action_client = node.create_client(ActionService, 'vla/execute_action')

        # Service servers for external requests
        self.process_command_srv = node.create_service(
            ProcessVLACommand,
            'vla/process_command',
            self.process_command_callback
        )

    async def process_command_callback(self, request, response):
        """
        Process a complete VLA command synchronously.
        """
        try:
            # Step 1: Process voice
            voice_request = VoiceRequest()
            voice_request.command = request.command
            voice_response = await self.voice_client.call_async(voice_request)

            # Step 2: Plan actions
            planning_request = PlanningRequest()
            planning_request.goal = voice_response.structured_command
            planning_request.context = request.context
            planning_response = await self.planning_client.call_async(planning_request)

            # Step 3: Process vision if needed
            vision_data = {}
            if self.plan_needs_vision(planning_response.plan):
                vision_request = VisionRequest()
                vision_request.targets = self.extract_targets(planning_response.plan)
                vision_response = await self.vision_client.call_async(vision_request)
                vision_data = vision_response.objects

            # Step 4: Execute actions
            action_request = ActionRequest()
            action_request.plan = self.update_plan_with_vision(
                planning_response.plan, vision_data
            )
            action_response = await self.action_client.call_async(action_request)

            response.success = action_response.success
            response.message = action_response.message
            response.execution_log = action_response.log

        except Exception as e:
            response.success = False
            response.message = f"VLA processing failed: {str(e)}"
            response.execution_log = [f"Error: {str(e)}"]

        return response

    def plan_needs_vision(self, plan: Any) -> bool:
        """Check if plan requires vision data."""
        # Implementation depends on your plan structure
        return False

    def extract_targets(self, plan: Any) -> List[str]:
        """Extract vision targets from plan."""
        return []

    def update_plan_with_vision(self, plan: Any, vision_data: Dict) -> Any:
        """Update plan with vision information."""
        return plan
```

## Launch File for Complete VLA System

Create a launch file to start all VLA components together:

```python
# launch/vla_complete_system.launch.py
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
            package='vla_voice_package',
            executable='voice_processor_node',
            name='voice_processor',
            parameters=[
                os.path.join(config_dir, 'voice_config.yaml')
            ],
            output='screen'
        ),

        # Language planning node
        Node(
            package='vla_planning_package',
            executable='language_planner_node',
            name='language_planner',
            parameters=[
                os.path.join(config_dir, 'planning_config.yaml')
            ],
            output='screen'
        ),

        # Vision processing node
        Node(
            package='vla_vision_package',
            executable='vision_processor_node',
            name='vision_processor',
            parameters=[
                os.path.join(config_dir, 'vision_config.yaml')
            ],
            output='screen'
        ),

        # Action execution node
        Node(
            package='vla_action_package',
            executable='action_executor_node',
            name='action_executor',
            parameters=[
                os.path.join(config_dir, 'action_config.yaml')
            ],
            output='screen'
        ),

        # VLA system integrator
        Node(
            package='vla_system',
            executable='vla_system_integrator',
            name='vla_integrator',
            parameters=[
                os.path.join(config_dir, 'vla_config.yaml')
            ],
            output='screen'
        ),

        # Camera driver (for vision system)
        Node(
            package='camera_driver',
            executable='camera_node',
            name='camera_driver',
            parameters=[
                os.path.join(config_dir, 'camera_config.yaml')
            ],
            output='screen'
        )
    ])
```

## Configuration Files

Create comprehensive configuration files for the integrated system:

```yaml
# config/vla_config.yaml
vla_system_integrator:
  ros__parameters:
    # System-wide parameters
    system_timeout: 60.0  # seconds
    max_concurrent_workflows: 5
    enable_monitoring: true
    monitoring_rate: 1.0  # Hz

    # Component health check parameters
    voice_system_timeout: 10.0
    planning_system_timeout: 15.0
    vision_system_timeout: 5.0
    action_system_timeout: 30.0

    # Workflow management
    workflow_cleanup_interval: 30.0  # seconds
    completed_workflow_retention: 100  # number of workflows to retain

    # Error handling
    max_retries_per_component: 3
    retry_delay: 2.0  # seconds
    error_recovery_enabled: true

    # Performance
    enable_profiling: false
    profile_output_file: "/tmp/vla_profile.json"
```

## Integration Testing

Implement comprehensive integration tests:

```python
import unittest
import rclpy
from std_msgs.msg import String
import json
from vla_system_integrator import VLASystemIntegrator

class TestVLAIntegration(unittest.TestCase):
    """
    Integration tests for the complete VLA system.
    """
    def setUp(self):
        rclpy.init()
        self.node = VLASystemIntegrator()
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_complete_vla_workflow(self):
        """
        Test complete VLA workflow from voice to action.
        """
        # Start a VLA workflow
        self.node.process_voice_command("Bring me the red cup", "test_workflow_1")

        # Wait for workflow to complete
        start_time = time.time()
        timeout = 30.0  # seconds

        while time.time() - start_time < timeout:
            if "test_workflow_1" not in self.node.active_workflows:
                # Workflow completed (removed from active workflows)
                break
            time.sleep(0.1)

        # Check if workflow completed successfully
        # This would check the workflow status and results
        self.assertTrue(True)  # Placeholder - actual implementation would check results

    def test_vision_integration(self):
        """
        Test vision system integration.
        """
        # Similar test for vision component integration
        pass

    def test_error_recovery(self):
        """
        Test error recovery mechanisms.
        """
        # Test that system recovers from component failures
        pass

def main():
    unittest.main()

if __name__ == '__main__':
    main()
```

## System Monitoring and Diagnostics

Implement monitoring for the integrated system:

```python
class VLADiagnosticsNode(Node):
    """
    Node for monitoring and diagnostics of the VLA system.
    """
    def __init__(self):
        super().__init__('vla_diagnostics')

        # Publishers for diagnostic information
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.system_status_pub = self.create_publisher(String, 'vla_system_diagnostics', 10)

        # Timer for periodic diagnostics
        self.diag_timer = self.create_timer(1.0, self.publish_diagnostics)

        # Component health trackers
        self.component_health = {
            'voice': {'last_seen': 0, 'status': 'unknown', 'error_count': 0},
            'planning': {'last_seen': 0, 'status': 'unknown', 'error_count': 0},
            'vision': {'last_seen': 0, 'status': 'unknown', 'error_count': 0},
            'action': {'last_seen': 0, 'status': 'unknown', 'error_count': 0}
        }

    def publish_diagnostics(self):
        """
        Publish system diagnostics.
        """
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        # Create diagnostic status for each component
        for comp_name, health_info in self.component_health.items():
            status = DiagnosticStatus()
            status.name = f"VLA_{comp_name}_System"
            status.hardware_id = f"vla_{comp_name}"

            # Determine status level
            if health_info['error_count'] > 5:
                status.level = DiagnosticStatus.ERROR
                status.message = "Component error rate too high"
            elif health_info['error_count'] > 0:
                status.level = DiagnosticStatus.WARN
                status.message = "Component has errors"
            else:
                status.level = DiagnosticStatus.OK
                status.message = "Component operating normally"

            # Add key-value pairs
            status.values.extend([
                KeyValue(key="last_seen", value=str(health_info['last_seen'])),
                KeyValue(key="error_count", value=str(health_info['error_count'])),
                KeyValue(key="status", value=health_info['status'])
            ])

            diag_array.status.append(status)

        self.diag_pub.publish(diag_array)

        # Also publish as JSON for easy consumption
        diag_msg = String()
        diag_msg.data = json.dumps({
            'timestamp': self.get_clock().now().nanoseconds,
            'component_health': self.component_health,
            'system_overall_status': self.get_system_overall_status()
        })
        self.system_status_pub.publish(diag_msg)

    def get_system_overall_status(self) -> str:
        """
        Get overall system status based on component health.
        """
        statuses = [info['status'] for info in self.component_health.values()]

        if 'error' in statuses:
            return 'error'
        elif 'warn' in statuses:
            return 'degraded'
        else:
            return 'ok'
```

## Best Practices for Integration

1. **Modular Design**: Keep components loosely coupled and easily replaceable
2. **Error Handling**: Implement comprehensive error handling and recovery
3. **Performance Monitoring**: Monitor system performance and identify bottlenecks
4. **Configuration Management**: Use configuration files for system parameters
5. **Testing**: Implement unit, integration, and system-level tests
6. **Documentation**: Maintain clear documentation for all interfaces
7. **Logging**: Implement comprehensive logging for debugging and monitoring

## Next Steps

After understanding complete system integration, the next step is to learn about the complete voice-to-manipulation pipeline. Continue with the [Voice Command to Manipulation Guide](./voice-to-manipulation.md) to understand how voice commands are transformed into physical robot actions.
---
sidebar_position: 3
title: 'Generating Executable Action Sequences'
---

# Generating Executable Action Sequences

This guide covers how to generate executable action sequences from LLM-planned actions. You'll learn how to convert high-level action plans into specific ROS 2 messages and services that can be executed by the robot.

## Overview

Once an LLM has planned a sequence of actions, these need to be converted into executable robot commands. This involves:

1. Converting high-level actions to specific ROS 2 message types
2. Sequencing actions with proper timing and coordination
3. Handling dependencies between actions
4. Managing error recovery and fallback behaviors

## Action Sequence Executor

Here's a complete implementation of an action sequence executor:

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import json
import time
import asyncio
from typing import List, Dict, Any, Optional
from enum import Enum

class ActionStatus(Enum):
    PENDING = "pending"
    EXECUTING = "executing"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELLED = "cancelled"

class RobotAction:
    def __init__(self, action_type: str, parameters: Dict[str, Any], description: str = ""):
        self.action_type = action_type
        self.parameters = parameters
        self.description = description
        self.status = ActionStatus.PENDING

class ActionSequenceExecutor(Node):
    def __init__(self):
        super().__init__('action_sequence_executor')

        # Publishers for different action types
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speech_pub = self.create_publisher(String, '/speech_output', 10)

        # Action clients for navigation
        self.nav_client = ActionClient(self, MoveBaseAction, 'move_base')

        # Subscriber for action sequence requests
        self.sequence_sub = self.create_subscription(
            String,
            'action_sequence',
            self.sequence_callback,
            10
        )

        # Publisher for sequence status
        self.status_pub = self.create_publisher(String, 'action_sequence_status', 10)

        self.get_logger().info('Action Sequence Executor initialized')

    def sequence_callback(self, msg):
        """
        Process incoming action sequence
        """
        try:
            sequence_data = json.loads(msg.data)
            actions = sequence_data.get('actions', [])

            self.get_logger().info(f'Received sequence with {len(actions)} actions')

            # Execute the sequence
            asyncio.create_task(self.execute_sequence(actions))

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in action sequence')
        except Exception as e:
            self.get_logger().error(f'Error processing action sequence: {e}')

    async def execute_sequence(self, actions: List[Dict[str, Any]]):
        """
        Execute a sequence of actions asynchronously
        """
        for i, action_data in enumerate(actions):
            action = RobotAction(
                action_type=action_data['action_type'],
                parameters=action_data['parameters'],
                description=action_data.get('description', f'Action {i+1}')
            )

            self.get_logger().info(f'Executing: {action.description}')

            # Update status
            status_msg = String()
            status_msg.data = json.dumps({
                'action_index': i,
                'action_type': action.action_type,
                'status': ActionStatus.EXECUTING.value,
                'description': action.description
            })
            self.status_pub.publish(status_msg)

            try:
                success = await self.execute_single_action(action)

                if success:
                    action.status = ActionStatus.SUCCEEDED
                    self.get_logger().info(f'Successfully executed: {action.description}')
                else:
                    action.status = ActionStatus.FAILED
                    self.get_logger().error(f'Failed to execute: {action.description}')
                    # Optionally stop execution on failure
                    break

            except Exception as e:
                action.status = ActionStatus.FAILED
                self.get_logger().error(f'Error executing action: {e}')
                break

        # Publish final status
        final_status = String()
        final_status.data = json.dumps({
            'sequence_complete': True,
            'status': 'completed'
        })
        self.status_pub.publish(final_status)

    async def execute_single_action(self, action: RobotAction) -> bool:
        """
        Execute a single action based on its type
        """
        if action.action_type == 'NAVIGATE':
            return await self.execute_navigation_action(action)
        elif action.action_type == 'PICK_UP':
            return await self.execute_pickup_action(action)
        elif action.action_type == 'PLACE':
            return await self.execute_place_action(action)
        elif action.action_type == 'SPEAK':
            return await self.execute_speak_action(action)
        elif action.action_type == 'DETECT':
            return await self.execute_detect_action(action)
        elif action.action_type == 'MOVE_ARM':
            return await self.execute_arm_action(action)
        else:
            self.get_logger().error(f'Unknown action type: {action.action_type}')
            return False

    async def execute_navigation_action(self, action: RobotAction) -> bool:
        """
        Execute navigation action
        """
        target_location = action.parameters.get('location')

        if not target_location:
            self.get_logger().error('Navigation action missing location parameter')
            return False

        # In a real implementation, you would look up the coordinates for the location
        # For this example, we'll use dummy coordinates
        if target_location.lower() == 'kitchen':
            x, y, theta = 2.0, 1.0, 0.0
        elif target_location.lower() == 'living_room':
            x, y, theta = 0.0, 0.0, 0.0
        elif target_location.lower() == 'bedroom':
            x, y, theta = -1.0, 2.0, 1.57
        else:
            self.get_logger().error(f'Unknown location: {target_location}')
            return False

        # Create navigation goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = theta

        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return False

        # Send goal
        future = self.nav_client.send_goal_async(goal)
        result_future = await future

        if result_future is not None:
            # Wait for result
            result = await result_future.get_result_async()
            return result.result.status == GoalStatus.SUCCEEDED

        return False

    async def execute_pickup_action(self, action: RobotAction) -> bool:
        """
        Execute pickup action (simulated)
        """
        object_name = action.parameters.get('object')
        location = action.parameters.get('location', 'default')

        self.get_logger().info(f'Attempting to pick up {object_name} from {location}')

        # Simulate pickup action
        # In a real implementation, this would involve:
        # 1. Moving arm to object location
        # 2. Grasping the object
        # 3. Verifying grasp success
        await asyncio.sleep(2.0)  # Simulate action time

        # For simulation purposes, assume success
        return True

    async def execute_place_action(self, action: RobotAction) -> bool:
        """
        Execute place action (simulated)
        """
        location = action.parameters.get('location', 'default')

        self.get_logger().info(f'Attempting to place object at {location}')

        # Simulate place action
        await asyncio.sleep(2.0)  # Simulate action time

        # For simulation purposes, assume success
        return True

    async def execute_speak_action(self, action: RobotAction) -> bool:
        """
        Execute speech action
        """
        text = action.parameters.get('text', 'Hello')

        self.get_logger().info(f'Speaking: {text}')

        # Publish speech command
        speech_msg = String()
        speech_msg.data = text
        self.speech_pub.publish(speech_msg)

        return True

    async def execute_detect_action(self, action: RobotAction) -> bool:
        """
        Execute object detection action (simulated)
        """
        object_name = action.parameters.get('object')

        self.get_logger().info(f'Detecting object: {object_name}')

        # Simulate detection
        await asyncio.sleep(1.0)  # Simulate detection time

        # For simulation, assume object was detected
        return True

    async def execute_arm_action(self, action: RobotAction) -> bool:
        """
        Execute arm movement action (simulated)
        """
        position = action.parameters.get('position', 'default')

        self.get_logger().info(f'Moving arm to position: {position}')

        # Simulate arm movement
        await asyncio.sleep(1.0)  # Simulate movement time

        # For simulation, assume success
        return True

def main(args=None):
    rclpy.init(args=args)
    node = ActionSequenceExecutor()

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

## Action Sequence Planning and Validation

Before executing action sequences, it's important to validate them:

```python
class ActionSequenceValidator:
    def __init__(self):
        # Define valid action types and their required parameters
        self.action_requirements = {
            'NAVIGATE': {'required': ['location']},
            'PICK_UP': {'required': ['object'], 'optional': ['location']},
            'PLACE': {'required': [], 'optional': ['location', 'object']},
            'SPEAK': {'required': ['text']},
            'DETECT': {'required': ['object']},
            'MOVE_ARM': {'required': ['position']}
        }

    def validate_sequence(self, actions: List[Dict[str, Any]]) -> tuple[bool, List[str]]:
        """
        Validate an action sequence for completeness and feasibility.
        """
        errors = []

        for i, action in enumerate(actions):
            action_type = action.get('action_type')
            parameters = action.get('parameters', {})

            if not action_type:
                errors.append(f"Action {i+1}: Missing action_type")
                continue

            if action_type not in self.action_requirements:
                errors.append(f"Action {i+1}: Unknown action type '{action_type}'")
                continue

            requirements = self.action_requirements[action_type]

            # Check required parameters
            for required_param in requirements['required']:
                if required_param not in parameters:
                    errors.append(f"Action {i+1}: Missing required parameter '{required_param}' for action '{action_type}'")

            # Check for unknown parameters
            allowed_params = set(requirements['required'] + requirements.get('optional', []))
            for param in parameters:
                if param not in allowed_params:
                    errors.append(f"Action {i+1}: Unknown parameter '{param}' for action '{action_type}'")

        return len(errors) == 0, errors

    def check_dependencies(self, actions: List[Dict[str, Any]]) -> List[str]:
        """
        Check for logical dependencies between actions.
        """
        errors = []

        for i in range(1, len(actions)):
            prev_action = actions[i-1]
            curr_action = actions[i]

            # Check if pickup action has a preceding navigation to object location
            if (curr_action.get('action_type') == 'PICK_UP' and
                prev_action.get('action_type') != 'NAVIGATE'):
                errors.append(f"Action {i+1}: Pickup action should be preceded by navigation")

            # Check if place action has a preceding pickup action
            if (curr_action.get('action_type') == 'PLACE' and
                not any(a.get('action_type') == 'PICK_UP' for a in actions[:i])):
                errors.append(f"Action {i+1}: Place action requires a preceding pickup action")

        return errors
```

## Conditional Action Execution

Sometimes actions need to be executed conditionally based on sensor feedback:

```python
class ConditionalActionExecutor(ActionSequenceExecutor):
    def __init__(self):
        super().__init__()

        # Subscriber for sensor feedback
        self.feedback_sub = self.create_subscription(
            String,
            'sensor_feedback',
            self.feedback_callback,
            10
        )

        self.sensor_feedback = {}
        self.waiting_for_feedback = False

    def feedback_callback(self, msg):
        """
        Handle sensor feedback
        """
        try:
            feedback_data = json.loads(msg.data)
            self.sensor_feedback.update(feedback_data)
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in sensor feedback')

    async def execute_conditional_action(self, action: RobotAction) -> bool:
        """
        Execute action with conditional logic based on sensor feedback
        """
        conditions = action.parameters.get('conditions', [])

        # Check conditions before executing
        for condition in conditions:
            if not self._evaluate_condition(condition):
                self.get_logger().info(f'Condition not met: {condition}')
                return False

        # Execute the action
        return await self.execute_single_action(action)

    def _evaluate_condition(self, condition: Dict[str, Any]) -> bool:
        """
        Evaluate a condition based on sensor feedback
        """
        sensor_name = condition.get('sensor')
        expected_value = condition.get('expected_value')
        comparison = condition.get('comparison', 'equals')  # equals, greater_than, less_than, etc.

        actual_value = self.sensor_feedback.get(sensor_name)

        if actual_value is None:
            return False

        if comparison == 'equals':
            return actual_value == expected_value
        elif comparison == 'greater_than':
            return actual_value > expected_value
        elif comparison == 'less_than':
            return actual_value < expected_value
        elif comparison == 'contains':
            return expected_value in actual_value

        return False
```

## Error Recovery and Fallback Strategies

Implement error recovery mechanisms for failed actions:

```python
class RobustActionExecutor(ConditionalActionExecutor):
    def __init__(self):
        super().__init__()
        self.max_retries = 3
        self.retry_delay = 2.0  # seconds

    async def execute_single_action_with_retry(self, action: RobotAction) -> bool:
        """
        Execute action with retry mechanism
        """
        for attempt in range(self.max_retries):
            try:
                success = await self.execute_single_action(action)

                if success:
                    return True

                self.get_logger().warn(f'Action failed on attempt {attempt + 1}, retrying...')

                # Add delay before retry
                time.sleep(self.retry_delay)

            except Exception as e:
                self.get_logger().error(f'Error on attempt {attempt + 1}: {e}')

                if attempt == self.max_retries - 1:
                    # Last attempt failed
                    return False

        return False

    def generate_fallback_actions(self, failed_action: RobotAction) -> List[RobotAction]:
        """
        Generate fallback actions when primary action fails
        """
        fallbacks = []

        if failed_action.action_type == 'NAVIGATE':
            # Try alternative path or ask for human assistance
            fallbacks.append(RobotAction(
                action_type='SPEAK',
                parameters={'text': 'I cannot navigate to that location. Can you help me?'},
                description='Request human assistance for navigation'
            ))

        elif failed_action.action_type == 'PICK_UP':
            # Try different grasp or alternative object
            fallbacks.append(RobotAction(
                action_type='SPEAK',
                parameters={'text': 'I cannot pick up that object. Trying alternative...'},
                description='Inform about pickup failure'
            ))

        return fallbacks
```

## Action Sequence Optimization

Optimize action sequences for efficiency:

```python
class OptimizedActionSequenceExecutor(RobustActionExecutor):
    def optimize_sequence(self, actions: List[RobotAction]) -> List[RobotAction]:
        """
        Optimize action sequence for efficiency
        """
        optimized = []

        i = 0
        while i < len(actions):
            current = actions[i]

            # Merge consecutive navigation actions to same location
            if (current.action_type == 'NAVIGATE' and
                i + 1 < len(actions) and
                actions[i + 1].action_type == 'NAVIGATE' and
                current.parameters.get('location') == actions[i + 1].parameters.get('location')):

                # Skip the duplicate navigation
                i += 2
                continue

            # Combine speak actions
            if (current.action_type == 'SPEAK' and
                i + 1 < len(actions) and
                actions[i + 1].action_type == 'SPEAK'):

                combined_text = f"{current.parameters.get('text', '')} {actions[i + 1].parameters.get('text', '')}"
                optimized.append(RobotAction(
                    action_type='SPEAK',
                    parameters={'text': combined_text.strip()},
                    description=f"Combined speak: {combined_text[:50]}..."
                ))
                i += 2
                continue

            optimized.append(current)
            i += 1

        return optimized

    async def execute_optimized_sequence(self, actions: List[RobotAction]):
        """
        Execute optimized action sequence
        """
        optimized_actions = self.optimize_sequence(actions)

        self.get_logger().info(f'Optimized sequence from {len(actions)} to {len(optimized_actions)} actions')

        return await self.execute_sequence([{
            'action_type': a.action_type,
            'parameters': a.parameters,
            'description': a.description
        } for a in optimized_actions])
```

## Best Practices

- **Modularity**: Keep action execution modular and reusable
- **Error Handling**: Implement comprehensive error handling and recovery
- **Validation**: Always validate action sequences before execution
- **Monitoring**: Provide real-time feedback on action execution status
- **Safety**: Implement safety checks to prevent harmful actions
- **Optimization**: Optimize sequences for efficiency and resource usage

## Next Steps

After learning how to generate executable action sequences, the next step is to implement high-level task planning for robots. Continue with the [High-Level Task Planning for Robots](./task-planning.md) guide.
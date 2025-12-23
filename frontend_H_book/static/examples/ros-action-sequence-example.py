"""
Sample ROS Action Sequence Definitions

This file demonstrates how action sequences can be defined and executed
using ROS 2 action interfaces.
"""

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from move_base_msgs.action import MoveBase
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
import json
import time
from typing import List, Dict, Any

class ActionSequenceDefinition:
    """
    Defines an action sequence for robot execution.
    """
    def __init__(self):
        self.id = ""
        self.name = ""
        self.description = ""
        self.actions = []
        self.created_at = time.time()

    def add_action(self, action_type: str, parameters: Dict[str, Any],
                   description: str = ""):
        """
        Add an action to the sequence.
        """
        action = {
            'type': action_type,
            'parameters': parameters,
            'description': description,
            'timeout': parameters.get('timeout', 30.0)  # Default 30 second timeout
        }
        self.actions.append(action)

    def to_ros_message(self):
        """
        Convert to a format suitable for ROS message transmission.
        """
        return String(data=json.dumps({
            'id': self.id,
            'name': self.name,
            'description': self.description,
            'actions': self.actions,
            'created_at': self.created_at
        }))

    def from_ros_message(self, msg: String):
        """
        Create from ROS message.
        """
        data = json.loads(msg.data)
        self.id = data['id']
        self.name = data['name']
        self.description = data['description']
        self.actions = data['actions']
        self.created_at = data['created_at']


class ActionSequenceExecutorNode(Node):
    """
    ROS Node that executes action sequences.
    """
    def __init__(self):
        super().__init__('action_sequence_executor')

        # Action clients for different robot capabilities
        self.move_base_client = ActionClient(self, MoveBase, 'move_base')
        self.arm_control_client = ActionClient(self, FollowJointTrajectory, 'arm_controller/follow_joint_trajectory')

        # Publishers and subscribers
        self.sequence_sub = self.create_subscription(
            String,
            'action_sequence',
            self.sequence_callback,
            10
        )

        self.status_pub = self.create_publisher(String, 'action_sequence_status', 10)
        self.feedback_pub = self.create_publisher(String, 'action_sequence_feedback', 10)

        # Service servers for sequence management
        self.execute_srv = self.create_service(
            ExecuteSequence,
            'execute_sequence',
            self.execute_sequence_service
        )

        self.get_logger().info('Action Sequence Executor initialized')

    def sequence_callback(self, msg: String):
        """
        Handle incoming action sequence.
        """
        try:
            sequence_def = ActionSequenceDefinition()
            sequence_def.from_ros_message(msg)

            self.get_logger().info(f'Received sequence: {sequence_def.name}')

            # Execute the sequence asynchronously
            self.execute_sequence_async(sequence_def)

        except Exception as e:
            self.get_logger().error(f'Error processing sequence: {e}')

    async def execute_sequence_async(self, sequence: ActionSequenceDefinition):
        """
        Execute an action sequence asynchronously.
        """
        self.get_logger().info(f'Starting execution of sequence: {sequence.name}')

        for i, action in enumerate(sequence.actions):
            self.get_logger().info(f'Executing action {i+1}/{len(sequence.actions)}: {action["description"]}')

            # Update status
            status_msg = String()
            status_msg.data = json.dumps({
                'sequence_id': sequence.id,
                'action_index': i,
                'action_type': action['type'],
                'status': 'executing',
                'timestamp': time.time()
            })
            self.status_pub.publish(status_msg)

            success = await self.execute_single_action(action)

            if not success:
                self.get_logger().error(f'Action {i+1} failed: {action["description"]}')

                # Publish failure status
                status_msg.data = json.dumps({
                    'sequence_id': sequence.id,
                    'action_index': i,
                    'action_type': action['type'],
                    'status': 'failed',
                    'timestamp': time.time(),
                    'error': f'Action failed: {action["description"]}'
                })
                self.status_pub.publish(status_msg)
                return False

        # Sequence completed successfully
        status_msg = String()
        status_msg.data = json.dumps({
            'sequence_id': sequence.id,
            'status': 'completed',
            'timestamp': time.time(),
            'actions_completed': len(sequence.actions)
        })
        self.status_pub.publish(status_msg)

        self.get_logger().info(f'Sequence completed: {sequence.name}')
        return True

    async def execute_single_action(self, action: Dict[str, Any]) -> bool:
        """
        Execute a single action based on its type.
        """
        action_type = action['type']
        parameters = action['parameters']

        if action_type == 'NAVIGATE_TO_POSE':
            return await self.execute_navigate_action(parameters)
        elif action_type == 'MOVE_ARM':
            return await self.execute_arm_action(parameters)
        elif action_type == 'GRASP_OBJECT':
            return await self.execute_grasp_action(parameters)
        elif action_type == 'SPEAK_TEXT':
            return await self.execute_speak_action(parameters)
        elif action_type == 'WAIT':
            return await self.execute_wait_action(parameters)
        else:
            self.get_logger().error(f'Unknown action type: {action_type}')
            return False

    async def execute_navigate_action(self, params: Dict[str, Any]) -> bool:
        """
        Execute navigation action.
        """
        try:
            goal_msg = MoveBase.Goal()
            goal_msg.target_pose.header.frame_id = params.get('frame_id', 'map')
            goal_msg.target_pose.header.stamp = self.get_clock().now().to_msg()

            # Set target pose from parameters
            goal_msg.target_pose.pose.position.x = params.get('x', 0.0)
            goal_msg.target_pose.pose.position.y = params.get('y', 0.0)
            goal_msg.target_pose.pose.position.z = params.get('z', 0.0)

            goal_msg.target_pose.pose.orientation.x = params.get('qx', 0.0)
            goal_msg.target_pose.pose.orientation.y = params.get('qy', 0.0)
            goal_msg.target_pose.pose.orientation.z = params.get('qz', 0.0)
            goal_msg.target_pose.pose.orientation.w = params.get('qw', 1.0)

            # Wait for action server
            if not self.move_base_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('Move base action server not available')
                return False

            # Send goal
            goal_future = self.move_base_client.send_goal_async(goal_msg)
            goal_handle = await goal_future

            if not goal_handle.accepted:
                self.get_logger().error('Navigation goal rejected')
                return False

            # Wait for result
            result_future = goal_handle.get_result_async()
            result = await result_future

            return result.result.status == 3  # SUCCEEDED status

        except Exception as e:
            self.get_logger().error(f'Navigation action failed: {e}')
            return False

    async def execute_arm_action(self, params: Dict[str, Any]) -> bool:
        """
        Execute arm movement action.
        """
        try:
            goal_msg = FollowJointTrajectory.Goal()

            # Set joint trajectory from parameters
            trajectory = JointTrajectory()
            trajectory.joint_names = params.get('joint_names', [])

            point = JointTrajectoryPoint()
            point.positions = params.get('positions', [])
            point.velocities = params.get('velocities', [0.0] * len(point.positions))
            point.accelerations = params.get('accelerations', [0.0] * len(point.positions))
            point.time_from_start.sec = params.get('duration_sec', 5)

            trajectory.points.append(point)
            goal_msg.trajectory = trajectory

            # Wait for action server
            if not self.arm_control_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('Arm control action server not available')
                return False

            # Send goal
            goal_future = self.arm_control_client.send_goal_async(goal_msg)
            goal_handle = await goal_future

            if not goal_handle.accepted:
                self.get_logger().error('Arm movement goal rejected')
                return False

            # Wait for result
            result_future = goal_handle.get_result_async()
            result = await result_future

            return result.result.error_code == 0  # SUCCESS

        except Exception as e:
            self.get_logger().error(f'Arm action failed: {e}')
            return False

    async def execute_grasp_action(self, params: Dict[str, Any]) -> bool:
        """
        Execute grasping action (simulated).
        """
        # This would interface with a grasping action server in a real implementation
        # For simulation, just wait for the specified time
        duration = params.get('duration', 2.0)
        time.sleep(duration)
        return True

    async def execute_speak_action(self, params: Dict[str, Any]) -> bool:
        """
        Execute speech action.
        """
        text = params.get('text', 'Hello')

        # Publish to speech synthesis topic
        speech_pub = self.create_publisher(String, '/tts_input', 10)
        speech_msg = String()
        speech_msg.data = text
        speech_pub.publish(speech_msg)

        return True

    async def execute_wait_action(self, params: Dict[str, Any]) -> bool:
        """
        Execute wait action.
        """
        duration = params.get('duration', 1.0)
        time.sleep(duration)
        return True


def create_coffee_service_sequence():
    """
    Example: Create a sequence for coffee service task.
    """
    sequence = ActionSequenceDefinition()
    sequence.id = "coffee_service_001"
    sequence.name = "Coffee Service Task"
    sequence.description = "Navigate to kitchen, prepare coffee, and serve to user"

    # Navigate to kitchen
    sequence.add_action(
        'NAVIGATE_TO_POSE',
        {
            'x': 2.0, 'y': 1.0, 'z': 0.0,
            'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0,
            'frame_id': 'map'
        },
        "Navigate to kitchen"
    )

    # Speak while navigating
    sequence.add_action(
        'SPEAK_TEXT',
        {'text': 'Heading to the kitchen to prepare your coffee.'},
        "Announce navigation"
    )

    # Wait for coffee preparation (simulated)
    sequence.add_action(
        'WAIT',
        {'duration': 5.0},
        "Wait for coffee preparation"
    )

    # Navigate back to user
    sequence.add_action(
        'NAVIGATE_TO_POSE',
        {
            'x': 0.0, 'y': 0.0, 'z': 0.0,
            'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0,
            'frame_id': 'map'
        },
        "Navigate back to user"
    )

    # Serve coffee
    sequence.add_action(
        'SPEAK_TEXT',
        {'text': 'Here is your coffee. Enjoy!'},
        "Serve coffee"
    )

    return sequence


# Example usage
if __name__ == '__main__':
    # Example of creating and using an action sequence
    coffee_sequence = create_coffee_service_sequence()

    print("Coffee Service Sequence:")
    print(f"ID: {coffee_sequence.id}")
    print(f"Name: {coffee_sequence.name}")
    print(f"Description: {coffee_sequence.description}")
    print(f"Number of actions: {len(coffee_sequence.actions)}")

    for i, action in enumerate(coffee_sequence.actions):
        print(f"  {i+1}. {action['description']} ({action['type']})")
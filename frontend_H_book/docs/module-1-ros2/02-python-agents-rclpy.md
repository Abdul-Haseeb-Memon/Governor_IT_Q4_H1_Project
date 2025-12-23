---
sidebar_position: 2
description: Creating Python-based ROS 2 nodes using rclpy to bridge AI decisions to robot controllers
---

# Python Agents with rclpy

## Learning Objectives

By the end of this chapter, you will be able to:
- Create Python-based ROS 2 nodes using the rclpy library
- Implement publishers and subscribers to bridge AI decisions to robot controllers
- Understand the conceptual communication flow between AI systems and robot control mechanisms
- Create service clients and servers for request-response communication patterns
- Integrate Python AI systems with ROS 2 robot controllers

## Prerequisites

- Understanding of ROS 2 fundamentals (covered in Chapter 1)
- Basic knowledge of Python programming
- Familiarity with object-oriented programming concepts

## Introduction to rclpy

rclpy is the Python client library for ROS 2. It provides Python bindings for the ROS 2 client library (rcl), allowing you to create ROS 2 nodes in Python. rclpy is part of the ROS 2 ecosystem and provides the same functionality as the C++ client library (rclcpp) but with Python's ease of use and rapid prototyping capabilities.

### Why Python for AI-Robot Integration?

Python is the dominant language in AI and machine learning, making it an ideal choice for bridging AI decisions to robot controllers:
- Rich ecosystem of AI/ML libraries (TensorFlow, PyTorch, scikit-learn)
- Easy to prototype and test algorithms
- Large community and extensive documentation
- Integration with ROS 2 through rclpy

## Creating Your First rclpy Node

Let's start by creating a simple rclpy node that serves as an AI decision bridge:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class AIDecisionBridge(Node):
    def __init__(self):
        super().__init__('ai_decision_bridge')

        # Create publisher for robot commands
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create subscriber for sensor data
        self.sensor_subscription = self.create_subscription(
            String,
            'sensor_data',
            self.sensor_callback,
            10
        )

        # Timer for AI decision making
        self.timer = self.create_timer(0.1, self.ai_decision_callback)

        self.get_logger().info('AI Decision Bridge node initialized')

    def sensor_callback(self, msg):
        self.get_logger().info(f'Received sensor data: {msg.data}')
        # Process sensor data and update internal state
        self.process_sensor_data(msg.data)

    def ai_decision_callback(self):
        # This is where AI logic would run
        # For now, we'll implement a simple decision
        cmd = self.make_decision()
        self.cmd_vel_publisher.publish(cmd)

    def process_sensor_data(self, data):
        # Process incoming sensor data
        # This could involve complex AI algorithms
        pass

    def make_decision(self):
        # Make a decision based on current state
        # Return a Twist message with velocity commands
        cmd = Twist()
        cmd.linear.x = 0.5  # Move forward at 0.5 m/s
        cmd.angular.z = 0.0  # No rotation
        return cmd

def main(args=None):
    rclpy.init(args=args)
    ai_bridge = AIDecisionBridge()

    try:
        rclpy.spin(ai_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        ai_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Publishers and Subscribers in Depth

### Publishers

Publishers are used to send messages to topics. In the context of AI-robot integration, publishers are often used to:
- Send AI-generated commands to robot controllers
- Broadcast AI system status
- Publish processed sensor data

### Subscribers

Subscribers receive messages from topics. For AI-robot integration, subscribers are typically used to:
- Receive sensor data from the robot
- Get feedback from control systems
- Listen to messages from other nodes in the system

### Example: Advanced Publisher-Subscriber Pattern

Here's a more complex example that demonstrates how AI systems can use publishers and subscribers to interact with robot controllers:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np

class AdvancedAIBridge(Node):
    def __init__(self):
        super().__init__('advanced_ai_bridge')

        # Publishers
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_publisher = self.create_publisher(Bool, '/ai_status', 10)

        # Subscribers
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        # Timer for AI processing
        self.ai_timer = self.create_timer(0.05, self.ai_processing_callback)

        # State variables
        self.obstacle_detected = False
        self.last_command_time = self.get_clock().now()

        self.get_logger().info('Advanced AI Bridge initialized')

    def laser_callback(self, msg):
        # Process laser scan data to detect obstacles
        min_distance = min([d for d in msg.ranges if not np.isnan(d)])

        if min_distance < 1.0:  # If obstacle within 1 meter
            self.obstacle_detected = True
            self.get_logger().warn('Obstacle detected! Stopping robot.')
        else:
            self.obstacle_detected = False

    def ai_processing_callback(self):
        # AI decision making based on sensor data
        cmd = Twist()

        if self.obstacle_detected:
            # Stop the robot if obstacle detected
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            # Move forward with slight random turning
            cmd.linear.x = 0.3
            cmd.angular.z = np.random.uniform(-0.2, 0.2)

        # Publish command
        self.cmd_publisher.publish(cmd)

        # Publish status
        status_msg = Bool()
        status_msg.data = True  # AI is active
        self.status_publisher.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    ai_bridge = AdvancedAIBridge()

    try:
        rclpy.spin(ai_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        ai_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Services for Request-Response Communication

Services provide a way for nodes to make requests and receive responses. This is useful for AI systems that need to query specific information from robot systems or request specific actions.

### Example: Service Client for Path Planning

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point
from nav_msgs.srv import GetPlan

class PathPlannerClient(Node):
    def __init__(self):
        super().__init__('path_planner_client')
        self.client = self.create_client(GetPlan, 'plan_path')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for path planning service...')

    def plan_path(self, start, goal):
        request = GetPlan.Request()
        request.start = start
        request.goal = goal

        future = self.client.call_async(request)
        return future

def main(args=None):
    rclpy.init(args=args)
    client = PathPlannerClient()

    # Example usage
    start_point = Point(x=0.0, y=0.0, z=0.0)
    goal_point = Point(x=5.0, y=5.0, z=0.0)

    future = client.plan_path(start_point, goal_point)

    rclpy.spin_until_future_complete(client, future)

    if future.result() is not None:
        plan = future.result().plan
        client.get_logger().info(f'Path received with {len(plan.poses)} poses')
    else:
        client.get_logger().error('Failed to get path')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Conceptual Communication Flow

The communication flow between AI systems and robot controllers typically follows this pattern:

### 1. Perception Phase
- Robot sensors publish data to topics
- AI system subscribes to sensor data
- AI processes sensor information to understand environment

### 2. Decision Phase
- AI system makes decisions based on processed information
- Decisions are formatted as ROS 2 messages
- AI system publishes commands to robot controllers

### 3. Action Phase
- Robot controllers receive commands
- Commands are executed by robot hardware
- Feedback is published back to the AI system

### 4. Monitoring Phase
- AI system monitors robot state and environment
- Adjustments are made based on feedback
- Loop continues for continuous operation

## Practical Example: AI-Driven Navigation

Let's implement a complete example that demonstrates how an AI system can control a robot's navigation:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
import numpy as np
import math

class AINavigationNode(Node):
    def __init__(self):
        super().__init__('ai_navigation_node')

        # Publishers
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Subscribers
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.status_subscriber = self.create_subscription(
            String,
            '/robot_status',
            self.status_callback,
            10
        )

        # AI processing timer
        self.ai_timer = self.create_timer(0.1, self.ai_navigation_callback)

        # Robot state
        self.scan_data = None
        self.robot_status = 'idle'
        self.goal_reached = False

        self.get_logger().info('AI Navigation System initialized')

    def scan_callback(self, msg):
        self.scan_data = msg.ranges

    def status_callback(self, msg):
        self.robot_status = msg.data

    def ai_navigation_callback(self):
        if self.scan_data is None:
            return

        cmd = Twist()

        # Simple navigation algorithm
        # Check for obstacles in front
        front_ranges = self.scan_data[330:30] + self.scan_data[330:360]  # wrap around
        min_front_dist = min([d for d in front_ranges if not math.isnan(d)])

        if min_front_dist < 0.8:  # Obstacle too close
            # Stop and rotate to find clear path
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Rotate right
        else:
            # Move forward toward goal
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0

        self.cmd_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    nav_node = AINavigationNode()

    try:
        rclpy.spin(nav_node)
    except KeyboardInterrupt:
        pass
    finally:
        nav_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercises

### Exercise 1: AI Decision Bridge
Create an rclpy node that subscribes to a topic called `/ai_commands` where AI system sends high-level commands (e.g., "move_forward", "turn_left", "stop"), and publishes appropriate Twist messages to control a robot's movement.

### Exercise 2: Sensor Data Processing
Create an rclpy node that subscribes to multiple sensor topics (LIDAR, camera, IMU), processes the data to detect specific conditions (e.g., obstacles, landmarks), and publishes the processed information to a new topic for the AI system to use.

### Exercise 3: Service Integration
Implement a service server that accepts goals from an AI system and returns whether the goal is achievable based on current robot state and environment data.

## Summary

Python agents using rclpy provide an effective bridge between AI decision-making systems and robot controllers. By understanding publishers, subscribers, and services, you can create sophisticated communication patterns that allow AI systems to effectively control humanoid robots.

In the next chapter, we'll explore how to describe humanoid robots using URDF (Unified Robot Description Format) and how these descriptions integrate with ROS 2 simulators.
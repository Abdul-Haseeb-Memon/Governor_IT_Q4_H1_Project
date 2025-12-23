---
sidebar_position: 1
description: Introduction to ROS 2 as the robotic middleware connecting AI logic to humanoid robot control
---

# ROS 2 Fundamentals

## Learning Objectives

By the end of this chapter, you will be able to:
- Explain the purpose of ROS 2 as robotic middleware
- Identify and describe the roles of nodes, topics, and services in a ROS 2 system
- Understand the high-level humanoid data flow from AI decisions to robot controllers
- Trace how data flows between AI decisions and robot control in a humanoid system

## Prerequisites

- Basic knowledge of Python programming
- Understanding of fundamental computer science concepts
- Familiarity with command-line interfaces

## What is ROS 2?

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

ROS 2 is not an operating system in the traditional sense, but rather a middleware that provides services designed for a heterogeneous computer cluster. It includes hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.

### Key Characteristics of ROS 2

- **Middleware**: ROS 2 acts as the communication layer between different parts of a robot system
- **Distributed**: Components can run on different machines and communicate over a network
- **Modular**: Systems can be built by connecting different components
- **Open Source**: Free and open-source software with a large community

## The Robotic Nervous System

Think of ROS 2 as the nervous system of a robot. Just as the nervous system in biological organisms connects the brain to muscles and sensors, ROS 2 connects AI decision-making systems to robot controllers and sensors.

In humanoid robots, this connection is especially critical because:
- AI systems make high-level decisions
- These decisions must be translated into specific motor commands
- Sensor data must flow back to inform the AI
- All of this must happen reliably and in real-time

## Nodes: The Building Blocks

Nodes are the fundamental building blocks of a ROS 2 system. A node is a process that performs computation. In a typical robot system, there might be one node that controls the robot's wheels, another that processes camera images, and another that listens to user commands.

### Characteristics of Nodes

- **Independent**: Each node runs independently of other nodes
- **Specialized**: Each node typically performs a specific function
- **Communicative**: Nodes communicate with each other through topics, services, and actions

### Creating a Node

In ROS 2, nodes are typically implemented in one of several supported languages, with Python and C++ being the most common. Each node must be initialized with a unique name and can then create publishers, subscribers, services, and clients.

## Topics and Messages: Data Streams

Topics are named buses over which nodes exchange messages. The communication is loosely coupled: publishers and subscribers are not aware of each other. A node can publish to multiple topics, and a topic can have multiple subscribers.

### Message Types

Messages are the data structures that are passed between nodes. ROS 2 provides many standard message types, and users can define custom message types as well. Messages are defined in .msg files and contain fields with specific data types.

### Publisher-Subscriber Pattern

The publisher-subscriber pattern is a communication pattern where:
- Publishers send data to a topic without knowing who will receive it
- Subscribers receive data from a topic without knowing who sent it
- The ROS 2 middleware handles the routing of messages

## Services: Request-Response Communication

Services provide a request-response communication pattern. A service client sends a request to a service server, which processes the request and sends back a response. This is different from the publisher-subscriber pattern, which is asynchronous and one-way.

### Service Types

Like messages, services have types that define the structure of the request and response. Service types are defined in .srv files, which contain both request and response parts.

### When to Use Services

Services are appropriate when:
- You need a synchronous request-response interaction
- The operation is expected to complete relatively quickly
- You need to ensure that a request has been processed and get a response

## High-Level Humanoid Data Flow

In a humanoid robot system, the data flow typically follows this pattern:

### AI Decision Path

1. **AI System**: Makes high-level decisions based on goals, sensor data, and world models
2. **Behavior Manager**: Translates high-level goals into specific robot behaviors
3. **Motion Planner**: Plans specific movements to achieve the behaviors
4. **Controller**: Sends specific commands to robot actuators

### Sensor Feedback Path

1. **Sensors**: Collect data from the environment (cameras, IMUs, joint encoders, etc.)
2. **Sensor Drivers**: Interface with hardware and publish sensor data
3. **Perception**: Processes sensor data to extract meaningful information
4. **State Estimation**: Maintains an estimate of the robot's state and environment
5. **AI System**: Uses the state information to make decisions

## Practical Example: Simple ROS 2 System

Let's consider a simple example of a humanoid robot that needs to walk toward a target:

```
[Target Detection Node] -> [Walking Command Topic] -> [Walking Controller Node]
                        -> [Target Position Service] -> [Path Planning Node]
```

In this example:
- The Target Detection Node publishes target positions to a topic
- The Walking Controller Node subscribes to the walking commands
- The Path Planning Node provides a service to calculate paths to targets

## Example: Publisher Node in Python

Here's a simple example of a publisher node in Python using rclpy:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Example: Subscriber Node in Python

And here's a corresponding subscriber node:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Example: Service Server in Python

Here's an example of a service server:

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercises

### Exercise 1: Concept Identification
Look at the following ROS 2 system description and identify the nodes, topics, and services:
- A camera node publishes images to a topic called `/camera/image_raw`
- A perception node subscribes to the image topic and publishes detected objects to `/detected_objects`
- A navigation node subscribes to detected objects and provides a service called `/get_navigation_plan`

### Exercise 2: Data Flow Analysis
Describe the high-level data flow in a humanoid robot system where:
- An AI decision system determines that the robot should walk to a specific location
- The system must consider obstacles detected by sensors
- The robot's current position is tracked by a localization system

Sketch the flow of information from the AI decision to the robot's motors, identifying what information travels in each direction.

### Exercise 3: Communication Pattern Selection
For each scenario below, decide whether to use topics (publish/subscribe) or services (request/response), and explain your reasoning:
1. Sending sensor data from multiple sensors to a central processing node
2. Requesting the robot to move to a specific position
3. Asking for the current battery level of the robot
4. Broadcasting the robot's current location to multiple nodes

## Summary

ROS 2 serves as the middleware that connects AI decision-making systems to robot controllers in humanoid robots. Through nodes, topics, and services, it enables complex robotic systems to be built from modular, reusable components that can communicate effectively.

In the next chapter, we'll explore how to create Python-based ROS 2 nodes using the rclpy library to implement these concepts in practice.
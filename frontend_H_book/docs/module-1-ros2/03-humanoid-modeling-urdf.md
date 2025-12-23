---
sidebar_position: 3
description: Understanding URDF for humanoid robot modeling and integration with ROS 2 simulators
---

# Humanoid Modeling with URDF

## Learning Objectives

By the end of this chapter, you will be able to:
- Explain the purpose and structure of URDF (Unified Robot Description Format) for humanoid modeling
- Identify and describe links, joints, and kinematic structure in humanoid robots
- Understand how URDF integrates with ROS 2 simulators
- Create basic URDF models for simple humanoid robots
- Use URDF files with ROS 2 simulation environments

## Prerequisites

- Understanding of ROS 2 fundamentals (covered in Chapter 1)
- Basic knowledge of robotics kinematics concepts
- Familiarity with XML format

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including its links (rigid parts), joints (connections between links), and other properties like inertial parameters, visual meshes, and collision properties.

### Why URDF is Important

URDF is crucial for humanoid robotics because it:
- Provides a standardized way to describe robot geometry
- Enables simulation of robot behavior before deployment
- Allows for visualization of the robot in RViz and other tools
- Supports kinematic and dynamic analysis
- Facilitates integration with ROS 2 tools and simulators

## URDF Structure

A URDF file is an XML document that describes a robot. The main elements include:

### Links
Links represent rigid bodies of the robot. Each link has:
- Visual properties (shape, color, mesh)
- Collision properties (shape for collision detection)
- Inertial properties (mass, center of mass, inertia tensor)

### Joints
Joints connect links together. Each joint defines:
- Type (revolute, continuous, prismatic, fixed, etc.)
- Axis of motion
- Limits (if applicable)
- Parent and child links

### Example: Simple URDF Structure

Here's a basic URDF for a simple robot with a base and a single rotating arm:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Arm link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.5"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Joint connecting base and arm -->
  <joint name="arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

## Links: The Building Blocks

Links are the rigid components of a robot. In humanoid robots, links represent body parts like:
- Torso
- Head
- Arms (upper arm, lower arm, hands)
- Legs (thigh, shin, feet)
- Individual finger segments

### Visual Properties
The visual element defines how the link appears in visualization tools:
- **Geometry**: Shape (box, cylinder, sphere, mesh)
- **Material**: Color and texture
- **Origin**: Position and orientation relative to joint

### Collision Properties
The collision element defines shapes used for collision detection:
- Usually simpler than visual geometry for performance
- Can be different shapes than visual representation

### Inertial Properties
The inertial element defines physical properties for simulation:
- **Mass**: How heavy the link is
- **Inertia**: How mass is distributed (inertia tensor)

## Joints: Connecting the Parts

Joints define how links can move relative to each other. In humanoid robots, joints model:
- Shoulder joints (ball joints with multiple degrees of freedom)
- Elbow joints (hinge joints)
- Hip joints (ball joints)
- Knee joints (hinge joints)
- Wrist joints (ball joints or multiple single-axis joints)

### Joint Types

1. **Fixed**: No movement between parent and child links
2. **Revolute**: Rotational movement around a single axis with limits
3. **Continuous**: Rotational movement around a single axis without limits
4. **Prismatic**: Linear movement along a single axis
5. **Planar**: Movement in a plane
6. **Floating**: 6 degrees of freedom (rarely used)

### Joint Parameters

Each joint typically includes:
- **Parent and child links**: Which links the joint connects
- **Origin**: Position and orientation of the joint
- **Axis**: Direction of movement
- **Limits**: Range of motion and physical constraints

## Humanoid Robot Structure

A typical humanoid robot URDF model includes:

### Kinematic Chain Structure
Humanoid robots have a tree structure with:
- **Base link**: Usually the torso or pelvis
- **Upper body**: Head, torso, arms
- **Lower body**: Legs and feet
- **End effectors**: Hands for manipulation

### Example: Simplified Humanoid URDF

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
  </link>

  <joint name="neck_joint" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </visual>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Right Arm -->
  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </visual>
  </link>

  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Left Leg -->
  <link name="left_thigh">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </visual>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_thigh"/>
    <origin xyz="0.1 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Right Leg -->
  <link name="right_thigh">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </visual>
  </link>

  <joint name="right_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_thigh"/>
    <origin xyz="-0.1 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

## Integration with ROS 2 Simulators

URDF models integrate with ROS 2 simulators like Gazebo and Ignition through:

### Robot State Publisher
- Publishes the robot's joint states as transforms
- Allows visualization of the robot in RViz
- Updates the TF tree with current joint positions

### Joint State Publisher
- Provides initial joint positions
- Can simulate joint movements for visualization

### Gazebo Plugins
- Physics simulation parameters
- Sensor simulation
- Actuator simulation

### Example: Launch File for URDF Integration

```xml
<launch>
  <!-- Load the URDF into the parameter server -->
  <param name="robot_description"
         value="$(find-pkg-share my_robot_description)/urdf/my_robot.urdf"/>

  <!-- Start the robot state publisher -->
  <node pkg="robot_state_publisher"
        exec="robot_state_publisher"
        name="robot_state_publisher">
    <param name="robot_description"
           value="$(var robot_description)"/>
  </node>

  <!-- Start the joint state publisher for visualization -->
  <node pkg="joint_state_publisher"
        exec="joint_state_publisher"
        name="joint_state_publisher"/>
</launch>
```

## Working with URDF in ROS 2

### Loading URDF into ROS 2
URDF files are typically loaded as a ROS 2 parameter named `robot_description`. This parameter is used by various ROS 2 nodes to access the robot model.

### URDF Tools
ROS 2 provides several tools for working with URDF:
- `check_urdf`: Validates URDF syntax
- `urdf_to_graphiz`: Creates a visual representation of the kinematic tree
- `xacro`: Preprocessor that allows macros and calculations in URDF

### Xacro: URDF with Macros
Xacro is an XML macro system that makes URDF files more maintainable:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_robot">

  <!-- Define properties -->
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_width" value="0.05"/>

  <!-- Macro for creating a wheel -->
  <xacro:macro name="wheel" params="prefix parent xyz">
    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </visual>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${xyz}"/>
      <axis xyz="0 1 0"/>
    </joint>
  </xacro:macro>

  <!-- Use the macro -->
  <xacro:wheel prefix="front_left" parent="base_link" xyz="0.2 0.2 0"/>
  <xacro:wheel prefix="front_right" parent="base_link" xyz="0.2 -0.2 0"/>
  <xacro:wheel prefix="back_left" parent="base_link" xyz="-0.2 0.2 0"/>
  <xacro:wheel prefix="back_right" parent="base_link" xyz="-0.2 -0.2 0"/>

</robot>
```

## Exercises

### Exercise 1: URDF Analysis
Given the following URDF snippet, identify:
1. How many links are defined?
2. What types of joints are used?
3. What is the kinematic chain from the base to the end effector?

```xml
<robot name="exercise_robot">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="link2"/>
  <link name="end_effector"/>

  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
  </joint>

  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
  </joint>

  <joint name="joint3" type="fixed">
    <parent link="link2"/>
    <child link="end_effector"/>
  </joint>
</robot>
```

### Exercise 2: Humanoid Arm Model
Create a URDF model for a simple humanoid arm with:
- Upper arm
- Lower arm
- Wrist
- 3 revolute joints connecting them
- Appropriate dimensions and joint limits

### Exercise 3: URDF Integration
Create a launch file that loads your URDF model and starts the necessary ROS 2 nodes to visualize it in RViz.

## Summary

URDF (Unified Robot Description Format) is essential for humanoid robot modeling in ROS 2. It provides a standardized way to describe robot geometry, kinematic structure, and physical properties. By understanding links, joints, and how URDF integrates with ROS 2 simulators, you can create accurate robot models that work with simulation environments and visualization tools.

With this knowledge, you now have a complete understanding of the three core components of ROS 2 as they apply to humanoid robotics: the fundamentals of the middleware, Python-based AI-robot bridges using rclpy, and robot modeling with URDF. These concepts form the foundation for developing sophisticated AI-driven humanoid robot systems.
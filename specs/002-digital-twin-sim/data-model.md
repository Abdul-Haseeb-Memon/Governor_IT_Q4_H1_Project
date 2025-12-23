# Data Model: Digital Twin Simulation Documentation

**Feature**: 002-digital-twin-sim
**Date**: 2025-12-18

## Content Entities

### Chapter
- **name**: String (e.g., "Physics Simulation with Gazebo", "High-Fidelity Environments with Unity", "Sensor Simulation")
- **slug**: String (URL-friendly identifier)
- **order**: Integer (determines sequence in module)
- **prerequisites**: Array of Strings (required knowledge)
- **learning_objectives**: Array of Strings (what student should learn)
- **content**: String (Markdown content)
- **examples**: Array of Example objects
- **exercises**: Array of Exercise objects

### Example
- **title**: String (descriptive title)
- **description**: String (what the example demonstrates)
- **code**: String (the actual code snippet or configuration)
- **language**: String (programming language or configuration format)
- **explanation**: String (detailed explanation of the example)

### Exercise
- **title**: String (descriptive title)
- **description**: String (what the student should implement)
- **difficulty**: Enum (beginner, intermediate, advanced)
- **expected_outcome**: String (what the student should achieve)
- **hints**: Array of Strings (guidance for students)

### Concept
- **name**: String (e.g., "Digital Twin", "Physics Simulation", "Sensor Fusion", "ROS 2 Integration")
- **definition**: String (clear explanation)
- **purpose**: String (why this concept exists)
- **examples**: Array of Example objects
- **related_concepts**: Array of Strings (other concepts this connects to)

### Simulation Component
- **name**: String (e.g., "Gazebo World", "Unity Scene", "Virtual Sensor")
- **type**: Enum (physics, visual, sensor, integration)
- **properties**: Object (configuration parameters specific to component type)
- **integration_points**: Array of String (how this connects to other components)
- **validation_rules**: Array of String (requirements for proper configuration)

## Content Relationships

- Module contains multiple Chapters
- Chapter contains multiple Examples and Exercises
- Chapter covers multiple Concepts
- Concepts may be related to other Concepts
- Examples demonstrate specific Concepts
- Simulation Components are configured within Chapters

## Validation Rules

- Each Chapter must have a unique slug within the module
- Chapter order must be sequential (1, 2, 3, etc.)
- Learning objectives must be specific and measurable
- Code examples must be syntactically correct for the target language
- Exercises must have appropriate difficulty levels based on covered concepts
- Simulation Components must have valid integration points with ROS 2

## State Transitions

- Content draft → reviewed → published
- Exercise difficulty: assigned → tested → validated
- Simulation Component: configured → validated → documented
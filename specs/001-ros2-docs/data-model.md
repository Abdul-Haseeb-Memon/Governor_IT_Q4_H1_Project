# Data Model: ROS 2 Documentation Module

**Feature**: 001-ros2-docs
**Date**: 2025-12-17

## Content Entities

### Chapter
- **name**: String (e.g., "ROS 2 Fundamentals", "Python Agents with rclpy", "Humanoid Modeling with URDF")
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
- **code**: String (the actual code snippet)
- **language**: String (programming language)
- **explanation**: String (detailed explanation of the code)

### Exercise
- **title**: String (descriptive title)
- **description**: String (what the student should implement)
- **difficulty**: Enum (beginner, intermediate, advanced)
- **expected_outcome**: String (what the student should achieve)
- **hints**: Array of Strings (guidance for students)

### Concept
- **name**: String (e.g., "Node", "Topic", "Service", "URDF")
- **definition**: String (clear explanation)
- **purpose**: String (why this concept exists)
- **examples**: Array of Example objects
- **related_concepts**: Array of Strings (other concepts this connects to)

## Content Relationships

- Module contains multiple Chapters
- Chapter contains multiple Examples and Exercises
- Chapter covers multiple Concepts
- Concepts may be related to other Concepts
- Examples demonstrate specific Concepts

## Validation Rules

- Each Chapter must have a unique slug within the module
- Chapter order must be sequential (1, 2, 3, etc.)
- Learning objectives must be specific and measurable
- Code examples must be syntactically correct for the target language
- Exercises must have appropriate difficulty levels based on covered concepts

## State Transitions

- Content draft → reviewed → published
- Exercise difficulty: assigned → tested → validated
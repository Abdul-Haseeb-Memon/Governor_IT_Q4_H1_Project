---
sidebar_position: 2
title: 'Translating Natural Language Goals to Actions'
---

# Translating Natural Language Goals to Actions

This guide covers how to use Large Language Models (LLMs) to translate natural language goals into executable robot actions. You'll learn how to design prompts that effectively convert high-level goals into specific robot behaviors.

## Overview

The core challenge in LLM-based robot planning is translating high-level, natural language goals (like "go to the kitchen and bring me a cup") into specific, executable robot actions. This requires understanding both the natural language input and the robot's capabilities.

## LLM Integration Setup

First, let's set up the LLM integration for goal translation:

```python
import openai
import json
from typing import Dict, List, Any
from dataclasses import dataclass

@dataclass
class RobotAction:
    action_type: str
    parameters: Dict[str, Any]
    description: str

class LLMGoalTranslator:
    def __init__(self, api_key: str, model: str = "gpt-4-turbo"):
        self.client = openai.OpenAI(api_key=api_key)
        self.model = model

    def translate_goal_to_actions(self, goal: str, robot_capabilities: List[str] = None) -> List[RobotAction]:
        """
        Translate a natural language goal into a sequence of robot actions.
        """
        if robot_capabilities is None:
            robot_capabilities = [
                "navigation", "object_manipulation", "grasping",
                "speech_synthesis", "object_detection", "arm_control"
            ]

        prompt = self._create_translation_prompt(goal, robot_capabilities)

        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self._get_system_prompt()},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.1,  # Low temperature for consistent outputs
                max_tokens=1000
            )

            actions_json = response.choices[0].message.content.strip()

            # Remove markdown code block markers if present
            if actions_json.startswith("```json"):
                actions_json = actions_json[7:]  # Remove ```json
            if "```" in actions_json:
                actions_json = actions_json.split("```")[0]  # Remove closing ```

            actions_data = json.loads(actions_json)

            # Convert to RobotAction objects
            actions = []
            for action_data in actions_data:
                action = RobotAction(
                    action_type=action_data["action_type"],
                    parameters=action_data["parameters"],
                    description=action_data["description"]
                )
                actions.append(action)

            return actions

        except Exception as e:
            print(f"Error translating goal to actions: {e}")
            return []

    def _create_translation_prompt(self, goal: str, capabilities: List[str]) -> str:
        """
        Create the prompt for translating goals to actions.
        """
        return f"""
Goal: {goal}

Robot Capabilities: {', '.join(capabilities)}

Translate this high-level goal into a sequence of specific robot actions.
Return a JSON array where each element has:
- action_type: The type of action (e.g., "NAVIGATE", "PICK_UP", "PLACE", "SPEAK")
- parameters: Object containing parameters for the action
- description: Human-readable description of the action

Example output format:
[
  {{
    "action_type": "NAVIGATE",
    "parameters": {{"location": "kitchen"}},
    "description": "Navigate to the kitchen"
  }},
  {{
    "action_type": "PICK_UP",
    "parameters": {{"object": "cup", "location": "counter"}},
    "description": "Pick up the cup from the counter"
  }},
  {{
    "action_type": "NAVIGATE",
    "parameters": {{"location": "user"}},
    "description": "Navigate back to the user"
  }},
  {{
    "action_type": "PLACE",
    "parameters": {{"location": "table"}},
    "description": "Place the cup on the table"
  }}
]

Return only the JSON array with no additional text.
"""

    def _get_system_prompt(self) -> str:
        """
        Get the system prompt for the LLM.
        """
        return """
You are a robot task planner that translates natural language goals into sequences of specific robot actions.
The robot has capabilities for navigation, object manipulation, grasping, speech synthesis, object detection, and arm control.
Return your response as a JSON array of actions with action_type, parameters, and description fields.
Keep the JSON minimal and valid with no additional text or explanations.
"""

# Example usage
if __name__ == "__main__":
    # translator = LLMGoalTranslator(api_key="your-api-key")
    # goal = "go to the kitchen and bring me a red cup"
    # actions = translator.translate_goal_to_actions(goal)
    #
    # for i, action in enumerate(actions):
    #     print(f"Step {i+1}: {action.description}")
    #     print(f"  Type: {action.action_type}")
    #     print(f"  Parameters: {action.parameters}")
    #     print()
    pass
```

## Context-Aware Goal Translation

For more sophisticated goal translation, we need to incorporate environmental context:

```python
@dataclass
class EnvironmentContext:
    current_location: str
    objects_in_environment: List[Dict[str, Any]]
    robot_state: Dict[str, Any]
    available_locations: List[str]

class ContextAwareTranslator(LLMGoalTranslator):
    def translate_goal_with_context(
        self,
        goal: str,
        context: EnvironmentContext
    ) -> List[RobotAction]:
        """
        Translate goal with environmental context.
        """
        prompt = self._create_contextual_prompt(goal, context)

        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self._get_system_prompt()},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.1,
                max_tokens=1000
            )

            actions_json = response.choices[0].message.content.strip()

            # Remove markdown code block markers if present
            if actions_json.startswith("```json"):
                actions_json = actions_json[7:]
            if "```" in actions_json:
                actions_json = actions_json.split("```")[0]

            actions_data = json.loads(actions_json)

            # Convert to RobotAction objects
            actions = []
            for action_data in actions_data:
                action = RobotAction(
                    action_type=action_data["action_type"],
                    parameters=action_data["parameters"],
                    description=action_data["description"]
                )
                actions.append(action)

            return actions

        except Exception as e:
            print(f"Error translating goal with context: {e}")
            return []

    def _create_contextual_prompt(self, goal: str, context: EnvironmentContext) -> str:
        """
        Create a prompt that includes environmental context.
        """
        objects_str = ", ".join([
            f"{obj['name']} at {obj['location']}"
            for obj in context.objects_in_environment
        ])

        available_locations_str = ", ".join(context.available_locations)

        return f"""
Goal: {goal}

Current Context:
- Current Location: {context.current_location}
- Available Locations: {available_locations_str}
- Objects in Environment: {objects_str}
- Robot State: {context.robot_state}

Translate this goal into specific robot actions considering the current environment.
The robot should navigate efficiently and use available objects appropriately.
Return a JSON array of actions with action_type, parameters, and description fields.
"""
```

## Handling Complex Multi-Step Goals

For complex goals that involve multiple subtasks, we can break them down first:

```python
class HierarchicalGoalTranslator(ContextAwareTranslator):
    def translate_complex_goal(self, goal: str, context: EnvironmentContext) -> List[RobotAction]:
        """
        Handle complex goals by first breaking them into subgoals.
        """
        # First, decompose the goal into subgoals
        subgoals = self._decompose_goal(goal, context)

        all_actions = []
        for subgoal in subgoals:
            subgoal_actions = self.translate_goal_with_context(subgoal, context)
            all_actions.extend(subgoal_actions)

        return all_actions

    def _decompose_goal(self, goal: str, context: EnvironmentContext) -> List[str]:
        """
        Decompose a complex goal into simpler subgoals.
        """
        prompt = f"""
Goal: {goal}

Current Context:
- Current Location: {context.current_location}
- Available Locations: {', '.join(context.available_locations)}
- Objects in Environment: {', '.join([obj['name'] for obj in context.objects_in_environment])}

Decompose this complex goal into a sequence of simpler, actionable subgoals.
Each subgoal should be achievable in one or few robot actions.
Return a JSON array of subgoals as strings.

Example for "go to kitchen and bring a cup":
[
  "navigate to kitchen",
  "identify a cup",
  "pick up the cup",
  "navigate back to user"
]
"""

        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": "You are a task decomposition expert. Break down complex goals into simpler subgoals. Return only a JSON array of subgoals as strings."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.1,
                max_tokens=500
            )

            subgoals_json = response.choices[0].message.content.strip()

            if subgoals_json.startswith("```json"):
                subgoals_json = subgoals_json[7:]
            if "```" in subgoals_json:
                subgoals_json = subgoals_json.split("```")[0]

            return json.loads(subgoals_json)

        except Exception as e:
            print(f"Error decomposing goal: {e}")
            # Fallback: return the original goal as a single subgoal
            return [goal]
```

## Error Handling and Validation

It's important to validate the actions generated by the LLM:

```python
class ValidatedGoalTranslator(HierarchicalGoalTranslator):
    def translate_goal_with_validation(
        self,
        goal: str,
        context: EnvironmentContext
    ) -> tuple[List[RobotAction], List[str]]:
        """
        Translate goal with validation, returning both actions and validation errors.
        """
        actions = self.translate_complex_goal(goal, context)
        errors = self._validate_actions(actions, context)

        return actions, errors

    def _validate_actions(
        self,
        actions: List[RobotAction],
        context: EnvironmentContext
    ) -> List[str]:
        """
        Validate the generated actions for feasibility.
        """
        errors = []

        for i, action in enumerate(actions):
            # Validate action type
            valid_action_types = ["NAVIGATE", "PICK_UP", "PLACE", "SPEAK", "DETECT", "MOVE_ARM"]
            if action.action_type not in valid_action_types:
                errors.append(f"Action {i+1}: Invalid action type '{action.action_type}'")

            # Validate navigation action
            if action.action_type == "NAVIGATE":
                target_location = action.parameters.get("location")
                if target_location and target_location not in context.available_locations:
                    errors.append(f"Action {i+1}: Cannot navigate to unknown location '{target_location}'")

            # Validate pick up action
            if action.action_type == "PICK_UP":
                target_object = action.parameters.get("object")
                if target_object:
                    # Check if object exists in environment
                    object_exists = any(
                        obj['name'].lower() == target_object.lower()
                        for obj in context.objects_in_environment
                    )
                    if not object_exists:
                        errors.append(f"Action {i+1}: Object '{target_object}' not found in environment")

        return errors

# Example usage
if __name__ == "__main__":
    # Example of how to use the validated translator
    context = EnvironmentContext(
        current_location="living_room",
        objects_in_environment=[
            {"name": "cup", "location": "kitchen_counter", "color": "red"},
            {"name": "book", "location": "table", "color": "blue"}
        ],
        robot_state={"battery_level": 0.8, "arm_free": True},
        available_locations=["living_room", "kitchen", "bedroom"]
    )

    # translator = ValidatedGoalTranslator(api_key="your-api-key")
    # goal = "go to kitchen and bring me the red cup"
    # actions, errors = translator.translate_goal_with_validation(goal, context)
    #
    # print(f"Generated {len(actions)} actions:")
    # for i, action in enumerate(actions):
    #     print(f"  {i+1}. {action.description}")
    #
    # if errors:
    #     print(f"\nValidation errors ({len(errors)}):")
    #     for error in errors:
    #         print(f"  - {error}")
    # else:
    #     print("\nAll actions are valid!")
    pass
```

## Best Practices

- **Prompt Engineering**: Craft clear, specific prompts that guide the LLM to generate the desired output format
- **Context Provision**: Provide relevant environmental context to improve planning accuracy
- **Validation**: Always validate generated actions for feasibility and safety
- **Error Recovery**: Implement fallback strategies when LLM generates invalid actions
- **Iterative Refinement**: Use feedback to improve the translation process over time

## Next Steps

After translating natural language goals to actions, the next step is to generate executable action sequences that can be sent to the robot. Continue with the [Generation of Executable Action Sequences](./action-sequences.md) guide.
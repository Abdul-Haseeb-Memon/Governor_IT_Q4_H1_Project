"""
LLM Prompt Engineering Examples for Robot Task Planning

This file contains examples of well-engineered prompts for various
robot planning tasks using Large Language Models.
"""

class PromptEngineeringExamples:
    """
    Collection of prompt engineering examples for LLM-based robot planning.
    """

    @staticmethod
    def goal_translation_prompt(goal: str, robot_capabilities: list, environment_context: dict) -> str:
        """
        Prompt for translating natural language goals to robot actions.
        """
        return f"""
Goal: {goal}

Robot Capabilities: {', '.join(robot_capabilities)}

Environmental Context:
- Current Location: {environment_context.get('current_location', 'unknown')}
- Available Locations: {', '.join(environment_context.get('available_locations', []))}
- Objects in Environment: {', '.join([obj['name'] for obj in environment_context.get('objects', [])])}
- Time of Day: {environment_context.get('time_of_day', 'unknown')}

Translate this high-level goal into a sequence of specific robot actions.
Each action should be executable by the robot given its capabilities.
Consider the environmental context when planning the sequence.

Return a JSON array with the following structure:
[
  {{
    "action_type": "NAVIGATE|PICK_UP|PLACE|SPEAK|DETECT|MOVE_ARM",
    "parameters": {{"location": "kitchen", "object": "cup", ...}},
    "description": "Human-readable description of the action"
  }}
]

Example for goal "go to kitchen and bring me a red cup":
[
  {{
    "action_type": "NAVIGATE",
    "parameters": {{"location": "kitchen"}},
    "description": "Navigate to the kitchen"
  }},
  {{
    "action_type": "DETECT",
    "parameters": {{"object": "red cup"}},
    "description": "Detect the red cup in the kitchen"
  }},
  {{
    "action_type": "PICK_UP",
    "parameters": {{"object": "red cup"}},
    "description": "Pick up the red cup"
  }},
  {{
    "action_type": "NAVIGATE",
    "parameters": {{"location": "current_user_location"}},
    "description": "Return to user location"
  }},
  {{
    "action_type": "PLACE",
    "parameters": {{"location": "table"}},
    "description": "Place the cup on the table"
  }}
]

Return ONLY the JSON array with no additional text.
"""

    @staticmethod
    def task_decomposition_prompt(complex_task: str) -> str:
        """
        Prompt for decomposing complex tasks into simpler subtasks.
        """
        return f"""
Complex Task: {complex_task}

Decompose this complex task into a sequence of simpler, actionable subtasks.
Each subtask should be achievable with basic robot capabilities like navigation,
object manipulation, and sensing.

The decomposition should consider:
1. Logical sequence of operations
2. Prerequisites for each step
3. Potential dependencies between subtasks
4. Error recovery considerations

Return a JSON array of subtasks with the following structure:
[
  {{
    "name": "descriptive_name",
    "description": "what this subtask accomplishes",
    "dependencies": ["subtask_name_1", "subtask_name_2"],  # previous subtasks this depends on
    "estimated_duration": 30.0  # in seconds
  }}
]

Example for "prepare dinner and set table":
[
  {{
    "name": "navigate_to_kitchen",
    "description": "Move to kitchen area to begin food preparation",
    "dependencies": [],
    "estimated_duration": 10.0
  }},
  {{
    "name": "find_ingredients",
    "description": "Locate necessary ingredients for the meal",
    "dependencies": ["navigate_to_kitchen"],
    "estimated_duration": 30.0
  }},
  {{
    "name": "prepare_food",
    "description": "Cook the meal using available ingredients",
    "dependencies": ["find_ingredients"],
    "estimated_duration": 120.0
  }}
]

Return ONLY the JSON array with no additional text.
"""

    @staticmethod
    def action_validation_prompt(actions: list, robot_state: dict) -> str:
        """
        Prompt for validating planned actions against robot state.
        """
        return f"""
Planned Actions: {json.dumps(actions, indent=2)}

Robot State: {json.dumps(robot_state, indent=2)}

Analyze the planned actions and validate them against the robot's current state.
Check for:
1. Capability constraints (can the robot perform these actions?)
2. Resource availability (does the robot have necessary resources?)
3. Physical constraints (are the actions physically possible?)
4. Safety considerations (do any actions pose risks?)

For each action, return validation results in the following format:
[
  {{
    "action_index": 0,
    "action_type": "NAVIGATE",
    "is_valid": true,
    "issues": [],
    "suggestions": []
  }}
]

If an action is invalid, include specific issues and potential suggestions for correction.

Return ONLY the JSON array with no additional text.
"""

    @staticmethod
    def context_integration_prompt(goal: str, context: dict) -> str:
        """
        Prompt for integrating environmental context into planning.
        """
        return f"""
Goal: {goal}

Environmental Context:
{json.dumps(context, indent=2)}

Create a task plan that incorporates the environmental context.
Consider factors like:
- Current location and available paths
- Objects present in the environment
- Time constraints or preferences
- User preferences and history
- Environmental conditions (lighting, noise, etc.)

The plan should be optimized for the current context while achieving the goal.
Return a detailed task plan as JSON with action sequences, timing considerations,
and context-aware optimizations.

Return ONLY the JSON plan with no additional text.
"""

    @staticmethod
    def error_recovery_prompt(failed_action: dict, error_context: dict) -> str:
        """
        Prompt for generating error recovery strategies.
        """
        return f"""
Failed Action: {json.dumps(failed_action, indent=2)}

Error Context: {json.dumps(error_context, indent=2)}

The robot attempted the above action but failed. Generate potential recovery strategies.
Consider:
1. Alternative approaches to achieve the same goal
2. Environmental factors that may have caused the failure
3. Available resources for recovery
4. Safety considerations for recovery actions

Return a JSON array of potential recovery strategies:
[
  {{
    "strategy": "alternative_approach",
    "description": "How to achieve the goal differently",
    "required_actions": ["list", "of", "actions"],
    "confidence": 0.8  # How likely this will work
  }}
]

Return ONLY the JSON array with no additional text.
"""

    @staticmethod
    def system_prompt_for_planning() -> str:
        """
        System prompt for the LLM to act as a robot task planner.
        """
        return """
You are an expert robot task planner that converts natural language goals into executable action sequences.
The robot has capabilities for navigation, manipulation, sensing, and communication.
Always return structured JSON responses with no additional text or explanations.
Consider environmental context, robot capabilities, and task dependencies when planning.
Prioritize safety and feasibility in all generated plans.
"""

    @staticmethod
    def few_shot_examples() -> list:
        """
        Provide few-shot examples for better LLM performance.
        """
        return [
            {
                "goal": "bring me a cup of water from the kitchen",
                "plan": [
                    {
                        "action_type": "NAVIGATE",
                        "parameters": {"location": "kitchen"},
                        "description": "Navigate to kitchen"
                    },
                    {
                        "action_type": "PICK_UP",
                        "parameters": {"object": "cup"},
                        "description": "Pick up a cup"
                    },
                    {
                        "action_type": "DETECT",
                        "parameters": {"object": "water_source"},
                        "description": "Locate water source"
                    },
                    {
                        "action_type": "PLACE",
                        "parameters": {"object": "cup", "location": "water_source"},
                        "description": "Place cup at water source"
                    }
                ]
            },
            {
                "goal": "find my keys and bring them to me",
                "plan": [
                    {
                        "action_type": "DETECT",
                        "parameters": {"object": "keys"},
                        "description": "Search for keys in common locations"
                    },
                    {
                        "action_type": "NAVIGATE",
                        "parameters": {"location": "keys_location"},
                        "description": "Go to where keys were found"
                    },
                    {
                        "action_type": "PICK_UP",
                        "parameters": {"object": "keys"},
                        "description": "Pick up the keys"
                    },
                    {
                        "action_type": "NAVIGATE",
                        "parameters": {"location": "user_location"},
                        "description": "Return to user"
                    }
                ]
            }
        ]


# Example usage
if __name__ == "__main__":
    import json

    examples = PromptEngineeringExamples()

    # Example of creating a goal translation prompt
    goal = "go to the kitchen and bring me a red cup"
    capabilities = ["navigation", "object_manipulation", "grasping", "speech"]
    context = {
        "current_location": "living_room",
        "available_locations": ["living_room", "kitchen", "bedroom"],
        "objects": [
            {"name": "red cup", "location": "kitchen_counter"},
            {"name": "book", "location": "table"}
        ],
        "time_of_day": "afternoon"
    }

    prompt = examples.goal_translation_prompt(goal, capabilities, context)
    print("Goal Translation Prompt:")
    print(prompt)
    print("\n" + "="*50 + "\n")

    # Example of task decomposition prompt
    complex_task = "prepare coffee and serve to guest in living room"
    task_prompt = examples.task_decomposition_prompt(complex_task)
    print("Task Decomposition Prompt:")
    print(task_prompt)
---
sidebar_position: 4
title: 'High-Level Task Planning for Robots'
---

# High-Level Task Planning for Robots

This guide covers implementing high-level task planning systems for humanoid robots using Large Language Models. You'll learn how to create sophisticated planning systems that can handle complex, multi-step tasks with dependencies and constraints.

## Overview

High-level task planning involves creating systems that can:
- Understand complex, multi-step tasks from natural language
- Generate detailed execution plans with proper sequencing
- Handle task dependencies and resource constraints
- Adapt plans based on environmental feedback
- Recover from failures and unexpected situations

## Hierarchical Task Planner

A hierarchical task planner breaks down complex tasks into manageable subtasks:

```python
import json
from typing import List, Dict, Any, Optional, Callable
from dataclasses import dataclass, field
from enum import Enum
import asyncio
import time

class TaskStatus(Enum):
    PENDING = "pending"
    PLANNING = "planning"
    EXECUTING = "executing"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"

@dataclass
class Task:
    id: str
    name: str
    description: str
    dependencies: List[str] = field(default_factory=list)
    subtasks: List['Task'] = field(default_factory=list)
    parameters: Dict[str, Any] = field(default_factory=dict)
    status: TaskStatus = TaskStatus.PENDING
    priority: int = 0  # Lower number = higher priority
    estimated_duration: float = 0.0  # in seconds
    actual_duration: float = 0.0
    result: Optional[Any] = None
    error: Optional[str] = None

class HierarchicalTaskPlanner:
    def __init__(self):
        self.tasks: Dict[str, Task] = {}
        self.task_counter = 0

    def create_task(self, name: str, description: str, dependencies: List[str] = None,
                   parameters: Dict[str, Any] = None, priority: int = 0,
                   estimated_duration: float = 0.0) -> Task:
        """
        Create a new task with the given parameters.
        """
        task_id = f"task_{self.task_counter:04d}"
        self.task_counter += 1

        task = Task(
            id=task_id,
            name=name,
            description=description,
            dependencies=dependencies or [],
            parameters=parameters or {},
            priority=priority,
            estimated_duration=estimated_duration
        )

        self.tasks[task_id] = task
        return task

    def create_composite_task(self, name: str, description: str, subtasks: List[Task]) -> Task:
        """
        Create a composite task that contains subtasks.
        """
        task_id = f"task_{self.task_counter:04d}"
        self.task_counter += 1

        composite_task = Task(
            id=task_id,
            name=name,
            description=description,
            subtasks=subtasks,
            status=TaskStatus.PENDING
        )

        self.tasks[task_id] = composite_task
        return composite_task

    def plan_task(self, goal: str) -> Task:
        """
        Plan a task based on a natural language goal.
        """
        # This would typically call an LLM to decompose the goal
        # For this example, we'll simulate the planning process
        self.tasks.clear()  # Reset for new plan

        # Example: Plan for "prepare coffee and bring it to the living room"
        root_task = self.create_task(
            name="root",
            description="Prepare coffee and bring to living room",
            priority=0
        )

        # Subtasks for the main goal
        navigate_to_kitchen = self.create_task(
            name="navigate_to_kitchen",
            description="Navigate to kitchen area",
            parameters={"destination": "kitchen"},
            priority=1
        )

        find_coffee_supplies = self.create_task(
            name="find_coffee_supplies",
            description="Locate coffee supplies (beans, filter, water)",
            dependencies=["navigate_to_kitchen"],
            priority=2
        )

        prepare_coffee = self.create_task(
            name="prepare_coffee",
            description="Prepare coffee using located supplies",
            dependencies=["find_coffee_supplies"],
            priority=3
        )

        navigate_to_living_room = self.create_task(
            name="navigate_to_living_room",
            description="Navigate to living room with prepared coffee",
            dependencies=["prepare_coffee"],
            parameters={"destination": "living_room"},
            priority=4
        )

        serve_coffee = self.create_task(
            name="serve_coffee",
            description="Serve coffee to user in living room",
            dependencies=["navigate_to_living_room"],
            priority=5
        )

        # Create composite task
        composite_task = self.create_composite_task(
            name="coffee_service",
            description="Complete coffee service task",
            subtasks=[navigate_to_kitchen, find_coffee_supplies, prepare_coffee,
                     navigate_to_living_room, serve_coffee]
        )

        return composite_task

    def get_ready_tasks(self) -> List[Task]:
        """
        Get tasks that are ready to execute (dependencies satisfied).
        """
        ready_tasks = []

        for task_id, task in self.tasks.items():
            if task.status != TaskStatus.PENDING:
                continue

            # Check if all dependencies are completed
            all_deps_satisfied = True
            for dep_id in task.dependencies:
                dep_task = self.tasks.get(dep_id)
                if dep_task and dep_task.status != TaskStatus.COMPLETED:
                    all_deps_satisfied = False
                    break

            if all_deps_satisfied:
                ready_tasks.append(task)

        # Sort by priority (lower number = higher priority)
        ready_tasks.sort(key=lambda t: t.priority)
        return ready_tasks

    def execute_task(self, task: Task) -> bool:
        """
        Execute a single task (simulated).
        """
        print(f"Executing task: {task.name} - {task.description}")

        # Simulate task execution time
        time.sleep(task.estimated_duration)

        # For simulation, assume success
        task.status = TaskStatus.COMPLETED
        task.actual_duration = task.estimated_duration
        task.result = f"Result of {task.name}"

        print(f"Completed task: {task.name}")
        return True
```

## Context-Aware Task Planning

For more sophisticated planning, we need to consider environmental context:

```python
@dataclass
class EnvironmentContext:
    locations: Dict[str, Dict[str, Any]]  # location_id -> properties
    objects: Dict[str, Dict[str, Any]]    # object_id -> properties
    robot_state: Dict[str, Any]
    time_of_day: str
    user_preferences: Dict[str, Any]

class ContextAwarePlanner(HierarchicalTaskPlanner):
    def __init__(self):
        super().__init__()
        self.context: Optional[EnvironmentContext] = None

    def set_context(self, context: EnvironmentContext):
        """
        Set the current environmental context.
        """
        self.context = context

    def plan_task_with_context(self, goal: str) -> Task:
        """
        Plan a task considering environmental context.
        """
        if not self.context:
            raise ValueError("Context must be set before planning")

        # Example: Adjust plan based on context
        # If it's nighttime, we might want to be quieter
        is_nighttime = self.context.time_of_day in ["evening", "night"]

        # If user prefers stronger coffee
        prefers_strong_coffee = self.context.user_preferences.get("coffee_strength") == "strong"

        # Adjust the plan based on context
        root_task = self.create_task(
            name="root",
            description=f"Prepare coffee{' (strong)' if prefers_strong_coffee else ''} and bring to living room",
            priority=0
        )

        # Navigation task with context
        navigate_to_kitchen = self.create_task(
            name="navigate_to_kitchen",
            description="Navigate to kitchen area",
            parameters={
                "destination": "kitchen",
                "speed": "slow" if is_nighttime else "normal"  # Move slower at night
            },
            priority=1
        )

        # Other tasks...
        find_coffee_supplies = self.create_task(
            name="find_coffee_supplies",
            description="Locate coffee supplies (beans, filter, water)",
            dependencies=["navigate_to_kitchen"],
            priority=2
        )

        # Adjust preparation based on user preference
        prepare_coffee = self.create_task(
            name="prepare_coffee",
            description=f"Prepare{' strong' if prefers_strong_coffee else ''} coffee",
            dependencies=["find_coffee_supplies"],
            parameters={
                "strength": "strong" if prefers_strong_coffee else "normal"
            },
            priority=3
        )

        navigate_to_living_room = self.create_task(
            name="navigate_to_living_room",
            description="Navigate to living room with prepared coffee",
            dependencies=["prepare_coffee"],
            parameters={
                "destination": "living_room",
                "speed": "slow" if is_nighttime else "normal"
            },
            priority=4
        )

        serve_coffee = self.create_task(
            name="serve_coffee",
            description="Serve coffee to user in living room",
            dependencies=["navigate_to_living_room"],
            priority=5
        )

        composite_task = self.create_composite_task(
            name="contextual_coffee_service",
            description=f"Context-aware coffee service{' (night mode)' if is_nighttime else ''}",
            subtasks=[navigate_to_kitchen, find_coffee_supplies, prepare_coffee,
                     navigate_to_living_room, serve_coffee]
        )

        return composite_task
```

## LLM-Integrated Task Planner

Integrate with LLMs for intelligent task decomposition:

```python
import openai
from typing import Union

class LLMTaskPlanner(ContextAwarePlanner):
    def __init__(self, api_key: str, model: str = "gpt-4-turbo"):
        super().__init__()
        self.client = openai.OpenAI(api_key=api_key)
        self.model = model

    def plan_task_with_llm(self, goal: str) -> Task:
        """
        Use LLM to generate a task plan for the given goal.
        """
        if not self.context:
            # Create a minimal context if none exists
            self.context = EnvironmentContext(
                locations={"kitchen": {}, "living_room": {}, "bedroom": {}},
                objects={"coffee": {}, "cup": {}},
                robot_state={"capabilities": ["navigation", "manipulation"]},
                time_of_day="day",
                user_preferences={}
            )

        prompt = self._create_planning_prompt(goal)

        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self._get_planning_system_prompt()},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.1,
                max_tokens=1500
            )

            plan_json = response.choices[0].message.content.strip()

            # Clean up the response
            if plan_json.startswith("```json"):
                plan_json = plan_json[7:]
            if "```" in plan_json:
                plan_json = plan_json.split("```")[0]

            plan_data = json.loads(plan_json)
            return self._build_task_from_plan(plan_data)

        except Exception as e:
            print(f"Error planning with LLM: {e}")
            # Fallback to basic planning
            return self.plan_task(goal)

    def _create_planning_prompt(self, goal: str) -> str:
        """
        Create the prompt for LLM-based task planning.
        """
        context_str = json.dumps({
            "locations": list(self.context.locations.keys()),
            "available_objects": list(self.context.objects.keys()),
            "robot_capabilities": self.context.robot_state.get("capabilities", []),
            "time_of_day": self.context.time_of_day,
            "user_preferences": self.context.user_preferences
        }, indent=2)

        return f"""
Goal: {goal}

Environmental Context:
{context_str}

Create a detailed task plan to achieve this goal. The plan should be a JSON object with:
- name: Name of the main task
- description: Description of the main task
- subtasks: Array of subtasks, each with:
  - name: Task name
  - description: Task description
  - dependencies: Array of task names this task depends on
  - parameters: Object with task-specific parameters
  - priority: Integer (lower = higher priority)
  - estimated_duration: Estimated time in seconds

Consider the environmental context when creating the plan. For example:
- Use available locations and objects
- Respect robot capabilities
- Consider time of day (e.g., move slower at night)
- Adapt to user preferences

Return only the JSON plan with no additional text.
"""

    def _get_planning_system_prompt(self) -> str:
        """
        Get the system prompt for task planning.
        """
        return """
You are a sophisticated task planning AI for a humanoid robot. Create detailed, executable task plans that consider environmental context and robot capabilities. Return your response as valid JSON only, with no additional text or explanations.
"""

    def _build_task_from_plan(self, plan_data: Dict[str, Any]) -> Task:
        """
        Build a Task object from the plan data returned by LLM.
        """
        # Clear existing tasks
        self.tasks.clear()

        # Create subtasks
        subtasks = []
        for subtask_data in plan_data.get('subtasks', []):
            subtask = self.create_task(
                name=subtask_data.get('name', 'unnamed'),
                description=subtask_data.get('description', ''),
                dependencies=subtask_data.get('dependencies', []),
                parameters=subtask_data.get('parameters', {}),
                priority=subtask_data.get('priority', 0),
                estimated_duration=subtask_data.get('estimated_duration', 0.0)
            )
            subtasks.append(subtask)

        # Create the main composite task
        composite_task = self.create_composite_task(
            name=plan_data.get('name', 'unnamed'),
            description=plan_data.get('description', ''),
            subtasks=subtasks
        )

        return composite_task
```

## Task Execution with Monitoring

Implement task execution with real-time monitoring and adaptation:

```python
class MonitoredTaskExecutor(LLMTaskPlanner):
    def __init__(self, api_key: str):
        super().__init__(api_key)
        self.execution_callbacks: Dict[str, List[Callable]] = {
            'on_task_start': [],
            'on_task_complete': [],
            'on_task_fail': [],
            'on_plan_complete': []
        }

    async def execute_plan(self, task: Task) -> bool:
        """
        Execute a planned task with monitoring and adaptation.
        """
        print(f"Starting execution of plan: {task.name}")

        # Execute subtasks in dependency order
        success = await self._execute_subtasks(task.subtasks)

        if success:
            print(f"Plan completed successfully: {task.name}")
            task.status = TaskStatus.COMPLETED
            self._trigger_callbacks('on_plan_complete', task)
        else:
            print(f"Plan failed: {task.name}")
            task.status = TaskStatus.FAILED

        return success

    async def _execute_subtasks(self, subtasks: List[Task]) -> bool:
        """
        Execute subtasks with dependency management.
        """
        remaining_tasks = subtasks.copy()
        completed_task_ids = set()

        while remaining_tasks:
            # Find tasks whose dependencies are satisfied
            ready_tasks = []
            for task in remaining_tasks:
                all_deps_met = all(dep_id in completed_task_ids for dep_id in task.dependencies)
                if all_deps_met:
                    ready_tasks.append(task)

            if not ready_tasks:
                print("No tasks ready to execute - possible circular dependency")
                return False

            # Execute ready tasks in priority order
            ready_tasks.sort(key=lambda t: t.priority)

            # Execute tasks (could be parallel in real implementation)
            for task in ready_tasks:
                print(f"Executing: {task.name}")

                # Trigger start callback
                self._trigger_callbacks('on_task_start', task)

                # Execute the task
                start_time = time.time()
                success = await self._execute_single_task(task)
                task.actual_duration = time.time() - start_time

                if success:
                    completed_task_ids.add(task.id)
                    remaining_tasks.remove(task)
                    task.status = TaskStatus.COMPLETED

                    # Trigger completion callback
                    self._trigger_callbacks('on_task_complete', task)
                else:
                    print(f"Task failed: {task.name}")
                    task.status = TaskStatus.FAILED

                    # Trigger failure callback
                    self._trigger_callbacks('on_task_fail', task)
                    return False

        return True

    async def _execute_single_task(self, task: Task) -> bool:
        """
        Execute a single task (simulated).
        """
        print(f"  Executing: {task.description}")

        # Simulate task execution
        await asyncio.sleep(task.estimated_duration)

        # Simulate potential failures
        import random
        if random.random() < 0.1:  # 10% chance of failure
            task.error = "Simulated task failure"
            return False

        task.result = f"Successfully completed {task.name}"
        return True

    def add_callback(self, event: str, callback: Callable):
        """
        Add a callback for a specific event.
        """
        if event in self.execution_callbacks:
            self.execution_callbacks[event].append(callback)

    def _trigger_callbacks(self, event: str, *args):
        """
        Trigger all callbacks for an event.
        """
        for callback in self.execution_callbacks.get(event, []):
            try:
                callback(*args)
            except Exception as e:
                print(f"Error in callback for {event}: {e}")
```

## Adaptive Planning

Implement the ability to adapt plans based on execution feedback:

```python
class AdaptiveTaskPlanner(MonitoredTaskExecutor):
    def __init__(self, api_key: str):
        super().__init__(api_key)
        self.max_replanning_attempts = 3

    async def execute_plan_with_adaptation(self, goal: str) -> bool:
        """
        Execute a plan with the ability to adapt when issues arise.
        """
        # Initial plan
        plan = self.plan_task_with_llm(goal)

        for attempt in range(self.max_replanning_attempts):
            print(f"Execution attempt {attempt + 1}")

            success = await self.execute_plan(plan)

            if success:
                return True

            # If failed, try to replan based on what went wrong
            if attempt < self.max_replanning_attempts - 1:
                print("Plan failed, attempting to replan...")
                plan = self._replan_after_failure(plan, goal)

        return False

    def _replan_after_failure(self, failed_plan: Task, goal: str) -> Task:
        """
        Create a new plan after the previous one failed.
        """
        # In a real implementation, you would analyze what failed
        # and create an alternative plan
        print(f"Replanning for goal: {goal}")

        # For this example, we'll just create a new plan
        return self.plan_task_with_llm(goal)

    def update_context_from_execution(self, task: Task):
        """
        Update environmental context based on task execution results.
        """
        # This would update the context based on what was learned during execution
        # For example, if a navigation task failed, update map with new obstacle info
        pass
```

## Best Practices

- **Modularity**: Design task planners to be modular and reusable
- **Context Awareness**: Always consider environmental context in planning
- **Adaptability**: Implement mechanisms to adapt plans when execution fails
- **Monitoring**: Provide real-time monitoring and feedback
- **Safety**: Include safety checks and validation in all plans
- **Scalability**: Design planners that can handle complex, multi-step tasks

## Next Steps

With high-level task planning implemented, you now have the complete LLM-based planning system. The next chapter covers the third component of VLA: Vision-Based Object Understanding. Continue with the [Vision-Based Object Understanding](../vision-understanding/index.md) chapter.
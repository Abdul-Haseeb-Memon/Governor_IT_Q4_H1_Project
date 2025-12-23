---
sidebar_position: 5
title: 'Debugging and Troubleshooting Guide'
---

# Debugging and Troubleshooting Guide

This guide covers debugging and troubleshooting techniques for the complete Vision-Language-Action (VLA) pipeline. You'll learn how to identify, diagnose, and resolve common issues in voice-to-manipulation systems.

## Overview

Debugging VLA systems requires understanding multiple interconnected components:
- Voice processing pipeline
- Language planning system
- Vision perception system
- Action execution system
- Integration between components

## Common Issues and Solutions

### Voice Processing Issues

#### Issue: Poor Speech Recognition Accuracy
**Symptoms:**
- Commands are frequently misunderstood
- High error rate in voice processing
- Inconsistent transcription results

**Solutions:**
1. **Check audio quality:**
   ```bash
   # Test audio input quality
   arecord -D hw:0,0 -f cd -d 5 test.wav
   aplay test.wav
   ```

2. **Verify Whisper API configuration:**
   ```python
   # Check API key and model availability
   import openai
   client = openai.OpenAI(api_key="your-api-key")

   # Test basic API connectivity
   try:
       transcription = client.audio.transcriptions.create(
           model="whisper-1",
           file=open("test.wav", "rb")
       )
       print("API connectivity: OK")
   except Exception as e:
       print(f"API connectivity: FAILED - {e}")
   ```

3. **Adjust confidence thresholds:**
   ```python
   # In your voice processing node
   CONFIDENCE_THRESHOLD = 0.7  # Adjust based on environment
   ```

#### Issue: Voice Processing Timeouts
**Symptoms:**
- Long delays in voice command processing
- Timeout errors in voice processing
- System appears unresponsive to voice commands

**Solutions:**
1. **Check API rate limits:**
   - Monitor OpenAI usage dashboard
   - Implement retry logic with exponential backoff
   - Consider caching common commands

2. **Optimize processing pipeline:**
   ```python
   # Use threading for non-blocking voice processing
   import threading

   def process_voice_async(audio_data):
       thread = threading.Thread(
           target=process_voice_command,
           args=(audio_data,)
       )
       thread.daemon = True
       thread.start()
   ```

### Language Planning Issues

#### Issue: Inappropriate Action Plans
**Symptoms:**
- Robot executes incorrect actions
- Planning system generates invalid action sequences
- Actions don't match user intent

**Solutions:**
1. **Review prompt engineering:**
   ```python
   # Ensure clear, specific prompts
   def create_planning_prompt(self, goal: str, context: dict) -> str:
       return f"""
       Goal: {goal}

       Context: {json.dumps(context, indent=2)}

       Plan actions that are:
       1. Safe for the robot and environment
       2. Feasible given robot capabilities
       3. Achieve the stated goal

       Return ONLY valid JSON with no additional text.
       """
   ```

2. **Validate generated plans:**
   ```python
   def validate_plan(self, plan: dict) -> tuple[bool, list]:
       errors = []

       # Check for required fields
       if 'actions' not in plan:
           errors.append("Plan missing 'actions' field")

       # Validate action types
       valid_actions = ['NAVIGATE', 'GRASP', 'PLACE', 'SPEAK', 'DETECT']
       for action in plan.get('actions', []):
           if action.get('action_type') not in valid_actions:
               errors.append(f"Invalid action type: {action.get('action_type')}")

       return len(errors) == 0, errors
   ```

#### Issue: Planning System Unresponsiveness
**Symptoms:**
- Long delays in plan generation
- System hangs during planning phase
- High resource usage during planning

**Solutions:**
1. **Implement planning timeouts:**
   ```python
   import signal

   def timeout_handler(signum, frame):
       raise TimeoutError("Planning timed out")

   def plan_with_timeout(self, goal: str, timeout: int = 30):
       signal.signal(signal.SIGALRM, timeout_handler)
       signal.alarm(timeout)
       try:
           plan = self.generate_plan(goal)
           signal.alarm(0)  # Cancel alarm
           return plan
       except TimeoutError:
           return {'actions': [], 'error': 'Planning timed out'}
   ```

2. **Optimize LLM calls:**
   - Use smaller, faster models for simple tasks
   - Implement result caching for common commands
   - Consider local LLM alternatives for faster response

### Vision System Issues

#### Issue: Poor Object Detection Accuracy
**Symptoms:**
- Objects not detected reliably
- Incorrect object classifications
- False positives in detection

**Solutions:**
1. **Check model configuration:**
   ```python
   # Verify model and confidence settings
   model = YOLO("yolov8n.pt")  # Ensure correct model path
   results = model(image, conf=0.5)  # Adjust confidence threshold
   ```

2. **Validate camera calibration:**
   ```python
   # Check camera matrix and distortion coefficients
   def validate_camera_calibration(self, camera_info):
       if camera_info.k == [0.0] * 9:
           raise ValueError("Camera not calibrated")

       # Check for reasonable focal length values
       fx, fy = camera_info.k[0], camera_info.k[4]
       if fx < 100 or fy < 100:
           raise ValueError(f"Unreasonable focal lengths: fx={fx}, fy={fy}")
   ```

3. **Improve lighting conditions:**
   - Ensure adequate lighting in the environment
   - Avoid direct sunlight or harsh shadows
   - Consider using infrared or thermal cameras for low-light conditions

#### Issue: 3D Position Estimation Errors
**Symptoms:**
- Robot navigates to wrong locations
- Grasping attempts fail due to incorrect positioning
- Distance measurements are inaccurate

**Solutions:**
1. **Verify camera calibration:**
   ```bash
   # Use ROS camera calibration tools
   ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.108 image:=/camera/image_raw
   ```

2. **Check depth sensor data:**
   ```python
   def validate_depth_data(self, depth_image):
       # Check for valid depth values
       valid_depths = depth_image[depth_image > 0]
       if len(valid_depths) == 0:
           raise ValueError("No valid depth data available")

       # Check for reasonable depth range
       if np.min(valid_depths) < 0.1 or np.max(valid_depths) > 10.0:
           raise ValueError(f"Depth values out of expected range: {np.min(valid_depths)} - {np.max(valid_depths)}")
   ```

### Integration Issues

#### Issue: Component Communication Failures
**Symptoms:**
- Messages not being passed between components
- Pipeline stalls at certain stages
- Components not receiving expected data

**Solutions:**
1. **Check ROS topic connections:**
   ```bash
   # List active topics
   ros2 topic list

   # Check topic types
   ros2 topic type /vla_commands

   # Echo topic to verify data flow
   ros2 topic echo /vla_commands
   ```

2. **Verify QoS profiles:**
   ```python
   # Ensure matching QoS profiles between publisher and subscriber
   qos_profile = QoSProfile(
       depth=10,
       reliability=ReliabilityPolicy.RELIABLE,
       durability=DurabilityPolicy.VOLATILE
   )
   ```

3. **Implement message validation:**
   ```python
   def validate_message(self, msg: String):
       try:
           data = json.loads(msg.data)
           required_fields = ['command_id', 'timestamp']
           for field in required_fields:
               if field not in data:
                   self.get_logger().error(f"Missing required field: {field}")
                   return False
           return True
       except json.JSONDecodeError:
           self.get_logger().error("Invalid JSON in message")
           return False
   ```

#### Issue: Pipeline State Management Problems
**Symptoms:**
- Pipeline gets stuck in certain states
- Multiple commands processed simultaneously
- State information becomes inconsistent

**Solutions:**
1. **Implement proper state management:**
   ```python
   class PipelineState(Enum):
       IDLE = "idle"
       VOICE_PROCESSING = "voice_processing"
       PLANNING = "planning"
       NAVIGATION = "navigation"
       MANIPULATION = "manipulation"
       COMPLETED = "completed"
       ERROR = "error"

   class VLAPipeline:
       def __init__(self):
           self.current_state = PipelineState.IDLE
           self.state_lock = threading.Lock()

       def transition_state(self, new_state: PipelineState):
           with self.state_lock:
               old_state = self.current_state
               self.current_state = new_state
               self.get_logger().info(f"State transition: {old_state.value} -> {new_state.value}")
   ```

2. **Add state timeouts:**
   ```python
   def check_state_timeouts(self):
       max_times = {
           PipelineState.VOICE_PROCESSING: 10.0,
           PipelineState.PLANNING: 15.0,
           PipelineState.NAVIGATION: 60.0,
           PipelineState.MANIPULATION: 30.0
       }

       time_in_state = time.time() - self.state_start_time
       max_time = max_times.get(self.current_state, 30.0)

       if time_in_state > max_time:
           self.get_logger().error(f"State timeout in {self.current_state.value}")
           self.handle_state_timeout()
   ```

## Debugging Tools and Techniques

### Logging and Monitoring

#### Comprehensive Logging Setup
```python
import logging
import json
from datetime import datetime

class VLAEventLogger:
    def __init__(self, log_file: str = "/tmp/vla_debug.log"):
        self.logger = logging.getLogger("VLA_Debug")
        self.logger.setLevel(logging.DEBUG)

        # Create file handler
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(logging.DEBUG)

        # Create console handler
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)

        # Create formatter
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        file_handler.setFormatter(formatter)
        console_handler.setFormatter(formatter)

        self.logger.addHandler(file_handler)
        self.logger.addHandler(console_handler)

    def log_pipeline_event(self, component: str, event: str, data: dict = None):
        log_entry = {
            'timestamp': datetime.now().isoformat(),
            'component': component,
            'event': event,
            'data': data or {}
        }
        self.logger.info(json.dumps(log_entry))

# Usage in pipeline components
event_logger = VLAEventLogger()
event_logger.log_pipeline_event(
    'voice_processor',
    'command_received',
    {'command': 'pick up the cup', 'command_id': 'cmd_123'}
)
```

#### Performance Monitoring
```python
import time
import psutil
from collections import deque

class PerformanceMonitor:
    def __init__(self):
        self.metrics = {
            'processing_times': deque(maxlen=100),
            'cpu_usage': deque(maxlen=100),
            'memory_usage': deque(maxlen=100),
            'success_rates': deque(maxlen=100)
        }

    def start_timer(self):
        return time.time()

    def end_timer(self, start_time: float, metric_name: str = 'processing_times'):
        elapsed = time.time() - start_time
        self.metrics[metric_name].append(elapsed)
        return elapsed

    def get_current_stats(self):
        return {
            'avg_processing_time': sum(self.metrics['processing_times']) / len(self.metrics['processing_times']) if self.metrics['processing_times'] else 0,
            'cpu_percent': psutil.cpu_percent(),
            'memory_percent': psutil.virtual_memory().percent,
            'success_rate': sum(self.metrics['success_rates']) / len(self.metrics['success_rates']) if self.metrics['success_rates'] else 0
        }
```

### Diagnostic Commands

#### System Health Check
```bash
#!/bin/bash
# vla_health_check.sh

echo "=== VLA System Health Check ==="

# Check ROS system
echo "ROS System Status:"
ros2 node list
echo

# Check VLA topics
echo "VLA Topics:"
ros2 topic list | grep -i vla
echo

# Check VLA services
echo "VLA Services:"
ros2 service list | grep -i vla
echo

# Check system resources
echo "System Resources:"
echo "CPU: $(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | awk -F'%' '{print $1}')%"
echo "Memory: $(free | grep Mem | awk '{printf("%.2f%%"), $3/$2 * 100.0}')"
echo

# Check API connectivity
echo "API Connectivity Tests:"
python3 -c "
import openai
try:
    client = openai.OpenAI(api_key='dummy_key')
    print('OpenAI API: Accessible (key validation would occur here)')
except ImportError:
    print('OpenAI API: Not installed')
except Exception as e:
    print(f'OpenAI API: Error - {e}')
"
```

#### Component-Specific Diagnostics
```python
class VLADiagnostics:
    def __init__(self):
        self.components = {
            'voice': self.check_voice_system,
            'planning': self.check_planning_system,
            'vision': self.check_vision_system,
            'action': self.check_action_system
        }

    def run_full_diagnostics(self) -> Dict[str, Any]:
        results = {}
        for component_name, check_func in self.components.items():
            try:
                results[component_name] = check_func()
            except Exception as e:
                results[component_name] = {
                    'status': 'error',
                    'error': str(e)
                }
        return results

    def check_voice_system(self) -> Dict[str, Any]:
        # Check voice processing system
        return {
            'status': 'ok',  # or 'error'
            'api_connected': True,
            'model_loaded': True,
            'last_error': None
        }

    def check_planning_system(self) -> Dict[str, Any]:
        # Check planning system
        return {
            'status': 'ok',
            'llm_accessible': True,
            'prompt_cache_size': 10,
            'last_plan_time': 2.5
        }

    def check_vision_system(self) -> Dict[str, Any]:
        # Check vision system
        return {
            'status': 'ok',
            'model_loaded': True,
            'camera_connected': True,
            'detections_per_second': 10
        }

    def check_action_system(self) -> Dict[str, Any]:
        # Check action execution system
        return {
            'status': 'ok',
            'navigation_available': True,
            'manipulation_available': True,
            'last_execution_time': 5.2
        }
```

## Troubleshooting Workflow

### Step-by-Step Debugging Process

1. **Identify the Problem**
   - What is the expected behavior?
   - What is the actual behavior?
   - When did the issue start occurring?

2. **Check System Status**
   ```bash
   # Run health check script
   ./vla_health_check.sh
   ```

3. **Review Logs**
   ```bash
   # Check VLA component logs
   tail -f /tmp/vla_debug.log
   ```

4. **Isolate the Issue**
   - Test individual components separately
   - Use simple test cases
   - Verify inputs and outputs at each stage

5. **Implement Fix**
   - Apply the most appropriate solution
   - Test the fix thoroughly
   - Verify no regressions were introduced

6. **Monitor After Fix**
   - Monitor system performance
   - Check for similar issues
   - Update documentation if needed

### Common Debugging Scenarios

#### Scenario 1: Voice Command Not Processed
```python
# Debugging steps:
# 1. Check if voice command is received
def voice_command_callback(self, msg: String):
    self.get_logger().info(f"Received voice command: {msg.data}")
    # Add this log to verify command reception

# 2. Check voice processing function
def process_voice_command(self, transcription: str):
    self.get_logger().info(f"Processing transcription: {transcription}")
    # Add logs to trace processing steps

# 3. Verify structured output
structured = self.structure_command(transcription)
self.get_logger().info(f"Structured command: {structured}")
```

#### Scenario 2: Robot Not Executing Actions
```python
# Debugging steps:
# 1. Check if plan is generated
def planning_result_callback(self, msg: String):
    self.get_logger().info(f"Received planning result: {msg.data}")

# 2. Check if plan is valid
plan = json.loads(msg.data)
is_valid, errors = self.validate_plan(plan)
if not is_valid:
    self.get_logger().error(f"Invalid plan: {errors}")

# 3. Check action execution
def execute_action(self, action: Dict[str, Any]):
    self.get_logger().info(f"Executing action: {action}")
    # Add success/failure logs
```

## Best Practices for Debugging VLA Systems

1. **Comprehensive Logging**: Log at each major pipeline stage with sufficient detail to trace execution flow
2. **Modular Testing**: Test components individually before integration testing
3. **Performance Monitoring**: Track processing times and resource usage to identify bottlenecks
4. **Error Recovery**: Implement graceful degradation when components fail
5. **Configuration Management**: Use configuration files for debugging settings that can be easily adjusted
6. **Version Control**: Keep track of working configurations and system states
7. **Documentation**: Maintain up-to-date documentation of debugging procedures and common solutions

## Next Steps

After mastering debugging and troubleshooting techniques, you have completed all the essential components of the VLA system. The complete Vision-Language-Action pipeline is now fully implemented and debugged. You can now build complete autonomous humanoid systems that understand voice commands and execute complex manipulation tasks.
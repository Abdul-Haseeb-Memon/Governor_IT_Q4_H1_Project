---
title: Nav2 for Humanoid Navigation
sidebar_label: Nav2 for Humanoid Navigation
description: Learn about configuring Nav2 for humanoid navigation, covering path planning concepts and navigation pipelines specifically adapted for bipedal robots rather than wheeled platforms, including how students can configure navigation pipelines that allow bipedal robots to navigate through environments using Nav2 with appropriate modifications for humanoid locomotion.
keywords: [nav2, navigation, path planning, bipedal, humanoid, robot, ros2, autonomous]
---

# Nav2 for Humanoid Navigation

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the Nav2 navigation framework and its components
- Configure Nav2 for bipedal locomotion patterns specific to humanoid robots
- Adapt path planning algorithms for humanoid robot kinematics
- Implement navigation pipelines that account for humanoid-specific constraints
- Integrate perception data with navigation for safe humanoid robot locomotion
- Validate navigation performance for bipedal robots in various environments

## Prerequisites

Before starting this chapter, you should:
- Have completed Module 1 (ROS 2 fundamentals) and Module 3, Chapters 1 and 2
- Understand basic concepts of path planning and navigation
- Have access to Nav2 navigation stack
- Possess knowledge of humanoid robot kinematics and locomotion

## Introduction to Nav2 for Humanoid Robots

Nav2 (Navigation 2) is the standard navigation framework for ROS 2, providing a flexible and extensible system for robot navigation. While originally designed for wheeled robots, Nav2 can be adapted for humanoid robots with careful consideration of the unique kinematic and dynamic constraints of bipedal locomotion.

### Key Components of Nav2

Nav2 consists of several key components:

- **Path Planner**: Generates global paths from start to goal positions
- **Controller**: Creates velocity commands to follow the global path
- **Recovery Behaviors**: Handles situations where the robot gets stuck
- **Lifecycle Manager**: Manages the state and configuration of navigation components
- **Behavior Trees**: Provides a framework for complex navigation behaviors

### Differences from Wheeled Robot Navigation

Humanoid robot navigation differs from wheeled navigation in several key ways:

- **Kinematic Constraints**: Bipedal locomotion has different mobility patterns
- **Dynamic Stability**: Requires maintaining balance during movement
- **Foot Placement**: Must consider precise footstep planning
- **Terrain Interaction**: Different ground contact patterns than wheels
- **Motion Planning**: More complex than simple differential or omnidirectional drive

## Nav2 Configuration for Humanoid Navigation

### Parameter Adaptation

Configuring Nav2 for humanoid robots requires adapting several key parameters:

- **Robot Footprint**: Accurate representation of the robot's ground contact area
- **Velocity Limits**: Appropriate values for bipedal walking speeds
- **Acceleration Limits**: Based on humanoid robot dynamics
- **Tolerance Values**: Accounting for walking gait inaccuracies
- **Costmap Parameters**: Reflecting humanoid-specific navigation constraints

### Costmap Configuration

The costmaps in Nav2 need special configuration for humanoid robots:

- **Inflation Radius**: Adjusted for humanoid body dimensions
- **Obstacle Handling**: Considering robot's ability to step over small obstacles
- **Voxel Layer**: For 3D obstacle representation relevant to humanoid height
- **Static Layer**: For map-based navigation in known environments

## Path Planning Concepts for Bipedal Robots

### Global Path Planning

Global planners in Nav2 for humanoid robots need to consider:

- **Walkable Areas**: Only planning through areas safe for bipedal locomotion
- **Terrain Slope**: Accounting for the robot's ability to handle inclines
- **Step Height**: Ensuring paths don't include steps too high for the robot
- **Surface Stability**: Avoiding unstable or slippery surfaces where possible

### Local Path Planning

Local planners must account for:

- **Dynamic Balance**: Maintaining stability during navigation
- **Footstep Planning**: Integrating with footstep planning algorithms
- **Gait Adaptation**: Adjusting walking pattern based on terrain
- **Reactive Avoidance**: Handling dynamic obstacles while maintaining balance

## Navigation Pipelines Adapted for Bipedal Locomotion

### Humanoid-Specific Navigation Architecture

A navigation pipeline for humanoid robots typically includes:

1. **Perception Integration**: Processing sensor data for environment understanding
2. **Terrain Analysis**: Identifying walkable surfaces and obstacles
3. **Footstep Planning**: Generating safe foot placement locations
4. **Path Planning**: Creating global paths suitable for bipedal locomotion
5. **Gait Control**: Managing the walking pattern to follow the path
6. **Balance Control**: Maintaining stability during navigation

### Integration with Humanoid Control Systems

The navigation system must integrate with:

- **Walking Controllers**: Low-level systems that control bipedal locomotion
- **Balance Controllers**: Systems that maintain dynamic stability
- **Footstep Planners**: Algorithms that determine safe foot placement
- **State Estimation**: Systems that track the robot's pose and state

## Navigation Pipelines Adapted for Bipedal Locomotion

### Humanoid-Specific Navigation Architecture

A navigation pipeline for humanoid robots typically includes:

1. **Perception Integration**: Processing sensor data for environment understanding
2. **Terrain Analysis**: Identifying walkable surfaces and obstacles
3. **Footstep Planning**: Generating safe foot placement locations
4. **Path Planning**: Creating global paths suitable for bipedal locomotion
5. **Gait Control**: Managing the walking pattern to follow the path
6. **Balance Control**: Maintaining stability during navigation

### Footstep Planning Integration

Footstep planning is a critical component for humanoid navigation that differs significantly from wheeled robot navigation:

- **Step Location Selection**: Algorithms that determine where each foot should be placed
- **Step Timing**: Coordinating foot placement with the robot's walking gait
- **Terrain Adaptation**: Adjusting foot placement based on ground conditions
- **Stability Constraints**: Ensuring each step maintains the robot's balance

### Gait Adaptation for Navigation

Humanoid robots must adapt their walking patterns during navigation:

- **Turning Gait**: Specialized walking patterns for changing direction
- **Obstacle Avoidance Gait**: Modified steps to navigate around obstacles
- **Slope Walking**: Adjusted gait patterns for inclines and declines
- **Step Climbing**: Specialized motions for navigating small steps or curbs

### Balance Control During Navigation

Maintaining balance during navigation involves:

- **Zero Moment Point (ZMP) Control**: Keeping the robot's center of pressure within safe limits
- **Center of Mass (CoM) Control**: Managing the robot's center of mass during movement
- **Reactive Balance**: Adjusting to unexpected disturbances during navigation
- **Predictive Balance**: Anticipating balance challenges based on planned paths

## Examples of Nav2 Configurations

### Basic Humanoid Navigation Configuration

```yaml
# Example Nav2 configuration for humanoid robot
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    default_nav_through_poses_bt_xml: /opt/nav2/share/nav2_bt_navigator/behavior_trees/navigate_w_replanning_and_recovery.xml
    default_nav_to_pose_bt_xml: /opt/nav2/share/nav2_bt_navigator/behavior_trees/navigate_w_replanning_and_recovery.xml
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_smooth_path_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_assisted_teleop_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_drive_on_heading_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_is_path_valid_condition_bt_node
    - nav2_is_goal_reached_condition_bt_node
    - nav2_is_path_blocked_condition_bt_node
    - nav2_are_error_codes_active_condition_bt_node
    - nav2_would_a_controller_recovery_help_condition_bt_node
    - nav2_would_a_planner_recovery_help_condition_bt_node
    - nav2_would_a_smoother_recovery_help_condition_bt_node
    - nav2_costmap_filter_info_service_bt_node
    - nav2_initial_pose_publisher_node_bt_node
    - nav2_speed_controller_node_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_truncate_path_local_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transformer_node_bt_node
    - nav2_get_path_through_poses_node_bt_node
    - nav2_get_path_to_pose_node_bt_node
    - nav2_get_local_plan_node_bt_node
    - nav2_rotate_action_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid-specific parameters
    FollowPath:
      plugin: "nav2_mppi_controller::MppiController"
      time_steps: 20
      control_freq: 10
      horizon_dt: 1.0
      xy_goal_tolerance: 0.3  # Larger tolerance for humanoid walking
      yaw_goal_tolerance: 0.3
      stateful: True
      critics: ["BaseObstacleCritic", "GoalCritic", "PathAlignCritic", "PathFollowCritic", "PreferForwardCritic"]

      BaseObstacleCritic:
        plugin: "nav2_critic_functions::BaseObstacleCritic"
        threshold_to_consider: 0.1
        obstacle_expansion_distance: 0.5  # Larger for humanoid safety
        cost_scaling_factor: 3.0

      GoalCritic:
        plugin: "nav2_critic_functions::GoalCritic"
        goal_dist_gain: 3.0

      PathAlignCritic:
        plugin: "nav2_critic_functions::PathAlignCritic"
        cost_power: 4
        alignment_dist: 0.5

      PathFollowCritic:
        plugin: "nav2_critic_functions::PathFollowCritic"
        cost_power: 2
        look_ahead_distance: 0.5

      PreferForwardCritic:
        plugin: "nav2_critic_functions::PreferForwardCritic"
        penalty_angle: 1.0
        cost_power: 2

local_costmap:
  ros__parameters:
    update_frequency: 5.0
    publish_frequency: 2.0
    global_frame: odom
    robot_base_frame: base_link
    use_sim_time: True
    rolling_window: true
    width: 10
    height: 10
    resolution: 0.05
    robot_radius: 0.3  # Adjusted for humanoid size
    plugins: ["voxel_layer", "inflation_layer"]

    voxel_layer:
      plugin: "nav2_costmap_2d::VoxelLayer"
      enabled: True
      voxel_size: 0.1
      max_voxels: 10000
      mark_threshold: 0
      observation_sources: scan
      scan:
        topic: /scan
        max_obstacle_height: 2.0
        clearing: True
        marking: True
        data_type: "LaserScan"
        raytrace_max_range: 10.0
        raytrace_min_range: 0.0
        obstacle_max_range: 5.0
        obstacle_min_range: 0.0

    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      cost_scaling_factor: 3.0
      inflation_radius: 1.0  # Larger for humanoid safety

global_costmap:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    use_sim_time: True
    update_frequency: 1.0
    publish_frequency: 1.0
    resolution: 0.05
    robot_radius: 0.3  # Adjusted for humanoid size
    plugins: ["static_layer", "obstacle_layer", "voxel_layer", "inflation_layer"]

    static_layer:
      plugin: "nav2_costmap_2d::StaticLayer"
      map_subscribe_transient_local: True

    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      enabled: True
      observation_sources: scan
      scan:
        topic: /scan
        max_obstacle_height: 2.0
        clearing: True
        marking: True
        data_type: "LaserScan"
        raytrace_max_range: 10.0
        raytrace_min_range: 0.0
        obstacle_max_range: 5.0
        obstacle_min_range: 0.0

    voxel_layer:
      plugin: "nav2_costmap_2d::VoxelLayer"
      enabled: True
      voxel_size: 0.1
      max_voxels: 10000
      mark_threshold: 0
      observation_sources: scan
      scan:
        topic: /scan
        max_obstacle_height: 2.0
        clearing: True
        marking: True
        data_type: "LaserScan"
        raytrace_max_range: 10.0
        raytrace_min_range: 0.0
        obstacle_max_range: 5.0
        obstacle_min_range: 0.0

    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      cost_scaling_factor: 3.0
      inflation_radius: 1.0  # Larger for humanoid safety

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5  # Larger tolerance for humanoid navigation
      use_astar: false
      allow_unknown: true

smoother_server:
  ros__parameters:
    use_sim_time: True
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: True
```

### Advanced Configuration with Footstep Planning Integration

```yaml
# Advanced Nav2 configuration with footstep planning for humanoid robots
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 5
    default_server_timeout: 30
    enable_groot_monitoring: True
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_is_path_valid_condition_bt_node
    - nav2_is_goal_reached_condition_bt_node
    - nav2_is_path_blocked_condition_bt_node
    - nav2_footstep_planner_condition_bt_node  # Custom footstep planning node
    - nav2_footstep_executor_action_bt_node    # Custom footstep execution node

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 10.0  # Lower frequency for more stable humanoid control
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FootstepController"]

    FootstepController:
      plugin: "nav2_footstep_controller::FootstepController"
      max_linear_speed: 0.2      # Slower speed for humanoid stability
      max_angular_speed: 0.3
      step_duration: 0.8         # Time per step
      step_height: 0.05          # Height of each step
      step_offset_x: 0.2         # Forward step distance
      step_offset_y: 0.1         # Lateral step distance
      step_tolerance: 0.05       # Tolerance for step placement

local_costmap:
  ros__parameters:
    update_frequency: 10.0
    publish_frequency: 5.0
    global_frame: odom
    robot_base_frame: base_link
    use_sim_time: True
    rolling_window: true
    width: 20
    height: 20
    resolution: 0.025  # Higher resolution for precise footstep planning
    robot_radius: 0.4  # Larger for humanoid safety margin
    plugins: ["voxel_layer", "inflation_layer", "footstep_layer"]  # Custom footstep layer

    voxel_layer:
      plugin: "nav2_costmap_2d::VoxelLayer"
      enabled: True
      voxel_size: 0.05
      max_voxels: 50000
      mark_threshold: 0
      observation_sources: scan
      scan:
        topic: /scan
        max_obstacle_height: 2.0
        clearing: True
        marking: True
        data_type: "LaserScan"
        raytrace_max_range: 15.0
        raytrace_min_range: 0.0
        obstacle_max_range: 8.0
        obstacle_min_range: 0.0

    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      cost_scaling_factor: 5.0  # Higher for humanoid safety
      inflation_radius: 1.5

    footstep_layer:
      plugin: "nav2_footstep_costmap::FootstepLayer"  # Custom plugin for footstep planning
      traversability_threshold: 0.7
      step_cost_penalty: 0.8

global_costmap:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    use_sim_time: True
    update_frequency: 2.0
    publish_frequency: 1.0
    resolution: 0.05
    robot_radius: 0.4
    plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

    static_layer:
      plugin: "nav2_costmap_2d::StaticLayer"
      map_subscribe_transient_local: True

    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      enabled: True
      observation_sources: scan
      scan:
        topic: /scan
        max_obstacle_height: 2.0
        clearing: True
        marking: True
        data_type: "LaserScan"
        raytrace_max_range: 15.0
        raytrace_min_range: 0.0
        obstacle_max_range: 8.0
        obstacle_min_range: 0.0

    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      cost_scaling_factor: 5.0
      inflation_radius: 1.5

planner_server:
  ros__parameters:
    expected_planner_frequency: 10.0
    use_sim_time: True
    planner_plugins: ["GridBased", "FootstepPlanner"]

    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.8
      use_astar: true
      allow_unknown: false

    FootstepPlanner:
      plugin: "nav2_footstep_planner::FootstepPlanner"  # Custom footstep planner
      max_step_width: 0.3
      max_step_height: 0.1
      max_step_length: 0.4
      step_spacing: 0.25
      safety_margin: 0.1

velocity_smoother:
  ros__parameters:
    use_sim_time: True
    smoothing_frequency: 20.0
    scale_velocities: False
    feedback: "OPEN_LOOP"
    velocity_timeout: 1.0
    max_velocity: [0.5, 0.0, 1.0]  # [x, y, theta] max velocities
    min_velocity: [-0.5, 0.0, -1.0]
    velocity_scaling_tolerance: 0.1
    velocity_threshold: 0.0

## Exercises

1. **Basic Navigation Setup**: Configure Nav2 for a humanoid robot simulation in Isaac Sim with the basic configuration parameters. Test the navigation system by setting a goal position and observing the robot's path planning and execution.

2. **Footstep Planning Integration**: Implement a footstep planning module that integrates with Nav2. Configure the costmap parameters to account for the robot's foot placement constraints and test navigation in an environment with obstacles that require careful footstep selection.

3. **Balance-Aware Navigation**: Modify the Nav2 configuration to include balance constraints. Adjust the controller parameters to ensure the humanoid robot maintains stability during navigation, especially during turns and obstacle avoidance maneuvers.

4. **Terrain Adaptation**: Configure Nav2 to handle different terrain types (flat ground, ramps, stairs). Test the navigation system's ability to adapt the robot's gait and footstep planning based on terrain analysis.

5. **Performance Optimization**: Measure and optimize the performance of the navigation pipeline by adjusting parameters such as costmap resolution, planner frequency, and controller update rates. Document the trade-offs between performance and navigation accuracy.

## Summary

Nav2 provides a comprehensive navigation framework that can be adapted for humanoid robots with careful consideration of their unique kinematic and dynamic constraints. By configuring appropriate parameters for footstep planning, balance control, and gait adaptation, humanoid robots can achieve autonomous navigation capabilities similar to wheeled robots while accounting for the complexities of bipedal locomotion. The integration of Nav2 with Isaac Sim and Isaac ROS creates a complete pipeline for developing, testing, and deploying navigation systems for humanoid robots in both simulation and real-world environments.

## Next Steps

Continue to the previous chapter to review perception concepts: [Isaac ROS Perception](./02-isaac-ros.md)

Return to the Module 3 overview to see all chapters in this module.
---
sidebar_position: 4
title: "4. NAV2 for Humanoid Path Planning"
---

# 4. NAV2 for Humanoid Path Planning

## Explanation: Adapting a Robust Navigation Stack for Bipedal Movement

**NAV2** is the second generation of the ROS Navigation Stack, providing a complete framework for autonomous mobile robot navigation within ROS 2. It is a highly modular and configurable system built on Behavior Trees, offering capabilities for global planning, local planning (controller), recovery behaviors, and costmap management. While NAV2 has proven extremely successful for wheeled and tracked robots, adapting it for **bipedal movement** in humanoids presents unique and significant challenges due to the humanoid's complex dynamics, non-holonomic constraints, and high degrees of freedom.

### Understanding the NAV2 Stack

The NAV2 stack is composed of several interoperable components that work together to enable a robot to navigate from a starting pose to a goal pose while avoiding obstacles:

*   **Behavior Tree (BT) Navigator**: The top-level orchestrator. It uses Behavior Trees to define complex navigation logic, including sequence of operations (e.g., plan, control, recover), parallel execution, and conditional branches. This makes NAV2 highly flexible and robust to unexpected situations.
*   **Global Planner**: Responsible for computing a long-range, collision-free path from the robot's current position to the goal. It operates on a global costmap and typically uses search algorithms like A* or Theta*.
*   **Local Planner (Controller)**: Responsible for generating short-term velocity commands (or equivalent) to follow the global path and perform immediate obstacle avoidance. It operates on a local costmap, which is a continuously updated, smaller representation of the robot's immediate surroundings. Popular local planners include DWA (Dynamic Window Approach) and TEB (Timed Elastic Band).
*   **Costmaps**: Grid-based representations of the environment that encode information about obstacles, inflation layers (representing robot size), and other costs. There are typically two costmaps: a static **global costmap** (for long-term planning) and a dynamic **local costmap** (for immediate obstacle avoidance).
*   **Recovery Behaviors**: Modules that trigger when the robot gets stuck or encounters an unplannable situation (e.g., clearing the local costmap, rotating in place to find a new path).
*   **Way-point Follower**: Enables the robot to visit a series of predefined locations.

### Challenges for Bipedal Movement

NAV2's default configurations and many of its standard plugins are optimized for differential drive or omnidirectional wheeled robots. Humanoids, however, introduce several complexities:

1.  **Non-Holonomic and Underactuated Nature**: Humanoids cannot move sideways easily without complex whole-body motions or foot placements. Their stability is highly dynamic. Standard local planners often assume holonomic motion or simpler non-holonomic constraints that don't apply to a walking robot.
2.  **Dynamic Footprint and Center of Mass (CoM)**: A wheeled robot has a relatively stable, static footprint. A humanoid's support polygon (area defined by feet on the ground) changes with every step, and its CoM dynamically shifts to maintain balance. Traditional 2D costmaps struggle to represent this dynamic stability.
3.  **Footstep Planning**: Instead of continuous velocity commands, bipedal locomotion requires discrete footstep plans (where to place the next foot) that are kinematically and dynamically feasible.
4.  **3D Environments and Terrain**: Humanoids can potentially navigate complex 3D terrain like stairs, uneven surfaces, and slopes. Standard NAV2 planners are primarily 2D or 2.5D, lacking true 3D planning capabilities for such environments.
5.  **High DoF and Balance Constraints**: Planning for a high-DoF humanoid must simultaneously consider balance, joint limits, collision avoidance, and the robot's overall momentum, making the state space for planning enormous.

### Footstep Planning

Given these challenges, humanoid navigation often relies on specialized **footstep planning**. This involves algorithms that generate a sequence of valid foot placements from the robot's current state to a target location, considering the robot's balance (e.g., Zero Moment Point - ZMP), kinematics, and the terrain. These footstep plans are then executed by a whole-body controller. Adapting NAV2 for humanoids typically means replacing or augmenting its standard global and local planners with modules that can interface with such footstep planners.

## Code Examples

Adapting NAV2 for humanoids usually involves custom plugins for global and local planners. Here, we'll show a conceptual outline of how a custom local planner might be structured to output footstep commands instead of `Twist` messages.

**Conceptual Custom Bipedal Local Planner (Python/C++ pseudocode)**

This would be a custom plugin loaded by NAV2's controller server.

```python
# bipedal_local_planner.py (Conceptual ROS 2 Nav2 Controller Plugin)

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path # Global path input
from geometry_msgs.msg import PoseStamped # Robot's current pose
from humanoid_msgs.msg import FootstepArray # Custom message for footstep commands
from nav2_core.controller import Controller # Base class for Nav2 controllers (conceptual)

class BipedalLocalPlanner(Controller): # Inherits from Nav2's controller interface
    def __init__(self):
        super().__init__()
        self.node = None # Will be set by Nav2 framework
        self.footstep_publisher = None
        self.footstep_sequence = []
        self.current_footstep_idx = 0

    def configure(self, node: Node, name: str, tf_buffer, costmap_ros):
        self.node = node
        self.node.get_logger().info(f"Configuring BipedalLocalPlanner: {name}")
        self.footstep_publisher = self.node.create_publisher(FootstepArray, 'humanoid/footstep_commands', 10)
        # Load parameters specific to bipedal walking (e.g., step length, step height)

    def activate(self):
        self.node.get_logger().info("BipedalLocalPlanner activated.")
        self.footstep_sequence = []
        self.current_footstep_idx = 0

    def deactivate(self):
        self.node.get_logger().info("BipedalLocalPlanner deactivated.")
        self.footstep_sequence = []

    def compute_and_publish_footsteps(self, global_plan: Path, robot_pose: PoseStamped):
        """
        Main logic: Generate or update footstep sequence based on global plan and current pose.
        This is where complex bipedal motion planning would occur.
        """
        if not self.footstep_sequence:
            # --- Conceptual Footstep Generation ---
            # In reality, this calls a dedicated footstep planner library/service
            self.node.get_logger().info("Generating new footstep sequence...")
            # Example: Plan 3 steps ahead
            # footstep_planner.plan(robot_pose, global_plan.poses[min(len(global_plan.poses)-1, 5)].pose)
            
            # Dummy footsteps for illustration
            footsteps = FootstepArray()
            footsteps.header.stamp = self.node.get_clock().now().to_msg()
            footsteps.header.frame_id = "odom"
            
            # Example: Forward steps
            step1 = Footstep() # Custom Footstep message
            step1.id = 0
            step1.foot_pose.position.x = robot_pose.pose.position.x + 0.2
            step1.foot_pose.position.y = robot_pose.pose.position.y + 0.1 # Right foot
            step1.leg = Footstep.RIGHT
            
            step2 = Footstep()
            step2.id = 1
            step2.foot_pose.position.x = robot_pose.pose.position.x + 0.4
            step2.foot_pose.position.y = robot_pose.pose.position.y - 0.1 # Left foot
            step2.leg = Footstep.LEFT

            footsteps.footsteps = [step1, step2]
            self.footstep_sequence = footsteps.footsteps
        
        # Publish the next one or two footsteps from the sequence
        if self.footstep_sequence:
            # Publish only the next few footsteps that the whole-body controller needs
            footsteps_to_publish = FootstepArray()
            footsteps_to_publish.footsteps = self.footstep_sequence[self.current_footstep_idx : self.current_footstep_idx + 2]
            self.footstep_publisher.publish(footsteps_to_publish)
            self.node.get_logger().info(f"Published {len(footsteps_to_publish.footsteps)} footsteps.")

    def compute_velocity_commands(self, robot_pose: PoseStamped, robot_speed: float):
        """
        NAV2 expects this function to return Twist, but for bipedal, it's about
        managing footstep execution.
        """
        # This method is primarily for wheeled robots. For humanoids,
        # we'd focus on generating footstep plans and managing their execution
        # based on global plan and robot's actual state.
        
        # We assume an external whole-body controller consumes `humanoid/footstep_commands`
        # and returns continuous odometry.
        
        # Here, we'd check current footstep execution status and update `current_footstep_idx`
        # and potentially call `compute_and_publish_footsteps` if more steps are needed.
        
        # For simplicity, if we don't have new footsteps, just stand still (return zero twist)
        from geometry_msgs.msg import Twist
        return Twist()

    def set_plan(self, path: Path):
        self.node.get_logger().info("Received new global plan. Will re-plan footsteps.")
        # Trigger footstep re-planning based on new global path
        self.footstep_sequence = [] # Clear old sequence
        self.current_footstep_idx = 0
        
    def is_goal_reached(self, robot_pose: PoseStamped, goal_pose: PoseStamped, threshold: float):
        # Check if the robot is close enough to the goal (using its current pose)
        # This might be tricky for humanoids as the exact "robot pose" might be dynamic (CoM, base_link)
        distance = ((robot_pose.pose.position.x - goal_pose.pose.position.x)**2 +
                    (robot_pose.pose.position.y - goal_pose.pose.position.y)**2)**0.5
        return distance < threshold
```

## Diagrams (in Markdown)

### Humanoid NAV2 Adaptation for Bipedal Movement

```mermaid
graph TD
    A[Global Goal (NAV2)] --> B{Behavior Tree Navigator};
    B --> C[Global Planner (e.g., A* on 2D/3D map)];
    C --> D[Global Path (Sequence of Poses)];
    D --> E[Custom Bipedal Local Planner (Plugin)];
    E --> F[Footstep Planner];
    F --> G[Footstep Sequence (Custom Msg)];
    G --> H[Whole-Body Controller (ROS 2 Node)];
    H --> I[Robot Actuators (Joint Commands)];
    I --> J[Humanoid Robot];
    J --> K[Odom/IMU (Current Pose)];
    K --> E;
    K --> B;
    L[Sensor Data (LiDAR, Depth)] --> M[Costmap Filter/Layer];
    M --> N[Costmaps (Global & Local)];
    N --> C;
    N --> E;
```
*Figure 3.4: Humanoid NAV2 Adaptation for Bipedal Movement. NAV2's Behavior Tree orchestrates global planning, generating a path. A custom bipedal local planner translates this into a footstep sequence, which is executed by a whole-body controller. Sensor data feeds into costmaps for obstacle avoidance. This diagram highlights the custom components needed to bridge NAV2's wheeled robot focus to humanoid locomotion.*

## Tables

| NAV2 Component        | Standard Function (Wheeled)                       | Adaptation for Humanoids                                   |
|-----------------------|---------------------------------------------------|------------------------------------------------------------|
| **Global Planner**    | A*/Theta* on 2D costmap.                          | A*/Theta* on 2D/2.5D costmap; Potentially 3D path planning. |
| **Local Planner**     | DWA/TEB: Generates continuous `Twist` commands.   | **Custom Plugin**: Generates `FootstepArray` or CoM trajectories. |
| **Costmaps**          | 2D grid, inflates obstacles, represents robot as circle/box. | **Custom Layers/3D Costmap**: Accounts for dynamic footprint, body sway, 3D terrain. |
| **Controller Server** | Executes local planner's `Twist` commands.        | Passes footstep commands to specialized Whole-Body Controller. |
| **Recovery Behaviors**| Spinning, clearing costmap.                       | **Custom Behaviors**: Re-balance, specific fall recovery maneuvers. |

## Callouts

:::tip
**Modular Design of NAV2**: Leverage NAV2's plugin architecture. Instead of rewriting the entire stack, focus on creating custom plugins for the global and local planners that specifically handle bipedal locomotion logic.
:::

:::warning
**Computational Load**: Bipedal locomotion planning is computationally intensive. Ensure your footstep planner and whole-body controller can meet the real-time constraints. Consider pre-computation, hierarchical planning, or GPU acceleration.
:::

## Bipedal Navigation

Bipedal navigation in humanoids is fundamentally different from wheeled robot navigation. Instead of continuous motion, it involves a sequence of discrete events: foot placements. The challenge is to generate these footsteps such that the robot remains dynamically stable, avoids collisions, and moves towards its goal.

### Footstep Planning

Footstep planning algorithms calculate a feasible sequence of foot contacts on the terrain. These algorithms must consider:

1.  **Kinematic Feasibility**: Can the robot's legs physically reach the desired foot placement? Are joint limits respected?
2.  **Dynamic Stability**: Does the planned footstep sequence maintain the robot's balance? Concepts like the **Zero Moment Point (ZMP)** and **Center of Mass (CoM)** trajectory are central here. The ZMP must remain within the support polygon (the convex hull of the contact points of the feet with the ground) to prevent tipping.
3.  **Collision Avoidance**: Are the planned footsteps and the robot's body free from collisions with environmental obstacles?
4.  **Terrain Adaptability**: Can the robot step over small obstacles, climb stairs, or navigate uneven ground?

### Obstacle Avoidance

While footstep planning handles large-scale obstacle negotiation, immediate obstacle avoidance (e.g., an unexpected moving object) requires a reactive layer. In NAV2's context, the local planner, augmented for bipedal motion, would need to:

*   Receive real-time sensor data (LiDAR, depth cameras) from costmaps.
*   Adjust footstep plans or even generate emergency evasive steps to avoid collisions.
*   This often involves tight integration with the whole-body controller to ensure a stable, reactive response.

## Step-by-step Guides

**Conceptual Adaptation of NAV2 for a Humanoid:**

1.  **Develop a Humanoid-Specific `robot_description`**: Ensure your URDF/XACRO defines the robot's collision models accurately for costmap generation.
2.  **Integrate Odometry**: Provide reliable odometry data for your humanoid (e.g., fused from IMU and joint encoders) to `/odom`. This is crucial for NAV2's localization.
3.  **Customize Global Planner (if 3D terrain)**: If navigating 3D terrain, you might need a custom global planner that operates on a 3D occupancy grid or a graph representing traversable surfaces, instead of a flat 2D map.
4.  **Develop a Custom Local Planner Plugin**: This is the most critical step. Create a C++ or Python plugin for NAV2's controller server that:
    *   Subscribes to the global plan and robot pose.
    *   Interfaces with an external (or internal) **footstep planner**.
    *   Generates `humanoid_msgs/FootstepArray` messages.
    *   Publishes these footstep commands to a dedicated topic for a whole-body controller.
    *   Instead of `Twist` commands, it would provide feedback on footstep execution progress.
5.  **Develop a Custom Whole-Body Controller**: This controller (often external to NAV2) subscribes to the footstep commands from your custom local planner and executes them on the physical robot while maintaining balance.
6.  **Customize Costmap Layers**: Develop custom costmap plugins that account for the humanoid's dynamic footprint, body sway, and potential 3D obstacles.
7.  **Behavior Tree Adaptation**: Design custom NAV2 Behavior Tree nodes that invoke your bipedal local planner and handle bipedal-specific recovery behaviors (e.g., `balance_recovery`).
8.  **Test in Simulation**: Rigorously test your adapted NAV2 stack in Isaac Sim or Gazebo, simulating various terrains and dynamic obstacles.

## Summary

NAV2, while a powerful navigation framework for mobile robots, requires significant adaptation to handle the complexities of bipedal movement in humanoids. This chapter detailed the core components of NAV2 and highlighted the challenges posed by a humanoid's dynamic footprint, non-holonomic constraints, and need for footstep planning. We explored how custom local planner plugins, specialized footstep planners, and adapted costmap layers are essential to bridge this gap. By thoughtfully integrating these humanoid-specific modules with NAV2's robust framework, we can enable bipedal robots to autonomously navigate complex environments, moving beyond simple wheeled locomotion to truly agile and versatile movement.

## Exercises

1.  **Footstep Planner Algorithm**: Research and briefly describe one well-known footstep planning algorithm used in humanoid robotics (e.g., model predictive control-based, RRT-based). How does it address kinematic feasibility and dynamic stability?
2.  **ZMP and Bipedal Stability**: Explain the Zero Moment Point (ZMP) concept. Why is it a crucial constraint for footstep planning and dynamic balance in humanoid robots? How would a ZMP violation manifest in a simulated humanoid?
3.  **NAV2 Custom Local Planner Requirements**: Beyond just outputting footstep commands, what additional information (e.g., current robot balance state, upcoming terrain information) would your custom bipedal local planner need to subscribe to in order to make robust decisions?
4.  **3D Costmap Alternatives**: If NAV2's 2D costmaps are insufficient for 3D humanoid navigation, research an alternative 3D environment representation (e.g., Octomap, voxel grid) that could be used by a custom NAV2 global planner for humanoids.
5.  **Recovery Behaviors for Humanoids**: Propose two new recovery behaviors for a humanoid robot that gets stuck or loses balance during navigation, which would be integrated into a customized NAV2 Behavior Tree. How would these differ from typical wheeled robot recovery behaviors?


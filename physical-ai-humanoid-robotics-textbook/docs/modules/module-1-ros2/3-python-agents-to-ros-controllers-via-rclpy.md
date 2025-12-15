---
sidebar_position: 3
title: "3. Python Agents to ROS Controllers via rclpy"
---

# 3. Python Agents to ROS Controllers via rclpy

## Explanation: Bridging High-Level Intelligence to Low-Level Actuation

The paradigm of AI-powered multi-agent systems often involves high-level cognitive processes (e.g., planning, decision-making, learning) interacting with low-level robotic hardware control. Python, with its rich ecosystem of AI/ML libraries, is a language of choice for developing these intelligent agents. However, robotic hardware, especially the precise, real-time control of joints and motors in a humanoid, typically relies on dedicated low-level controllers, often implemented in C++ or specialized firmware. **`rclpy`**, the Python client library for ROS 2, acts as the crucial bridge, enabling Python-based AI agents to seamlessly command and receive feedback from ROS 2 controllers.

### `rclpy`: The Python Interface to ROS 2

`rclpy` provides a Pythonic interface to the ROS 2 client library (rcl), allowing developers to write ROS 2 nodes, publishers, subscribers, service clients, and service servers entirely in Python. It leverages the underlying `rcl` C library and the DDS middleware, ensuring interoperability with C++ ROS 2 nodes and maintaining the performance characteristics of ROS 2.

The core functionalities `rclpy` exposes are:

*   **Node Creation**: Every ROS 2 application starts with a `Node` instance. `rclpy.node.Node` provides the base class for your Python node, giving it a name and access to ROS 2 functionalities like logging, parameter management, and communication primitives.
*   **Publishers**: Used by agents to send commands or state information to ROS 2 topics. For example, a Python-based gait planner might publish desired joint positions to a `joint_group_commands` topic.
*   **Subscribers**: Used by agents to receive information from ROS 2 topics. A perception agent, for instance, might subscribe to camera image topics or IMU data topics.
*   **Services and Actions**: `rclpy` also supports creating service clients/servers and action clients/servers, allowing Python agents to engage in synchronous request-reply or long-running goal-oriented tasks.

### Integrating Python Agents with ROS Controllers

The typical workflow for a Python-based AI agent controlling a humanoid robot via ROS 2 involves:

1.  **Perception Data Acquisition**: The Python agent subscribes to various ROS 2 topics (e.g., `sensor_msgs/JointState` for joint encoder readings, `sensor_msgs/Imu` for orientation, `sensor_msgs/Image` for camera feeds) to gather its current observations of the robot's state and environment.
2.  **Cognition and Decision-Making**: The Python agent processes these observations using its internal AI logic (e.g., reinforcement learning policy, planning algorithm, LLM-based reasoning). Based on its goals and the current state, it computes a desired high-level action or a set of low-level control targets.
3.  **Command Generation**: The agent translates its computed actions into appropriate ROS 2 message types (e.g., `std_msgs/Float64MultiArray` for desired joint positions, `geometry_msgs/Twist` for base velocity commands, or custom message types for specific humanoid gaits).
4.  **Action Execution**: The agent publishes these command messages to specific ROS 2 topics that are monitored by the robot's low-level controllers. These controllers (often implemented in C++ and part of the `ros2_control` framework) then interpret these commands and directly actuate the physical hardware.

This clear separation allows Python to handle the complex, data-intensive AI algorithms, while ROS 2's `ros2_control` ecosystem (written in C++) manages the deterministic, high-frequency control loops necessary for stable and dynamic bipedal movement.

## Code Examples

### Python Agent Publishing Joint Commands to ROS 2

This example demonstrates a simplified Python agent that acts as a `JointCommander` node. It periodically publishes desired joint positions to a ROS 2 topic, simulating a high-level agent instructing a robot to move its joints.

**1. Define a custom message type (if `std_msgs/Float64MultiArray` isn't sufficient).**
For this example, we'll use `std_msgs/Float64MultiArray` for simplicity.
If you were defining a custom message, it would be in `my_custom_msgs/msg/JointCommand.msg`:
```
string[] joint_names
float64[] positions
float64[] velocities
float64[] efforts
```
And then used in Python as `from my_custom_msgs.msg import JointCommand`.



## Diagrams (in Markdown)

### Python Agent to ROS Controller Bridge

```mermaid
graph TD
    subgraph Python AI Agent (rclpy Node)
        P[Perception (Subscribes to /joint_states, /imu/data)]
        D{Decision/Planning (AI Logic)}
        A[Action Generation (Publishes to /joint_group_commands)]
    end

    subgraph ROS 2 Middleware (DDS)
        T_States(Topic: /joint_states)
        T_Commands(Topic: /joint_group_commands)
    end

    subgraph Low-Level Controller (C++ ros2_control)
        C_Sub[Subscriber to /joint_group_commands]
        C_Exec[Execute Hardware Control]
        C_Pub[Publisher of /joint_states]
    end

    RobotHardware[Robot Hardware (Joints, Motors)]

    T_States --> P;
    P --> D;
    D --> A;
    A --> T_Commands;
    T_Commands --> C_Sub;
    C_Sub --> C_Exec;
    C_Exec --> RobotHardware;
    RobotHardware --> C_Pub;
    C_Pub --> T_States;

    style P fill:#add8e6,stroke:#333,stroke-width:1px;
    style D fill:#ffc,stroke:#333,stroke-width:1px;
    style A fill:#a2e0ff,stroke:#333,stroke-width:1px;
    style C_Sub fill:#cfc,stroke:#333,stroke-width:1px;
    style C_Exec fill:#dee,stroke:#333,stroke-width:1px;
    style C_Pub fill:#cfc,stroke:#333,stroke-width:1px;
    style T_States fill:#90ee90,stroke:#333,stroke-width:1px;
    style T_Commands fill:#90ee90,stroke:#333,stroke-width:1px;
    style RobotHardware fill:#f9f,stroke:#333,stroke-width:2px;
```
*Figure 1.3: Python Agent to ROS Controller Bridge. A high-level Python AI agent, implemented as an `rclpy` node, perceives robot state via ROS 2 topics, processes it with its internal logic, and publishes commands back to ROS 2 topics. These commands are then executed by low-level `ros2_control` components that interface directly with the robot's hardware.*

## Tables

| Component                     | Description                                                   | `rclpy` Role                    | Humanoid Application                                          |
|-------------------------------|---------------------------------------------------------------|---------------------------------|---------------------------------------------------------------|
| **Python Agent (High-Level)** | Implements AI/ML algorithms, planning, decision-making.       | ROS 2 Node (using `rclpy`)      | Gait planning, gesture recognition, natural language understanding. |
| **`rclpy`**                   | Python client library for ROS 2.                              | Interface to ROS 2 primitives.  | Enables Python agents to publish/subscribe/service/action with ROS 2. |
| **ROS 2 Topics**              | Asynchronous data streams.                                    | `create_publisher`, `create_subscription` | Real-time joint states, IMU data, camera feeds, command publication. |
| **ROS 2 Services/Actions**    | Synchronous RPC / Goal-oriented tasks.                        | `create_client`, `create_server` | Complex queries (e.g., inverse kinematics), long-term navigation goals. |
| **ROS Controllers (Low-Level)** | Manages physical hardware, often in C++ with `ros2_control`. | Interacts with ROS 2 topics/services. | Direct motor control, sensor data acquisition, safety limits. |

## Callouts

:::tip
**Asynchronous Programming**: For agents performing complex computations or interacting with external APIs, consider using Python's `asyncio` with `rclpy` to prevent blocking the ROS 2 event loop. This ensures the agent remains responsive to new percepts and commands.
:::

:::warning
**Real-time vs. Python**: While `rclpy` is performant, Python's Global Interpreter Lock (GIL) can limit true parallelism for CPU-bound tasks. For hard real-time control loops (e.g., less than 10ms), C++ `rclcpp` or dedicated microcontrollers are generally preferred, with Python agents providing higher-level supervision.
:::

## Step-by-step Guides

**Creating a Python Agent Node with `rclpy`:**

1.  **Set up ROS 2 Environment**: Ensure ROS 2 (e.g., Humble, Iron) is installed and sourced.
2.  **Create a ROS 2 Package**:
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_python my_humanoid_agent_pkg --dependencies rclpy std_msgs sensor_msgs
    # Add other message types like geometry_msgs, custom messages as needed
    ```
3.  **Define Node Class**: In your package's `my_humanoid_agent_pkg/my_humanoid_agent_pkg/joint_commander_agent.py` file, create a class inheriting from `rclpy.node.Node`.
4.  **Implement `__init__`**:
    *   Call `super().__init__('your_node_name')`.
    *   Create publishers using `self.create_publisher(MessageType, 'topic_name', qos_profile)`.
    *   Create subscribers using `self.create_subscription(MessageType, 'topic_name', self.callback_function, qos_profile)`.
    *   Set up timers for periodic tasks using `self.create_timer(period_seconds, self.callback_function)`.
5.  **Implement Callbacks**: Write methods for subscribers and timers to handle incoming messages and periodic logic.
6.  **`main()` Function**: Include the standard `rclpy.init()`, node creation, `rclpy.spin()`, `node.destroy_node()`, and `rclpy.shutdown()` boilerplate.
7.  **Update `setup.py`**: Add an entry point for your node.
    ```python
    # setup.py
    # ...
    entry_points={
        'console_scripts': [
            'joint_commander_agent = my_humanoid_agent_pkg.joint_commander_agent:main',
        ],
    },
    # ...
    ```
8.  **Build and Source**:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_humanoid_agent_pkg
    source install/setup.bash
    ```
9.  **Run Your Agent**:
    ```bash
    ros2 run my_humanoid_agent_pkg joint_commander_agent
    ```

## Summary

This chapter has elucidated the crucial role of `rclpy` in empowering Python-based AI agents to interact with ROS 2 controllers, forming the bridge between high-level intelligence and low-level hardware actuation. We explored how `rclpy` provides Pythonic access to ROS 2's fundamental communication primitives—nodes, topics, services, and actions—enabling agents to perceive robot state, execute cognitive processes, generate commands, and publish them to controllers. The detailed code example demonstrated a Python agent publishing joint commands and subscribing to joint states, showcasing the practical implementation of this vital connection. This integration is essential for harnessing Python's AI/ML capabilities while leveraging ROS 2's robust framework for real-time robotic control, particularly for complex humanoid systems.

## Exercises

1.  **Sensor Feedback Agent**: Create a Python ROS 2 node using `rclpy` that subscribes to the `/imu/data` topic (of type `sensor_msgs/Imu`). The agent should continuously monitor the linear acceleration along the Z-axis. If the absolute value of this acceleration exceeds a certain threshold (e.g., `1.5 g`), it should log a `WARNING` message.
2.  **Service-Oriented Task Agent**: Design a Python ROS 2 node that acts as a client for a hypothetical service `/humanoid/adjust_balance` (which takes a `float64` for lean angle and returns a `boolean` for success). Implement a simple agent that, when a button is pressed (simulated by a timer), calls this service to adjust the humanoid's balance.
3.  **Command Validation**: Modify the `JointCommanderAgent` example to include a simple validation check. Before publishing, ensure that no `target_position` exceeds a predefined `MAX_JOINT_ANGLE` (e.g., +/- 1.57 radians). If a command would exceed this, clip it or log a warning.
4.  **Asynchronous Agent Logic**: For a more complex Python agent where the `control_loop_callback` might take longer (e.g., involving an LLM inference), how could you refactor it using Python's `asyncio` and `rclpy`'s `create_task` to ensure the ROS 2 node remains responsive and doesn't block other callbacks?
5.  **Data Processing Agent**: Create an `rclpy` node that subscribes to `/humanoid/camera/image_raw` (of type `sensor_msgs/Image`), but instead of displaying the image, it processes it. For simplicity, convert the image to grayscale (conceptually) and then publish a `std_msgs/String` message with the text "Processed grayscale image" to a new topic `/processed_image_status`.


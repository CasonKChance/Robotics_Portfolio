# ROS 2 Unicycle Robot Simulator

A 2D unicycle robot simulator built in **C++20 and ROS 2**. This project extends my standalone unicycle simulator from project 1 into a distributed ROS 2 system using nodes, topics, services, parameters, custom interfaces, and a Python-based visualization.

The project simulates a robot moving through a configurable 2D world containing obstacles and a goal region. Velocity commands are published through ROS 2, consumed by the simulator, which updates the robot, and the robot's pose is then published to the visualization node in real time.

## Project Goals

This project was built to develop practical familiarity with the core concepts of ROS 2 while continuing to develop the robotics fundamentals from my previous project.

Specifically, the project focuses on:

* ROS 2 nodes and the ROS graph
* Publishers and subscribers
* Topics and message passing
* Services and clients
* ROS 2 parameters
* Custom ROS 2 messages and services
* C++ ROS 2 development with `rclcpp`
* Python ROS 2 development with `rclpy`
* ROS 2 package and CMake configuration
* Integration testing
* Separating robotics models from ROS-specific infrastructure
* Real-time visualization of simulated robot state

---

## Architecture

The simulator is divided into three primary ROS 2 nodes:

```text
              ┌──────────────────────────┐
              │    Command Velocity      │
              │    Publisher Node        │
              └────────────┬─────────────┘
                           │
                       /cmd_vel
                           │
                           ▼
              ┌──────────────────────────┐
              │     Simulator Node       │
              │                          │
              └──────┬──────────┬────────┘
                     │          │    ▲
                     │          │    │
                /robot_pose  /send_world_data
                     │          │    │     
                     ▼          ▼    │
              ┌──────────────────────┴───┐
              │    Visualization Node    │
              │                          │
              └──────────────────────────┘
```

### Command Velocity Publisher

`CommandVelocityPublisherNode` publishes velocity commands to:

```text
/cmd_vel
```

using:

```text
geometry_msgs/msg/Twist
```

The node exposes ROS 2 parameters for the commanded linear and angular velocities.

Commands are published at 100 Hz.

### Simulator Node

`SimulatorNode` owns and manages the simulation. It is implemented in C++ using `rclcpp`.

It:

* Subscribes to `/cmd_vel`
* Updates the robot state
* Advances the simulation using a fixed timestep
* Publishes the robot pose
* Checks world boundaries
* Checks collisions with obstacles
* Checks whether the robot has reached the goal
* Provides world information through a ROS 2 service
* Publishes to `/robot_pose`

The simulator is built around separate C++ domain classes:

```text
Robot
World
Goal
Obstacle
```

This keeps the robotics simulation logic independent from the ROS communication layer.

### Visualization Node

`VisualizationNode` is implemented in Python using `rclpy` and Matplotlib.

It:

* Requests the initial world configuration through a ROS 2 service
* Subscribes to `/robot_pose`
* Displays the simulation world
* Displays obstacles
* Displays the goal region
* Displays the robot's current position and heading
* Tracks the robot's trajectory
* Updates the visualization as new pose messages arrive

The ROS executor runs in a background thread while Matplotlib owns the main GUI thread.

---

## Robot Model

The simulated robot uses a **unicycle kinematic model**.

The robot state is represented by:

```text
q = [x, y, θ]ᵀ
```

where:

* `x` — world-frame x position
* `y` — world-frame y position
* `θ` — robot heading

The commanded velocity is represented by:

```text
u = [v, ω]ᵀ
```

where:

* `v` — linear velocity
* `ω` — angular velocity

The robot is updated using Forward Euler integration:

```text
ẋ = v cos(θ)
ẏ = v sin(θ)
θ̇ = ω
```

which gives the discrete update:

```text
xₖ₊₁ = xₖ + v cos(θₖ) Δt

yₖ₊₁ = yₖ + v sin(θₖ) Δt

θₖ₊₁ = θₖ + ω Δt
```

The heading is normalized to the range:

```text
[-π, π]
```

The robot also models acceleration toward commanded velocity and limits its linear and angular velocity.

Current limits are:

```text
Maximum linear velocity:   5.0 m/s
Maximum angular velocity:  π rad/s

Linear acceleration:       2.5 m/s²
Angular acceleration:      π/2 rad/s²
```

---

## Simulation World

The world is configurable through ROS 2 parameters in:

```text
config/world.yaml
```

The configuration defines:

* The size of the world (length and width in meters)
* A configurable circular goal position and radius
* Configurable circular obstacle positions and radii

Current world configuration:

```text
World:
  X: [0, 10] m
  Y: [0, 10] m

Goal:
  Position: (8, 8) m
  Radius:   1 m

Obstacles:
  (2, 4), radius 1 m
  (4, 2), radius 1 m
  (6, 6), radius 1.75 m
```

The simulator terminates when the robot:

1. Reaches the goal
2. Collides with an obstacle
3. Leaves the world boundaries

---

## ROS 2 Interfaces

### Topics

#### `/cmd_vel`

Type:

```text
geometry_msgs/msg/Twist
```

Used to send linear and angular velocity commands to the simulator.

```text
CommandVelocityPublisherNode
            │
            ▼
         /cmd_vel
            │
            ▼
       SimulatorNode
```

#### `/robot_pose`

Type:

```text
project_2_ros_unicycle_robot_sim_2d/msg/RobotPose
```

Contains:

```text
float64 x
float64 y
float64 theta
```

The simulator publishes the robot's current pose to subscribers.

```text
SimulatorNode
      │
      ▼
 /robot_pose
      │
      ▼
VisualizationNode
```

---

### Service

#### `/send_world_data`

Type:

```text
project_2_ros_unicycle_robot_sim_2d/srv/SendWorldData
```

The visualization node requests the current world configuration from the simulator.

The response contains:

* World boundaries
* Goal information
* Obstacle information
* Current robot pose

This demonstrates request/response communication between ROS 2 nodes.

---

## Custom ROS 2 Interfaces

The project defines three custom messages:

```text
msg/
├── RobotPose.msg
├── Goal.msg
└── Obstacle.msg
```

and one custom service:

```text
srv/
└── SendWorldData.srv
```

These interfaces allow the ROS nodes to communicate using data specific to this simulation rather than relying exclusively on standard ROS message types.

---

## Visualization

The simulator includes a real-time 2D visualization implemented with **Python, rclpy, and Matplotlib**.

The visualization displays:

* World boundaries
* Goal region
* Obstacles
* Robot position
* Robot heading
* Robot trajectory

The ROS executor runs in a background thread so that Matplotlib can maintain control of the main GUI thread.

<img width="962" height="792" alt="Screenshot 2026-09-16 at 11 00 31 AM" src="https://github.com/user-attachments/assets/11d84be3-73d9-43bd-9535-0a4adc8e04be" />

---

## ROS Graph

<img width="984" height="337" alt="Screenshot 2026-09-16 at 10 58 28 AM" src="https://github.com/user-attachments/assets/0b5485e3-5a33-499e-8156-47bf8bac7f55" />

---

## Demo Video

https://github.com/user-attachments/assets/9fd7d09a-ae33-4b14-9f04-5d560d7d94d9

---

## Project Structure

```text
project_2_ros_unicycle_robot_sim_2d/
│
├── CMakeLists.txt
├── package.xml
├── LICENSE
│
├── config/
│   └── world.yaml
│
├── include/
│   └── project_2_ros_unicycle_robot_sim_2d/
│       ├── CommandVelocityPublisherNode.h
│       ├── Goal.h
│       ├── Obstacle.h
│       ├── Pose.h
│       ├── Robot.h
│       ├── SimulatorNode.h
│       ├── VelocityCommand.h
│       └── World.h
│
├── msg/
│   ├── Goal.msg
│   ├── Obstacle.msg
│   └── RobotPose.msg
│
├── srv/
│   └── SendWorldData.srv
│
├── src/
│   ├── CommandVelocityPublisherNode.cpp
│   ├── CommandVelocityPublisherNodeMain.cpp
│   ├── Goal.cpp
│   ├── Obstacle.cpp
│   ├── Robot.cpp
│   ├── SimulatorNode.cpp
│   ├── SimulatorNodeMain.cpp
│   ├── VisualizationNode.py
│   └── World.cpp
│
└── tests/
    ├── CommandVelocityPublisherNodeTests.cpp
    └── SimulatorNodeTests.cpp
```

---

## Building

This project was developed on:

```text
Ubuntu 26.04
ROS 2 Lyrical
C++20
Python 3
CMake
colcon
```

Clone this repository:

```bash
git clone https://github.com/CasonKChance/Robotics_Portfolio
```

Build the package:

```bash
cd ~/Robotics_Portfolio/project_2_ros_unicycle_robot_sim_2d
colcon build --packages-select project_2_ros_unicycle_robot_sim_2d
```

Source the workspace:

```bash
source install/setup.bash
```

---

## Running

Start the command publisher in a new terminal window:

```bash
source install/setup.bash
```

```bash
ros2 run project_2_ros_unicycle_robot_sim_2d CommandVelocityPublisherNode
```

Start the simulator in a new terminal window:

```bash
source install/setup.bash
```

```bash
ros2 run project_2_ros_unicycle_robot_sim_2d SimulatorNode \
    --ros-args \
    --params-file \
    src/project_2_ros_unicycle_robot_sim_2d/config/world.yaml
```

Start the visualization in a new terminal window:

```bash
source install/setup.bash
```

```bash
ros2 run project_2_ros_unicycle_robot_sim_2d VisualizationNode
```

The command publisher can be configured using ROS 2 parameters.

For example:

```bash
ros2 param set /command_velocity_publisher_node linear_velocity 1.0
ros2 param set /command_velocity_publisher_node angular_velocity 0.2
```

---

## Inspecting the ROS System

The ROS 2 CLI can be used to inspect the running system.

List nodes:

```bash
ros2 node list
```

Inspect a node:

```bash
ros2 node info /simulator_node
```

List topics:

```bash
ros2 topic list
```

Inspect the velocity command topic:

```bash
ros2 topic info /cmd_vel
```

View robot pose messages:

```bash
ros2 topic echo /robot_pose
```

List services:

```bash
ros2 service list
```

Inspect the custom service:

```bash
ros2 interface show project_2_ros_unicycle_robot_sim_2d/srv/SendWorldData
```

The ROS graph can also be visualized using:

```bash
rqt_graph
```

---

## Testing

The project uses **GoogleTest through `ament_cmake_gtest`**.

Build the project with testing enabled:

```bash
colcon build --packages-select project_2_ros_unicycle_robot_sim_2d
```

Run the tests:

```bash
colcon test --packages-select project_2_ros_unicycle_robot_sim_2d
```

View the test results:

```bash
colcon test-result --verbose
```

The test suite includes tests for:

* Command velocity publisher behavior
* Simulator behavior
* ROS topic communication
* Robot interaction with the simulated world
* Goal and obstacle conditions

---

## Design Decisions

### ROS Nodes vs. Simulation Classes

The robotics simulation logic is kept separate from ROS-specific code.

For example:

```text
Robot
World
Goal
Obstacle
```

are normal C++ classes and do not depend on ROS.

The `SimulatorNode` provides the ROS interface around those classes.

This separation makes the underlying simulation logic easier to test and reuse outside of ROS.

### Topics for Continuous Data

Velocity commands and robot pose use topics because they represent data that can be continuously published and consumed.

```text
/cmd_vel
/robot_pose
```

### Service for World Data

World configuration is requested through a service because the visualization node is making a discrete request for information rather than continuously streaming world configuration.

```text
/send_world_data
```

### Python Visualization

The visualization is implemented in Python rather than C++ to take advantage of the rapid development and plotting capabilities of Matplotlib while keeping the core simulation in C++.

---

## What I Learned

This project introduced the core communication model of ROS 2 and provided experience building a multi-node robotic system.

Key concepts practiced:

* ROS 2 nodes
* ROS 2 topics
* Publishers and subscribers
* Services and clients
* ROS 2 parameters
* Custom messages
* Custom services
* `rclcpp`
* `rclpy`
* ROS 2 package structure
* `ament_cmake`
* `colcon`
* ROS graph introspection
* ROS/C++ integration
* ROS/Python integration
* Integration testing

One of the most important architectural lessons was separating the **robotics model** from the **ROS communication layer**. The `Robot`, `World`, `Goal`, and `Obstacle` classes represent the simulation itself, while `SimulatorNode` exposes that simulation through ROS 2.

---

## Challenges

### Integrating ROS with the Existing Simulator

The original simulator was a standalone C++ program. Converting it into a continuously running ROS 2 system required changing the execution model from a traditional simulation loop into a ROS node driven by callbacks and timers.

### Real-Time Visualization

Matplotlib owns the GUI event loop, while ROS needs to process incoming messages. The visualization therefore uses a background thread for the ROS executor while keeping the Matplotlib event loop on the main thread.

A mutex protects shared state between the ROS callback thread and visualization code.

### Custom Interfaces

The project uses custom ROS messages and a custom service to communicate simulation-specific data between nodes. This required integrating ROS interface generation into the CMake build process.

### Testing ROS Behavior

Testing individual C++ classes is relatively straightforward. Testing ROS communication required creating publishers/subscribers, spinning executors, waiting for messages, and verifying behavior across multiple ROS components.

---

## Technologies

* **C++20**
* **Python 3**
* **ROS 2**
* **rclcpp**
* **rclpy**
* **CMake**
* **ament_cmake**
* **colcon**
* **GoogleTest**
* **Matplotlib**
* **Ubuntu 26.04**

---

## Related Project

This project is an extension of Project 1:

**2D Unicycle Robot Simulator**

Project 1 established the underlying robot simulation and kinematic model. This project builds a ROS 2 software architecture around that simulation.

# ROS 2 Unicycle Robot Simulator 2.0

A 2D unicycle robot simulator built in C++20 and ROS 2. This project extends my standalone unicycle simulator from projects 1 and 2 by adding user facing controls using ROS2 actions, configurable parameters to set physical limits for the simulated robot, and a launch process.

## Demo

**[Insert GIF/video here.]**

---

## Overview

The project simulates a configurable unicycle robot moving through a configurable 2D world containing bounds, obstacles, and a goal region. Goal poses, given by the user through an operator node, in the world frame are supplied via a ROS2 action to a controller node. The controller calculates and communicates velocity commands to the simulator through a ROS2 publisher/subscriber relationship and utilizes a feedback loop and successive states to orient the robot toward the goal position, translate the robot to the goal position, and finally orient the robot into the goal pose. The simulator provides world and robot state information to a visualization node so the user can see interactions in real time.

---

## System Architecture

              ┌───────────────────────────────────┐
              │          Operator Node            │
              │                                   │
              └──────┬────────────────────────────┘
                     │                  ▲
                /go_to_pose             │
                action goal             │
                     │              /go_to_pose
                     │          action feedback/result
                     ▼                  │
              ┌─────────────────────────┴─────────┐
              │         Controller Node           │
              │                                   │
              └──────┬────────────────────────────┘
                     │          ▲          ▲
                 /cmd_vel       │          │
                     │    /robot_state     │
                     ▼          │    /simulator_status
              ┌─────────────────┴──────────┴──────┐
              │          Simulator Node           │
              │                                   │
              └──────┬─────────────────┬──────────┘
                     │                 │     ▲
                     │                 │     │
                /robot_state      /send_world_data
                     │                 │     │     
                     ▼                 ▼     │
              ┌──────────────────────────────┴────┐
              │         Visualization Node        │
              │                                   │
              └───────────────────────────────────┘

### ROS Graph

**[Insert image here.]**

### Nodes

#### Robot Operator

**Responsibility:**  
The operator is the user-facing node that communicates to the controller where the user wants the robot to go.

**Inputs:**  
The operator receives a goal pose from the user in the form of [x, y, θ]ᵀ where x and y are coordinates in the world frame (measured in meters) and θ is the intended heading (measured in radians).

**Outputs:**  
The operater contains a GoToPose action client which sends the goal pose received from the user to the Robot Controller node.

---

#### Robot Controller

**Responsibility:**  
The controller uses a provided goal pose to calculate and publish velocity commands intended to move the robot to the goal pose. It utilizes stateful architecture and a feedback loop with the simulator node to move the robot through three phases: rotate towards the goal position, translate to the goal position, and rotate to the final heading. The controller will validate the robot's final position against the goal pose and calculate and publish velocity commands to correct the robot's position so that the final position is within a provided tolerance of the goal pose.

**Inputs:**  
The controller receives the goal pose via a ROS2 action goal from the operator. It also subscribes to the /robot_state topic so that it can receive up-to-date information about the robot's current pose within the world frame as well as information about the robot's current velocity. Additionally, it subscribes to the /simulator_status topic so that it can provide clean up operations when the simulator terminates.

**Outputs:**  
The controller provides ROS2 action feedback and results back to the operator as the action is being completed so the user can see information regarding the distance remaining to reach the goal pose. It also publishes velocity commands to /cmd_vel for simulator consumption.

---

#### Simulator

**Responsibility:**  
The simulator owns both the robot and world and coordinates interactions between the two. It is responsible for building the world, configuring the robot, advancing time, applying velocity commands to the robot, performing collision checking between the world and the robot, and communicating relevant information to the controller and the visualization.

**Inputs:**  
The simulator subscribes to the /cmd_vel topic from which it receives velocity commands from the controller to be applied to the robot. It also receives parameters from world.yaml and robot.yaml so that it knows how to build the world and configure the robot

**Outputs:**  
The simulator publishes the robot's current pose and velocity to the /robot_state topic which is consumed by both the visualization and controller. It also publishes its own status to the /simulator_status topic so the controller can know whether or not the simulator is still running. Additionally, it acts as a server for the /send_world_data service where it receives a request for the world data from the visualization node upon that node's startup.

---

#### Visualization

**Responsibility:**  
The visualization node uses world and robot data provided by the simulator to create and update a Matplotlib visualization of the simulation in real time.

**Inputs:**
The visualization node requests and receives world data on startup from the simulator via the /send_world_data service, it uses this data to build a visualization of the world. It also subscribes to the /robot_state topic from which it receives the robot's current pose which it then uses to update the visualization of the robot's position in real time.

---

### Interfaces

#### Actions

##### `GoToPose.action`

```text
# Goal
float64 x
float64 y
float64 theta
---
# Result
float64 x
float64 y
float64 theta
---
# Feedback
float64 distance_remaining
float64 rotation_remaining
```

The goal is the intended pose for the robot. The result is the actual pose of the robot once the action has been completed. The feedback, which is sent back to the operator for user viewing, is the distance and rotation remaining to be completed for the robot to be in the final pose. I choose an action for this process as it is a long-running process and benefits from having action cancelating and being able to provide feedback.

#### Services

##### `SendWorldData.srv`

```text
# No data needed from client
---
float64 max_x
float64 max_y
Goal[<=1] goal
Obstacle[] obstacles
RobotState robot_state
```

The visualization node sends a request for world data over this service. It is then provided with the max_x and max_y of the world, with which it can construct the world bounds, an optional circular goal region, a list of circular obstacles, and the robot's state which it uses to place the robot within the world.

#### Messages

##### RobotState.msg

```text
float64 x
float64 y
float64 theta
float64 linear_velocity
float64 angular_velocity
```

The RobotState message is used by the simulator to provide the current position, heading, and velocity of the robot to consumers.

##### SimulatorStatus.msg

```text
bool is_simulator_running
```

The SimulatorStatus message is used by the simulator to tell relevant nodes whether or not it has reached a termination state.

##### Goal.msg

```text
geometry_msgs/Point center
float64 radius
```

The Goal message is used to describe the circular goal region's position and radius.

##### Obstacle.msg

```text
geometry_msgs/Point center
float64 radius
```

The obstacle message is used to describe a circular obstacle region's position and radius.

---

## Robot Controller: In Depth Review

The Robot Controller is responsible for converting a user-provided goal pose into velocity commands that move the simulated robot to the requested position and orientation. Rather than commanding the entire maneuver at once, the controller operates as a state machine and uses continuous feedback from the simulator to determine when to accelerate, brake, transition between stages, and correct errors in the robot's final pose.

### Objective

The controller solves a point-to-point pose control problem for a unicycle robot.

Given:

```text
Current pose: (x, y, θ)
Target pose:  (x_goal, y_goal, θ_goal)
```

the controller must bring the robot to the requested position and final orientation while respecting the robot's configured maximum linear velocity, maximum angular velocity, linear acceleration, and angular acceleration.

Because the simulated robot follows a unicycle motion model, it cannot translate sideways. The robot's translational velocity acts along its current heading, so reaching an arbitrary pose requires coordinating rotation and translation rather than independently changing `x`, `y`, and `θ`.

The controller receives the current robot state from the simulator through `/robot_state` and executes its control loop every 10 ms. This creates a feedback loop in which each control decision is based on the latest simulated pose and velocity.

### Control Strategy

Navigation is divided into three primary maneuvers:

1. Rotate until the robot faces the target position.
2. Drive forward until the robot reaches the target position.
3. Rotate in place until the robot reaches the requested final heading.

These maneuvers are implemented as a state machine:

```text
RotatingToGoalPosition
          │
          ▼
WaitingForRotationToGoalPositionStop
          │
          ▼
DrivingToGoalPose
          │
          ▼
WaitingForDrivingToGoalPoseStop
          │
          ▼
RotatingToGoalPose
          │
          ▼
WaitingForRotationToGoalPoseStop
          │
          ▼
GoalPoseReached
```

When a new `GoToPose` action is accepted, the controller begins in `RotatingToGoalPosition`.

The required initial rotation is calculated by finding the direction from the robot to the goal position:

```text
target_heading = atan2(y_goal - y, x_goal - x)
```

and comparing that direction with the robot's current heading:

```text
rotation_remaining = target_heading - θ
```

Angular differences are normalized to `[-π, π]`. This causes the controller to select the shorter rotation direction instead of, for example, rotating almost an entire revolution to reach an equivalent heading.

After completing this rotation, the controller enters `WaitingForRotationToGoalPositionStop`. The controller does not immediately begin translating because a zero angular velocity command does not always produce zero angular velocity due to floating point error. It waits until the measured angular velocity is sufficiently close to zero before beginning the next maneuver.

During `DrivingToGoalPose`, the controller commands forward linear motion toward the target. The remaining distance is calculated using Euclidean distance:

```text
distance_remaining = √((x_goal - x)² + (y_goal - y)²)
```

Once braking needs to begin, the controller commands zero velocity and enters `WaitingForDrivingToGoalPoseStop`. It again waits for the robot's actual velocity to fall sufficiently close to zero before continuing.

The final maneuver, `RotatingToGoalPose`, rotates the stationary robot from its current heading to `θ_goal`. Once braking has completed and the angular velocity has settled, the state machine reaches `GoalPoseReached`.

Separating the maneuver into rotation, translation, and final rotation keeps the controller intentionally simple and deterministic. It also matches the nonholonomic nature of the unicycle model: the robot first aligns the direction in which it can move, moves along that direction, and then independently establishes its desired final orientation.

The waiting states are necessary because the simulator models acceleration. A command of zero velocity therefore represents a request to decelerate rather than an instantaneous stop.

### Braking / Motion Constraints

The robot has two configurable motion constraints:

- Maximum linear velocity
- Maximum angular velocity
- Linear acceleration
- Angular acceleration

The controller uses the maximum velocities when commanding motion. The simulator then changes the robot's actual velocity toward those commands according to its configured acceleration limits.

As a result, the controller cannot simply command maximum velocity until the robot reaches the exact target and then command zero. At nonzero velocity, the robot requires time and distance to decelerate. Waiting until the target itself is reached before braking would cause the robot to overshoot it.

To determine when braking should begin, the controller calculates stopping distance using:

```text
d_stop = v² / (2a)
```

where:

```text
d_stop = stopping distance
v      = current linear or angular velocity
a      = corresponding linear or angular acceleration
```

For translational motion, `v` is the current linear velocity and `a` is the configured linear acceleration. For rotational motion, the same calculation is applied using angular velocity and angular acceleration, producing an angular stopping distance.

During each moving state, the controller compares the remaining distance or rotation with the calculated stopping distance:

```text
remaining > stopping distance
        │
        ├── yes ──> continue commanding motion
        │
        └── no  ──> command zero velocity and begin braking
```

The simulator's acceleration model then progressively moves the actual velocity toward zero.

The controller considers the robot stopped when the relevant absolute velocity is less than:

```text
1e-3
```

This braking strategy allows the controller to account for the robot's simulated dynamics instead of treating velocity commands as instantaneous changes in motion.

### Goal Tolerances and Corrections

Stopping distance allows the controller to begin braking near the desired pose, but discrete simulation updates, acceleration, and floating-point calculations mean the robot should not be expected to stop at an exactly identical floating-point position and heading.

The controller therefore defines two configurable tolerances:

```text
goal_pose_positional_tolerance
goal_pose_heading_tolerance
```

The positional tolerance determines how close the robot's `(x, y)` position must be to the requested goal position. The heading tolerance determines how close the robot's final orientation must be to `θ_goal`.

Once the primary three-stage maneuver has completed, `GoalPoseReached` validates the actual robot state.

If:

```text
distance_remaining > positional_tolerance
```

the controller returns to `RotatingToGoalPosition` and performs another position maneuver. Because the robot may now be slightly past or offset from the goal, the desired direction is recalculated using the robot's current state.

If the position is valid but:

```text
abs(rotation_remaining) > heading_tolerance
```

the controller returns to `RotatingToGoalPose` and performs another heading correction.

The action succeeds only after both conditions are satisfied:

```text
distance_remaining <= positional_tolerance

AND

abs(rotation_remaining) <= heading_tolerance
```

The controller then returns the robot's actual final `x`, `y`, and `θ` to the operator as the action result.

This correction process allows the controller to converge on the requested pose without relying on exact floating-point equality or assuming that the initial braking maneuver will stop at precisely the requested coordinates.

---

## Action Lifecycle

Robot navigation is exposed to the operator through the ROS 2 `GoToPose` action.

A complete navigation command follows this lifecycle:

```text
User enters target pose
        ↓
Operator sends GoToPose goal
        ↓
Controller receives goal request
        ↓
Controller checks for an existing active goal
        ↓
Goal accepted or rejected
        ↓
Controller executes maneuver
        ↓
Feedback published while moving
        ↓
Controller validates final pose
        ↓
Correction passes performed if necessary
        ↓
Robot reaches requested pose within tolerance
        ↓
Result returned to operator
```

When the controller receives a goal request, it first determines whether another goal is already active. If an active goal exists, the new request is rejected. Otherwise, the request is accepted for execution.

After acceptance, the goal handle becomes the controller's active goal and the state machine enters `RotatingToGoalPosition`.

While executing the maneuver, the controller publishes action feedback containing:

```text
distance_remaining
rotation_remaining
```

The remaining distance represents the Euclidean distance between the robot and the requested goal position. The remaining rotation represents the normalized difference between the robot's current heading and the requested final heading.

Once the robot's position and heading are both within their configured tolerances, the controller marks the action as succeeded and returns:

```text
x
y
θ
```

containing the robot's actual final pose.

The controller then clears the active goal and returns to `Idle`, where it waits for another request.

### Cancellation

The `GoToPose` action also supports cancellation.

When a cancellation request reaches the controller's action server, the controller accepts the request. On the next execution of the control loop, the active goal handle reports that the goal is canceling.

The controller then immediately publishes:

```text
linear velocity  = 0.0
angular velocity = 0.0
```

and transitions to `GoalCanceled`.

Because acceleration limits are modeled by the simulator, commanding zero velocity does not mean that the robot has physically stopped yet. The controller therefore waits in `GoalCanceled` until both:

```text
abs(linear_velocity)  <= 1e-3
abs(angular_velocity) <= 1e-3
```

Only after the robot has completed its deceleration does the controller mark the action as canceled.

The result contains the robot's actual pose at the end of the cancellation:

```text
x
y
theta
```

The controller then clears the active goal and returns to `Idle`.

This ensures that cancellation represents a completed stop rather than merely the instant at which the stop command was issued.

---

## Simulation Model

The simulator uses an intentionally simplified 2D kinematic model. Its purpose is to model the behavior relevant to basic mobile-robot control—pose, velocity, acceleration, world geometry, and collisions—without introducing the complexity of a full rigid-body physics engine.

### Robot State

The simulated robot's primary pose is:

```text
x
y
θ
```

where `x` and `y` represent its position in the world frame and `θ` represents its heading.

The robot also maintains two different velocity representations:

```text
commanded velocity
actual velocity
```

Each contains:

```text
linear velocity
angular velocity
```

The commanded velocity represents what the controller has requested through `/cmd_vel`.

The actual velocity represents how quickly the simulated robot is currently moving. These values are intentionally separate because the robot cannot instantaneously change from one velocity to another.

The simulator publishes the actual state through `/robot_state`, including:

```text
x
y
theta
linear_velocity
angular_velocity
```

This provides the controller with the feedback required to make subsequent control decisions.

### Motion Model

The robot follows the standard planar unicycle kinematic model.

At each simulation update, its current linear velocity `v`, angular velocity `ω`, heading `θ`, and simulation time step `dt` determine how its pose changes.

The continuous model is:

```text
dx/dt = v cos(θ)

dy/dt = v sin(θ)

dθ/dt = ω
```

The simulator advances this model using discrete Forward Euler integration:

```text
x_next = x + v cos(θ) dt

y_next = y + v sin(θ) dt

θ_next = θ + ω dt
```

The resulting heading is normalized back into:

```text
[-π, π]
```

using:

```text
atan2(sin(θ), cos(θ))
```

The simulator uses a fixed:

```text
dt = 0.01 seconds
```

corresponding to a 100 Hz update loop.

This model captures the defining constraint of a unicycle robot: translational motion occurs in the direction of the robot's current heading. There is no independent lateral velocity.

### Acceleration

Velocity commands do not directly become the robot's actual velocity.

Instead, during each simulation update, the robot compares its commanded velocity with its current actual velocity. It then changes the actual velocity toward the command according to the configured acceleration.

For linear motion, the maximum velocity change during one update is determined by:

```text
Δv = linear_acceleration * dt
```

For angular motion:

```text
Δω = angular_acceleration * dt
```

The simulator determines whether it needs to accelerate or decelerate based on the sign of the difference between the commanded and actual velocities.

Conceptually:

```text
velocity_error = commanded_velocity - actual_velocity

actual_velocity += direction * acceleration * dt
```

If an update would move the actual velocity beyond the requested velocity, the value is set directly to the requested velocity instead. This prevents the acceleration update itself from oscillating around or overshooting the command.

Afterward, the actual velocities are clamped to:

```text
[-maximum_linear_velocity, maximum_linear_velocity]

[-maximum_angular_velocity, maximum_angular_velocity]
```

for linear and angular motion respectively.

This separation between commanded and actual velocity is what creates acceleration and braking behavior in the simulator. It is also why the controller calculates stopping distance and contains explicit states that wait for the robot to finish decelerating.

### World

The simulated world is a bounded 2D Cartesian environment.

Its geometry consists of:

- World bounds
- An optional circular goal region
- Zero or more circular obstacles

The lower world boundary is `(0, 0)`, while the upper boundary is configured using `max_x` and `max_y`. A valid point therefore satisfies:

```text
0 <= x <= max_x
0 <= y <= max_y
```

Obstacles are represented as circles defined by a center position `(x, y)` and a radius. The optional world goal is represented using the same circular geometry.

During world construction, the simulator validates that configured obstacles and the optional goal fit inside the world bounds. The radius of each region is taken into account, so it is not sufficient for only the center of an obstacle or goal to lie within the world. A configuration that places any portion of one of these regions outside the world results in an error.

After each robot update, the simulator checks the robot's new position for three termination conditions:

```text
Out of world bounds
Collision with an obstacle
Collision with the goal region
```

If none occur, the simulator remains in its `Running` state.

If any condition occurs, the simulator transitions to the corresponding termination state and stops its update loop. It then publishes a simulator-status message indicating that the simulator is no longer running.

Collision checking is intentionally geometric rather than physical. The robot is treated as a point located at its `(x, y)` position, while obstacles and the goal are circular regions. The simulator therefore detects whether the robot's position lies inside one of these regions; it does not calculate rigid-body contact forces or collision response.

The simulation intentionally does **not** model effects such as:

- Robot body dimensions
- Wheel geometry
- Wheel slip
- Mass or inertia
- Friction
- Motor dynamics
- Traction limits
- Contact forces
- Collision response or bouncing
- Sensor noise
- Localization uncertainty
- Terrain
- Full rigid-body dynamics

Likewise, the simulator does not model individual wheels. Linear and angular velocity are applied directly to the unicycle kinematic model.

These simplifications are intentional. The simulator is designed to isolate and demonstrate ROS 2 communication, action-based robot control, acceleration-constrained motion, feedback control, basic unicycle kinematics, and interactions with a configurable 2D environment rather than reproduce the complete physics of a physical mobile robot.

---

## Configuration

### Robot Configuration

```yaml
robot:
  max_linear_velocity: ...
  max_angular_velocity: ...
  linear_acceleration: ...
  angular_acceleration: ...
  goal_pose_positional_tolerance: ...
  goal_pose_heading_tolerance: ...
```

The robot.yaml configuration file is used to configure the robot's physical limits as well as tolerance for goal poses.

### World Configuration

```yaml
world:
    bounds:
    max_x: ...
    max_y: ...

    goal:
    enabled: true/false
    x: ...
    y: ...
    radius: ...

    obstacles:
    x: [..., ...]
    y: [..., ...]
    radius: [..., ...]
```

The world.yaml configuration file is used to provide the simulator with information about the world to build. The simulator will relay this information to the visualization node for visualization.

---

## Visualization

**[Insert screenshot/GIF.]**

---

## Building and Running

### Prerequisites

```text
Ubuntu: 26.04
ROS 2: lyrical
C++: 20
Python: 3.14.4
```

### Build

```bash
colcon build
```

### Start the simulator

#### Source the Workspace

In a separate tab

```bash
source install/setup.bas
```

#### Launch the Simulator

```bash
ros2 launch project_3_ros_unicycle_robot_sim_2d_v2 simulator.launch.py
```

### Run the operator

#### Source the Workspace
In a separate tab

```bash
source install/setup.bas
```

#### Start the operator
```bash
ros2 run project_3_ros_unicycle_robot_sim_2d_v2 RobotOperatorNode
```

---

## Testing

Run tests for the Simulator and Controller functionality.

### Run Tests

```bash
colcon test
```

---

## Technologies

```text
* C++20
* Python 3
* ROS 2
* rclcpp
* rclpy
* CMake
* ament_cmake
* colcon
* oogleTest
* Matplotlib
* Ubuntu 26.04
```

---

## Related Projects

This project is an extension of Projects 1 and 2:

### 2D Unicycle Robot Simulator
### ROS 2D Unicycle Robot Simulator

Project 1 established the underlying robot simulation and kinematic model. Project 2 build ROS2 system architecture around that simulator.
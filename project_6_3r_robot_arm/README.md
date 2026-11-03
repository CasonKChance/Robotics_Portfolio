# Project 6 — ROS 2 3R Robotic Arm Kinematics

A simulated three-revolute-joint robotic arm used to demonstrate forward and inverse kinematics in three-dimensional space. Built with C++20, Eigen, and ROS 2 using a custom `RoboticArmKinematics` library.

---

## Demo

### Default (FK) Mode

https://github.com/user-attachments/assets/46b3e1c7-088b-47f2-96b8-3f73f98c8d71

### IK Mode

https://github.com/user-attachments/assets/14f69bf7-d2e2-4c82-adca-8ccc014a2415

---

## Overview

This project introduces robotic manipulator kinematics through a simulated three-revolute-joint robotic arm. The robot uses a Z-Y-Y joint-axis configuration, allowing the end effector to move through three-dimensional Cartesian space.

All kinematic calculations are performed using a custom standalone `RoboticArmKinematics` C++ library. The library implements forward kinematics, spatial and linear-velocity Jacobians, end-effector velocity calculations, analytic inverse kinematics for simple planar manipulators, and numerical inverse kinematics for general revolute-joint manipulators.

The ROS 2 application uses the kinematics library to calculate the current end-effector position and to move the 3R robot to requested Cartesian positions. A custom `GoToPosition` action accepts an `(x, y, z)` target, calculates the required joint configuration using numerical inverse kinematics, and commands the simulated joints until the requested end-effector position is reached.

The robot can also be launched in a default mode using `joint_state_publisher_gui`, allowing the individual joints to be manipulated manually while observing the resulting end-effector position and TF transforms in RViz.

---

## System Architecture

### Robot Arm

Responsibilities:

- Represents the simulated 3R robotic arm
- Loads the robot geometry, joint axes, joint limits, inertial properties, damping, and friction values from the URDF
- Maintains the current joint positions, velocities, and accelerations
- Simulates joint motion from commanded joint velocities
- Enforces the velocity and effort limits defined in the URDF
- Calculates the current end-effector position using the `RoboticArmKinematics` library
- Publishes joint positions and velocities to `/joint_states`
- Publishes the current robot state to `/robot_state`
- Receives commanded joint velocities from `/joint_velocities` when operating in IK mode

The simulated joint motion uses an approximate dynamics model based on joint inertia, motor effort limits, damping, friction, and a proportional velocity controller. This is not intended to replace a full physics simulator, but provides more realistic joint acceleration and deceleration than directly assigning joint positions.

In the default operating mode, the Robot Arm instead subscribes to `/joint_states` from `joint_state_publisher_gui` and calculates the resulting end-effector position.

### Robot Arm Controller

Responsibilities:

- Provides the `/go_to_position` ROS 2 action server
- Receives Cartesian end-effector goals in `(x, y, z)`
- Reads the current robot configuration from `/robot_state`
- Uses the current joint configuration as the initial guess for numerical inverse kinematics
- Calculates target joint angles using the `RoboticArmKinematics` library
- Calculates the remaining rotation required for each joint
- Publishes velocity commands to `/joint_velocities`
- Enforces the maximum joint velocities provided by the robot
- Publishes action feedback containing the remaining joint rotation
- Supports goal cancellation
- Verifies the final Cartesian end-effector position before reporting success
- Re-runs inverse kinematics if residual Cartesian error exceeds the configured goal tolerance

The high-level goal flow is:

```text
Cartesian Goal (x, y, z)
          │
          ▼
   GoToPosition Action
          │
          ▼
   RobotArmController
          │
          ├──── Current Robot State
          │
          ▼
 Numerical Inverse Kinematics
          │
          ▼
   Target Joint Angles
          │
          ▼
 Joint Velocity Commands
          │
          ▼
       RobotArm
          │
          ├──── /joint_states
          │
          └──── /robot_state
```

---

## Custom ROS Interfaces

The project includes a separate `project_6_3r_robot_arm_interfaces` package containing the messages and action used by the robot and controller.

### `GoToPosition.action`

Accepts a desired Cartesian end-effector position:

```text
# Goal
float64 x
float64 y
float64 z
---
# Result
float64 x
float64 y
float64 z
---
# Feedback
float64 joint_pedestal_arm1_rotation_remaining
float64 joint_arm1_arm2_rotation_remaining
float64 joint_arm2_arm3_rotation_remaining
```

The result contains the measured final end-effector position.

Feedback reports the remaining angular displacement for each revolute joint.

### `JointVelocities.msg`

Contains the commanded velocity for each revolute joint:

```text
float64 joint_pedestal_arm1_velocity
float64 joint_arm1_arm2_velocity
float64 joint_arm2_arm3_velocity
```

### `RobotState.msg`

Contains the robot state required by the controller:

```text
float64 end_effector_x
float64 end_effector_y
float64 end_effector_z
float64 arm1_length
float64 arm2_length
float64 arm3_length
float64 joint_pedestal_arm1_angle
float64 joint_arm1_arm2_angle
float64 joint_arm2_arm3_angle
float64 joint_pedestal_arm1_velocity
float64 joint_arm1_arm2_velocity
float64 joint_arm2_arm3_velocity
float64 joint_pedestal_arm1_maximum_acceleration
float64 joint_arm1_arm2_maximum_acceleration
float64 joint_arm2_arm3_maximum_acceleration
float64 joint_pedestal_arm1_maximum_velocity
float64 joint_arm1_arm2_maximum_velocity
float64 joint_arm2_arm3_maximum_velocity
float64 x_offset
float64 y_offset
float64 z_offset
```

---

## ROS Graph

### Default (FK) Mode

<img width="1356" height="650" alt="Screenshot 2026-11-02 at 6 44 37 PM" src="https://github.com/user-attachments/assets/8ee9c657-646c-46a7-ba6a-45c12950b32b" />

### IK Mode

<img width="1356" height="650" alt="Screenshot 2026-11-02 at 6 44 06 PM" src="https://github.com/user-attachments/assets/f7a83577-c84f-4f32-99ad-d443ca0867ab" />

---

## TF Tree

<img width="383" height="791" alt="Screenshot 2026-11-02 at 6 49 26 PM" src="https://github.com/user-attachments/assets/0ac7e4ba-b1c1-41dc-b031-1d1a8eccb669" />

### TF Hierarchy

```text
world
└── base_link
    └── pedestal_link
        └── arm_link_1
            └── arm_link_2
                └── arm_link_3
                    └── end_effector_link
```

The fixed transforms define the world, base, pedestal, and end-effector relationships. The transforms between the three arm links are controlled by the revolute joints:

```text
joint_pedestal_arm1    → Z axis
joint_arm1_arm2        → Y axis
joint_arm2_arm3        → Y axis
```

`robot_state_publisher` reads `/joint_states` and publishes the resulting dynamic TF transforms for visualization in RViz.

---


## Robotic Arm Kinematics Library

The `RoboticArmKinematics` library is a standalone C++ library built on Eigen. It implements the underlying kinematic mathematics independently of ROS 2.

The library provides:

```text
forwardKinematics()

isWithinReachBounds()

spatialJacobian()

linearVelocityJacobian()

endEffectorTwist()

endEffectorLinearVelocity()

analyticInverseKinematics()

numericalInverseKinematics()
```

### Forward Kinematics

`forwardKinematics()` calculates the Cartesian position of the end effector from the robot's joint configuration.

Conceptually:

```text
Joint Configuration
       q
       │
       ▼
Forward Kinematics
       │
       ▼
End-Effector Position
     (x, y, z)
```

The implementation supports arbitrary link counts and arbitrary revolute-joint axes.

Each joint rotation and link translation is composed into a transformation chain to determine the final position of the end effector.

### Reach Bounds

`isWithinReachBounds()` performs a geometric check to determine whether a target lies within the minimum and maximum possible radial reach of the arm.

For link lengths $L_i$:

$$
r_{\max} = \sum_i L_i
$$

and:

$$
r_{\min} = \max\left(0,\ 2L_{\max} - \sum_i L_i\right)
$$

This provides a fast rejection test before attempting inverse kinematics.

Because arbitrary joint-axis configurations can impose additional constraints, passing the reach-bound test does not necessarily guarantee that a particular robot geometry can reach the target.

### Spatial Jacobian

`spatialJacobian()` constructs a 6×N spatial Jacobian.

Each column contains the screw vector associated with one revolute joint:

$$
S_i =
\begin{bmatrix}
\omega_i \\
v_i
\end{bmatrix}
$$

where:

$$
v_i = -\omega_i \times q_i
$$

Here, $\omega_i$ is the joint's rotation axis and $q_i$ is a point on that axis.

The resulting Jacobian maps joint velocities to an end-effector spatial twist:

$$
V = J(q)\dot{q}
$$

### Linear-Velocity Jacobian

`linearVelocityJacobian()` converts the spatial Jacobian into the 3×N Jacobian required to calculate the Cartesian linear velocity of the end effector.

For each joint:

$$
J_{v,i} = \omega_i \times p_e + v_i
$$

where $p_e$ is the current end-effector position, and $\omega_i$ and $v_i$ are the spatial screw components expressed in the base frame.

The resulting relationship is:

$$
\dot{p} = J_v(q)\dot{q}
$$

### End-Effector Twist and Velocity

`endEffectorTwist()` calculates the six-dimensional end-effector twist:

$$
V = J(q)\dot{q}
$$

while `endEffectorLinearVelocity()` calculates only the Cartesian linear velocity:

$$
\dot{p} = J_v(q)\dot{q}
$$

These functions demonstrate how joint-space motion is mapped into task-space motion.

### Analytic Inverse Kinematics

`analyticInverseKinematics()` provides closed-form inverse kinematics for simple planar manipulators containing one or two links.

For a two-link arm, the solver can produce:

- Zero solutions for unreachable positions
- One solution at singular workspace boundaries
- Two solutions for ordinary reachable positions

The two normal solutions correspond to the different possible elbow configurations.

The analytic solver is primarily included as an exercise in understanding the geometry and trigonometry behind inverse kinematics. More complicated manipulators generally require geometry-specific analytic solutions.

### Numerical Inverse Kinematics

`numericalInverseKinematics()` provides a more general inverse-kinematics implementation for manipulators with arbitrary numbers of revolute joints.

Starting from an initial joint configuration $q_k$, the solver calculates the current Cartesian error:

$$
e_k = p_{\text{target}} - FK(q_k)
$$

and uses the pseudoinverse of the linear-velocity Jacobian:

$$
\Delta q = J_v(q_k)^+ e_k
$$

to iteratively update the joint configuration:

$$
q_{k+1} = q_k + \Delta q
$$

Iteration continues until the Cartesian error falls within the requested tolerance or the maximum number of iterations is reached.

Unlike analytic IK, numerical IK is not tied to one specific manipulator geometry. The 3R ROS robot therefore uses numerical inverse kinematics to solve Cartesian goals in three-dimensional space.

---

## Kinematics Testing

The `RoboticArmKinematics` library includes a GoogleTest test suite covering the primary mathematical operations and edge cases.

Tests include:

- Forward kinematics
- Single-link and multi-link manipulators
- Multiple revolute-joint axes
- Workspace reach bounds
- Spatial Jacobians
- Linear-velocity Jacobians
- End-effector twists
- End-effector linear velocities
- Analytic inverse kinematics
- Numerical inverse kinematics
- Singular workspace boundaries
- Unreachable targets
- Invalid inputs

One important validation compares the velocity calculated using the Jacobian against a finite-difference approximation of the forward kinematics:

$$
v \approx
\frac{FK(q+\dot{q}\Delta t)-FK(q)}
{\Delta t}
$$

This provides an independent check that the Jacobian correctly maps joint velocities into Cartesian end-effector velocity.

Inverse-kinematics results are also validated by passing the calculated joint configuration back through forward kinematics and verifying that the resulting end-effector position matches the requested target.

---

## Operating Modes

The robot can be launched in two operating modes.

### Default (FK) Mode

Default mode launches `joint_state_publisher_gui`.

This allows each revolute joint to be manipulated manually while RViz displays the resulting robot configuration.

The Robot Arm subscribes to `/joint_states` and uses forward kinematics to calculate and log the resulting end-effector position.

```text
joint_state_publisher_gui
            │
            │ /joint_states
            ▼
        RobotArm
            │
            ▼
   Forward Kinematics
            │
            ▼
 End-Effector Position
```

### IK Mode

IK mode launches `RobotArmController` instead of `joint_state_publisher_gui`.

The controller receives Cartesian goals through `/go_to_position`, calculates a target configuration using numerical inverse kinematics, and commands joint velocities until the arm reaches the requested position.

The allowed final Cartesian error can be configured using:

```text
goal_position_tolerance
```

The default tolerance is:

```text
0.001 m
```

---

## Building and Running

### Prerequisites

```text
Ubuntu: 26.04
ROS 2: Lyrical
C++: 20
```

Required libraries/packages include:

- Eigen3
- GoogleTest
- ROS 2 `rclcpp`
- ROS 2 `rclcpp_action`
- `sensor_msgs`
- `std_msgs`
- `urdf`
- `robot_state_publisher`
- `joint_state_publisher_gui`
- RViz2
- Xacro

### Clone the repository and move to the project directory

```bash
git clone https://github.com/CasonKChance/Robotics_Portfolio/
cd Robotics_Portfolio/project_6_3r_robot_arm
```

### Source the ROS 2 workspace

```bash
source /opt/ros/lyrical/setup.bash
```

### Build the project

```bash
colcon build
```

### Source the project

```bash
source install/setup.bash
```

## Running in Default (FK) Mode

Launch the robot, `joint_state_publisher_gui`, `robot_state_publisher`, and RViz:

```bash
ros2 launch project_6_3r_robot_arm 3r_robot_arm.launch.py
```

The GUI can then be used to manually change the three joint angles.

## Running in IK Mode

Launch the robot with the inverse-kinematics controller:

```bash
ros2 launch project_6_3r_robot_arm 3r_robot_arm.launch.py mode:=IK
```

A custom end-effector position tolerance can also be supplied:

```bash
ros2 launch project_6_3r_robot_arm 3r_robot_arm.launch.py mode:=IK goal_position_tolerance:=0.001
```

### Sending an IK Goal

With the robot running in IK mode, send a Cartesian position goal using:

```bash
ros2 action send_goal --feedback /go_to_position project_6_3r_robot_arm_interfaces/action/GoToPosition "{x: ..., y: ..., z: ...}"
```

The action server publishes the remaining angular displacement for each joint while the arm is moving.

When the robot stops, the controller verifies that the measured end-effector position is within the configured tolerance before returning success.

---

## Project Structure

```text
Robotics_Portfolio/
│
├── RoboticArmKinematics/
│   ├── CMakeLists.txt
│   ├── include/
│   │   └── RoboticArmKinematics/
│   │       └── Kinematics.h
│   ├── src/
│   │   └── Kinematics.cpp
│   └── tests/
│       └── TestKinematics.cpp
│
└── project_6_3r_robot_arm/
    ├── README.md
    └── src/
        ├── project_6_3r_robot_arm/
        │   ├── CMakeLists.txt
        │   ├── package.xml
        │   ├── include/
        │   │   └── project_6_3r_robot_arm/
        │   │       ├── RobotArm.h
        │   │       └── RobotArmController.h
        │   ├── src/
        │   │   ├── RobotArm.cpp
        │   │   ├── RobotArmMain.cpp
        │   │   ├── RobotArmController.cpp
        │   │   └── RobotArmControllerMain.cpp
        │   ├── urdf/
        │   │   ├── 3r_robot_arm.urdf.xacro
        │   │   ├── components.xacro
        │   │   └── properties.xacro
        │   ├── launch/
        │   │   └── 3r_robot_arm.launch.py
        │   └── rviz/
        │       └── 3r_robot_arm.rviz
        │
        └── project_6_3r_robot_arm_interfaces/
            ├── CMakeLists.txt
            ├── package.xml
            ├── action/
            │   └── GoToPosition.action
            └── msg/
                ├── JointVelocities.msg
                └── RobotState.msg
```

---

## Technologies

- C++20
- ROS 2
- Eigen3
- GoogleTest
- URDF
- Xacro
- RViz2
- CMake / `ament_cmake`
- colcon

---

## What I'd Improve

Possible future extensions include:

- Add end-effector orientation to the inverse-kinematics objective
- Add joint limits directly to the numerical IK solver
- Implement damped least-squares inverse kinematics for improved behavior near singularities
- Add explicit singularity detection and reporting
- Add Cartesian trajectory generation between end-effector goals
- Add collision checking
- Add obstacle-aware motion planning
- Replace the custom joint simulation with a full physics simulator
- Expand the manipulator to additional degrees of freedom

# Project 4 — ROS 2 Differential-Drive Robot

---

A differential-drive robot model described in a URDF, visualized in RViz, and controlled with a custom teleop controller. Built with C++20 and ROS2.

---

## Demo

[Watch Demo Video](DEMO_VIDEO_URL)

---

## Overview

This project describes a differential-drive robot model and simulates motion by using differential-drive kinematics, leveraging TF broadcasting, and joint state publishing. A controller captures keyboard input and uses it to publish command velocities on `/cmd_vel` as `geometry_msgs/msg/Twist` message. A robot node then uses the robot model defined in `differential_drive_robot.urdf.xacro`, the twist messages, and differential-drive kinematics to calculate the robot's pose if the command velocities were applied to the wheels. The node then broadcasts a new transform from the `base_footprint` frame to the `odom` frame to simulate motion. Wheel rotation is simulated by calculating rotation using the twist messages and publishing new wheel joint states to `/joint_states`. The simulated motion is visualized using RViz.

---

## System Architecture

### Nodes

#### `Robot`

Responsibilities:

- Loads the robot description.
- Extracts differential-drive physical parameters from the URDF.
- Subscribes to velocity commands.
- Calculates target wheel velocities using differential-drive inverse kinematics.
- Enforces wheel velocity limits.
- Models acceleration using wheel effort and moment of inertia.
- Calculates the robot's resulting linear/angular velocity.
- Integrates robot pose over time.
- Publishes wheel joint states.
- Broadcasts the dynamic robot transform.

#### `RobotController`

Responsibilities:

- Creates an SDL keyboard-control window.
- Reads arrow-key input.
- Converts keyboard input into linear/angular velocity commands.
- Publishes `geometry_msgs/msg/Twist` messages to `/cmd_vel`.
- Stops the robot if the controller loses focus or shuts down.

Controls:

| Key | Action |
| --- | --- |
| ↑ | Forward |
| ↓ | Reverse |
| ← | Rotate left |
| → | Rotate right |

#### `FrameInspector`

Responsibilities:

- Maintains a TF2 buffer and listener.
- Looks up the transform between arbitrary frames.
- Reports translation and quaternion rotation.
- Uses configurable `from_frame` and `to_frame` parameters.

---

## ROS Graph

![ROS 2 Node Graph](docs/images/ros_graph.png)

# Project 4 — ROS 2 Differential-Drive Robot

A differential-drive robot model described in a URDF, visualized in RViz, and controlled with a custom teleop controller. Built with C++20 and ROS2.

---

## Demo

https://github.com/user-attachments/assets/77696b15-db9e-492d-9538-5ef2f8388ae8

---

## Overview

This project describes a differential-drive robot model and simulates motion by using differential-drive kinematics, leveraging TF broadcasting, and joint state publishing. A controller captures keyboard input and uses it to publish command velocities on `/cmd_vel` as `geometry_msgs/msg/Twist` message. A robot node then uses the robot model defined in `differential_drive_robot.urdf.xacro`, the twist messages, and differential-drive kinematics to calculate the robot's supposed pose if the command velocities were applied to the wheels. The node then broadcasts a new transform based on this pose from the `odom` frame to the `base_footprint` frame to simulate motion. Wheel rotation is simulated by calculating rotation using the twist messages and publishing new wheel joint states to `/joint_states`. The simulated motion is visualized using RViz.

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

<img width="1354" height="653" alt="Screenshot 2026-10-06 at 11 23 33 AM" src="https://github.com/user-attachments/assets/d77066cc-c195-4a96-9d85-ddc1e603f891" />

---

## TF Tree

<img width="1171" height="695" alt="Screenshot 2026-10-06 at 11 22 10 AM" src="https://github.com/user-attachments/assets/73f61ebb-ea38-4e29-8296-2a62787b9a05" />

### TF Hierarchy

```text
odom
└── base_footprint
    └── base_link
        ├── left_wheel_base_link
        │   └── left_wheel_link
        ├── right_wheel_base_link
        │   └── right_wheel_link
        ├── caster_wheel_base_link
        │   └── caster_wheel_link
        ├── laser_sensor_link
        └── camera_sensor_link
```

In the hierarchy, `odom` acts as the stable frame the robot moves around in and `base_footprint` acts as a convenience frame used to project the movement of the robot on a flat surface. Anything from `base_link` down represents the robot. The `*_wheel_base_link`s form anchor points for the wheels and their joints, the `*_wheel_link`s represent the wheels, and the `*_sensor_link`s act as enclosures for sensors that don't actually exist in this model but function as learning objectives for describing models in a URDF. The transforms from `odom` to `base_footprint` and from `*_wheel_base_link`s to `*_wheel_link`s are dynamic and can change as the robot move's around and the wheels rotate, all other frames are static with respect to their immediate parent.

---

## Robot Model

The description of the robot is separated into three files:

```text
urdf/
├── differential_drive_robot.urdf.xacro
├── components.xacro
└── properties.xacro
```

Where `properties.xacro` describes the physical properties of the robot and it's parts, these properties include:

- Robot dimensions
- Robot mass
- Wheel dimensions
- Wheel mass
- Sensor dimensions
- Wheel velocity limits
- Wheel effort limits

`components.xacro` describes reusable components such as wheel bases, wheels, wheel joints, and sensor housings.

`differential_drive_robot.urdf.xacro` assembles the robot using the properties and components, defines the relationship between the components, defines visual and collision geometry, and calculates moments of inertia.

The robot itself follows a differential-drive model. It is made up of a central chassis, with two sensor housings on top, these housings were included only for the sake of learning robot modeling and reasoning about transforms. Underneath the chassis are mounts for the wheels. There are two cylindrical wheels in the rear  used to drive the robot. Up front is a spherical caster wheel. Due to constraints with URDF, the joint between the caster wheel and its mount is a fixed joint and the wheel itself is given a very low friction coefficient to simulate how the wheel would function if it were free-rolling.

---

## Building and Running

### Prerequisites
```text
Ubuntu: 26.04
ROS 2: lyrical
C++: 20
```

### Clone the repository and move to project directory
```bash
git clone https://github.com/CasonKChance/Robotics_Portfolio/
cd robotics_portfolio && cd project_4_differential_drive_robot
```

### Source the workspace
```bash
source /opt/ros/lyrical/setup.bash
```

### Build the project
```bash
colcon build
```

### Source the project
In a separate tab:
```bash
source install/setup.bash
```

### Launch the nodes and RViz
```
ros2 launch project_4_differential_drive_robot differential_drive_robot.launch.py
```

### Run the FrameInspector
In a separate tab

#### Source the project
```bash
source install/setup.bash
```

#### Run the FrameInspector node, giving the origin frame and the destination frame for the transform
```bash
ros2 run project_4_differential_drive_robot FrameInspector --ros-args -p from_frame:="REPLACE_WITH_ORIGIN_FRAME_NAME" -p to_frame:="REPLACE_WITH_DESTINATION_FRAME"
```

---

## Project Structure

```text
project_4_differential_drive_robot/
└── src/
    └── project_4_differential_drive_robot/
        ├── CMakeLists.txt
        ├── package.xml
        │
        ├── include/
        │   └── project_4_differential_drive_robot/
        │       ├── FrameInspector.h
        │       ├── Pose.h
        │       ├── Robot.h
        │       ├── RobotController.h
        │       └── Twist.h
        │
        ├── src/
        │   ├── FrameInspector.cpp
        │   ├── FrameInspectorMain.cpp
        │   ├── Robot.cpp
        │   ├── RobotMain.cpp
        │   ├── RobotController.cpp
        │   └── RobotControllerMain.cpp
        │
        ├── urdf/
        │   ├── components.xacro
        │   ├── differential_drive_robot.urdf.xacro
        │   └── properties.xacro
        │
        ├── launch/
        │   └── differential_drive_robot.launch.py
        │
        └── rviz/
            └── differential_drive_robot.rviz
```

--- 

## Technologies

- C++20
- ROS 2
- rclcpp
- tf2
- URDF
- Xacro
- RViz2
- SDL2
- CMAKE / ament_cmake
- colcon

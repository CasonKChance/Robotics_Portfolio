# Project 5 — ROS 2 Rigid-Body Transformations

A differential-drive robot and simulated landmark sensor used to demonstrate rigid-body transformations between world, robot, and sensor coordinate frames. Built with C++20, Eigen, and ROS 2.

---

## Demo

INSERT DEMO VIDEO

---

## Overview

This project extends the differential-drive robot model and simulated motion from Project 4. It instantiates a landmark object in the world which publishes it's position in the world to the `/landmark_position` topic. A "landmark sensor" which is rigidly attached to the robot simulates sensor data by reading data from the /landmark_position topic and converting the read position to the sensor's frame and publishing the transformed position to the `/sensor_data` topic. The robot reads the /sensor_data position and transforms it back into the world frame then logs the position. All transformation calculations are done with a custom `Transformations` library build specially for this project. Relationships between the world frame and robot frame and the robot frame and sensor frame are provided by `TF2` and formed into `Eigen` transformation matrices using the Transformations library.

---

## System Architecture

### Robot

Responsibilities:

- Simulates differential-drive motion
- Maintains/broadcasts it's dynamic transform
- Receives landmark position measurements in the sensor frame
- Constructs the required transform chain
- Converts position measurements from the sensor frame to the world frame
- Displays/logs estimated world landmark position

A more detailed overview of the robot's motion architecture can be found in the Project 4 README.

### Robot Controller

Responsibilities:

- Reads keyboard input
- Publishes velocity commands to `/cmd_vel`

A more detailed overview of the robot controller's architecture can be found in the Project 4 README.

### Landmark Sensor

Responsibilities:

- Stores/receives the landmark position in the world frame
- Retrieves the robot and sensor transforms
- Constructs the world-to-sensor relationship
- Inverts the transformation chain
- Expresses the world landmark in the sensor coordinate frame
- Publishes the resulting sensor measurement to `/sensor_data`

### Landmark

Responsibilities:

- Represents the fixed landmark position in the world
- Publishes a marker object to the `/visualization_marker` topic for RViz visualization
- Publishes the position to the `/landmark_position` topic

---

## ROS Graph

INSERT ROS GRAPH

---

## TF Tree

INSERT TF TREE

### TF Hierarchy

```text
world
└── base_footprint
    └── base_link
        ├── left_wheel_base_link
        │   └── left_wheel_link
        ├── right_wheel_base_link
        │   └── right_wheel_link
        ├── caster_wheel_base_link
        │   └── caster_wheel_link
        ├── landmark_sensor_link
```

A more detailed description of the TF hierarch can be found in the Project 4 README with the exception of the landmark_sensor_link which replaces Project 4's non-functional camera_sensor_link and laser_sensor_link.

---

## Rigid Body Transformations

INSERT RIGID BODY TRANSFORMATIONS HERE

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
cd Robotics_Portfolio && cd project_5_rigid_body_transformations
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
ros2 launch project_5_rigid_body_transformations differential_drive_robot.launch.py
```

---

## Project Structure

```text
project_5_rigid_body_transformations/
├── Transformation/
│   ├── CMakeLists.txt
│   ├── include/
│   │   └── Transformation/
│   │       ├── Rotation.h
│   │       └── Transformation.h
│   ├── src/
│   │   ├── Rotation.cpp
│   │   └── Transformation.cpp
│   └── tests/
│       └── TestTransformation.cpp
│
└── project_5_rigid_body_transformations/
    └── src/
        └── project_5_rigid_body_transformations/
            ├── CMakeLists.txt
            ├── package.xml
            ├── include/
            │   └── project_5_rigid_body_transformations/
            │       ├── LandmarkSensor.h
            │       ├── Pose.h
            │       ├── Robot.h
            │       ├── RobotController.h
            │       └── Twist.h
            ├── src/
            │   ├── Landmark.cpp
            │   ├── LandmarkSensor.cpp
            │   ├── LandmarkSensorMain.cpp
            │   ├── Robot.cpp
            │   ├── RobotMain.cpp
            │   ├── RobotController.cpp
            │   └── RobotControllerMain.cpp
            ├── urdf/
            ├── launch/
            └── rviz/
```

---

## Technologies

- C++20
- ROS 2
- rclcpp
- tf2/tf2_ros
- Eigen3
- GoogleTest
- URDF
- Xacro
- RViz2
- SDL2
- CMAKE / ament_cmake
- colcon
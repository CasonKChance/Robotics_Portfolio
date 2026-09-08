# 2D Unicycle Robot Simulator

A C++20 simulation of a 2D mobile robot using the **unicycle kinematic model**, with configurable velocity and acceleration limits, a bounded world with obstacles and goals, simulation-state logging, and automated tests.

This project was built as the first project in my robotics software portfolio to develop a foundation in **robot kinematics, numerical simulation, C++, and robotics data analysis**.

---

## Overview

The simulator models a mobile robot whose state is represented by its planar position and orientation:

$$\begin{pmatrix} x \\ y \\ θ \end{pmatrix}$$

The robot accepts commanded linear and angular velocities:

$$\begin{bmatrix} v \\ ω \end{bmatrix}$$

and updates its state according to the unicycle model:

ẋ = v cos(θ)

ẏ = v sin(θ)

θ̇ = ω

The continuous-time equations are integrated numerically using discrete simulation timesteps.

The simulator extends the basic kinematic model with:

* Linear and angular velocity limits
* Linear and angular acceleration limits
* Actual velocity tracking of commanded velocity
* Heading normalization to `[-π, π]`
* A bounded 2D world
* Obstacles
* Goal regions
* Collision detection
* Out-of-bounds detection
* Goal-reached detection
* Robot and world CSV logging
* Python-based simulation-data analysis
* Automated tests for robot, simulator, and world behavior

---

## Goals

The primary goals of this project were to:

1. Implement a mobile-robot motion model from first principles.
2. Understand how velocity commands affect robot pose over time.
3. Build a modular simulator in modern C++.
4. Introduce realistic constraints between commanded and actual velocity.
5. Model basic interactions between a robot and its environment.
6. Validate the implementation with automated tests.
7. Build a data pipeline for analyzing simulation behavior.

---

# Robotics Model

## Robot State

The robot pose is represented by:

```text
x      Position along the world X-axis
y      Position along the world Y-axis
θ      Robot heading
```

The orientation is normalized to the range:

```text
[-π, π]
```

The implementation performs this normalization using:

```cpp
std::atan2(std::sin(angle), std::cos(angle))
```

This keeps orientation values bounded while preserving angular equivalence.

## Control Input

The robot receives a velocity command containing:

```text
linear velocity   v
angular velocity  ω
```

These values represent the desired robot motion rather than necessarily the instantaneous velocity of the robot.

---

## Unicycle Kinematics

The robot's pose evolves according to:

```text
ẋ     = v cos(θ)
ẏ     = v sin(θ)
θ̇     = ω
```

For a discrete timestep `dt`, the simulator updates the state using:

```text
x     ← x + v cos(θ) dt
y     ← y + v sin(θ) dt
θ     ← θ + ω dt
```

The implementation uses the robot's **actual velocity** when performing this integration.

This distinction becomes important once acceleration limits are introduced.

---

# Velocity and Acceleration Constraints

A commanded velocity is not applied instantaneously.

Instead, the robot maintains both:

```text
commanded velocity
actual velocity
```

When a command changes, the actual velocity moves toward the commanded velocity according to configured acceleration limits.

Conceptually:

```text
  commanded velocity
          │
          ▼
┌───────────────────┐
│ Acceleration limit│
└─────────┬─────────┘
          │
          ▼
   actual velocity
          │
          ▼
    robot kinematics
          │
          ▼
       new pose
```

Both linear and angular velocity are also clamped to their configured maximum values.

This provides a more realistic model than directly assigning the commanded velocity to the robot at every timestep.

---

# World Model

The simulator contains a 2D world that can represent:

* World boundaries
* Obstacles
* Goal regions

After each simulation step, the simulator checks the robot's current position against the world.

The simulation can enter one of several terminal states:

```text
Running
OutOfBounds
ObstacleCollision
GoalReached
```

This allows the simulator to represent basic interaction between robot motion and its environment.

---

# Simulator Architecture

The implementation separates the robot, environment, simulation loop, and logging responsibilities.

```text
                 ┌────────────────────┐
                 │       Main         │
                 │ Simulation Scenario│
                 └─────────┬──────────┘
                           │
                           ▼
                 ┌────────────────────┐
                 │     Simulator      │
                 │                    │
                 │  Simulation Clock  │
                 │  Simulation Loop   │
                 │  Collision Checks  │
                 │  Data Logging      │
                 └───────┬──────┬─────┘
                         │      │
                ┌────────┘      └────────┐
                ▼                        ▼
        ┌───────────────┐        ┌───────────────┐
        │     Robot     │        │     World     │
        │               │        │               │
        │ Pose          │        │ Bounds        │
        │ Velocity      │        │ Obstacles     │
        │ Kinematics    │        │ Goals         │
        └───────────────┘        └───────────────┘
                │                        │
                └──────────┬─────────────┘
                           ▼
                    ┌──────────────┐
                    │ CSV Logging  │
                    └──────┬───────┘
                           ▼
                    Python Analysis
```

---

# Components

## `Robot`

Responsible for the robot's internal state and motion model.

The robot:

* Stores its current pose.
* Stores commanded velocity.
* Tracks actual velocity.
* Applies acceleration limits.
* Applies maximum velocity limits.
* Integrates the unicycle equations.
* Normalizes its heading.

The core motion update is implemented in `Robot::update()`.

---

## `Simulator`

Responsible for advancing the simulation.

The simulator:

* Maintains simulation time.
* Uses a configurable timestep.
* Updates the robot.
* Checks world conditions.
* Detects collisions.
* Detects goal completion.
* Detects out-of-bounds conditions.
* Logs robot state.
* Logs world state.

The simulator also validates that the timestep is positive and supports running a simulation for an arbitrary duration, including a final partial timestep when necessary.

---

## `World`

Responsible for representing the robot's environment.

The world provides the spatial information required to determine whether the robot:

* Remains within the simulation boundaries.
* Has collided with an obstacle.
* Has reached a goal.

---

## `SimulatorDataLogger`

Simulation state is written to CSV files so that the output of the simulator can be analyzed independently from the simulation itself.

The simulator creates an `output` directory in the build tree and writes separate robot and world logs.

---

# Project Structure

```text
unicycle_robot_sim_2d/
├── data_analysis/
│   ├── analyze_simulation.py
│   └── requirements.txt
│
├── include/
│   └── simulator/
│       ├── Obstacle.h
│       ├── Pose.h
│       ├── Robot.h
│       ├── Simulator.h
│       ├── SimulatorDataLogger.h
│       ├── VelocityCommand.h
│       └── World.h
│
├── src/
│   ├── simulator/
|   |   ├── CMakeLists.txt
|   |   ├── Obstacle.cpp
|   |   ├── Robot.cpp
|   |   ├── Simulator.cpp
|   |   ├── SimulatorDataLogger.cpp
|   |   └── World.cpp
|   |
│   ├── CMakeLists.txt
│   └── Main.cpp
│
├── tests/
│   ├── CMakeLists.txt
│   ├── TestRobot.cpp
│   ├── TestSimulator.cpp
│   └── TestWorld.cpp
│
├── CMakeLists.txt
└── README.md
```

The C++ project is configured for **C++20** and requires **CMake 3.23 or newer**. Testing is enabled through the CMake build configuration.

---

# Testing

Testing is divided into three areas:

```text
tests/
├── TestRobot.cpp
├── TestSimulator.cpp
└── TestWorld.cpp
```

This separation allows the robot's mathematical behavior, simulation behavior, and world behavior to be validated independently.

## Robot Tests

The robot test suite verifies:

* Stationary behavior
* Forward motion
* Backward motion
* In-place rotation
* Heading normalization
* Square trajectories
* Maximum velocity constraints

For example, the square-trajectory test commands the robot to travel four 10-meter legs with 90° turns and verifies that it returns to its starting pose.

The velocity-limit tests also verify that commanded velocities cannot cause the robot's actual velocity to exceed the configured maximum values.

## Simulator Tests

The simulator test suite validates behavior of the simulation loop and its interaction with the robot and world.

## World Tests

The world test suite validates environmental behavior such as spatial boundaries, obstacles, and goal conditions.

---

# Data Analysis

The simulator separates data generation from data analysis.

The C++ simulator produces CSV output, while the Python tooling in:

```text
data_analysis/analyze_simulation.py
```

is responsible for analyzing the recorded simulation data.

This separation makes it possible to:

1. Run the same simulation repeatedly.
2. Record the resulting state.
3. Analyze the data independently.
4. Experiment with different visualization or analysis techniques without modifying the simulator.

---

# Build

## Requirements

* C++20-compatible compiler
* CMake 3.23+
* Python 3.x for data analysis

The CMake project explicitly requires C++20 and CMake 3.23.

## Clone

```bash
git clone https://github.com/CasonKChance/Robotics_Portfolio.git
cd Robotics_Portfolio/unicycle_robot_sim_2d
```

## Configure

```bash
mkdir build
cd build

cmake ..
```

## Build

```bash
cmake --build .
```

---

# Running the Simulator

After building, run the generated simulator executable from the build directory.

The simulation writes its output logs to:

```text
build/output/
```

including:

```text
SimulatorDataLog.csv
WorldDataLog.csv
```

The exact executable location is determined by the CMake configuration in the repository.

---

# Running Tests

Tests are integrated into the CMake build and can be run with:

```bash
ctest --test-dir build --output-on-failure
```

The repository contains dedicated robot, simulator, and world test executables.

---

# Running Data Analysis

Install the Python dependencies:

```bash
cd data_analysis
pip install -r requirements.txt
```

Then run:

```bash
python3 analyze_simulation.py
```

The analysis operates on the simulation data generated by the C++ simulator.

---

# Engineering Decisions

## Why C++20?

C++ was chosen because modern robotics software frequently requires a combination of:

* Performance
* Deterministic execution
* Strong type safety
* Low-level hardware access
* Efficient numerical computation
* Modern software-engineering practices

Using C++20 also provided an opportunity to practice modern C++ features while building something directly relevant to robotics software.

---

## Why Separate Commanded and Actual Velocity?

A real robot cannot instantaneously change its velocity.

Treating commanded velocity and actual velocity as separate states provides a simple way to model actuator limitations:

```text
Command
   │
   ▼
Acceleration Constraint
   │
   ▼
Actual Velocity
   │
   ▼
Robot Motion
```

This also creates a natural foundation for later projects involving controllers and trajectory tracking.

---

## Why a Fixed Simulation Timestep?

A discrete simulation timestep provides a simple and deterministic way to advance the robot's state.

The tradeoff is that a larger timestep can introduce greater numerical approximation error, while a smaller timestep increases the number of simulation updates required.

The simulator therefore treats timestep size as an explicit configuration parameter.

---

## Why Separate the Robot and World?

Separating the robot from the world keeps the responsibilities of each component clear.

The `Robot` owns its state and motion model.

The `World` owns environmental information.

The `Simulator` coordinates the two.

This architecture makes each component easier to test independently and provides a foundation for extending the simulator with more sophisticated robot models, sensors, planners, and controllers.

---

# Limitations

This simulator intentionally focuses on the fundamentals of mobile-robot simulation rather than physical realism.

The current model does **not** simulate:

* Wheel slip
* Individual wheel dynamics
* Motor torque
* Motor current
* Battery behavior
* Wheel-ground interaction
* Sensor noise
* IMU measurements
* Wheel encoder measurements
* GPS
* Localization uncertainty
* Realistic collision physics
* Three-dimensional motion

The robot is also modeled kinematically rather than dynamically.

These limitations are intentional. The purpose of this project is to establish a clean foundation for progressively more sophisticated robotics systems.

---

# What I Learned

## Robotics

* Unicycle robot kinematics
* Pose representation
* Linear and angular velocity
* Numerical integration
* Heading normalization
* Velocity and acceleration constraints
* Basic robot/environment interaction

## Software Engineering

* C++20
* CMake
* Simulation-state logging
* Data-analysis pipelines

## Simulation

* Discrete-time simulation
* Numerical approximation
* Simulation state management
* Terminal simulation conditions
* Reproducible data collection
* Validating mathematical models through tests

---

# Future Work

This project establishes the foundation for progressively more advanced robotics systems.

Potential extensions include:

* Differential-drive wheel modeling
* Wheel encoder simulation
* IMU simulation
* Sensor noise
* PID velocity control
* Trajectory tracking
* Odometry
* Coordinate-frame transformations
* Localization
* Kalman filtering
* SLAM
* Path planning
* ROS2 integration
* Gazebo simulation

These extensions will build toward the larger goal of developing a complete autonomous mobile-robot software stack.

---

## Project Status

**Project 1 — Complete**

The simulator, world model, logging pipeline, data-analysis tooling, and automated test suites are implemented and available in this repository.

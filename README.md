# ROS2 Sensor Fusion For A Differential Drive Robot

## Project Overview

This project implements a full ROS2 software stack for a simulated differential drive robot operating in Gazebo. The system fuses IMU and wheel odometry data through a custom complementary filter to produce a reliable pose estimate, which an autonomous proportional controller consumes to drive the robot to a user-defined goal position.

The objective was to build a complete, engineering-grade robotics pipeline rather than a minimal proof-of-concept. The sensor fusion algorithm, controller logic, custom message definitions, Gazebo world and model configuration, and launch infrastructure were each implemented as distinct, modular components.

---

## What Was Built

This project includes the following components:

- A custom sensor fusion node implementing a complementary filter over IMU and odometry inputs
- A proportional controller node computing heading error and distance-to-goal to generate velocity commands
- Simulated IMU and odometry publishers in Python producing realistic sensor data streams
- A custom `FusedPose.msg` type encoding pose, velocity, and a sensor confidence score
- A Gazebo 11 simulation environment including a custom differential drive robot model and world file
- A ROS2 launch file orchestrating the full simulation stack from a single command
- A YAML configuration file externalizing all tunable robot and controller parameters

---

## Tools And Libraries

The following tools and libraries were used throughout this project:

- **ROS2 Humble** — primary middleware for node communication, topic management, and launch infrastructure
- **Gazebo 11** — physics-based simulation environment for robot modeling and sensor emulation
- **C++17** — implementation language for the sensor fusion and controller nodes
- **Python 3.10+** — implementation language for the simulated IMU and odometry publishers
- **colcon** — build system for compiling and installing the ROS2 package
- **rosdep** — dependency resolution for ROS and system-level packages

All robot parameters and controller gains are managed through a YAML-driven configuration system, enabling parameter iteration without requiring any modification to the underlying source code.

---

## Repository Structure

```
ros2-sensor-fusion/
├── src/
│   └── differential_drive_robot/
│       ├── config/
│       │   └── robot.yaml              # Tunable robot and controller parameters
│       ├── launch/
│       │   └── simulation.launch.py    # Full stack launch orchestration
│       ├── msg/
│       │   └── FusedPose.msg           # Custom fused pose message definition
│       └── src/
│           ├── sensor_fusion_node.cpp  # Complementary filter fusion node
│           ├── controller_node.cpp     # Proportional goal-seeking controller
│           ├── imu_publisher.py        # Simulated IMU data publisher
│           └── odom_publisher.py       # Simulated odometry data publisher
├── gazebo/
│   ├── worlds/
│   │   └── robot_world.sdf             # Simulation world definition
│   └── models/
│       └── diff_drive_robot/
│           ├── model.sdf               # Robot model geometry and physics
│           └── model.config
└── README.md
```

---

## How The Control Loop Works

At each simulation time step, the following operations are performed in sequence:

1. The IMU publisher and odometry publisher emit sensor readings on `/imu_data` and `/odom_data`
2. The sensor fusion node applies a complementary filter to combine high-frequency IMU orientation updates with lower-frequency odometry position updates, producing a single `/fused_pose` output
3. The controller node reads `/fused_pose` and `/goal_pose`, computes heading error and Euclidean distance to goal, and applies proportional gains to generate a `/cmd_vel` command
4. Gazebo receives the velocity command, advances the simulation, and the cycle repeats

This architecture ensures that the full perception-to-actuation pipeline remains modular, observable via standard ROS2 topic tooling, and reproducible across simulation runs.

---

## Node Summary

| Node | Language | Subscribes | Publishes |
|---|---|---|---|
| `imu_publisher` | Python | — | `/imu_data` |
| `odom_publisher` | Python | — | `/odom_data` |
| `sensor_fusion_node` | C++ | `/imu_data`, `/odom_data` | `/fused_pose` |
| `controller_node` | C++ | `/fused_pose`, `/goal_pose` | `/cmd_vel` |

---

## Sensor Fusion Implementation Details

The complementary filter was implemented entirely from first principles and reflects several practical design decisions that matter in real deployed systems:

- **IMU Orientation Tracking** — high-frequency IMU orientation updates provide low-latency heading estimates between odometry updates
- **Odometry Correction** — wheel odometry supplies position estimates and periodically corrects heading through a weighted blend
- **Complementary Weighting** — a tunable alpha parameter controls the trust split between sensor streams, configurable in `robot.yaml` without touching node source
- **Confidence Score** — the custom `FusedPose.msg` includes a per-message confidence value derived from sensor availability and freshness, available to downstream consumers
- **Decoupled Update Rates** — the fusion node handles asynchronous arrival of IMU and odometry messages without requiring synchronized timestamps

---

## Custom Message Definition

`FusedPose.msg` combines position, orientation, velocity, and a sensor confidence score from the fusion of both input streams into a single message type consumed by the controller:

```
std_msgs/Header header
geometry_msgs/Pose2D pose
float64 linear_velocity
float64 angular_velocity
float64 confidence
```

---

## How To Run

**Install Dependencies**
```bash
git clone https://github.com/Vivaan-Vora/ros2-sensor-fusion.git
cd ros2-sensor-fusion
rosdep install --from-paths src --ignore-src -r -y
```

**Build And Source**
```bash
colcon build --symlink-install
source install/setup.bash
```

**Launch Full Simulation**
```bash
ros2 launch differential_drive_robot simulation.launch.py
```

**Set A Goal Position**
```bash
ros2 topic pub /goal_pose geometry_msgs/msg/Pose2D "{x: 3.0, y: 2.0, theta: 0.0}"
```

---

## Engineering Lessons From This Project

**Sensor fusion requires choosing what to trust and when.** A complementary filter makes the tradeoff explicit: IMU orientation is trusted for short-horizon heading estimates where latency matters, and odometry is trusted for position correction where drift accumulates. Getting the alpha weighting wrong produces either a sluggish or unstable pose estimate regardless of controller quality.

**Message design shapes system architecture.** Embedding pose, velocity, and confidence in `FusedPose.msg` kept the controller interface clean and made it straightforward to implement confidence-weighted behaviors without restructuring the topic graph.

**Asynchronous sensor streams require explicit handling.** IMU and odometry data arrive at different rates. Designing the fusion node to handle missing messages from the start prevented significant debugging overhead later.

**Configuration-driven design significantly improves iteration velocity.** Externalizing controller gains and sensor weights to `robot.yaml` allowed rapid tuning experiments without introducing regression risk in node logic.

---

## Summary

This project constitutes a complete, practical demonstration of applied robotics software engineering — spanning ROS2 node architecture, complementary filter sensor fusion, proportional goal-seeking control, Gazebo simulation integration, custom message definitions, and configuration-driven parameter management. The system is designed to be readable, modular, and extensible for future localization and navigation experimentation.

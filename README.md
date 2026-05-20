# ROS2 Sensor Fusion And Autonomous Control

---

## Project Overview

This project implements a full ROS2 software stack for a simulated differential drive robot operating in Gazebo. IMU and wheel odometry data are fused through a custom complementary filter to produce a reliable pose estimate, which an autonomous proportional controller consumes to drive the robot to a user-defined goal position.

The objective was to build a complete, engineering-grade robotics pipeline rather than a minimal proof-of-concept. The sensor fusion algorithm, controller logic, custom message definitions, Gazebo world and model configuration, and launch infrastructure were each implemented as distinct, modular components.

---

## What Was Built

- A custom sensor fusion node implementing a complementary filter over IMU and odometry inputs
- A proportional controller node that computes heading error and distance-to-goal to generate velocity commands
- Simulated IMU and odometry publishers in Python producing realistic sensor data streams
- A custom `FusedPose.msg` type encoding position, orientation, velocity, and a sensor confidence score
- A Gazebo 11 simulation environment including a custom differential drive robot model and world file
- A ROS2 launch file orchestrating the full simulation stack from a single command
- A YAML configuration file externalizing all tunable robot and controller parameters

---

## Tools And Libraries

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
ROS2-Sensor-Fusion-And-Autonomous-Control/
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
├── docs/
│   └── images/
│       ├── gazebo-simulation-preview.png
│       ├── ros2-topic-graph-preview.png
│       ├── fused-pose-terminal-preview.png
│       ├── control-loop-architecture.png
│       ├── complementary-filter-timeline.png
│       ├── robot-model-overview.png
│       └── config-tuning-preview.png
└── README.md
```

---

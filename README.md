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

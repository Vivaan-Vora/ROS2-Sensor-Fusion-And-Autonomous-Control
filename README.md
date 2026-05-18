# ROS2 Sensor Fusion And Autonomous Control

---

## Project Overview

This project implements a full ROS2 software stack for a simulated differential drive robot operating in Gazebo. IMU and wheel odometry data are fused through a custom complementary filter to produce a reliable pose estimate, which an autonomous proportional controller consumes to drive the robot to a user-defined goal position.

The objective was to build a complete, engineering-grade robotics pipeline rather than a minimal proof-of-concept. The sensor fusion algorithm, controller logic, custom message definitions, Gazebo world and model configuration, and launch infrastructure were each implemented as distinct, modular components.

---

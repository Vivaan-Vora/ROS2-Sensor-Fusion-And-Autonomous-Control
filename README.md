# ros2-sensor-fusion

A ROS2 project simulating a differential drive robot in Gazebo with IMU and wheel odometry sensor fusion and autonomous goal-seeking behavior.

## Overview

This project implements a full ROS2 software stack for a simulated differential drive robot. It integrates IMU and wheel odometry data through a custom sensor fusion node to produce a reliable pose estimate, which is then consumed by a proportional controller that autonomously drives the robot to a target goal position.

## Architecture

```
/imu_data  ──────────────────┐
                              ▼
                        [sensor_fusion_node]  ──▶  /fused_pose
                              ▲
/odom_data  ─────────────────┘
                                                        │
                                                        ▼
                                              [controller_node]  ──▶  /cmd_vel
```

## Package Structure

```
ros2-sensor-fusion/
  src/
    differential_drive_robot/
      config/
        robot.yaml
      launch/
        simulation.launch.py
      msg/
        FusedPose.msg
      src/
        sensor_fusion_node.cpp
        controller_node.cpp
        imu_publisher.py
        odom_publisher.py
      CMakeLists.txt
      package.xml
  gazebo/
    worlds/
      robot_world.sdf
    models/
      diff_drive_robot/
        model.sdf
        model.config
  README.md
```

## Dependencies

- ROS2 Humble
- Gazebo 11
- colcon
- Python 3.10+
- C++17

## Setup

```bash
# Clone the repo
git clone https://github.com/yourusername/ros2-sensor-fusion.git
cd ros2-sensor-fusion

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source
source install/setup.bash
```

## Running

```bash
# Launch full simulation
ros2 launch differential_drive_robot simulation.launch.py

# Set a goal position (x, y, theta)
ros2 topic pub /goal_pose geometry_msgs/msg/Pose2D "{x: 3.0, y: 2.0, theta: 0.0}"
```

## Nodes

| Node | Language | Subscribes | Publishes |
|---|---|---|---|
| imu_publisher | Python | — | /imu_data |
| odom_publisher | Python | — | /odom_data |
| sensor_fusion_node | C++ | /imu_data, /odom_data | /fused_pose |
| controller_node | C++ | /fused_pose, /goal_pose | /cmd_vel |

## Custom Message

`FusedPose.msg` combines position, orientation, and confidence score from the fusion of both sensors into a single message type consumed by the controller.

## How It Works

The sensor fusion node runs a complementary filter combining high-frequency IMU orientation updates with lower-frequency odometry position updates. The controller node computes heading error and distance to goal and applies proportional gains to generate linear and angular velocity commands published to `/cmd_vel`.

#!/usr/bin/env python3
"""Publish simulated IMU data for the differential drive robot stack."""

import math
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Return an x, y, z, w quaternion for a planar yaw angle."""
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class ImuPublisher(Node):
    """Generate a configurable planar IMU stream."""

    def __init__(self) -> None:
        super().__init__("imu_publisher")
        self.declare_parameter("topic_name", "/imu_data")
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("angular_velocity_z", 0.25)
        self.declare_parameter("linear_acceleration_x", 0.0)
        self.declare_parameter("orientation_noise_stddev", 0.002)
        self.declare_parameter("angular_velocity_noise_stddev", 0.001)

        topic_name = self.get_parameter("topic_name").value
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        timer_period = 1.0 / max(publish_rate_hz, 1.0)

        self.publisher = self.create_publisher(Imu, topic_name, 10)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(timer_period, self.publish_imu)
        self.get_logger().info(f"Publishing simulated IMU data on {topic_name}")

    def publish_imu(self) -> None:
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9

        angular_velocity_z = float(self.get_parameter("angular_velocity_z").value)
        linear_acceleration_x = float(self.get_parameter("linear_acceleration_x").value)
        orientation_noise = random.gauss(
            0.0, float(self.get_parameter("orientation_noise_stddev").value)
        )
        gyro_noise = random.gauss(
            0.0, float(self.get_parameter("angular_velocity_noise_stddev").value)
        )

        yaw = angular_velocity_z * elapsed + orientation_noise
        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        msg = Imu()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.angular_velocity.z = angular_velocity_z + gyro_noise
        msg.linear_acceleration.x = linear_acceleration_x

        msg.orientation_covariance[0] = 0.01
        msg.orientation_covariance[4] = 0.01
        msg.orientation_covariance[8] = 0.02
        msg.angular_velocity_covariance[8] = 0.01
        msg.linear_acceleration_covariance[0] = 0.05

        self.publisher.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ImuPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

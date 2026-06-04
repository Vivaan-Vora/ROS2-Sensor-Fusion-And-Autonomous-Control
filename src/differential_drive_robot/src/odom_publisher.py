#!/usr/bin/env python3
"""Publish simulated wheel odometry for the differential drive robot stack."""

import math
import random

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Return an x, y, z, w quaternion for a planar yaw angle."""
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class OdomPublisher(Node):
    """Generate wheel odometry from a simple differential-drive motion model."""

    def __init__(self) -> None:
        super().__init__("odom_publisher")
        self.declare_parameter("topic_name", "/odom_data")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("linear_velocity", 0.4)
        self.declare_parameter("angular_velocity", 0.25)
        self.declare_parameter("position_noise_stddev", 0.01)
        self.declare_parameter("theta_noise_stddev", 0.002)

        topic_name = self.get_parameter("topic_name").value
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        timer_period = 1.0 / max(publish_rate_hz, 1.0)

        self.publisher = self.create_publisher(Odometry, topic_name, 10)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(timer_period, self.publish_odom)
        self.get_logger().info(f"Publishing simulated odometry on {topic_name}")

    def publish_odom(self) -> None:
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9

        linear_velocity = float(self.get_parameter("linear_velocity").value)
        angular_velocity = float(self.get_parameter("angular_velocity").value)
        position_noise_stddev = float(
            self.get_parameter("position_noise_stddev").value
        )
        theta_noise_stddev = float(self.get_parameter("theta_noise_stddev").value)

        theta = angular_velocity * elapsed
        if abs(angular_velocity) < 1e-6:
            x = linear_velocity * elapsed
            y = 0.0
        else:
            radius = linear_velocity / angular_velocity
            x = radius * math.sin(theta)
            y = radius * (1.0 - math.cos(theta))

        x += random.gauss(0.0, position_noise_stddev)
        y += random.gauss(0.0, position_noise_stddev)
        theta += random.gauss(0.0, theta_noise_stddev)
        qx, qy, qz, qw = yaw_to_quaternion(theta)

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.child_frame_id = str(self.get_parameter("child_frame_id").value)
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = linear_velocity
        msg.twist.twist.angular.z = angular_velocity

        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.02
        msg.twist.covariance[0] = 0.02
        msg.twist.covariance[35] = 0.02

        self.publisher.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

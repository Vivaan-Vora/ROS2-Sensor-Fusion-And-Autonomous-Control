#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "differential_drive_robot/msg/fused_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

double normalize_angle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double yaw_from_quaternion(double x, double y, double z, double w)
{
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double blend_angle(double base, double target, double target_weight)
{
  return normalize_angle(base + target_weight * normalize_angle(target - base));
}

}  // namespace

class SensorFusionNode : public rclcpp::Node
{
public:
  SensorFusionNode()
  : Node("sensor_fusion_node")
  {
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu_data");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom_data");
    fused_pose_topic_ = declare_parameter<std::string>("fused_pose_topic", "/fused_pose");
    frame_id_ = declare_parameter<std::string>("frame_id", "odom");
    imu_orientation_weight_ = declare_parameter<double>("imu_orientation_weight", 0.98);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);

    imu_orientation_weight_ = std::clamp(imu_orientation_weight_, 0.0, 1.0);
    const double timer_period = 1.0 / std::max(publish_rate_hz_, 1.0);

    fused_pose_pub_ =
      create_publisher<differential_drive_robot::msg::FusedPose>(fused_pose_topic_, 10);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 10, std::bind(&SensorFusionNode::handle_imu, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, std::bind(&SensorFusionNode::handle_odom, this, std::placeholders::_1));
    publish_timer_ = create_wall_timer(
      std::chrono::duration<double>(timer_period),
      std::bind(&SensorFusionNode::publish_fused_pose, this));

    RCLCPP_INFO(
      get_logger(), "Fusing IMU topic '%s' and odometry topic '%s' into '%s'",
      imu_topic_.c_str(), odom_topic_.c_str(), fused_pose_topic_.c_str());
  }

private:
  void handle_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    latest_imu_yaw_ = yaw_from_quaternion(
      msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    latest_angular_velocity_ = msg->angular_velocity.z;
    last_imu_stamp_ = msg->header.stamp;
    has_imu_ = true;

    if (has_fused_pose_) {
      fused_theta_ = blend_angle(fused_theta_, latest_imu_yaw_, imu_orientation_weight_);
    }
  }

  void handle_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    fused_x_ = msg->pose.pose.position.x;
    fused_y_ = msg->pose.pose.position.y;
    latest_linear_velocity_ = msg->twist.twist.linear.x;
    latest_angular_velocity_ = msg->twist.twist.angular.z;
    last_odom_stamp_ = msg->header.stamp;

    const double odom_yaw = yaw_from_quaternion(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    fused_theta_ = has_imu_ ? blend_angle(odom_yaw, latest_imu_yaw_, imu_orientation_weight_) : odom_yaw;

    has_odom_ = true;
    has_fused_pose_ = true;
  }

  void publish_fused_pose()
  {
    if (!has_fused_pose_) {
      return;
    }

    differential_drive_robot::msg::FusedPose msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.pose.x = fused_x_;
    msg.pose.y = fused_y_;
    msg.pose.theta = normalize_angle(fused_theta_);
    msg.linear_velocity = latest_linear_velocity_;
    msg.angular_velocity = latest_angular_velocity_;
    msg.confidence = compute_confidence();  // derived from sensor freshness
    fused_pose_pub_->publish(msg);
  }

  double compute_confidence()
  {
    if (!has_imu_ || !has_odom_) {
      return 0.35;
    }

    const rclcpp::Time current_time = now();
    const double imu_age = std::max(0.0, (current_time - rclcpp::Time(last_imu_stamp_)).seconds());
    const double odom_age = std::max(0.0, (current_time - rclcpp::Time(last_odom_stamp_)).seconds());
    const double age_penalty = std::min(0.6, imu_age * 0.5 + odom_age * 0.3);
    return std::clamp(1.0 - age_penalty, 0.0, 1.0);
  }

  std::string imu_topic_;
  std::string odom_topic_;
  std::string fused_pose_topic_;
  std::string frame_id_;
  double imu_orientation_weight_{0.98};
  double publish_rate_hz_{30.0};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<differential_drive_robot::msg::FusedPose>::SharedPtr fused_pose_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  bool has_imu_{false};
  bool has_odom_{false};
  bool has_fused_pose_{false};
  double fused_x_{0.0};
  double fused_y_{0.0};
  double fused_theta_{0.0};
  double latest_imu_yaw_{0.0};
  double latest_linear_velocity_{0.0};
  double latest_angular_velocity_{0.0};
  builtin_interfaces::msg::Time last_imu_stamp_;
  builtin_interfaces::msg::Time last_odom_stamp_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorFusionNode>());
  rclcpp::shutdown();
  return 0;
}

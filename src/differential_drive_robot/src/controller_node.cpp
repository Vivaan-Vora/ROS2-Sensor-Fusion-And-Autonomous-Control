#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "differential_drive_robot/msg/fused_pose.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

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

}  // namespace

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode()
  : Node("controller_node")
  {
    fused_pose_topic_ = declare_parameter<std::string>("fused_pose_topic", "/fused_pose");
    goal_pose_topic_ = declare_parameter<std::string>("goal_pose_topic", "/goal_pose");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);
    k_linear_ = declare_parameter<double>("k_linear", 0.8);
    k_angular_ = declare_parameter<double>("k_angular", 1.8);
    k_heading_ = declare_parameter<double>("k_heading", 1.2);
    max_linear_velocity_ = declare_parameter<double>("max_linear_velocity", 0.6);
    max_angular_velocity_ = declare_parameter<double>("max_angular_velocity", 1.5);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.08);
    heading_tolerance_ = declare_parameter<double>("heading_tolerance", 0.05);

    const double timer_period = 1.0 / std::max(control_rate_hz_, 1.0);

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    fused_pose_sub_ = create_subscription<differential_drive_robot::msg::FusedPose>(
      fused_pose_topic_, 10,
      std::bind(&ControllerNode::handle_fused_pose, this, std::placeholders::_1));
    goal_pose_sub_ = create_subscription<geometry_msgs::msg::Pose2D>(
      goal_pose_topic_, 10,
      std::bind(&ControllerNode::handle_goal_pose, this, std::placeholders::_1));
    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(timer_period),
      std::bind(&ControllerNode::publish_control, this));

    RCLCPP_INFO(
      get_logger(), "Controller waiting for '%s' and '%s'",
      fused_pose_topic_.c_str(), goal_pose_topic_.c_str());
  }

private:
  void handle_fused_pose(const differential_drive_robot::msg::FusedPose::SharedPtr msg)
  {
    current_pose_ = *msg;
    has_pose_ = true;
  }

  void handle_goal_pose(const geometry_msgs::msg::Pose2D::SharedPtr msg)
  {
    goal_pose_ = *msg;
    has_goal_ = true;
    goal_reached_logged_ = false;
    RCLCPP_INFO(
      get_logger(), "Received goal x=%.2f y=%.2f theta=%.2f",
      goal_pose_.x, goal_pose_.y, goal_pose_.theta);
  }

  void publish_control()
  {
    if (!has_pose_ || !has_goal_) {
      return;
    }

    const double dx = goal_pose_.x - current_pose_.pose.x;
    const double dy = goal_pose_.y - current_pose_.pose.y;
    const double distance = std::hypot(dx, dy);
    const double target_heading = std::atan2(dy, dx);
    const double heading_error = normalize_angle(target_heading - current_pose_.pose.theta);
    const double final_heading_error = normalize_angle(goal_pose_.theta - current_pose_.pose.theta);

    geometry_msgs::msg::Twist cmd;
    if (distance > goal_tolerance_) {
      const double heading_scale = std::max(0.0, std::cos(heading_error));
      cmd.linear.x = std::clamp(
        k_linear_ * distance * heading_scale, -max_linear_velocity_, max_linear_velocity_);
      cmd.angular.z = std::clamp(
        k_angular_ * heading_error, -max_angular_velocity_, max_angular_velocity_);
    } else if (std::abs(final_heading_error) > heading_tolerance_) {
      cmd.angular.z = std::clamp(
        k_heading_ * final_heading_error, -max_angular_velocity_, max_angular_velocity_);
    } else if (!goal_reached_logged_) {
      RCLCPP_INFO(get_logger(), "Goal reached within tolerance");
      goal_reached_logged_ = true;
    }

    cmd_vel_pub_->publish(cmd);
  }

  std::string fused_pose_topic_;
  std::string goal_pose_topic_;
  std::string cmd_vel_topic_;
  double control_rate_hz_{20.0};
  double k_linear_{0.8};
  double k_angular_{1.8};
  double k_heading_{1.2};
  double max_linear_velocity_{0.6};
  double max_angular_velocity_{1.5};
  double goal_tolerance_{0.08};
  double heading_tolerance_{0.05};

  rclcpp::Subscription<differential_drive_robot::msg::FusedPose>::SharedPtr fused_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr goal_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  bool has_pose_{false};
  bool has_goal_{false};
  bool goal_reached_logged_{false};
  differential_drive_robot::msg::FusedPose current_pose_;
  geometry_msgs::msg::Pose2D goal_pose_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}

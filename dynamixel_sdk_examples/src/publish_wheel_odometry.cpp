#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/get_velocity.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

#include "publish_wheel_odometry.hpp"

// Default setting
#define WHEEL_RADIUS 0.035  // Replace with actual value [m]
#define WHEEL_BASE 0.2    // Replace with actual value [m]

PublishWheelOdometry::PublishWheelOdometry()
: Node("publish_wheel_odometry")
{
  RCLCPP_INFO(this->get_logger(), "Run publish wheel odometry node");

  x_ = 0.0;
  y_ = 0.0;
  theta_ = 0.0;

  last_time_ = this->now();

  odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  get_velocity_subscriber_ =
    this->create_subscription<GetVelocity>(
    "get_velocity",
    10,
    [this](const GetVelocity::SharedPtr msg) -> void
    {
      rclcpp::Time current_time = this->now();
      double dt = (current_time - last_time_).seconds();
      float left_velocity = 0.0f;
      float right_velocity = 0.0f;
      
      left_velocity = msg->left_vel; // Left wheel velocity
      right_velocity = msg->right_vel; // Right wheel velocity

      // Convert rpm to m/s using wheel radius
      double left_speed_mps = (left_velocity * WHEEL_RADIUS * 2 * M_PI) / 60; // Convert rpm to rad/s
      double right_speed_mps = (right_velocity * WHEEL_RADIUS * 2 * M_PI) / 60; // Convert rpm to rad/s

      // Calculate robot's linear and angular velocity
      double v = (left_speed_mps + right_speed_mps) / 2;
      double w = (right_speed_mps - left_speed_mps) / WHEEL_BASE;
      RCLCPP_INFO(this->get_logger(), "left: %f, right: %f", left_velocity, right_velocity);
      RCLCPP_INFO(this->get_logger(), "v: %f, w: %f", v, w);

      // Calculate pose changes
      double dx = v * cos(theta_) * dt;
      double dy = v * sin(theta_) * dt;
      double dtheta = w * dt;

      // Update robot's pose
      x_ += dx;
      y_ += dy;
      theta_ += dtheta;

      // Calculate quaternion for orientation
      tf2::Quaternion quaternion;
      quaternion.setRPY(0, 0, theta_);

      // Publish odometry message
      auto odom = nav_msgs::msg::Odometry();
      odom.header.stamp = this->now();
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_link";
      odom.pose.pose.position.x = x_;
      odom.pose.pose.position.y = y_;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = tf2::toMsg(quaternion);
      odometry_publisher_->publish(odom);

      last_time_ = current_time;
    }
  );
}

PublishWheelOdometry::~PublishWheelOdometry()
{
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto publishwheelodometry = std::make_shared<PublishWheelOdometry>();
    rclcpp::spin(publishwheelodometry);
    rclcpp::shutdown();
    return 0;
}
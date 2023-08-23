#ifndef PUBLISH_WHEEL_ODOMETRY_HPP_
#define PUBLISH_WHEEL_ODOMETRY_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/get_velocity.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class PublishWheelOdometry : public rclcpp::Node
{
public:
  using GetVelocity = dynamixel_sdk_custom_interfaces::msg::GetVelocity;
  PublishWheelOdometry();
  virtual ~PublishWheelOdometry();

private:
  rclcpp::Subscription<GetVelocity>::SharedPtr get_velocity_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;

  double x_;
  double y_;
  double theta_;

  rclcpp::Time last_time_;
  
};

#endif  // PUBLISH_WHEEL_ODOMETRY_HPP_
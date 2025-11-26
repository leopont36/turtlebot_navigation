#ifndef INITIAL_POSE_SETTER_HPP
#define INITIAL_POSE_SETTER_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "nav2_msgs/srv/set_initial_pose.hpp"

class InitialPoseSetter: public rclcpp::Node
{
public:
  InitialPoseSetter();

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Client<nav2_msgs::srv::SetInitialPose>::SharedPtr initial_pose_client_;
  bool setted_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};

#endif // INITIAL_POSE_PUBLISHER_HPP

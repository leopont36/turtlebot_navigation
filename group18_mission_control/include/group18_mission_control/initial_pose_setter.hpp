#ifndef INITIAL_POSE_SETTER_HPP
#define INITIAL_POSE_SETTER_HPP

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

class InitialPoseSetter: public rclcpp::Node
{
public:
  InitialPoseSetter();

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr tr_event_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  bool amcl_on_, published_;

  void event_callback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};

#endif // INITIAL_POSE_SETTER_HPP

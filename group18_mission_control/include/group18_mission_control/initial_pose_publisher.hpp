#ifndef INITIAL_POSE_PUBLISHER_HPP
#define INITIAL_POSE_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <lifecycle_msgs/msg/transition_event.hpp>
#include <lifecycle_msgs/msg/state.hpp>

class InitialPosePublisher : public rclcpp::Node
{
public:
  InitialPosePublisher();

private:
  rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr amcl_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_;
    
  bool activated_;
  bool odom_received_;
  bool published_;
    
  int publish_attempts_;
  int max_attempts_;

  void amcl_callback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void TryPublish();
};

#endif // INITIAL_POSE_PUBLISHER_HPP
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
/**
 * @brief Constructor for the InitialPoseSetter node.
 * Initializes initial pose setter, odom subscribers.
 */
  InitialPoseSetter();

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr tr_event_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  bool amcl_on_, published_;

/**
 * @brief Callback function for lifecycle transition event.
 * This method set the flag amcl_on_ true when amcl is activated
 * @param msg Lifecycle transition event
 */
  void eventCallback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg);

/**
 * @brief Callback function for odometry messages.
 * This method stores the robot's position and orientation as the initial pose.
 * The initial pose is only updated until it has been published.
 * @param msg Odometry message containing pose and covariance of the robot
 */
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
};

#endif // INITIAL_POSE_SETTER_HPP

#include "group18_mission_control/initial_pose_setter.hpp"

InitialPoseSetter::InitialPoseSetter() : Node("initial_pose_setter"), amcl_on_(false), published_(false)
{
  // Create subscriber to odometry
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&InitialPoseSetter::odomCallback, this, std::placeholders::_1));

  tr_event_sub_ = create_subscription<lifecycle_msgs::msg::TransitionEvent>( "/amcl/transition_event", 10, std::bind(&InitialPoseSetter::eventCallback, this, std::placeholders::_1));

  // Create publisher for the initial pose
  initial_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

  RCLCPP_INFO(this->get_logger(), "InitialPoseSetter node has been started.");
}

void InitialPoseSetter::eventCallback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg)
{
  if (msg->goal_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    RCLCPP_INFO(this->get_logger(), "AMCL ready to get initial pose");
    amcl_on_ = true;
  }
}

void InitialPoseSetter::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!amcl_on_ || published_)
    return;

  RCLCPP_INFO(this->get_logger(), "Getting initial pose from /odom");

  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
  initial_pose.header.stamp = this->now();
  initial_pose.header.frame_id = "map";
  initial_pose.pose = msg->pose;

  initial_pose_pub_->publish(initial_pose);

  RCLCPP_INFO(this->get_logger(), "Initial pose published to /initialpose");

  published_ = true;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InitialPoseSetter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

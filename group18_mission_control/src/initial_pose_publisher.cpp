#include "group18_mission_control/initial_pose_publisher.hpp"

InitialPosePublisher::InitialPosePublisher() : Node("initial_pose_publisher"), activated_(false), odom_received_(false), published_(false), publish_attempts_(0), max_attempts_(10)
{
  amcl_sub_ = create_subscription<lifecycle_msgs::msg::TransitionEvent>( "/amcl/transition_event", 10, std::bind(&InitialPosePublisher::amcl_callback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&InitialPosePublisher::odom_callback, this, std::placeholders::_1));
  initial_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
  publish_timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&InitialPosePublisher::TryPublish, this));
  RCLCPP_INFO(get_logger(), "InitialPosePublisher started, waiting for AMCL startup...");
}

void InitialPosePublisher::amcl_callback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg)
{
  if (msg->goal_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    RCLCPP_INFO(get_logger(), "AMCL is active");
    activated_ = true;
  }
}

void InitialPosePublisher::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (published_)
    return;

  initial_pose_.header.stamp = this->now();
  initial_pose_.header.frame_id = "map";
  initial_pose_.pose.pose.position = msg->pose.pose.position;
  initial_pose_.pose.pose.orientation = msg->pose.pose.orientation;
  initial_pose_.pose.covariance = msg->pose.covariance;

  if (!odom_received_) {
    RCLCPP_INFO(this->get_logger(), "First odom received");
    odom_received_ = true;
  }
}

void InitialPosePublisher::TryPublish()
{
  if (published_) 
  {
    publish_timer_->cancel();
    return;
  }

  if (!odom_received_ || !activated_)
    return;

  if (initial_pose_pub_->get_subscription_count() == 0) 
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for subscriber on /initialpose...");
    return;
  }

  initial_pose_.header.stamp = this->now();
  initial_pose_pub_->publish(initial_pose_);
  publish_attempts_++;

  RCLCPP_INFO(this->get_logger(), "Initial pose published (attempt %d): x=%.3f, y=%.3f", publish_attempts_, initial_pose_.pose.pose.position.x, initial_pose_.pose.pose.position.y);

  if (publish_attempts_ >= max_attempts_) 
  {
    published_ = true;
    publish_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Initial pose publishing complete");
  }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InitialPosePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
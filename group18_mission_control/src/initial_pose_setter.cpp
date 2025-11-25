#include "group18_mission_control/initial_pose_setter.hpp"

/**
 * @brief Constructor for the InitialPoseSetter node.
 * Initializes initial pose setter, odom subscribers.
 */
InitialPoseSetter::InitialPoseSetter() : Node("initial_pose_setter"), setted_(false)
{
  // Create subscriber to odometry
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&InitialPoseSetter::odom_callback, this, std::placeholders::_1));

  // Create client for set_initial_pose service
  initial_pose_client_ = create_client<nav2_msgs::srv::SetInitialPose>("/set_initial_pose");

  RCLCPP_INFO(this->get_logger(), "InitialPoseSetter node has been started.");
}

/**
 * @brief Callback function for odometry messages.
 * This method stores the robot's position and orientation as the initial pose.
 * The initial pose is only updated until it has been published.
 * @param msg Odometry message containing pose and covariance of the robot
 */
void InitialPoseSetter::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!initial_pose_client_ ->service_is_ready()|| setted_)
    return;

  RCLCPP_INFO(this->get_logger(), "AMCL ready to get initial pose");

  auto request = std::make_shared<nav2_msgs::srv::SetInitialPose::Request>();
  request->pose.header.stamp = this->now();
  request->pose.header.frame_id = "map";
  request->pose.pose = msg->pose;

  auto future = initial_pose_client_->async_send_request(request,
    [this](rclcpp::Client<nav2_msgs::srv::SetInitialPose>::SharedFuture future) {
      (void) future;
      RCLCPP_INFO(this->get_logger(), "Initial pose setted in AMCL");
      setted_ = true;
  });

}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InitialPoseSetter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

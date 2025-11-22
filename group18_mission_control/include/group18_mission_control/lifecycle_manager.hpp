#ifndef LIFECYCLE_MANAGER_HPP_
#define LIFECYCLE_MANAGER_HPP_

#include <memory>
#include <string>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"

class LifecycleManager : public rclcpp::Node
{
public:
  LifecycleManager();

private:
  void startupSequence();
  bool callStartup(rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr client);

  
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr client_localization_;
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr client_navigation_;
  std::thread startup_thread_;
};

#endif  // LIFECYCLE_MANAGER_HPP_

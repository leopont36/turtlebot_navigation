#ifndef LIFECYCLE_MANAGER_HPP
#define LIFECYCLE_MANAGER_HPP

#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"

class LifecycleManager : public rclcpp::Node
{
public:
  LifecycleManager();
  void startup();

private:
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr client_localization_;
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr client_navigation_;
  
  void callStartup(rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr client, const std::string & name);
};

#endif  // LIFECYCLE_MANAGER_HPP
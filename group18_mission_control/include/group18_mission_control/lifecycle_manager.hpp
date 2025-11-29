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
/**
 * @brief Constructor for the LifecycleManager node.
 * * Initializes the Service Clients for the Nav2 localization and navigation.
 * It also create a thread to handle the startup sequence
 */
  LifecycleManager();

private:
/**
 * @brief Initialize the startup stack for the navigation
 * * The localization start before the navigation is attempted.
 */
  void startupSequence();

/**
 * @brief Function to send startup to the lifecycle service
 * * @param client The shared pointer of a service client
 * @return True if the transition to 'active' state was successful. False otherwise.
 */
  bool callStartup(rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr client);

  
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr client_localization_;
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr client_navigation_;
  std::thread startup_thread_;
};

#endif  // LIFECYCLE_MANAGER_HPP_

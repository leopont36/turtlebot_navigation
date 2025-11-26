#include "group18_mission_control/lifecycle_manager.hpp"
using namespace std::chrono_literals;

/**
 * @brief Constructor for the LifecycleManager node.
 * * Initializes the Service Clients for the Nav2 localization and navigation.
 * It also create a thread to handle the startup sequence
 */
LifecycleManager::LifecycleManager() : Node("lifecycle_manager_client")
{
  client_localization_ = create_client<nav2_msgs::srv::ManageLifecycleNodes>("/lifecycle_manager_localization/manage_nodes");
  client_navigation_ = create_client<nav2_msgs::srv::ManageLifecycleNodes>("/lifecycle_manager_navigation/manage_nodes");
  startup_thread_ = std::thread(&LifecycleManager::startupSequence, this);
  startup_thread_.detach();
}

/**
 * @brief Initialize the startup stack for the navigation
 * * The localization start before the navigation is attempted.
 */
void LifecycleManager::startupSequence()
{
  
  // Delay to allow system to load other node
  std::this_thread::sleep_for(500ms);
  RCLCPP_INFO(get_logger(), "Starting Nav2 stack...");

  // Attempt to activate the Localization
  if (!callStartup(client_localization_))
  {
    RCLCPP_ERROR(get_logger(), "Failed to start localization, aborting");
    return;
  }

  // Attempt to activate Navigation
  if (!callStartup(client_navigation_))
  {
    RCLCPP_ERROR(get_logger(), "Failed to start navigation");
    return;
  }

  RCLCPP_INFO(get_logger(), "Nav2 stack started successfully!");
}

/**
 * @brief Function to send startup to the lifecycle service
 * * @param client The shared pointer of a service client
 * @return True if the transition to 'active' state was successful. False otherwise.
 */
bool LifecycleManager::callStartup(rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr client)
{
  RCLCPP_INFO(get_logger(), "Waiting for service...");

  // Wait 10 seconds for the lifecycle manager service to become active
  if (!client->wait_for_service(10s))
  {
    RCLCPP_ERROR(get_logger(), "Service not available");
    return false;
  }

  // Construct the service request
  auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
  request->command = nav2_msgs::srv::ManageLifecycleNodes::Request::STARTUP;

  // Send request asynchronously
  RCLCPP_INFO(get_logger(), "Sending STARTUP...");
  auto future = client->async_send_request(request);

  // Wait 30 seconds to complete the transition
  auto status = future.wait_for(30s);
  if (status != std::future_status::ready)
  {
    RCLCPP_ERROR(get_logger(), "Startup timeout");
    return false;
  }

  // Analize the response
  auto response = future.get();
  if (response->success)
  {
    RCLCPP_INFO(get_logger(), "Started successfully");
    return true;
  }

  RCLCPP_ERROR(get_logger(), "Startup failed");
  return false;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LifecycleManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

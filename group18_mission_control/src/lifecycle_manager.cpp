#include "lifecycle_manager.hpp"

LifecycleManager::LifecycleManager() : Node("lifecycle_manager_client")
{
  client_localization_ = create_client<nav2_msgs::srv::ManageLifecycleNodes>("/lifecycle_manager_localization/manage_nodes");
  client_navigation_ = create_client<nav2_msgs::srv::ManageLifecycleNodes>("/lifecycle_manager_navigation/manage_nodes");
}

void LifecycleManager::startup()
{
  callStartup(client_localization_, "Localization");
  callStartup(client_navigation_, "Navigation");
}

void LifecycleManager::callStartup(rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr client, const std::string & name)
{
  if (!client->wait_for_service(std::chrono::seconds(5))) 
  {
    RCLCPP_ERROR(get_logger(), "%s service not available", name.c_str());
    return;
  }

  auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
  request->command = nav2_msgs::srv::ManageLifecycleNodes::Request::STARTUP;

  auto future = client->async_send_request(request);
  
  if (rclcpp::spin_until_future_complete(shared_from_this(), future) == rclcpp::FutureReturnCode::SUCCESS && future.get()->success)
    RCLCPP_INFO(get_logger(), "%s started", name.c_str());
  
  else 
    RCLCPP_ERROR(get_logger(), "%s startup failed", name.c_str());
  
}
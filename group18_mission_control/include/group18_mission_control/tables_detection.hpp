#ifndef TABLES_DETECTION_HPP
#define TABLES_DETECTION_HPP

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "group18_interfaces/srv/table_count.hpp"
#include "group18_mission_control/circle.hpp" 

class TablesDetection : public rclcpp::Node
{
public:
  TablesDetection();

private:
  // member functions
  void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);
  
  void send_n_tables(
    const std::shared_ptr<group18_interfaces::srv::TableCount::Request> request,
    std::shared_ptr<group18_interfaces::srv::TableCount::Response> response);

  // variables
  int table_count;

  // ROS Interfaces
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Service<group18_interfaces::srv::TableCount>::SharedPtr service_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
  
  // TF2
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

#endif // TABLES_DETECTION_HPP
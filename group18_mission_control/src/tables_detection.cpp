#include "group18_mission_control/tables_detection.hpp"

#include <numeric>
#include "tf2/exceptions.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

TablesDetection::TablesDetection()
: Node("tables_detection"), table_count(0)
{
  // service for counting tables
  service_ = this->create_service<group18_interfaces::srv::TableCount>(
    "table_count", 
    std::bind(&TablesDetection::send_n_tables, this, _1, _2));
  
  RCLCPP_INFO(this->get_logger(), "Ready to check the number of tables.");

  // publisher for tables positions
  publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("tables", 10);

  // TF2
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscription
  subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 10, 
    std::bind(&TablesDetection::scan_callback, this, _1));
}

void TablesDetection::send_n_tables(
  const std::shared_ptr<group18_interfaces::srv::TableCount::Request> /*request*/,
  std::shared_ptr<group18_interfaces::srv::TableCount::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Incoming request for detection of tables...");
  response->n_tables = table_count;
  RCLCPP_INFO(this->get_logger(), "sending back response: [%ld]", response->n_tables);
}

void TablesDetection::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg) 
{
  std::vector<float> ranges = msg->ranges;
  float index_start_cluster = -1;
  float cluster_threshold = 0.6;
  int local_table_count = 0;
  
  std::vector<geometry_msgs::msg::Pose> table_poses;
  std::vector<geometry_msgs::msg::Point> current_cluster_points;

  for (size_t i = 0; i < ranges.size() - 1; i++) {
    float r = ranges[i];
    bool is_range_valid = (msg->range_min < r && r < msg->range_max);

    if (is_range_valid) {
      if (index_start_cluster == -1) {
        index_start_cluster = i;
        current_cluster_points.clear(); 
      }

      // polar to cartesian in 'base_scan'
      float angle = msg->angle_min + i * msg->angle_increment;
      geometry_msgs::msg::Point p;
      p.x = r * std::cos(angle);
      p.y = r * std::sin(angle);
      p.z = 0;
      current_cluster_points.push_back(p);

      // Check if cluster ends
      if (std::abs(r - ranges[i+1]) > cluster_threshold || !is_range_valid) {
        
        // min points check, must be greater that 3 to form a circle
        if (current_cluster_points.size() >= 3) {
          
          Circle cluster_circle(current_cluster_points);
          cluster_circle.fit();
          
          if (cluster_circle.is_valid) {
            local_table_count += 1;

            geometry_msgs::msg::PointStamped point_laser;
            point_laser.header.frame_id = "base_scan";
            point_laser.header.stamp = msg->header.stamp;
            point_laser.point = cluster_circle.center;

            // transform to odom to publish it
            geometry_msgs::msg::TransformStamped transform_stamped;
            std::string toFrameRel = "odom";
            std::string fromFrameRel = "base_scan";
            
            try {
              transform_stamped = tf_buffer_->lookupTransform(
                toFrameRel, fromFrameRel, tf2::TimePointZero);
                
              geometry_msgs::msg::PointStamped transformed_point;
              tf2::doTransform(point_laser, transformed_point, transform_stamped);
              
              geometry_msgs::msg::Pose transformed_pose;
              transformed_pose.position = transformed_point.point;
              transformed_pose.orientation.w = 1.0; 

              table_poses.push_back(transformed_pose);

            } catch (const tf2::TransformException & ex) {
              RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s",
                toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
              // keep continue, but reset cluster logic happens after this block anyway
            }
          }
        }
        // reset cluster
        index_start_cluster = -1;
        current_cluster_points.clear();
      }
        
    } else {
      index_start_cluster = -1;
      current_cluster_points.clear();
    }
  }
  
  // update state and publish
  table_count = local_table_count;
  
  auto pose_array_msg = geometry_msgs::msg::PoseArray();
  pose_array_msg.header.stamp = this->now();
  pose_array_msg.header.frame_id = "odom";
  pose_array_msg.poses = table_poses;
  this->publisher_->publish(pose_array_msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TablesDetection>());
  rclcpp::shutdown();
  return 0;
}
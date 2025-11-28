#ifndef TABLES_DETECTION_HPP
#define TABLES_DETECTION_HPP

#include <vector>
#include <deque>
#include <cmath>
#include <algorithm>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "group18_interfaces/srv/table_count.hpp"

struct DetectedObject {
    double x;
    double y;
};

class  TablesDetection : public rclcpp::Node
{
public:
     TablesDetection();

private:
    // threshold to split scan points into distinct clusters
    const double SEGMENT_JUMP = 0.40; 
    
    // width filters to reject object too big/small to be tables
    const double TABLE_MIN_WIDTH = 0.05; // 5cm
    const double TABLE_MAX_WIDTH = 0.40; // 40cm

    // circle geometry filters
    const double TABLE_RADIUS_MIN = 0.02;
    const double TABLE_RADIUS_MAX = 0.20; 
    const double CIRCLE_FIT_MSE_THRESH = 0.03; // Max mean squared error for curve fit

    // to detect consistency between scans
    const double MERGE_TOLERANCE = 0.60; // Max dist to merge detections between frames
    const int MIN_VOTES = 2; // How many frames must see a table to confirm it
    

    std::deque<sensor_msgs::msg::LaserScan::SharedPtr> scan_buffer_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr viz_pub_;
    rclcpp::Service<group18_interfaces::srv::TableCount>::SharedPtr service_;


    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    void service_callback(const std::shared_ptr<group18_interfaces::srv::TableCount::Request> request,
                          std::shared_ptr<group18_interfaces::srv::TableCount::Response> response);

    // Main processing pipeline for a single scan
    std::vector<DetectedObject> find_objects_in_scan(const sensor_msgs::msg::LaserScan::SharedPtr& scan);

    // Tries to fit a circle to points. Returns true if valid, updates out_x/y.
    bool fit_circle_geometry(const std::vector<std::pair<double, double>>& points, double& out_x, double& out_y);

    // Validates a cluster and transforms it to the Odom frame
    void process_cluster(const std::vector<std::pair<double, double>>& points, 
                         std::vector<DetectedObject>& objects,
                         const geometry_msgs::msg::TransformStamped& tf);

    // aggregates detections over time to remove noise
    int filter_and_count(const std::vector<DetectedObject>& candidates);
};

#endif // TABLES_DETECTION_HPP
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

/**
 * @brief Simple structure to hold the 2D coordinates of a detected object.
 */
struct DetectedObject {
    double x;
    double y;
};

/**
 * @brief Node responsible for detecting circular tables from LaserScan data.
 * * It buffers incoming scans and, upon service request, processes the buffer 
 * to detect, filter, and count tables based on geometric constraints.
 */
class TablesDetection : public rclcpp::Node
{
public:
    /**
     * @brief Constructor. Initializes ROS interfaces, parameters, and TF buffer.
     */
    TablesDetection();

private:
    // Threshold (in meters) to split scan points into distinct clusters.
    // If adjacent points are further apart than this, a new cluster begins.
    const double SEGMENT_JUMP = 0.10; 

    // Geometric constraints for valid tables (radius in meters).
    const double TABLE_RADIUS_MIN = 0.02;
    const double TABLE_RADIUS_MAX = 0.20; 
    
    // Maximum Mean Squared Error allowed for a cluster to be considered a circle.
    const double CIRCLE_FIT_MSE_THRESH = 0.03; 

    // Maximum distance (in meters) to consider two detections as the same object.
    const double MERGE_TOLERANCE = 0.30; 
    
    // Minimum number of detections across the buffer required to confirm a table exists (out of 15 scans).
    const int MIN_VOTES = 5; 

    // Buffer to store recent laser scans for temporal averaging/filtering.
    std::deque<sensor_msgs::msg::LaserScan::SharedPtr> scan_buffer_;
    
    // TF2 buffer and listener for coordinate transforms.
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ROS Communication interfaces.
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr viz_pub_;
    rclcpp::Service<group18_interfaces::srv::TableCount>::SharedPtr service_;


    /**
     * @brief Callback for incoming LaserScan messages.
     * Fills the scan buffer up to a fixed size.
     */
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    /**
     * @brief Service callback to count tables.
     * Processes the buffered scans to find stable table detections.
     */
    void service_callback(const std::shared_ptr<group18_interfaces::srv::TableCount::Request> request,
                          std::shared_ptr<group18_interfaces::srv::TableCount::Response> response);

    /**
     * @brief Pipeline to find table candidates in a single scan.
     * Segments the scan into clusters and processes each one.
     */
    std::vector<DetectedObject> find_objects_in_scan(const sensor_msgs::msg::LaserScan::SharedPtr& scan);

    /**
     * @brief Attempts to fit a circle to a set of points (Geometric Circle Fit).
     * @param points Input cluster points (x, y).
     * @param out_x Output center X.
     * @param out_y Output center Y.
     * @return true if a valid circle within parameters is found.
     */
    bool fit_circle_geometry(const std::vector<std::pair<double, double>>& points, double& out_x, double& out_y);

    /**
     * @brief Evaluates a cluster of points to see if it represents a table.
     * Performs convexity checks and calls circle fitting.
     * @param points Input points in the cluster.
     * @param objects Reference to the list of valid detected objects.
     * @param tf Transform to convert the detected point to the target frame (e.g., Odom or Map).
     */
    void process_cluster(const std::vector<std::pair<double, double>>& points, 
                         std::vector<DetectedObject>& objects,
                         const geometry_msgs::msg::TransformStamped& tf);

    /**
     * @brief Aggregates candidates from multiple scans to remove noise.
     * Merges close detections and applies a voting threshold.
     * @param candidates All potential tables found across the scan buffer.
     * @return The final count of confirmed tables.
     */
    int filter_and_count(const std::vector<DetectedObject>& candidates);
};

#endif // TABLES_DETECTION_HPP
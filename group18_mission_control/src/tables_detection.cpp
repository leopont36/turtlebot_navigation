#include "group18_mission_control/tables_detection.hpp"

TablesDetection::TablesDetection() : Node("tables_detection")
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&TablesDetection::scan_callback, this, std::placeholders::_1));

    service_ = this->create_service<group18_interfaces::srv::TableCount>(
        "table_count", std::bind(&TablesDetection::service_callback, this, std::placeholders::_1, std::placeholders::_2));

    viz_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("detected_tables", 10);

    RCLCPP_INFO(this->get_logger(), "Tables detection Ready.");
}

void TablesDetection::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    scan_buffer_.push_back(msg);
    if (scan_buffer_.size() > 15) {
        scan_buffer_.pop_front();
    }
}

void TablesDetection::service_callback(const std::shared_ptr<group18_interfaces::srv::TableCount::Request>,
                                    std::shared_ptr<group18_interfaces::srv::TableCount::Response> response)
{
    if (scan_buffer_.empty()) {
        response->n_tables = 0;
        return;
    }

    std::vector<DetectedObject> all_detections;

    for (const sensor_msgs::msg::LaserScan::SharedPtr& scan : scan_buffer_) {
        // These objects will be returned in the MAP frame (stable)
        std::vector<DetectedObject> objects = find_objects_in_scan(scan);
        all_detections.insert(all_detections.end(), objects.begin(), objects.end());
    }

    // filters map-frame objects, then converts to odom for publishing
    int final_count = filter_and_count(all_detections);
    response->n_tables = final_count;
    RCLCPP_INFO(this->get_logger(), "Counted %d tables.", final_count);
}

std::vector<DetectedObject> TablesDetection::find_objects_in_scan(const sensor_msgs::msg::LaserScan::SharedPtr& scan) 
{
    std::vector<DetectedObject> objects;
    geometry_msgs::msg::TransformStamped tf;
    
    try {
        // in map frame to reduce motion blur.
        tf = tf_buffer_->lookupTransform("map", scan->header.frame_id, scan->header.stamp, rclcpp::Duration::from_seconds(0.1));
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
        return objects; 
    }

    std::vector<std::pair<double, double>> cluster_points;
    double prev_r = scan->ranges[0];
    // RCLCPP_INFO(this->get_logger(), "\n\n---- SCAN with %.2ld points ----\n\n", scan->ranges.size());

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double r = scan->ranges[i];
        bool valid = (r > scan->range_min && r < scan->range_max);
        
        if (std::abs(r - prev_r) > SEGMENT_JUMP || !valid) {
            // RCLCPP_INFO(this->get_logger(), "CLUSTER DETECTED with %.2ld points", cluster_points.size());
            process_cluster(cluster_points, objects, tf);
            cluster_points.clear();
        }

        if (valid) {
            double angle = scan->angle_min + i * scan->angle_increment;
            double x = r * std::cos(angle);
            double y = r * std::sin(angle);
            cluster_points.push_back({x, y});
            prev_r = r;
        } else {
            prev_r = -1.0; 
        }
    }
    process_cluster(cluster_points, objects, tf);
    return objects;
}


bool TablesDetection::fit_circle_geometry(const std::vector<std::pair<double, double>>& points, double& out_x, double& out_y) 
{
    size_t n = points.size();
    
    // Select 3 points
    std::pair<double, double> p1 = points[0];
    std::pair<double, double> p2 = points[n / 2];
    std::pair<double, double> p3 = points[n - 1];

    double x1 = p1.first, y1 = p1.second;
    double x2 = p2.first, y2 = p2.second;
    double x3 = p3.first, y3 = p3.second;

    double D = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
    
    // Check if points are collinear (a wall)
    if (std::abs(D) < 0.001)  {
        // RCLCPP_INFO(this->get_logger(), "Not a circle, likely a wall");
        return false;
    } 

    // Calculate Circumcenter
    double center_x = ((x1*x1 + y1*y1)*(y2 - y3) + (x2*x2 + y2*y2)*(y3 - y1) + (x3*x3 + y3*y3)*(y1 - y2)) / D;
    double center_y = ((x1*x1 + y1*y1)*(x3 - x2) + (x2*x2 + y2*y2)*(x1 - x3) + (x3*x3 + y3*y3)*(x2 - x1)) / D;

    double radius = std::hypot(center_x - x1, center_y - y1);

    // Filter by radius size
    if (radius < TABLE_RADIUS_MIN || radius > TABLE_RADIUS_MAX) {
        // RCLCPP_INFO(this->get_logger(), "Radius out of bounds");
        return false;
    }

    // Filter by MSE (how well other points fit this circle)
    double total_error_sq = 0.0;
    for (const std::pair<double, double>& p : points) {
        double dist_to_center = std::hypot(p.first - center_x, p.second - center_y);
        double err = dist_to_center - radius;
        total_error_sq += err * err;
    }
    double mse = std::sqrt(total_error_sq / n);

    if (mse > CIRCLE_FIT_MSE_THRESH) {
        // RCLCPP_INFO(this->get_logger(), "CIRCLE_FIT_MSE_THRESH not respected");
        return false;
    }

    // Success: Output the calculated center
    out_x = center_x;
    out_y = center_y;
    // RCLCPP_INFO(this->get_logger(), "!!!!WE HAVE A CIRCLE!!!!");
    return true;
}

void TablesDetection::process_cluster(const std::vector<std::pair<double, double>>& points, 
                 std::vector<DetectedObject>& objects,
                 const geometry_msgs::msg::TransformStamped& tf)
{
    size_t n = points.size();
    if (n < 3) return; // too few points to be anything useful

    // convexiry check
    double r_start = std::hypot(points.front().first, points.front().second);
    double r_end   = std::hypot(points.back().first,  points.back().second);
    double r_mid   = std::hypot(points[n/2].first,    points[n/2].second);

    // check if middle point is closer to the robot
    if (r_mid > r_start || r_mid > r_end) {
        //  corner is concave, not a circle
        return; 
    }

    double final_x = 0.0;
    double final_y = 0.0;

    if (!fit_circle_geometry(points, final_x, final_y)) {
        return; 
    }

    // transform and store (map frame)
    geometry_msgs::msg::PointStamped p_in, p_out;
    p_in.point.x = final_x;
    p_in.point.y = final_y;
    
    try {
        tf2::doTransform(p_in, p_out, tf);
        objects.push_back({p_out.point.x, p_out.point.y});
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "TF Exception: %s", ex.what());
        return; 
    }
}

int TablesDetection::filter_and_count(const std::vector<DetectedObject>& candidates) {
    
    struct Bin { double x, y; int votes; };
    std::vector<Bin> bins;

    for (const DetectedObject& cand : candidates) {
        bool matched = false;
        for (Bin& bin : bins) {
            if (std::hypot(cand.x - bin.x, cand.y - bin.y) < MERGE_TOLERANCE) {
                bin.x = (bin.x * bin.votes + cand.x) / (bin.votes + 1);
                bin.y = (bin.y * bin.votes + cand.y) / (bin.votes + 1);
                bin.votes++;
                matched = true;
                break;
            }
        }
        if (!matched) {
            bins.push_back({cand.x, cand.y, 1});
        }
    }

    // prepare output in odom frame
    geometry_msgs::msg::PoseArray viz_msg;
    viz_msg.header.frame_id = "odom"; 
    viz_msg.header.stamp = this->now();

    // get map -> odom (to convert back)
    geometry_msgs::msg::TransformStamped map_to_odom;
    bool can_transform = false;
    try {
        map_to_odom = tf_buffer_->lookupTransform("odom", "map", tf2::TimePointZero);
        can_transform = true;
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Cannot transform map->odom: %s", ex.what());
    }

    int confirmed_count = 0;
    for (const Bin& bin : bins) {
        if (bin.votes >= MIN_VOTES) {
            confirmed_count++;
            
            // create pose in map frame
            geometry_msgs::msg::PoseStamped p_map;
            p_map.pose.position.x = bin.x;
            p_map.pose.position.y = bin.y;
            
            // transform to odom
            if (can_transform) {
                geometry_msgs::msg::PoseStamped p_odom;
                tf2::doTransform(p_map, p_odom, map_to_odom);
                viz_msg.poses.push_back(p_odom.pose);
            } else {
                // Fallback (shouldn't happen if TF is healthy)
                viz_msg.poses.push_back(p_map.pose);
            }
        }
    }
    viz_pub_->publish(viz_msg);

    return confirmed_count;
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TablesDetection>());
    rclcpp::shutdown();
    return 0;
}
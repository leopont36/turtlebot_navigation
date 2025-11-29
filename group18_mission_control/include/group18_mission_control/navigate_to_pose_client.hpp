#ifndef NAVIGATE_TO_POSE_CLIENT_
#define NAVIGATE_TO_POSE_CLIENT_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "group18_interfaces/srv/table_count.hpp"

class NavigateToPoseClient : public rclcpp::Node
{
public:
    using NavigateToPoseAction = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPoseAction>;

    NavigateToPoseClient();

private:
    rclcpp_action::Client<NavigateToPoseAction>::SharedPtr action_client_;

    // final position to reach
    geometry_msgs::msg::PoseStamped goal_pose; 
    // to make sure the goal is sent
    bool goal_sent_; 

    // to repeatily check if apriltag's positions and initial position are published
    void timer_callback();

    // to track if the positions of the apriltags are published and obtain rhem
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr timer_;

    // to track if initial pose is published
    bool initial_pose_received_; 
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    // to initialize goal_pose to position between apriltags
    bool goal_calculated_;
    bool calculate_goal_pose(); 


    void send_goal();
    void feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPoseAction::Feedback> feedback);
    // to manage start of tables detection
    void result_callback(const GoalHandle::WrappedResult & result);

    // service client for table detection
    void start_table_counting_service();
    rclcpp::Client<group18_interfaces::srv::TableCount>::SharedPtr table_client_;
    void table_response_callback(rclcpp::Client<group18_interfaces::srv::TableCount>::SharedFuture future);
};

#endif // NAVIGATE_TO_POSE_CLIENT_
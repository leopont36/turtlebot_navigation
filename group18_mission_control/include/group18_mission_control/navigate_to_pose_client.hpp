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

/**
 * @brief Main mission logic: Wait for pose, detect AprilTags, navigate to midpoint, count tables.
 */
class NavigateToPoseClient : public rclcpp::Node
{
public:
    using NavigateToPoseAction = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPoseAction>;

    /**
     * @brief Initializes Nav2 action client, TF2 listener, and main control timer.
     */
    NavigateToPoseClient();

private:
    // Navigation Action Client
    rclcpp_action::Client<NavigateToPoseAction>::SharedPtr action_client_;

    // Target position (midpoint between tags)
    geometry_msgs::msg::PoseStamped goal_pose; 

    // Flag to ensure goal is sent once
    bool goal_sent_; 

    // Main Loop Timer
    rclcpp::TimerBase::SharedPtr timer_;

    /**
     * @brief 1Hz Control Loop. Checks pose, calculates goal, and sends navigation request.
     */
    void timer_callback();

    // TF2 (Transforms)
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // Initial Pose Subscription
    bool initial_pose_received_; 
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

    /**
     * @brief Verifies AMCL localization has started.
     */
    void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    // Goal Calculation Logic
    bool goal_calculated_;

    /**
     * @brief Calculates the navigation goal pose based on the AprilTag position.
     * * This function looks up the transforms for 'tag36h11:1' and 'tag36h11:10' in the 'map' frame.
     * It compute the mid point between these two tags and use it as a final destination.
     * * @return True if both tags are detected and the goal is calculated. False otherwise.
     */
    bool calculate_goal_pose(); 

    // Action Client Callbacks

    /**
     * @brief Send the calculated pose(middle pose) to the Action Server.
     * * Configure the goal option and sends the request asynchronously.
     */
    void send_goal();

    /**
     * @brief Callback function to handle the feedback from the Action Server.
     * @param Goalhandle The goal handle.
     * @param feedback The feedback message that contains the current navigation status.
     */
    void feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPoseAction::Feedback> feedback);

    /**
     * @brief Handles navigation result. Triggers table counting on success.
     * @param Goalhandle The result.
     */
    void result_callback(const GoalHandle::WrappedResult & result);

    // Table Counting Service
    rclcpp::Client<group18_interfaces::srv::TableCount>::SharedPtr table_client_;

    /**
     * @brief Calls 'table_count' service.
     */
    void start_table_counting_service();

    /**
     * @brief Logs table count from service response.
     */
    void table_response_callback(rclcpp::Client<group18_interfaces::srv::TableCount>::SharedFuture future);
};

#endif // NAVIGATE_TO_POSE_CLIENT_
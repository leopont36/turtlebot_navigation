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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class NavigateToPoseClient : public rclcpp::Node
{
public:
    using NavigateToPoseAction = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPoseAction>;

    NavigateToPoseClient();

private:
    rclcpp_action::Client<NavigateToPoseAction>::SharedPtr action_client_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::PoseStamped goal_pose; // final position to reach
    bool goal_sent_ = false; // to make sure the goal is sent

    void timer_callback(); // to repeatily check if transforms of the apriltag's positions have been published
    bool calculate_goal_pose(); // to initialize goal_pose to position between apriltags

    void send_goal();
    void feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPoseAction::Feedback> feedback);
};

#endif // NAVIGATE_TO_POSE_MANAGER_
#include "group18_mission_control/navigate_to_pose_client.hpp"

using namespace std::chrono_literals;

NavigateToPoseClient::NavigateToPoseClient() : Node("navigate_to_pose_client")
{
    // create action client
    action_client_ = rclcpp_action::create_client<NavigateToPoseAction>(
        this, "navigate_to_pose");
    
    // setup if2
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // timer to loop every 1 second to check if the tags are visible
    timer_ = this->create_wall_timer(
        1000ms, std::bind(&NavigateToPoseClient::timer_callback, this));
}

void NavigateToPoseClient::timer_callback()
{
    // if goal is sent, return
    if (goal_sent_) {
        timer_->cancel();
        return;
    }

    // try to compute goal's position
    if (calculate_goal_pose()) {
        send_goal();
        goal_sent_ = true;
    }
}

bool NavigateToPoseClient::calculate_goal_pose()
{
    geometry_msgs::msg::TransformStamped ts_apriltag1;
    geometry_msgs::msg::TransformStamped ts_apriltag2;

    try {
        // check if transforms exist before looking them up (prevents error spam)
        if (!tf_buffer_->canTransform("map", "tag36h11:1", tf2::TimePointZero) ||
            !tf_buffer_->canTransform("map", "tag36h11:10", tf2::TimePointZero)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for AprilTags to be detected...");
            return false; 
        }

        ts_apriltag1 = tf_buffer_->lookupTransform("map", "tag36h11:1", tf2::TimePointZero);
        ts_apriltag2 = tf_buffer_->lookupTransform("map", "tag36h11:10", tf2::TimePointZero);
    } 
    catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "TF Exception: %s", ex.what());
        return false; 
    }

    // get middle position between apriltags
    geometry_msgs::msg::PoseStamped middle_pose;
    middle_pose.header.stamp = this->get_clock()->now();
    middle_pose.header.frame_id = "map";
    // calculate middle point
    middle_pose.pose.position.x = (ts_apriltag1.transform.translation.x + ts_apriltag2.transform.translation.x) / 2.0;
    middle_pose.pose.position.y = (ts_apriltag1.transform.translation.y + ts_apriltag2.transform.translation.y) / 2.0;
    middle_pose.pose.position.z = ts_apriltag1.transform.translation.z;
    // set orientation as first tag
    middle_pose.pose.orientation = ts_apriltag1.transform.rotation;

    // set goal pose as middle pose
    this->goal_pose = middle_pose;
    
    RCLCPP_INFO(this->get_logger(), "Goal calculated: [%.2f, %.2f]", 
        middle_pose.pose.position.x, middle_pose.pose.position.y);

    return true;
}

void NavigateToPoseClient::send_goal()
{
    auto goal_msg = NavigateToPoseAction::Goal();
    goal_msg.pose = this->goal_pose;
    
    action_client_->wait_for_action_server();
    
    auto send_goal_options = rclcpp_action::Client<NavigateToPoseAction>::SendGoalOptions();
    send_goal_options.feedback_callback = 
        std::bind(&NavigateToPoseClient::feedback_callback, this, 
                    std::placeholders::_1, std::placeholders::_2);
    
    action_client_->async_send_goal(goal_msg, send_goal_options);
}

void NavigateToPoseClient::feedback_callback(GoalHandle::SharedPtr, 
                            const std::shared_ptr<const NavigateToPoseAction::Feedback> feedback)
{
    RCLCPP_INFO(this->get_logger(), "Received feedback");
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigateToPoseClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
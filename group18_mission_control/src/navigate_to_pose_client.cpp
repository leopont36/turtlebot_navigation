#include "group18_mission_control/navigate_to_pose_client.hpp"

using namespace std::chrono_literals;

/**
 * @brief Constructor for the NavigateToPoseClient node.
 * * Initializes the action client for navigation, sets up the TF2 buffer and listener
 *  and starts a timer to check every second for AprilTag visibility.
 */
NavigateToPoseClient::NavigateToPoseClient() : Node("navigate_to_pose_client")
{
    // Create Action Client
    action_client_ = rclcpp_action::create_client<NavigateToPoseAction>(
        this, "navigate_to_pose");
    
    // Setup tf2 variable
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Timer to loop every 1 second to check if the tags are visible
    timer_ = this->create_wall_timer(
        1000ms, std::bind(&NavigateToPoseClient::timer_callback, this));
}

/**
 * @brief Periodic callback function.
 * * Checks if the navigation goal has already been sent. If not, it attempts to calculate
 * the goal pose. Once the goal is successfully calculated and sent, the timer is cancelled.
 */
void NavigateToPoseClient::timer_callback()
{
    // If goal is sent, return
    if (goal_sent_) {
        timer_->cancel();
        return;
    }

    // Try to compute goal's position, and send it
    if (calculate_goal_pose()) {
        send_goal();
        goal_sent_ = true;
    }
}

/**
 * @brief Calculates the navigation goal pose based on the AprilTag position.
 * * This function looks up the transforms for 'tag36h11:1' and 'tag36h11:10' in the 'map' frame.
 * It compute the mid point between these two tags and use it as a final destination.
 * * @return True if both tags are detected and the goal is calculated. False otherwise.
 */
bool NavigateToPoseClient::calculate_goal_pose()
{
    // Variable to store transforms for the AprilTags
    geometry_msgs::msg::TransformStamped ts_apriltag1;
    geometry_msgs::msg::TransformStamped ts_apriltag2;

    try {
        
        // Check if transforms exist before attempting lookup
        if (!tf_buffer_->canTransform("map", "tag36h11:1", tf2::TimePointZero) ||
            !tf_buffer_->canTransform("map", "tag36h11:10", tf2::TimePointZero)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for AprilTags to be detected...");
            return false; 
        }
        
        // If transforms exist, calculate the apriltag's transforms w.r.t. 'map' frame
        ts_apriltag1 = tf_buffer_->lookupTransform("map", "tag36h11:1", tf2::TimePointZero);
        ts_apriltag2 = tf_buffer_->lookupTransform("map", "tag36h11:10", tf2::TimePointZero);
    } 
    catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "TF Exception: %s", ex.what());
        return false; 
    }

    // Initialize the middle pose between AprilTags
    geometry_msgs::msg::PoseStamped middle_pose;
    middle_pose.header.stamp = this->get_clock()->now();
    middle_pose.header.frame_id = "map";
    
    // Calculate the geometric middle point
    middle_pose.pose.position.x = (ts_apriltag1.transform.translation.x + ts_apriltag2.transform.translation.x) / 2.0;
    middle_pose.pose.position.y = (ts_apriltag1.transform.translation.y + ts_apriltag2.transform.translation.y) / 2.0;
    middle_pose.pose.position.z = ts_apriltag1.transform.translation.z;
    
    // Set the orientation as the first tag
    middle_pose.pose.orientation = ts_apriltag1.transform.rotation;

    // Set the goal pose as middle pose
    this->goal_pose = middle_pose;
    
    RCLCPP_INFO(this->get_logger(), "Goal calculated: [%.2f, %.2f]", 
        middle_pose.pose.position.x, middle_pose.pose.position.y);

    return true;
}

/**
 * @brief Send the calculated pose(middle pose) to the Action Server.
 * * Configure the goal option and sends the request asynchronously.
 */
void NavigateToPoseClient::send_goal()
{
    auto goal_msg = NavigateToPoseAction::Goal();
    goal_msg.pose = this->goal_pose;

    // Ensure the Action Server is running
    action_client_->wait_for_action_server();

    // Define the oprion for the goal
    auto send_goal_options = rclcpp_action::Client<NavigateToPoseAction>::SendGoalOptions();
    send_goal_options.feedback_callback = 
        std::bind(&NavigateToPoseClient::feedback_callback, this, 
                    std::placeholders::_1, std::placeholders::_2);

    // Send the goal to the Server
    action_client_->async_send_goal(goal_msg, send_goal_options);
}

/**
 * @brief Callback function to handle the feedback from the Action Server.
 * * @param Goalhandle The goal handle.
 * @param feedback The feedback message that contains the current navigation status.
 */
void NavigateToPoseClient::feedback_callback(GoalHandle::SharedPtr, 
                            const std::shared_ptr<const NavigateToPoseAction::Feedback> feedback)
{
    // Log of the received feedback
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

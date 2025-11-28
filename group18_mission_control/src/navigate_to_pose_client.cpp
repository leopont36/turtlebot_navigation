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

    
    // subscription to check if initial pose has been published
    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 
        10, 
        std::bind(&NavigateToPoseClient::initial_pose_callback, this, std::placeholders::_1));

    // initialize client for table detection
    table_client_ = this->create_client<group18_interfaces::srv::TableCount>("table_count");

    goal_sent_ = false;
    initial_pose_received_ = false; 
    goal_calculated_ = false;

    // timer to loop every 1 second to try computing apriltags and send goal pose
    timer_ = this->create_wall_timer(
        1000ms, std::bind(&NavigateToPoseClient::timer_callback, this));

}

void NavigateToPoseClient::initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr /*msg*/)
{   

    if (!initial_pose_received_) {
        RCLCPP_INFO(this->get_logger(), "Initial Pose received! System is ready.");
        initial_pose_received_ = true;
        // unsubscribe now to save resources
        //initial_pose_sub_.reset(); 
    }
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

    // wait that the initial pose is published
    if (!initial_pose_received_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
            "Waiting for Initial Pose to be published...");
        return;
    }

    // try to compute goal's position, if it succeeds send goal
    if (!goal_calculated_) {
        goal_calculated_ = calculate_goal_pose();
        return;
    }
    
    send_goal();
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

        RCLCPP_INFO(this->get_logger(), "Apriltags detected at positions: [%.2f, %.2f] and [%.2f, %.2f]", 
            ts_apriltag1.transform.translation.x, ts_apriltag1.transform.translation.y,
            ts_apriltag2.transform.translation.x, ts_apriltag2.transform.translation.y);
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
    
    RCLCPP_INFO(this->get_logger(), "Goal calculated (position between apriltags): [%.2f, %.2f]", 
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

    // Define the options for the goal
    auto send_goal_options = rclcpp_action::Client<NavigateToPoseAction>::SendGoalOptions();

    // for updates while moving (not used)
    send_goal_options.feedback_callback = 
        std::bind(&NavigateToPoseClient::feedback_callback, this, 
                    std::placeholders::_1, std::placeholders::_2);
    
    // result callback to manage start of table detection node
    send_goal_options.result_callback = 
        std::bind(&NavigateToPoseClient::result_callback, this, std::placeholders::_1);
    
    send_goal_options.goal_response_callback = 
        [this](const GoalHandle::SharedPtr & goal_handle) {
            
            if (goal_handle) {
                RCLCPP_INFO(this->get_logger(), "Nav goal sent.");
                goal_sent_ = true; // to cancel timer_callback
            } 
            else {
                // REJECTED!
                RCLCPP_WARN(this->get_logger(), "Nav goal rejected.");
            }
        };
    
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
    // RCLCPP_INFO(this->get_logger(), "Received feedback");
}

void NavigateToPoseClient::result_callback(const GoalHandle::WrappedResult & result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal reached! Calling table counter service...");
            // TO DO: call service for starting tables
            start_table_counting_service();
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            goal_sent_ = false;
            initial_pose_received_ = false;
            if (timer_->is_canceled())
                timer_->reset();
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
    }
}

void NavigateToPoseClient::start_table_counting_service()
{
    // check if service is ready
    if (!table_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "Table counting service not available!");
        return;
    }

    auto request = std::make_shared<group18_interfaces::srv::TableCount::Request>();

    RCLCPP_INFO(this->get_logger(), "Calling Table Counter Service...");

    // send asynchronously
    table_client_->async_send_request(request, 
        std::bind(&NavigateToPoseClient::table_response_callback, this, std::placeholders::_1));
}

void NavigateToPoseClient::table_response_callback(rclcpp::Client<group18_interfaces::srv::TableCount>::SharedFuture future)
{
    try {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Service Success! Number of tables detected: %ld", response->n_tables);
    } 
    catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
    }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigateToPoseClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

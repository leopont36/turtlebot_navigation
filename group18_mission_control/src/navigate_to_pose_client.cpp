#include "group18_mission_control/navigate_to_pose_client.hpp"

using namespace std::chrono_literals;

NavigateToPoseClient::NavigateToPoseClient() : Node("navigate_to_pose_client")
{
    // create action client
    action_client_ = rclcpp_action::create_client<NavigateToPoseAction>(
        this, "navigate_to_pose");
    
    // setup tf2
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
        initial_pose_sub_.reset(); 
    }
}

void NavigateToPoseClient::timer_callback()
{
    // if goal is sent, return
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

bool NavigateToPoseClient::calculate_goal_pose()
{
    // to store transforms for the apriltags
    geometry_msgs::msg::TransformStamped ts_apriltag1;
    geometry_msgs::msg::TransformStamped ts_apriltag2;

    try {
        // check if transforms exist before looking them up, if not return false
        if (!tf_buffer_->canTransform("map", "tag36h11:1", tf2::TimePointZero) ||
            !tf_buffer_->canTransform("map", "tag36h11:10", tf2::TimePointZero)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for AprilTags to be detected...");
            return false; 
        }
        // transforms exist, get apriltag's transforms w.r.t. map frame
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
    
    RCLCPP_INFO(this->get_logger(), "Goal calculated (position between apriltags): [%.2f, %.2f]", 
        middle_pose.pose.position.x, middle_pose.pose.position.y);

    return true;
}

void NavigateToPoseClient::send_goal()
{
    auto goal_msg = NavigateToPoseAction::Goal();
    goal_msg.pose = this->goal_pose;
    
    action_client_->wait_for_action_server();
    
    // to manage feedbacks / result
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
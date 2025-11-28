#include "group18_mission_control/corridor_navigator.hpp"

CorridorNavigator::CorridorNavigator() : Node("corridor_navigation_node"), in_corridor_(false), nav_off_(false)
{
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&CorridorNavigator::scanCallback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&CorridorNavigator::odomCallback, this, std::placeholders::_1));
  
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  initial_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

  stop_nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

  RCLCPP_INFO(this->get_logger(), "Corridor Navigator node has been started.");
}

void CorridorNavigator::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::vector<Point2D> left_points, right_points;
  float angle = msg->angle_min;
         
  for (size_t i = 0; i < msg->ranges.size(); i++)
  {
    float range = msg->ranges[i];
    //extract only points at least 1.5m from the robot
    if (range > msg->range_min && range < 1.5)    
    { 
      float x = range * cos(angle);               
      float y = range * sin(angle);
      
      //split right and left points      
      if (y > 0) 
        left_points.push_back({x, y});  
      else  
        right_points.push_back({x, y});
    }
    angle += msg->angle_increment;
  }

  // check the minimun number of points
  if (right_points.size() < 30 || left_points.size() < 30)
  {
    if(nav_off_)
      selfStop();
    in_corridor_ = false;
    return;
  }

  float left_slope = regressionSlope(left_points);
  float right_slope = regressionSlope(right_points);

  //check if left and right lines are parallel (equal slope) with 0.01 tollerance
  if (std::abs(left_slope - right_slope) > 0.01)
  {
    if(nav_off_)
      selfStop();
    in_corridor_ = false;
    return;
  }

  if (!in_corridor_)
  {
    stopNavigation();
    RCLCPP_INFO(this->get_logger(), "Detected corridor - taking manual control");
  }

  in_corridor_ = true;

  //compute angolar speed to send to /cmd_vel
  float corridor_angle = (left_slope + right_slope) / 2.0;
  geometry_msgs::msg::Twist cmd_msg;
  cmd_msg.linear.x = 1;
  cmd_msg.angular.z =  corridor_angle; 
  //cmd_msg.angular.z = std::clamp(-1.5f * corridor_angle - 0.3f * lateral_offset, -0.5f, 0.5f);
  vel_pub_->publish(cmd_msg);
  

  RCLCPP_INFO(this->get_logger(), "Sending vel_cmd: v = %.2f, w = %.10f",cmd_msg.linear.x, cmd_msg.angular.z);
}

//gestisce in automatico la ripartenza
void CorridorNavigator::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if(in_corridor_ || !nav_off_)
    return;

  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
  initial_pose.header.stamp = this->now();
  initial_pose.header.frame_id = "map";
  initial_pose.pose = msg->pose;
  initial_pose_pub_->publish(initial_pose);
  nav_off_ = false;
  RCLCPP_INFO(this->get_logger(), "Exited corridor, published initial pose for navigation restart");

}

void CorridorNavigator::stopNavigation()
{
  if (!stop_nav_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available");
    return;
  }
  auto cancel_future = stop_nav_client_->async_cancel_all_goals();
  RCLCPP_INFO(this->get_logger(), "Stopped navigation2 - starting corridor self-control");
  nav_off_= true;
}

void CorridorNavigator::selfStop()
{
  geometry_msgs::msg::Twist cmd_msg;
  vel_pub_->publish(cmd_msg);
}

float CorridorNavigator::regressionSlope(const std::vector<Point2D>& points)
{
  if (points.empty()) 
    return 0.0f;      

  double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
  int n = points.size();
        
  for (const auto& p : points) 
  {
    sumX  += p.x;
    sumY  += p.y;
    sumXY += p.x * p.y;
    sumX2 += p.x * p.x;
  }
        
  double den = n * sumX2 - sumX * sumX;
  if (std::abs(den) < 1e-6)
    return 0.0f;
            
  return static_cast<float>((n * sumXY - sumX * sumY) / den);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CorridorNavigator>());
  rclcpp::shutdown();
  return 0;
}



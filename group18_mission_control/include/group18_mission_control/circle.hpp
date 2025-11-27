#ifndef CIRCLE_HPP
#define CIRCLE_HPP

#include <vector>
#include <tuple>
#include <cmath>
#include <map>
#include "geometry_msgs/msg/point.hpp"

class Circle {
public:
  // Constructor
  Circle(const std::vector<geometry_msgs::msg::Point>& cluster_points);

  // Core function to calculate the circle
  void fit();

  // Public members to access results
  const std::vector<geometry_msgs::msg::Point>& points;
  geometry_msgs::msg::Point center;
  double radius = 0.0;
  bool is_valid = false;

private:
  // Configuration constants
  const double MIN_RADIUS = 0.05; 
  const double MAX_RADIUS = 0.40; 
  const double RADIUS_STEP = 0.02; 
  const double GRID_RESOLUTION = 0.02; 
  
  int min_votes_for_table; 
};

#endif // CIRCLE_HPP
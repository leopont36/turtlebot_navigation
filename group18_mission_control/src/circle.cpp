#include "group18_mission_control/circle.hpp"

Circle::Circle(const std::vector<geometry_msgs::msg::Point>& cluster_points) :
  points(cluster_points),
  min_votes_for_table(cluster_points.size() - 2) 
{
  if (min_votes_for_table < 3) {
    min_votes_for_table = 3;
  }
}

void Circle::fit() {
  is_valid = false;
  if (points.empty()) return;

  // Accumulator: <GridX, GridY, RadiusIndex> -> VoteCount
  std::map<std::tuple<int, int, int>, int> accumulator;

  int r_index = 0;
  for (double r = MIN_RADIUS; r <= MAX_RADIUS; r += RADIUS_STEP) {
    
    for (const auto& p : points) {
      // Step 10 deg for speed
      for (int angle_deg = 0; angle_deg < 360; angle_deg += 10) { 

        double angle_rad = angle_deg * M_PI / 180.0;

        double cx = p.x + r * std::cos(angle_rad);
        double cy = p.y + r * std::sin(angle_rad);

        int grid_x = static_cast<int>(cx / GRID_RESOLUTION);
        int grid_y = static_cast<int>(cy / GRID_RESOLUTION);

        accumulator[{grid_x, grid_y, r_index}]++;
      }
    }
    r_index++;
  }

  if (accumulator.empty()) return;

  int max_votes = 0;
  std::tuple<int, int, int> best_params;

  for (const auto& entry : accumulator) {
    if (entry.second > max_votes) {
      max_votes = entry.second;
      best_params = entry.first;
    }
  }

  if (max_votes >= min_votes_for_table) {
    int best_x = std::get<0>(best_params);
    int best_y = std::get<1>(best_params);
    int best_r_idx = std::get<2>(best_params);

    this->center.x = (best_x + 0.5) * GRID_RESOLUTION;
    this->center.y = (best_y + 0.5) * GRID_RESOLUTION;
    this->radius = MIN_RADIUS + (best_r_idx * RADIUS_STEP);
    this->is_valid = true;
  }
}
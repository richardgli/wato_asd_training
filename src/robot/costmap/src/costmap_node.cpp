#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_core_(robot::CostmapCore(this->get_logger())) {
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    initializeCostmap();
 
    for (size_t i = 0; i < scan->ranges.size(); ++i) {

        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];

        if (range < scan->range_max && range > scan->range_min) {
            int x_grid, y_grid;
            convertToGrid(x_grid, y_grid, range, angle);
            markAndInflateObstacle(x_grid, y_grid);
        }
    }
 
    costmap_.header.stamp = this->get_clock()->now();
    costmap_.header.frame_id = scan->header.frame_id;

    publishCostmap();
}

void CostmapNode::initializeCostmap() {
  width_ = 300;
  height_ = 300;
  resolution_ = 0.1;
  inflation_radius_ = 1;
  origin_position_x_ = -15;
  origin_position_y_ = -15;

  costmap_.info.width = width_;
  costmap_.info.height = height_;
  costmap_.info.resolution = resolution_;
  costmap_.info.origin.position.x = origin_position_x_;
  costmap_.info.origin.position.y = origin_position_y_;

  grid_.assign(width_, std::vector<int>(height_, 0));
}

void CostmapNode::convertToGrid(int& x_grid, int& y_grid, double range, double angle) {
  double x_coord = range * std::cos(angle);
  double y_coord = range * std::sin(angle);

  x_grid = static_cast<int>((x_coord - origin_position_x_) / resolution_);
  y_grid = static_cast<int>((y_coord - origin_position_y_) / resolution_);
}

void CostmapNode::markAndInflateObstacle(int x_grid, int y_grid) {
  int radius_ = inflation_radius_ / resolution_;
  const int MAX_COST = 100;

  if (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_) {
    grid_[y_grid][x_grid] = MAX_COST;

    for (int dy_grid = y_grid - radius_; dy_grid <= y_grid + radius_; dy_grid++) {
      for (int dx_grid = x_grid - radius_; dx_grid <= x_grid + radius_; dx_grid++) {

        if (dy_grid < 0 || dy_grid >= height_ || dx_grid < 0 || dx_grid >= width_) continue;

        double distance_ = std::hypot(dx_grid - x_grid, dy_grid - y_grid) * resolution_;

        if (distance_ > inflation_radius_) continue;

        int cost_ = static_cast<int>(MAX_COST * (1 - distance_ / inflation_radius_));

        grid_[dy_grid][dx_grid] = std::max(cost_, grid_[dy_grid][dx_grid]);
      }
    }
  }
}

void CostmapNode::publishCostmap() {
  costmap_.data.resize(width_ * height_);

  for (int y = 0; y < height_; y++) {
    for (int x = 0; x < width_; x++) {
      costmap_.data[y * width_ + x] = grid_[y][x];
    }
  }
  costmap_pub_->publish(costmap_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
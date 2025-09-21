#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void initializeCostmap();
    void convertToGrid(int& x_grid, int& y_grid, double range, double angle);
    void markAndInflateObstacle(int x_grid, int y_grid);
    void publishCostmap();

  private:
    robot::CostmapCore costmap_core_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    nav_msgs::msg::OccupancyGrid costmap_;
    std::vector<std::vector<int>> grid_;

    int width_;
    int height_;
    double resolution_;
    double inflation_radius_;
    int origin_position_x_;
    int origin_position_y_;
};

#endif 
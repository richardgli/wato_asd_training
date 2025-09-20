#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();

    void publishMessage();

  private:
    robot::CostmapCore costmap_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr nav_pub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::TimerBase::SharedPtr timer_;  
    
    void convertToGrid(double range, double angle) {
      /* 
      * The origin coordinates are x and y.
      * To convert LaserScan to grid indices, calculate the
      *   Cartesian coordinates of the detected point using
      *   the formulas
      * These coordinates are relative to the origin coordinates,
      *   so you add the origin coordinates to the Cartesian
      *   coordinates to get the grid indices
      * Divide the resulting grid indices by the resolution
      */
      
      /* int x = range * std::cos(angle);
      int y = range * std::sin(angle); */

  }

  void markObstacle(int x, int y) {
      if (x >= 0 && x < width_ && y >= 0 && y < height_) {
      grid_[y][x] = 100;
      }
  }

  void inflateObstacles() {
      /* 
      * Loop through all the cells in the map
      * If the cell is an obstacle (i.e., has a cost of 100),
      *   then loop through all the cells around the obstacle
      *   cell within the inflation radius
      * Calculate the Euclidean distance of the cell to the 
      *   obstacle cell
      * Calculate the cost of the cell using the formula and
      *   assign the new cost if the calculated cost is greater
      *   than the current cell's cost
      */ 
      // int cost = max_cost * (1 - (distance / inflation_radius));
  }
    void laserCallback(const sensor_msgs:msg:LaserScan::SharedPtr scan_pub) {

        initializeCostmap();

        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            double angle = scan_pub->angle_min + i * scan_pub->angle_increment;
            double range = scan_pub->ranges[i];
            if (range < scan_pub->range_max && range > scan_pub->range_min) {
                // Calculate grid coordinates
                int x_grid, y_grid;
                convertToGrid(range, angle, x_grid, y_grid);
                markObstacle(x_grid, y_grid);
            }

            inflateObstacles();

            // Step 4: Publish costmap
            publishCostmap();
        }
    }    
};

#endif 
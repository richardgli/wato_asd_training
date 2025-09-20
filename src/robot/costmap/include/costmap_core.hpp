#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    void initializeCostmap();

  private:
    void convertToGrid(double range, double angle, int x, int y);
    void markObstacle(int x, int y);
    void inflateObstacles();

    rclcpp::Logger logger_;
    int width_;
    int height_;
    double resolution_;
    int inflation_radius;
    std::vector<std::vector<int>> grid_;

};

}  

#endif  
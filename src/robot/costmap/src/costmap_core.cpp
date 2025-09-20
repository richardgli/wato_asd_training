#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger)
 : logger_(logger),
    width_(300),
    height_(300),
    resolution_(0.1),
    inflation_radius(1)
{
    grid_.resize(height_, std::vector<int>(width_, 0));
}

}
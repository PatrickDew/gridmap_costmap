#include <rclcpp/rclcpp.hpp>
#include "gridmap_costmap/gridmap_costmap_converter.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<gridmap_costmap::GridmapCostmapConverter>();
    
    RCLCPP_INFO(node->get_logger(), "Starting GridmapCostmapConverter node");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}

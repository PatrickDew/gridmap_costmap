#ifndef COSTMAP_GENERATOR_HPP
#define COSTMAP_GENERATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/costmap.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace gridmap_costmap
{

class CostmapGenerator
{
public:
    CostmapGenerator();
    ~CostmapGenerator();
    
    // Main generation methods
    nav_msgs::msg::OccupancyGrid generateOccupancyGrid(const grid_map::GridMap& grid_map);
    nav_msgs::msg::Costmap generateCostmap2D(const grid_map::GridMap& grid_map);
    
    // Costmap processing
    void processCostmap(nav_msgs::msg::OccupancyGrid& costmap);
    void inflateObstacles(nav_msgs::msg::OccupancyGrid& costmap, double inflation_radius);
    void smoothCostmap(nav_msgs::msg::OccupancyGrid& costmap);
    
    // Cost calculation
    int calculateCostFromTerrain(double slope, double roughness, double elevation);
    int calculateCostFromGridMap(const grid_map::GridMap& grid_map, int x, int y);
    
    // Coordinate transformations
    geometry_msgs::msg::PoseStamped gridMapToWorld(const grid_map::GridMap& grid_map, 
                                                   const grid_map::Index& index);
    grid_map::Index worldToGridMap(const grid_map::GridMap& grid_map, 
                                  const geometry_msgs::msg::Point& point);
    
    // Set parameters
    void setResolution(double resolution) { resolution_ = resolution; }
    void setMapWidth(double width) { map_width_ = width; }
    void setMapHeight(double height) { map_height_ = height; }
    void setMapFrame(const std::string& frame) { map_frame_ = frame; }
    void setInflationRadius(double radius) { inflation_radius_ = radius; }
    
    // Cost thresholds
    void setFreeThreshold(int threshold) { free_threshold_ = threshold; }
    void setOccupiedThreshold(int threshold) { occupied_threshold_ = threshold; }
    void setMaxCost(int cost) { max_cost_ = cost; }
    
private:
    // Parameters
    double resolution_;
    double map_width_;
    double map_height_;
    std::string map_frame_;
    double inflation_radius_;
    
    // Cost thresholds
    int free_threshold_;
    int occupied_threshold_;
    int max_cost_;
    
    // Terrain cost parameters
    double max_slope_;
    double max_roughness_;
    double slope_weight_;
    double roughness_weight_;
    double elevation_weight_;
    
    // Utility methods
    void initializeCostmap(nav_msgs::msg::OccupancyGrid& costmap);
    void setCostmapMetadata(nav_msgs::msg::OccupancyGrid& costmap);
    void applyInflationKernel(nav_msgs::msg::OccupancyGrid& costmap, int center_x, int center_y, double radius);
    bool isValidCostmapIndex(const nav_msgs::msg::OccupancyGrid& costmap, int x, int y);
    
    // Cost calculation helpers
    int slopeToCost(double slope);
    int roughnessToCost(double roughness);
    int elevationToCost(double elevation);
    int combineCosts(int slope_cost, int roughness_cost, int elevation_cost);
};

} // namespace gridmap_costmap

#endif // COSTMAP_GENERATOR_HPP


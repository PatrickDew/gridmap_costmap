#include "gridmap_costmap/costmap_generator.hpp"
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace gridmap_costmap
{

CostmapGenerator::CostmapGenerator()
    : resolution_(0.1)
    , map_width_(50.0)
    , map_height_(50.0)
    , map_frame_("map")
    , inflation_radius_(0.5)
    , free_threshold_(0)
    , occupied_threshold_(100)
    , max_cost_(255)
    , max_slope_(0.5)
    , max_roughness_(0.3)
    , slope_weight_(0.4)
    , roughness_weight_(0.3)
    , elevation_weight_(0.3)
{
}

CostmapGenerator::~CostmapGenerator()
{
}

nav_msgs::msg::OccupancyGrid CostmapGenerator::generateOccupancyGrid(const grid_map::GridMap& grid_map)
{
    nav_msgs::msg::OccupancyGrid costmap;
    
    // Initialize costmap
    initializeCostmap(costmap);
    setCostmapMetadata(costmap);
    
    // Get grid map dimensions
    int width = grid_map.getSize().x();
    int height = grid_map.getSize().y();
    
    // Resize costmap data
    costmap.data.resize(width * height);
    
    // Convert grid map to costmap
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            grid_map::Index index(x, y);
            int cost = calculateCostFromGridMap(grid_map, x, y);
            costmap.data[y * width + x] = cost;
        }
    }
    
    // Process costmap
    processCostmap(costmap);
    
    return costmap;
}

nav_msgs::msg::Costmap CostmapGenerator::generateCostmap2D(const grid_map::GridMap& grid_map)
{
    nav_msgs::msg::Costmap costmap_2d;
    
    // Set header
    costmap_2d.header.frame_id = map_frame_;
    costmap_2d.header.stamp = rclcpp::Clock().now();
    
    // Set metadata
    costmap_2d.metadata.resolution = resolution_;
    costmap_2d.metadata.size_x = static_cast<uint32_t>(map_width_ / resolution_);
    costmap_2d.metadata.size_y = static_cast<uint32_t>(map_height_ / resolution_);
    costmap_2d.metadata.origin.position.x = -map_width_ / 2.0;
    costmap_2d.metadata.origin.position.y = -map_height_ / 2.0;
    costmap_2d.metadata.origin.position.z = 0.0;
    costmap_2d.metadata.origin.orientation.w = 1.0;
    
    // Get grid map dimensions
    int width = grid_map.getSize().x();
    int height = grid_map.getSize().y();
    
    // Resize costmap data
    costmap_2d.data.resize(width * height);
    
    // Convert grid map to costmap
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            grid_map::Index index(x, y);
            int cost = calculateCostFromGridMap(grid_map, x, y);
            costmap_2d.data[y * width + x] = static_cast<uint8_t>(cost);
        }
    }
    
    return costmap_2d;
}

void CostmapGenerator::processCostmap(nav_msgs::msg::OccupancyGrid& costmap)
{
    // Apply inflation
    inflateObstacles(costmap, inflation_radius_);
    
    // Apply smoothing
    smoothCostmap(costmap);
}

void CostmapGenerator::inflateObstacles(nav_msgs::msg::OccupancyGrid& costmap, double inflation_radius)
{
    int inflation_cells = static_cast<int>(inflation_radius / resolution_);
    
    // Create a copy for inflation
    std::vector<int8_t> original_data = costmap.data;
    
    for (int y = 0; y < costmap.info.height; ++y) {
        for (int x = 0; x < costmap.info.width; ++x) {
            int index = y * costmap.info.width + x;
            
            if (original_data[index] >= occupied_threshold_) {
                // Inflate around this obstacle
                applyInflationKernel(costmap, x, y, inflation_radius);
            }
        }
    }
}

void CostmapGenerator::applyInflationKernel(nav_msgs::msg::OccupancyGrid& costmap, int center_x, int center_y, double radius)
{
    int inflation_cells = static_cast<int>(radius / resolution_);
    
    for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
        for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
            int x = center_x + dx;
            int y = center_y + dy;
            
            if (isValidCostmapIndex(costmap, x, y)) {
                double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
                
                if (distance <= radius) {
                    int index = y * costmap.info.width + x;
                    int inflation_cost = static_cast<int>((1.0 - distance / radius) * max_cost_);
                    costmap.data[index] = std::max(costmap.data[index], static_cast<int8_t>(inflation_cost));
                }
            }
        }
    }
}

void CostmapGenerator::smoothCostmap(nav_msgs::msg::OccupancyGrid& costmap)
{
    // Simple smoothing using a 3x3 kernel
    std::vector<int8_t> smoothed_data = costmap.data;
    
    for (int y = 1; y < costmap.info.height - 1; ++y) {
        for (int x = 1; x < costmap.info.width - 1; ++x) {
            int index = y * costmap.info.width + x;
            
            // Calculate average of 3x3 neighborhood
            int sum = 0;
            int count = 0;
            
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    int neighbor_index = (y + dy) * costmap.info.width + (x + dx);
                    sum += costmap.data[neighbor_index];
                    count++;
                }
            }
            
            smoothed_data[index] = static_cast<int8_t>(sum / count);
        }
    }
    
    costmap.data = smoothed_data;
}

int CostmapGenerator::calculateCostFromGridMap(const grid_map::GridMap& grid_map, int x, int y)
{
    grid_map::Index index(x, y);
    
    if (!grid_map.isValid(index)) {
        return -1; // Unknown
    }
    
    double slope = 0.0;
    double roughness = 0.0;
    double elevation = 0.0;
    
    // Get terrain values
    if (grid_map.exists("slope")) {
        slope = grid_map.at("slope", index);
    }
    if (grid_map.exists("roughness")) {
        roughness = grid_map.at("roughness", index);
    }
    if (grid_map.exists("elevation")) {
        elevation = grid_map.at("elevation", index);
    }
    
    return calculateCostFromTerrain(slope, roughness, elevation);
}

int CostmapGenerator::calculateCostFromTerrain(double slope, double roughness, double elevation)
{
    int slope_cost = slopeToCost(slope);
    int roughness_cost = roughnessToCost(roughness);
    int elevation_cost = elevationToCost(elevation);
    
    return combineCosts(slope_cost, roughness_cost, elevation_cost);
}

int CostmapGenerator::slopeToCost(double slope)
{
    if (slope > max_slope_) {
        return max_cost_;
    }
    return static_cast<int>((slope / max_slope_) * max_cost_);
}

int CostmapGenerator::roughnessToCost(double roughness)
{
    if (roughness > max_roughness_) {
        return max_cost_;
    }
    return static_cast<int>((roughness / max_roughness_) * max_cost_);
}

int CostmapGenerator::elevationToCost(double elevation)
{
    // Simple elevation cost - can be customized based on requirements
    if (elevation < -1.0 || elevation > 1.0) {
        return max_cost_;
    }
    return static_cast<int>(std::abs(elevation) * max_cost_);
}

int CostmapGenerator::combineCosts(int slope_cost, int roughness_cost, int elevation_cost)
{
    // Weighted combination of costs
    int combined_cost = static_cast<int>(
        slope_weight_ * slope_cost +
        roughness_weight_ * roughness_cost +
        elevation_weight_ * elevation_cost
    );
    
    return std::min(combined_cost, max_cost_);
}

void CostmapGenerator::initializeCostmap(nav_msgs::msg::OccupancyGrid& costmap)
{
    // Set header
    costmap.header.frame_id = map_frame_;
    costmap.header.stamp = rclcpp::Clock().now();
    
    // Set info
    costmap.info.resolution = resolution_;
    costmap.info.width = static_cast<uint32_t>(map_width_ / resolution_);
    costmap.info.height = static_cast<uint32_t>(map_height_ / resolution_);
    costmap.info.origin.position.x = -map_width_ / 2.0;
    costmap.info.origin.position.y = -map_height_ / 2.0;
    costmap.info.origin.position.z = 0.0;
    costmap.info.origin.orientation.w = 1.0;
}

void CostmapGenerator::setCostmapMetadata(nav_msgs::msg::OccupancyGrid& costmap)
{
    // Set additional metadata if needed
    costmap.info.origin.orientation.x = 0.0;
    costmap.info.origin.orientation.y = 0.0;
    costmap.info.origin.orientation.z = 0.0;
}

bool CostmapGenerator::isValidCostmapIndex(const nav_msgs::msg::OccupancyGrid& costmap, int x, int y)
{
    return x >= 0 && x < static_cast<int>(costmap.info.width) &&
           y >= 0 && y < static_cast<int>(costmap.info.height);
}

} // namespace gridmap_costmap


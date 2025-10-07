#include "gridmap_costmap/gridmap_costmap_converter.hpp"
#include "gridmap_costmap/terrain_analyzer.hpp"
#include "gridmap_costmap/costmap_generator.hpp"

namespace gridmap_costmap
{

GridmapCostmapConverter::GridmapCostmapConverter()
    : Node("gridmap_costmap_converter")
{
    // Initialize parameters
    this->declare_parameter("resolution", 0.1);
    this->declare_parameter("map_width", 50.0);
    this->declare_parameter("map_height", 50.0);
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("robot_frame", "base_link");
    this->declare_parameter("elevation_layer", "elevation");
    this->declare_parameter("slope_layer", "slope");
    this->declare_parameter("roughness_layer", "roughness");
    this->declare_parameter("costmap_layer", "traversability");
    
    // Cost calculation parameters
    this->declare_parameter("max_slope", 0.5);
    this->declare_parameter("max_roughness", 0.3);
    this->declare_parameter("min_elevation", -2.0);
    this->declare_parameter("max_elevation", 2.0);
    this->declare_parameter("slope_weight", 0.4);
    this->declare_parameter("roughness_weight", 0.3);
    this->declare_parameter("elevation_weight", 0.3);
    
    // Get parameters
    resolution_ = this->get_parameter("resolution").as_double();
    map_width_ = this->get_parameter("map_width").as_double();
    map_height_ = this->get_parameter("map_height").as_double();
    map_frame_ = this->get_parameter("map_frame").as_string();
    robot_frame_ = this->get_parameter("robot_frame").as_string();
    elevation_layer_ = this->get_parameter("elevation_layer").as_string();
    slope_layer_ = this->get_parameter("slope_layer").as_string();
    roughness_layer_ = this->get_parameter("roughness_layer").as_string();
    costmap_layer_ = this->get_parameter("costmap_layer").as_string();
    
    max_slope_ = this->get_parameter("max_slope").as_double();
    max_roughness_ = this->get_parameter("max_roughness").as_double();
    min_elevation_ = this->get_parameter("min_elevation").as_double();
    max_elevation_ = this->get_parameter("max_elevation").as_double();
    slope_weight_ = this->get_parameter("slope_weight").as_double();
    roughness_weight_ = this->get_parameter("roughness_weight").as_double();
    elevation_weight_ = this->get_parameter("elevation_weight").as_double();
    
    // Initialize TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Initialize publishers
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", 10);
    costmap_2d_pub_ = this->create_publisher<nav_msgs::msg::Costmap>("costmap_2d", 10);
    processed_grid_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("processed_grid_map", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("terrain_markers", 10);
    
    // Initialize subscribers
    grid_map_sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
        "grid_map", 10,
        std::bind(&GridmapCostmapConverter::gridMapCallback, this, std::placeholders::_1));
    
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "pointcloud", 10,
        std::bind(&GridmapCostmapConverter::pointCloudCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "GridmapCostmapConverter initialized");
}

GridmapCostmapConverter::~GridmapCostmapConverter()
{
}

void GridmapCostmapConverter::gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
{
    try {
        // Convert ROS message to grid map
        grid_map::GridMapRosConverter::fromMessage(*msg, grid_map_);
        
        RCLCPP_DEBUG(this->get_logger(), "Received grid map with size: %f x %f", 
                    grid_map_.getLength().x(), grid_map_.getLength().y());
        
        // Process the grid map
        processGridMap();
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing grid map: %s", e.what());
    }
}

void GridmapCostmapConverter::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    try {
        // Convert point cloud to grid map (if needed)
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);
        
        // This could be used to update the grid map with new point cloud data
        // For now, we'll just log the received point cloud
        RCLCPP_DEBUG(this->get_logger(), "Received point cloud with %zu points", cloud.size());
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing point cloud: %s", e.what());
    }
}

void GridmapCostmapConverter::processGridMap()
{
    try {
        // Calculate terrain costs
        calculateTerrainCosts();
        
        // Generate costmap
        generateCostmap();
        
        // Publish results
        publishCostmap();
        publishMarkers();
        
        RCLCPP_DEBUG(this->get_logger(), "Grid map processed successfully");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing grid map: %s", e.what());
    }
}

void GridmapCostmapConverter::calculateTerrainCosts()
{
    // Initialize terrain analyzer
    TerrainAnalyzer analyzer;
    analyzer.setMaxSlope(max_slope_);
    analyzer.setMaxRoughness(max_roughness_);
    analyzer.setSlopeWeight(slope_weight_);
    analyzer.setRoughnessWeight(roughness_weight_);
    analyzer.setElevationWeight(elevation_weight_);
    
    // Analyze terrain
    analyzer.analyzeTerrain(grid_map_);
    
    RCLCPP_DEBUG(this->get_logger(), "Terrain analysis completed");
}

void GridmapCostmapConverter::generateCostmap()
{
    // Initialize costmap generator
    CostmapGenerator generator;
    generator.setResolution(resolution_);
    generator.setMapWidth(map_width_);
    generator.setMapHeight(map_height_);
    generator.setMapFrame(map_frame_);
    
    // Generate costmaps
    costmap_ = generator.generateOccupancyGrid(grid_map_);
    costmap_2d_ = generator.generateCostmap2D(grid_map_);
    
    RCLCPP_DEBUG(this->get_logger(), "Costmap generation completed");
}

void GridmapCostmapConverter::publishCostmap()
{
    // Publish occupancy grid
    if (costmap_pub_->get_subscription_count() > 0) {
        costmap_pub_->publish(costmap_);
    }
    
    // Publish costmap 2D
    if (costmap_2d_pub_->get_subscription_count() > 0) {
        costmap_2d_pub_->publish(costmap_2d_);
    }
    
    // Publish processed grid map
    if (processed_grid_map_pub_->get_subscription_count() > 0) {
        grid_map_msgs::msg::GridMap processed_msg;
        grid_map::GridMapRosConverter::toMessage(grid_map_, processed_msg);
        processed_grid_map_pub_->publish(processed_msg);
    }
}

void GridmapCostmapConverter::publishMarkers()
{
    // Create marker array for terrain visualization
    visualization_msgs::msg::MarkerArray marker_array;
    
    // This could include markers for obstacles, terrain features, etc.
    // For now, we'll create a simple marker
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "terrain";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;
    
    marker_array.markers.push_back(marker);
    
    if (marker_pub_->get_subscription_count() > 0) {
        marker_pub_->publish(marker_array);
    }
}

} // namespace gridmap_costmap


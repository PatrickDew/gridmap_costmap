#ifndef GRIDMAP_COSTMAP_CONVERTER_HPP
#define GRIDMAP_COSTMAP_CONVERTER_HPP

#include <rclcpp/rclcpp.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_costmap_2d/grid_map_costmap_2d.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/costmap.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace gridmap_costmap
{

class GridmapCostmapConverter : public rclcpp::Node
{
public:
    GridmapCostmapConverter();
    ~GridmapCostmapConverter();

private:
    // ROS2 subscribers and publishers
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Publisher<nav_msgs::msg::Costmap>::SharedPtr costmap_2d_pub_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr processed_grid_map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    
    // TF2
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    
    // Grid map and costmap data
    grid_map::GridMap grid_map_;
    nav_msgs::msg::OccupancyGrid costmap_;
    nav_msgs::msg::Costmap costmap_2d_;
    
    // Parameters
    double resolution_;
    double map_width_;
    double map_height_;
    std::string map_frame_;
    std::string robot_frame_;
    std::string elevation_layer_;
    std::string slope_layer_;
    std::string roughness_layer_;
    std::string costmap_layer_;
    
    // Cost calculation parameters
    double max_slope_;
    double max_roughness_;
    double min_elevation_;
    double max_elevation_;
    double slope_weight_;
    double roughness_weight_;
    double elevation_weight_;
    
    // Methods
    void gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg);
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void processGridMap();
    void calculateTerrainCosts();
    void generateCostmap();
    void publishCostmap();
    void publishMarkers();
    
    // Terrain analysis methods
    void calculateSlope();
    void calculateRoughness();
    void calculateElevationCosts();
    void applyGaussianFilter();
    void applyMorphologicalOperations();
    
    // Utility methods
    double calculateSlopeAtPoint(const grid_map::Index& index);
    double calculateRoughnessAtPoint(const grid_map::Index& index);
    double calculateCostFromTerrain(double slope, double roughness, double elevation);
    bool isValidIndex(const grid_map::Index& index);
    geometry_msgs::msg::PoseStamped transformPose(const geometry_msgs::msg::PoseStamped& pose, const std::string& target_frame);
};

} // namespace gridmap_costmap

#endif // GRIDMAP_COSTMAP_CONVERTER_HPP


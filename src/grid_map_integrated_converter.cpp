#include <rclcpp/rclcpp.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_costmap_2d/grid_map_costmap_2d.hpp>
#include <grid_map_filters/grid_map_filters.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <mutex>
#include <cmath>
#include <vector>

class GridMapIntegratedConverter : public rclcpp::Node
{
public:
    GridMapIntegratedConverter() : Node("grid_map_integrated_converter")
    {
        // Initialize parameters
        this->declare_parameter("resolution", 0.1);
        this->declare_parameter("map_length_x", 50.0);
        this->declare_parameter("map_length_y", 50.0);
        this->declare_parameter("map_frame", "map");
        this->declare_parameter("robot_frame", "base_link");
        this->declare_parameter("lidar_frame", "livox");
        this->declare_parameter("camera_frame", "camera_link");
        
        // Terrain analysis parameters
        this->declare_parameter("max_slope", 0.5);
        this->declare_parameter("max_roughness", 0.3);
        this->declare_parameter("min_elevation", -2.0);
        this->declare_parameter("max_elevation", 10.0);
        this->declare_parameter("slope_weight", 0.4);
        this->declare_parameter("roughness_weight", 0.3);
        this->declare_parameter("elevation_weight", 0.3);
        
        // Grid map parameters
        this->declare_parameter("grid_map_resolution", 0.1);
        this->declare_parameter("grid_map_length_x", 50.0);
        this->declare_parameter("grid_map_length_y", 50.0);
        
        // Get parameters
        resolution_ = this->get_parameter("resolution").as_double();
        map_length_x_ = this->get_parameter("map_length_x").as_double();
        map_length_y_ = this->get_parameter("map_length_y").as_double();
        map_frame_ = this->get_parameter("map_frame").as_string();
        robot_frame_ = this->get_parameter("robot_frame").as_string();
        lidar_frame_ = this->get_parameter("lidar_frame").as_string();
        camera_frame_ = this->get_parameter("camera_frame").as_string();
        
        max_slope_ = this->get_parameter("max_slope").as_double();
        max_roughness_ = this->get_parameter("max_roughness").as_double();
        min_elevation_ = this->get_parameter("min_elevation").as_double();
        max_elevation_ = this->get_parameter("max_elevation").as_double();
        slope_weight_ = this->get_parameter("slope_weight").as_double();
        roughness_weight_ = this->get_parameter("roughness_weight").as_double();
        elevation_weight_ = this->get_parameter("elevation_weight").as_double();
        
        grid_map_resolution_ = this->get_parameter("grid_map_resolution").as_double();
        grid_map_length_x_ = this->get_parameter("grid_map_length_x").as_double();
        grid_map_length_y_ = this->get_parameter("grid_map_length_y").as_double();
        
        // Initialize TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Initialize grid map
        grid_map_.setFrameId(map_frame_);
        grid_map_.setGeometry(grid_map::Length(grid_map_length_x_, grid_map_length_y_), 
                             grid_map_resolution_, 
                             grid_map::Position(0.0, 0.0));
        
        // Add layers to grid map
        grid_map_.add("elevation");
        grid_map_.add("slope");
        grid_map_.add("roughness");
        grid_map_.add("traversability");
        grid_map_.add("obstacles");
        grid_map_.add("normal_x");
        grid_map_.add("normal_y");
        grid_map_.add("normal_z");
        
        // Initialize publishers
        grid_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("grid_map", 10);
        costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", 10);
        elevation_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("elevation_map", 10);
        slope_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("slope_map", 10);
        roughness_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("roughness_map", 10);
        processed_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("processed_pointcloud", 10);
        
        // Initialize subscribers
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "livox_points", 10,
            std::bind(&GridMapIntegratedConverter::pointCloudCallback, this, std::placeholders::_1));
        
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&GridMapIntegratedConverter::laserCallback, this, std::placeholders::_1));
        
        camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", 10,
            std::bind(&GridMapIntegratedConverter::cameraCallback, this, std::placeholders::_1));
        
        // Create a timer for periodic processing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&GridMapIntegratedConverter::processGridMap, this));
        
        RCLCPP_INFO(this->get_logger(), "GridMapIntegratedConverter initialized with grid map size: %f x %f", 
                   grid_map_length_x_, grid_map_length_y_);
    }

private:
    // ROS2 subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr elevation_map_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr slope_map_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr roughness_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_pointcloud_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // TF2
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    
    // Grid map
    grid_map::GridMap grid_map_;
    
    // Parameters
    double resolution_;
    double map_length_x_;
    double map_length_y_;
    std::string map_frame_;
    std::string robot_frame_;
    std::string lidar_frame_;
    std::string camera_frame_;
    
    // Terrain analysis parameters
    double max_slope_;
    double max_roughness_;
    double min_elevation_;
    double max_elevation_;
    double slope_weight_;
    double roughness_weight_;
    double elevation_weight_;
    
    // Grid map parameters
    double grid_map_resolution_;
    double grid_map_length_x_;
    double grid_map_length_y_;
    
    // Sensor data
    pcl::PointCloud<pcl::PointXYZ> latest_pointcloud_;
    sensor_msgs::msg::LaserScan latest_laser_;
    cv::Mat latest_camera_image_;
    std::mutex sensor_mutex_;
    
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        pcl::fromROSMsg(*msg, latest_pointcloud_);
        
        // Process point cloud to grid map
        processPointCloudToGridMap();
        
        RCLCPP_DEBUG(this->get_logger(), "Received point cloud with %zu points", latest_pointcloud_.size());
    }
    
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        latest_laser_ = *msg;
        RCLCPP_DEBUG(this->get_logger(), "Received laser scan with %zu ranges", msg->ranges.size());
    }
    
    void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        try {
            latest_camera_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
            RCLCPP_DEBUG(this->get_logger(), "Received camera image: %dx%d", 
                        latest_camera_image_.cols, latest_camera_image_.rows);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
    
    void processPointCloudToGridMap()
    {
        if (latest_pointcloud_.empty()) return;
        
        // Clear elevation layer
        grid_map_["elevation"].setConstant(NAN);
        
        // Process each point
        for (const auto& point : latest_pointcloud_.points) {
            if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
                continue;
            }
            
            // Check if point is within grid map bounds
            if (point.x < -grid_map_length_x_/2 || point.x > grid_map_length_x_/2 ||
                point.y < -grid_map_length_y_/2 || point.y > grid_map_length_y_/2) {
                continue;
            }
            
            // Convert to grid map position
            grid_map::Position position(point.x, point.y);
            
            // Check if position is valid
            if (!grid_map_.isValid(position)) continue;
            
            // Get current elevation at this position
            double current_elevation = grid_map_.atPosition("elevation", position);
            
            // Update elevation (use maximum height for overlapping points)
            if (std::isnan(current_elevation) || point.z > current_elevation) {
                grid_map_.atPosition("elevation", position) = point.z;
            }
        }
        
        // Publish processed point cloud
        sensor_msgs::msg::PointCloud2 processed_msg;
        pcl::toROSMsg(latest_pointcloud_, processed_msg);
        processed_msg.header.frame_id = lidar_frame_;
        processed_msg.header.stamp = this->get_clock()->now();
        processed_pointcloud_pub_->publish(processed_msg);
    }
    
    void processGridMap()
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        
        // Update grid map timestamp
        grid_map_.setTimestamp(this->get_clock()->now().nanoseconds());
        
        // Calculate terrain features
        calculateSlopeMap();
        calculateRoughnessMap();
        calculateNormalMap();
        calculateTraversabilityMap();
        
        // Apply filters
        applyFilters();
        
        // Publish grid map
        grid_map_msgs::msg::GridMap grid_map_msg;
        grid_map::GridMapRosConverter::toMessage(grid_map_, grid_map_msg);
        grid_map_pub_->publish(grid_map_msg);
        
        // Publish individual maps
        publishCostmap();
        publishElevationMap();
        publishSlopeMap();
        publishRoughnessMap();
        
        RCLCPP_DEBUG(this->get_logger(), "Processed grid map with %zu x %zu cells", 
                    grid_map_.getSize().x(), grid_map_.getSize().y());
    }
    
    void calculateSlopeMap()
    {
        // Calculate slope using gradient
        for (grid_map::GridMapIterator it(grid_map_); !it.isPastEnd(); ++it) {
            const grid_map::Index index(*it);
            
            if (!grid_map_.isValid(index)) continue;
            
            double slope = calculateSlopeAtPoint(index);
            grid_map_.at("slope", index) = slope;
        }
    }
    
    void calculateRoughnessMap()
    {
        // Calculate roughness using local variance
        for (grid_map::GridMapIterator it(grid_map_); !it.isPastEnd(); ++it) {
            const grid_map::Index index(*it);
            
            if (!grid_map_.isValid(index)) continue;
            
            double roughness = calculateRoughnessAtPoint(index);
            grid_map_.at("roughness", index) = roughness;
        }
    }
    
    void calculateNormalMap()
    {
        // Calculate surface normals
        for (grid_map::GridMapIterator it(grid_map_); !it.isPastEnd(); ++it) {
            const grid_map::Index index(*it);
            
            if (!grid_map_.isValid(index)) continue;
            
            Eigen::Vector3d normal = calculateNormalAtPoint(index);
            grid_map_.at("normal_x", index) = normal.x();
            grid_map_.at("normal_y", index) = normal.y();
            grid_map_.at("normal_z", index) = normal.z();
        }
    }
    
    void calculateTraversabilityMap()
    {
        // Calculate traversability based on terrain features
        for (grid_map::GridMapIterator it(grid_map_); !it.isPastEnd(); ++it) {
            const grid_map::Index index(*it);
            
            if (!grid_map_.isValid(index)) continue;
            
            double elevation = grid_map_.at("elevation", index);
            double slope = grid_map_.at("slope", index);
            double roughness = grid_map_.at("roughness", index);
            
            // Calculate individual costs
            double slope_cost = calculateSlopeCost(slope);
            double roughness_cost = calculateRoughnessCost(roughness);
            double elevation_cost = calculateElevationCost(elevation);
            
            // Combine costs
            double traversability = slope_weight_ * slope_cost + 
                                  roughness_weight_ * roughness_cost + 
                                  elevation_weight_ * elevation_cost;
            
            grid_map_.at("traversability", index) = traversability;
        }
    }
    
    void applyFilters()
    {
        // Apply Gaussian filter to smooth terrain
        grid_map::GridMapFilters::applyGaussianFilter(grid_map_, "elevation", "elevation", 1.0);
        grid_map::GridMapFilters::applyGaussianFilter(grid_map_, "slope", "slope", 0.5);
        grid_map::GridMapFilters::applyGaussianFilter(grid_map_, "roughness", "roughness", 0.5);
        
        // Apply median filter to remove noise
        grid_map::GridMapFilters::applyMedianFilter(grid_map_, "elevation", "elevation", 3);
        grid_map::GridMapFilters::applyMedianFilter(grid_map_, "slope", "slope", 3);
        grid_map::GridMapFilters::applyMedianFilter(grid_map_, "roughness", "roughness", 3);
    }
    
    double calculateSlopeAtPoint(const grid_map::Index& index)
    {
        if (!grid_map_.isValid(index)) return 0.0;
        
        double resolution = grid_map_.getResolution();
        double elevation = grid_map_.at("elevation", index);
        
        // Calculate gradients in x and y directions
        double grad_x = 0.0, grad_y = 0.0;
        
        // X gradient
        if (grid_map_.isValid(grid_map::Index(index.x() + 1, index.y()))) {
            double elevation_right = grid_map_.at("elevation", grid_map::Index(index.x() + 1, index.y()));
            grad_x = (elevation_right - elevation) / resolution;
        }
        
        // Y gradient
        if (grid_map_.isValid(grid_map::Index(index.x(), index.y() + 1))) {
            double elevation_up = grid_map_.at("elevation", grid_map::Index(index.x(), index.y() + 1));
            grad_y = (elevation_up - elevation) / resolution;
        }
        
        // Calculate slope magnitude
        double slope = std::sqrt(grad_x * grad_x + grad_y * grad_y);
        
        return slope;
    }
    
    double calculateRoughnessAtPoint(const grid_map::Index& index)
    {
        if (!grid_map_.isValid(index)) return 0.0;
        
        int radius = 2; // Local neighborhood radius
        double variance = calculateLocalVariance(index, radius);
        
        return std::sqrt(variance);
    }
    
    double calculateLocalVariance(const grid_map::Index& index, int radius)
    {
        double sum = 0.0;
        double sum_squared = 0.0;
        int count = 0;
        
        double center_elevation = grid_map_.at("elevation", index);
        
        for (int dx = -radius; dx <= radius; ++dx) {
            for (int dy = -radius; dy <= radius; ++dy) {
                grid_map::Index neighbor_index(index.x() + dx, index.y() + dy);
                
                if (grid_map_.isValid(neighbor_index)) {
                    double elevation = grid_map_.at("elevation", neighbor_index);
                    double diff = elevation - center_elevation;
                    sum += diff;
                    sum_squared += diff * diff;
                    count++;
                }
            }
        }
        
        if (count == 0) return 0.0;
        
        double mean = sum / count;
        double variance = (sum_squared / count) - (mean * mean);
        
        return std::max(0.0, variance);
    }
    
    Eigen::Vector3d calculateNormalAtPoint(const grid_map::Index& index)
    {
        if (!grid_map_.isValid(index)) return Eigen::Vector3d(0, 0, 1);
        
        double resolution = grid_map_.getResolution();
        
        // Calculate gradients
        double grad_x = 0.0, grad_y = 0.0;
        
        if (grid_map_.isValid(grid_map::Index(index.x() + 1, index.y()))) {
            double elevation_right = grid_map_.at("elevation", grid_map::Index(index.x() + 1, index.y()));
            double elevation_left = grid_map_.at("elevation", grid_map::Index(index.x() - 1, index.y()));
            grad_x = (elevation_right - elevation_left) / (2.0 * resolution);
        }
        
        if (grid_map_.isValid(grid_map::Index(index.x(), index.y() + 1))) {
            double elevation_up = grid_map_.at("elevation", grid_map::Index(index.x(), index.y() + 1));
            double elevation_down = grid_map_.at("elevation", grid_map::Index(index.x(), index.y() - 1));
            grad_y = (elevation_up - elevation_down) / (2.0 * resolution);
        }
        
        // Calculate normal vector
        Eigen::Vector3d normal(-grad_x, -grad_y, 1.0);
        normal.normalize();
        
        return normal;
    }
    
    double calculateSlopeCost(double slope)
    {
        if (slope > max_slope_) {
            return 1.0; // Maximum cost
        }
        return slope / max_slope_;
    }
    
    double calculateRoughnessCost(double roughness)
    {
        if (roughness > max_roughness_) {
            return 1.0; // Maximum cost
        }
        return roughness / max_roughness_;
    }
    
    double calculateElevationCost(double elevation)
    {
        if (std::isnan(elevation)) return 1.0;
        
        if (elevation < min_elevation_ || elevation > max_elevation_) {
            return 1.0; // Maximum cost
        }
        
        // Normalize elevation cost
        double normalized_elevation = (elevation - min_elevation_) / (max_elevation_ - min_elevation_);
        return std::abs(normalized_elevation - 0.5) * 2.0; // Cost increases away from center
    }
    
    void publishCostmap()
    {
        nav_msgs::msg::OccupancyGrid costmap;
        grid_map::GridMapRosConverter::toOccupancyGrid(grid_map_, "traversability", 0.0, 1.0, costmap);
        costmap_pub_->publish(costmap);
    }
    
    void publishElevationMap()
    {
        nav_msgs::msg::OccupancyGrid elevation_map;
        grid_map::GridMapRosConverter::toOccupancyGrid(grid_map_, "elevation", min_elevation_, max_elevation_, elevation_map);
        elevation_map_pub_->publish(elevation_map);
    }
    
    void publishSlopeMap()
    {
        nav_msgs::msg::OccupancyGrid slope_map;
        grid_map::GridMapRosConverter::toOccupancyGrid(grid_map_, "slope", 0.0, max_slope_, slope_map);
        slope_map_pub_->publish(slope_map);
    }
    
    void publishRoughnessMap()
    {
        nav_msgs::msg::OccupancyGrid roughness_map;
        grid_map::GridMapRosConverter::toOccupancyGrid(grid_map_, "roughness", 0.0, max_roughness_, roughness_map);
        roughness_map_pub_->publish(roughness_map);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<GridMapIntegratedConverter>();
    
    RCLCPP_INFO(node->get_logger(), "Starting GridMapIntegratedConverter node");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}

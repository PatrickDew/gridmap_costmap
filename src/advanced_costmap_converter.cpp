#include <rclcpp/rclcpp.hpp>
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

class AdvancedCostmapConverter : public rclcpp::Node
{
public:
    AdvancedCostmapConverter() : Node("advanced_costmap_converter")
    {
        // Initialize parameters
        this->declare_parameter("resolution", 0.1);
        this->declare_parameter("map_width", 100.0);
        this->declare_parameter("map_height", 100.0);
        this->declare_parameter("map_frame", "map");
        this->declare_parameter("robot_frame", "base_link");
        this->declare_parameter("lidar_frame", "velodyne");
        this->declare_parameter("camera_frame", "camera_link");
        
        // Terrain analysis parameters
        this->declare_parameter("max_slope", 0.5);
        this->declare_parameter("max_roughness", 0.3);
        this->declare_parameter("min_elevation", -2.0);
        this->declare_parameter("max_elevation", 5.0);
        this->declare_parameter("slope_weight", 0.4);
        this->declare_parameter("roughness_weight", 0.3);
        this->declare_parameter("elevation_weight", 0.3);
        
        // Sensor processing parameters
        this->declare_parameter("pointcloud_voxel_size", 0.1);
        this->declare_parameter("pointcloud_max_range", 30.0);
        this->declare_parameter("pointcloud_min_range", 0.1);
        this->declare_parameter("laser_max_range", 30.0);
        this->declare_parameter("laser_min_range", 0.1);
        
        // Get parameters
        resolution_ = this->get_parameter("resolution").as_double();
        map_width_ = this->get_parameter("map_width").as_double();
        map_height_ = this->get_parameter("map_height").as_double();
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
        
        pointcloud_voxel_size_ = this->get_parameter("pointcloud_voxel_size").as_double();
        pointcloud_max_range_ = this->get_parameter("pointcloud_max_range").as_double();
        pointcloud_min_range_ = this->get_parameter("pointcloud_min_range").as_double();
        laser_max_range_ = this->get_parameter("laser_max_range").as_double();
        laser_min_range_ = this->get_parameter("laser_min_range").as_double();
        
        // Initialize TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Initialize publishers
        costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", 10);
        elevation_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("elevation_map", 10);
        slope_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("slope_map", 10);
        roughness_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("roughness_map", 10);
        processed_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("processed_pointcloud", 10);
        
        // Initialize subscribers
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "velodyne_points", 10,
            std::bind(&AdvancedCostmapConverter::pointCloudCallback, this, std::placeholders::_1));
        
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&AdvancedCostmapConverter::laserCallback, this, std::placeholders::_1));
        
        camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", 10,
            std::bind(&AdvancedCostmapConverter::cameraCallback, this, std::placeholders::_1));
        
        // Create a timer for periodic costmap generation
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&AdvancedCostmapConverter::generateCostmap, this));
        
        RCLCPP_INFO(this->get_logger(), "AdvancedCostmapConverter initialized");
    }

private:
    // ROS2 subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr elevation_map_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr slope_map_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr roughness_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_pointcloud_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // TF2
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    
    // Parameters
    double resolution_;
    double map_width_;
    double map_height_;
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
    
    // Sensor processing parameters
    double pointcloud_voxel_size_;
    double pointcloud_max_range_;
    double pointcloud_min_range_;
    double laser_max_range_;
    double laser_min_range_;
    
    // Sensor data
    pcl::PointCloud<pcl::PointXYZ> latest_pointcloud_;
    sensor_msgs::msg::LaserScan latest_laser_;
    cv::Mat latest_camera_image_;
    std::mutex sensor_mutex_;
    
    // Terrain maps
    std::vector<std::vector<double>> elevation_map_;
    std::vector<std::vector<double>> slope_map_;
    std::vector<std::vector<double>> roughness_map_;
    std::vector<std::vector<double>> costmap_data_;
    
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        pcl::fromROSMsg(*msg, latest_pointcloud_);
        
        // Process point cloud
        processPointCloud();
        
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
    
    void processPointCloud()
    {
        if (latest_pointcloud_.empty()) return;
        
        // Voxel grid filtering
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(latest_pointcloud_.makeShared());
        voxel_filter.setLeafSize(pointcloud_voxel_size_, pointcloud_voxel_size_, pointcloud_voxel_size_);
        voxel_filter.filter(*cloud_filtered);
        
        // Range filtering
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_range_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> range_filter;
        range_filter.setInputCloud(cloud_filtered);
        range_filter.setFilterFieldName("z");
        range_filter.setFilterLimits(min_elevation_, max_elevation_);
        range_filter.filter(*cloud_range_filtered);
        
        // Distance filtering
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_distance_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> distance_filter;
        distance_filter.setInputCloud(cloud_range_filtered);
        distance_filter.setFilterFieldName("x");
        distance_filter.setFilterLimits(-pointcloud_max_range_, pointcloud_max_range_);
        distance_filter.filter(*cloud_distance_filtered);
        
        latest_pointcloud_ = *cloud_distance_filtered;
        
        // Publish processed point cloud
        sensor_msgs::msg::PointCloud2 processed_msg;
        pcl::toROSMsg(latest_pointcloud_, processed_msg);
        processed_msg.header.frame_id = lidar_frame_;
        processed_msg.header.stamp = this->get_clock()->now();
        processed_pointcloud_pub_->publish(processed_msg);
    }
    
    void generateCostmap()
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        
        // Initialize terrain maps
        int width = static_cast<int>(map_width_ / resolution_);
        int height = static_cast<int>(map_height_ / resolution_);
        
        elevation_map_.assign(height, std::vector<double>(width, 0.0));
        slope_map_.assign(height, std::vector<double>(width, 0.0));
        roughness_map_.assign(height, std::vector<double>(width, 0.0));
        costmap_data_.assign(height, std::vector<double>(width, 0.0));
        
        // Process point cloud data
        if (!latest_pointcloud_.empty()) {
            processPointCloudToMaps();
        }
        
        // Process laser data
        if (!latest_laser_.ranges.empty()) {
            processLaserToMaps();
        }
        
        // Process camera data
        if (!latest_camera_image_.empty()) {
            processCameraToMaps();
        }
        
        // Calculate terrain features
        calculateSlopeMap();
        calculateRoughnessMap();
        calculateCostmap();
        
        // Publish maps
        publishCostmap();
        publishElevationMap();
        publishSlopeMap();
        publishRoughnessMap();
        
        RCLCPP_INFO(this->get_logger(), "Generated advanced costmap with %dx%d cells", width, height);
    }
    
    void processPointCloudToMaps()
    {
        int width = static_cast<int>(map_width_ / resolution_);
        int height = static_cast<int>(map_height_ / resolution_);
        
        for (const auto& point : latest_pointcloud_.points) {
            if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
                continue;
            }
            
            // Convert to grid coordinates
            int grid_x = static_cast<int>((point.x + map_width_ / 2.0) / resolution_);
            int grid_y = static_cast<int>((point.y + map_height_ / 2.0) / resolution_);
            
            if (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height) {
                elevation_map_[grid_y][grid_x] = point.z;
            }
        }
    }
    
    void processLaserToMaps()
    {
        int width = static_cast<int>(map_width_ / resolution_);
        int height = static_cast<int>(map_height_ / resolution_);
        
        double angle_increment = latest_laser_.angle_increment;
        double angle_min = latest_laser_.angle_min;
        
        for (size_t i = 0; i < latest_laser_.ranges.size(); ++i) {
            double range = latest_laser_.ranges[i];
            if (range < laser_min_range_ || range > laser_max_range_) {
                continue;
            }
            
            double angle = angle_min + i * angle_increment;
            double x = range * std::cos(angle);
            double y = range * std::sin(angle);
            double z = 0.0; // Assume ground level for laser
            
            // Convert to grid coordinates
            int grid_x = static_cast<int>((x + map_width_ / 2.0) / resolution_);
            int grid_y = static_cast<int>((y + map_height_ / 2.0) / resolution_);
            
            if (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height) {
                elevation_map_[grid_y][grid_x] = z;
            }
        }
    }
    
    void processCameraToMaps()
    {
        // Simple edge detection for obstacles
        cv::Mat edges;
        cv::Canny(latest_camera_image_, edges, 50, 150);
        
        // Convert to costmap (simplified)
        int width = static_cast<int>(map_width_ / resolution_);
        int height = static_cast<int>(map_height_ / resolution_);
        
        // This is a simplified approach - in reality, you'd need proper camera calibration
        // and 3D reconstruction to map camera data to world coordinates
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                // Simple heuristic based on image edges
                if (x < edges.cols && y < edges.rows) {
                    if (edges.at<uchar>(y, x) > 0) {
                        costmap_data_[y][x] += 0.1; // Add some cost for detected edges
                    }
                }
            }
        }
    }
    
    void calculateSlopeMap()
    {
        int width = static_cast<int>(map_width_ / resolution_);
        int height = static_cast<int>(map_height_ / resolution_);
        
        for (int y = 1; y < height - 1; ++y) {
            for (int x = 1; x < width - 1; ++x) {
                // Calculate gradient
                double dz_dx = (elevation_map_[y][x + 1] - elevation_map_[y][x - 1]) / (2.0 * resolution_);
                double dz_dy = (elevation_map_[y + 1][x] - elevation_map_[y - 1][x]) / (2.0 * resolution_);
                
                double slope = std::sqrt(dz_dx * dz_dx + dz_dy * dz_dy);
                slope_map_[y][x] = slope;
            }
        }
    }
    
    void calculateRoughnessMap()
    {
        int width = static_cast<int>(map_width_ / resolution_);
        int height = static_cast<int>(map_height_ / resolution_);
        int radius = 2; // Local neighborhood radius
        
        for (int y = radius; y < height - radius; ++y) {
            for (int x = radius; x < width - radius; ++x) {
                double sum = 0.0;
                double sum_squared = 0.0;
                int count = 0;
                
                double center_elevation = elevation_map_[y][x];
                
                for (int dy = -radius; dy <= radius; ++dy) {
                    for (int dx = -radius; dx <= radius; ++dx) {
                        double elevation = elevation_map_[y + dy][x + dx];
                        double diff = elevation - center_elevation;
                        sum += diff;
                        sum_squared += diff * diff;
                        count++;
                    }
                }
                
                double mean = sum / count;
                double variance = (sum_squared / count) - (mean * mean);
                roughness_map_[y][x] = std::sqrt(std::max(0.0, variance));
            }
        }
    }
    
    void calculateCostmap()
    {
        int width = static_cast<int>(map_width_ / resolution_);
        int height = static_cast<int>(map_height_ / resolution_);
        
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                double slope = slope_map_[y][x];
                double roughness = roughness_map_[y][x];
                double elevation = elevation_map_[y][x];
                
                // Calculate individual costs
                double slope_cost = calculateSlopeCost(slope);
                double roughness_cost = calculateRoughnessCost(roughness);
                double elevation_cost = calculateElevationCost(elevation);
                
                // Combine costs
                double total_cost = slope_weight_ * slope_cost + 
                                  roughness_weight_ * roughness_cost + 
                                  elevation_weight_ * elevation_cost;
                
                costmap_data_[y][x] = std::min(1.0, total_cost);
            }
        }
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
        costmap.header.frame_id = map_frame_;
        costmap.header.stamp = this->get_clock()->now();
        costmap.info.resolution = resolution_;
        costmap.info.width = static_cast<uint32_t>(map_width_ / resolution_);
        costmap.info.height = static_cast<uint32_t>(map_height_ / resolution_);
        costmap.info.origin.position.x = -map_width_ / 2.0;
        costmap.info.origin.position.y = -map_height_ / 2.0;
        costmap.info.origin.position.z = 0.0;
        costmap.info.origin.orientation.w = 1.0;
        
        int width = costmap.info.width;
        int height = costmap.info.height;
        costmap.data.resize(width * height);
        
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int index = y * width + x;
                int cost = static_cast<int>(costmap_data_[y][x] * 100);
                costmap.data[index] = std::min(100, std::max(0, cost));
            }
        }
        
        costmap_pub_->publish(costmap);
    }
    
    void publishElevationMap()
    {
        nav_msgs::msg::OccupancyGrid elevation_map;
        elevation_map.header.frame_id = map_frame_;
        elevation_map.header.stamp = this->get_clock()->now();
        elevation_map.info.resolution = resolution_;
        elevation_map.info.width = static_cast<uint32_t>(map_width_ / resolution_);
        elevation_map.info.height = static_cast<uint32_t>(map_height_ / resolution_);
        elevation_map.info.origin.position.x = -map_width_ / 2.0;
        elevation_map.info.origin.position.y = -map_height_ / 2.0;
        elevation_map.info.origin.position.z = 0.0;
        elevation_map.info.origin.orientation.w = 1.0;
        
        int width = elevation_map.info.width;
        int height = elevation_map.info.height;
        elevation_map.data.resize(width * height);
        
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int index = y * width + x;
                int elevation = static_cast<int>((elevation_map_[y][x] + 2.0) * 25.0); // Normalize to 0-100
                elevation_map.data[index] = std::min(100, std::max(0, elevation));
            }
        }
        
        elevation_map_pub_->publish(elevation_map);
    }
    
    void publishSlopeMap()
    {
        nav_msgs::msg::OccupancyGrid slope_map;
        slope_map.header.frame_id = map_frame_;
        slope_map.header.stamp = this->get_clock()->now();
        slope_map.info.resolution = resolution_;
        slope_map.info.width = static_cast<uint32_t>(map_width_ / resolution_);
        slope_map.info.height = static_cast<uint32_t>(map_height_ / resolution_);
        slope_map.info.origin.position.x = -map_width_ / 2.0;
        slope_map.info.origin.position.y = -map_height_ / 2.0;
        slope_map.info.origin.position.z = 0.0;
        slope_map.info.origin.orientation.w = 1.0;
        
        int width = slope_map.info.width;
        int height = slope_map.info.height;
        slope_map.data.resize(width * height);
        
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int index = y * width + x;
                int slope = static_cast<int>((slope_map_[y][x] / max_slope_) * 100.0);
                slope_map.data[index] = std::min(100, std::max(0, slope));
            }
        }
        
        slope_map_pub_->publish(slope_map);
    }
    
    void publishRoughnessMap()
    {
        nav_msgs::msg::OccupancyGrid roughness_map;
        roughness_map.header.frame_id = map_frame_;
        roughness_map.header.stamp = this->get_clock()->now();
        roughness_map.info.resolution = resolution_;
        roughness_map.info.width = static_cast<uint32_t>(map_width_ / resolution_);
        roughness_map.info.height = static_cast<uint32_t>(map_height_ / resolution_);
        roughness_map.info.origin.position.x = -map_width_ / 2.0;
        roughness_map.info.origin.position.y = -map_height_ / 2.0;
        roughness_map.info.origin.position.z = 0.0;
        roughness_map.info.origin.orientation.w = 1.0;
        
        int width = roughness_map.info.width;
        int height = roughness_map.info.height;
        roughness_map.data.resize(width * height);
        
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int index = y * width + x;
                int roughness = static_cast<int>((roughness_map_[y][x] / max_roughness_) * 100.0);
                roughness_map.data[index] = std::min(100, std::max(0, roughness));
            }
        }
        
        roughness_map_pub_->publish(roughness_map);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<AdvancedCostmapConverter>();
    
    RCLCPP_INFO(node->get_logger(), "Starting AdvancedCostmapConverter node");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <cmath>

class GridmapCostmapConverter : public rclcpp::Node
{
public:
    GridmapCostmapConverter() : Node("gridmap_costmap_converter")
    {
        // Initialize parameters
        this->declare_parameter("resolution", 0.1);
        this->declare_parameter("map_width", 50.0);
        this->declare_parameter("map_height", 50.0);
        this->declare_parameter("map_frame", "map");
        this->declare_parameter("robot_frame", "base_link");
        
        // Get parameters
        resolution_ = this->get_parameter("resolution").as_double();
        map_width_ = this->get_parameter("map_width").as_double();
        map_height_ = this->get_parameter("map_height").as_double();
        map_frame_ = this->get_parameter("map_frame").as_string();
        robot_frame_ = this->get_parameter("robot_frame").as_string();
        
        // Initialize TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Initialize publishers
        costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", 10);
        
        // Initialize subscribers
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "pointcloud", 10,
            std::bind(&GridmapCostmapConverter::pointCloudCallback, this, std::placeholders::_1));
        
        // Create a timer for periodic costmap generation
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&GridmapCostmapConverter::generateCostmap, this));
        
        RCLCPP_INFO(this->get_logger(), "GridmapCostmapConverter initialized");
    }

private:
    // ROS2 subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
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
    
    // Point cloud data
    sensor_msgs::msg::PointCloud2 latest_cloud_;
    std::mutex cloud_mutex_;
    
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        latest_cloud_ = *msg;
        RCLCPP_DEBUG(this->get_logger(), "Received point cloud with %d points", msg->width * msg->height);
    }
    
    void generateCostmap()
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        
        // Generate costmap even without point cloud data for testing
        
        // Create occupancy grid
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
        
        // Initialize costmap data
        int width = costmap.info.width;
        int height = costmap.info.height;
        costmap.data.resize(width * height, -1); // Unknown
        
        // Simple costmap generation - create a basic pattern for testing
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int index = y * width + x;
                
                // Create a simple pattern for testing
                double center_x = width / 2.0;
                double center_y = height / 2.0;
                double distance = std::sqrt((x - center_x) * (x - center_x) + (y - center_y) * (y - center_y));
                
                if (distance < 5) {
                    costmap.data[index] = 100; // Obstacle in center
                } else if (distance < 10) {
                    costmap.data[index] = 50;   // High cost around center
                } else {
                    costmap.data[index] = 0;   // Free space
                }
            }
        }
        
        // Publish costmap
        costmap_pub_->publish(costmap);
        
        RCLCPP_INFO(this->get_logger(), "Generated costmap with %dx%d cells", width, height);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<GridmapCostmapConverter>();
    
    RCLCPP_INFO(node->get_logger(), "Starting GridmapCostmapConverter node");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}

#include "gridmap_costmap/terrain_analyzer.hpp"
#include <grid_map_cv/grid_map_cv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

namespace gridmap_costmap
{

TerrainAnalyzer::TerrainAnalyzer()
    : max_slope_(0.5)
    , max_roughness_(0.3)
    , min_elevation_(-2.0)
    , max_elevation_(2.0)
    , slope_weight_(0.4)
    , roughness_weight_(0.3)
    , elevation_weight_(0.3)
    , kernel_size_(3)
    , gaussian_sigma_(1.0)
    , median_kernel_size_(3)
    , morphological_kernel_size_(3)
{
}

TerrainAnalyzer::~TerrainAnalyzer()
{
}

void TerrainAnalyzer::analyzeTerrain(grid_map::GridMap& grid_map)
{
    RCLCPP_INFO(rclcpp::get_logger("terrain_analyzer"), "Starting terrain analysis");
    
    // Calculate slope map
    calculateSlopeMap(grid_map);
    
    // Calculate roughness map
    calculateRoughnessMap(grid_map);
    
    // Calculate traversability map
    calculateTraversabilityMap(grid_map);
    
    RCLCPP_INFO(rclcpp::get_logger("terrain_analyzer"), "Terrain analysis completed");
}

void TerrainAnalyzer::calculateSlopeMap(grid_map::GridMap& grid_map)
{
    if (!grid_map.exists(elevation_layer_)) {
        RCLCPP_WARN(rclcpp::get_logger("terrain_analyzer"), "Elevation layer not found");
        return;
    }
    
    // Add slope layer if it doesn't exist
    if (!grid_map.exists(slope_layer_)) {
        grid_map.add(slope_layer_);
    }
    
    // Calculate slope for each cell
    for (grid_map::GridMapIterator it(grid_map); !it.isPastEnd(); ++it) {
        const grid_map::Index index(*it);
        double slope = calculateSlopeAtPoint(grid_map, index);
        grid_map.at(slope_layer_, index) = slope;
    }
    
    // Apply smoothing
    applyGaussianFilter(grid_map, slope_layer_);
    
    RCLCPP_DEBUG(rclcpp::get_logger("terrain_analyzer"), "Slope map calculated");
}

void TerrainAnalyzer::calculateRoughnessMap(grid_map::GridMap& grid_map)
{
    if (!grid_map.exists(elevation_layer_)) {
        RCLCPP_WARN(rclcpp::get_logger("terrain_analyzer"), "Elevation layer not found");
        return;
    }
    
    // Add roughness layer if it doesn't exist
    if (!grid_map.exists(roughness_layer_)) {
        grid_map.add(roughness_layer_);
    }
    
    // Calculate roughness for each cell
    for (grid_map::GridMapIterator it(grid_map); !it.isPastEnd(); ++it) {
        const grid_map::Index index(*it);
        double roughness = calculateRoughnessAtPoint(grid_map, index);
        grid_map.at(roughness_layer_, index) = roughness;
    }
    
    // Apply smoothing
    applyGaussianFilter(grid_map, roughness_layer_);
    
    RCLCPP_DEBUG(rclcpp::get_logger("terrain_analyzer"), "Roughness map calculated");
}

void TerrainAnalyzer::calculateTraversabilityMap(grid_map::GridMap& grid_map)
{
    if (!grid_map.exists(slope_layer_) || !grid_map.exists(roughness_layer_)) {
        RCLCPP_WARN(rclcpp::get_logger("terrain_analyzer"), "Required layers not found");
        return;
    }
    
    // Add traversability layer if it doesn't exist
    if (!grid_map.exists("traversability")) {
        grid_map.add("traversability");
    }
    
    // Calculate traversability for each cell
    for (grid_map::GridMapIterator it(grid_map); !it.isPastEnd(); ++it) {
        const grid_map::Index index(*it);
        
        double slope = grid_map.at(slope_layer_, index);
        double roughness = grid_map.at(roughness_layer_, index);
        double elevation = grid_map.at(elevation_layer_, index);
        
        double traversability = calculateTraversabilityCost(slope, roughness, elevation);
        grid_map.at("traversability", index) = traversability;
    }
    
    RCLCPP_DEBUG(rclcpp::get_logger("terrain_analyzer"), "Traversability map calculated");
}

double TerrainAnalyzer::calculateSlopeAtPoint(const grid_map::GridMap& grid_map, const grid_map::Index& index)
{
    if (!isValidIndex(grid_map, index)) {
        return 0.0;
    }
    
    double resolution = grid_map.getResolution();
    double elevation = grid_map.at(elevation_layer_, index);
    
    // Calculate gradients in x and y directions
    double grad_x = 0.0, grad_y = 0.0;
    
    // X gradient
    if (isValidIndex(grid_map, grid_map::Index(index.x() + 1, index.y()))) {
        double elevation_right = grid_map.at(elevation_layer_, grid_map::Index(index.x() + 1, index.y()));
        grad_x = (elevation_right - elevation) / resolution;
    }
    
    // Y gradient
    if (isValidIndex(grid_map, grid_map::Index(index.x(), index.y() + 1))) {
        double elevation_up = grid_map.at(elevation_layer_, grid_map::Index(index.x(), index.y() + 1));
        grad_y = (elevation_up - elevation) / resolution;
    }
    
    // Calculate slope magnitude
    double slope = std::sqrt(grad_x * grad_x + grad_y * grad_y);
    
    return slope;
}

double TerrainAnalyzer::calculateRoughnessAtPoint(const grid_map::GridMap& grid_map, const grid_map::Index& index)
{
    if (!isValidIndex(grid_map, index)) {
        return 0.0;
    }
    
    int radius = 2; // Local neighborhood radius
    double variance = calculateLocalVariance(grid_map, index, radius);
    
    return std::sqrt(variance);
}

double TerrainAnalyzer::calculateLocalVariance(const grid_map::GridMap& grid_map, const grid_map::Index& index, int radius)
{
    double sum = 0.0;
    double sum_squared = 0.0;
    int count = 0;
    
    double center_elevation = grid_map.at(elevation_layer_, index);
    
    for (int dx = -radius; dx <= radius; ++dx) {
        for (int dy = -radius; dy <= radius; ++dy) {
            grid_map::Index neighbor_index(index.x() + dx, index.y() + dy);
            
            if (isValidIndex(grid_map, neighbor_index)) {
                double elevation = grid_map.at(elevation_layer_, neighbor_index);
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

double TerrainAnalyzer::calculateTraversabilityCost(double slope, double roughness, double elevation)
{
    double slope_cost = calculateSlopeCost(slope);
    double roughness_cost = calculateRoughnessCost(roughness);
    double elevation_cost = calculateElevationCost(elevation);
    
    return slope_weight_ * slope_cost + 
           roughness_weight_ * roughness_cost + 
           elevation_weight_ * elevation_cost;
}

double TerrainAnalyzer::calculateSlopeCost(double slope)
{
    if (slope > max_slope_) {
        return 1.0; // Maximum cost
    }
    return slope / max_slope_;
}

double TerrainAnalyzer::calculateRoughnessCost(double roughness)
{
    if (roughness > max_roughness_) {
        return 1.0; // Maximum cost
    }
    return roughness / max_roughness_;
}

double TerrainAnalyzer::calculateElevationCost(double elevation)
{
    if (elevation < min_elevation_ || elevation > max_elevation_) {
        return 1.0; // Maximum cost
    }
    
    // Normalize elevation cost
    double normalized_elevation = (elevation - min_elevation_) / (max_elevation_ - min_elevation_);
    return std::abs(normalized_elevation - 0.5) * 2.0; // Cost increases away from center
}

void TerrainAnalyzer::applyGaussianFilter(grid_map::GridMap& grid_map, const std::string& layer)
{
    if (!grid_map.exists(layer)) {
        return;
    }
    
    // Convert to OpenCV Mat
    cv::Mat cv_mat = gridMapToCvMat(grid_map, layer);
    
    // Apply Gaussian filter
    cv::Mat filtered_mat;
    cv::GaussianBlur(cv_mat, filtered_mat, cv::Size(kernel_size_, kernel_size_), gaussian_sigma_);
    
    // Convert back to grid map
    cvMatToGridMap(filtered_mat, grid_map, layer);
}

void TerrainAnalyzer::applyMedianFilter(grid_map::GridMap& grid_map, const std::string& layer)
{
    if (!grid_map.exists(layer)) {
        return;
    }
    
    // Convert to OpenCV Mat
    cv::Mat cv_mat = gridMapToCvMat(grid_map, layer);
    
    // Apply median filter
    cv::Mat filtered_mat;
    cv::medianBlur(cv_mat, filtered_mat, median_kernel_size_);
    
    // Convert back to grid map
    cvMatToGridMap(filtered_mat, grid_map, layer);
}

void TerrainAnalyzer::applyMorphologicalFilter(grid_map::GridMap& grid_map, const std::string& layer)
{
    if (!grid_map.exists(layer)) {
        return;
    }
    
    // Convert to OpenCV Mat
    cv::Mat cv_mat = gridMapToCvMat(grid_map, layer);
    
    // Apply morphological operations
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, 
                                              cv::Size(morphological_kernel_size_, morphological_kernel_size_));
    cv::Mat filtered_mat;
    cv::morphologyEx(cv_mat, filtered_mat, cv::MORPH_CLOSE, kernel);
    
    // Convert back to grid map
    cvMatToGridMap(filtered_mat, grid_map, layer);
}

cv::Mat TerrainAnalyzer::gridMapToCvMat(const grid_map::GridMap& grid_map, const std::string& layer)
{
    if (!grid_map.exists(layer)) {
        return cv::Mat();
    }
    
    // Get the matrix data
    const grid_map::Matrix& matrix = grid_map[layer];
    
    // Convert to OpenCV Mat
    cv::Mat cv_mat;
    grid_map::GridMapCvConverter::toImage<unsigned char, 1>(grid_map, layer, CV_8UC1, 0.0, 1.0, cv_mat);
    
    return cv_mat;
}

void TerrainAnalyzer::cvMatToGridMap(const cv::Mat& cv_mat, grid_map::GridMap& grid_map, const std::string& layer)
{
    if (cv_mat.empty()) {
        return;
    }
    
    // Convert from OpenCV Mat to grid map
    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(cv_mat, layer, grid_map, 0.0, 1.0);
}

bool TerrainAnalyzer::isValidIndex(const grid_map::GridMap& grid_map, const grid_map::Index& index)
{
    return grid_map.isValid(index);
}

} // namespace gridmap_costmap


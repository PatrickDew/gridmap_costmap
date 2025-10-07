#ifndef TERRAIN_ANALYZER_HPP
#define TERRAIN_ANALYZER_HPP

#include <grid_map_core/grid_map_core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

namespace gridmap_costmap
{

class TerrainAnalyzer
{
public:
    TerrainAnalyzer();
    ~TerrainAnalyzer();
    
    // Main analysis methods
    void analyzeTerrain(grid_map::GridMap& grid_map);
    void calculateSlopeMap(grid_map::GridMap& grid_map);
    void calculateRoughnessMap(grid_map::GridMap& grid_map);
    void calculateTraversabilityMap(grid_map::GridMap& grid_map);
    
    // Terrain feature extraction
    void extractTerrainFeatures(const grid_map::GridMap& grid_map, 
                               std::vector<double>& slopes,
                               std::vector<double>& roughness,
                               std::vector<double>& elevations);
    
    // Cost calculation methods
    double calculateTraversabilityCost(double slope, double roughness, double elevation);
    double calculateSlopeCost(double slope);
    double calculateRoughnessCost(double roughness);
    double calculateElevationCost(double elevation);
    
    // Filtering and smoothing
    void applyGaussianFilter(grid_map::GridMap& grid_map, const std::string& layer);
    void applyMedianFilter(grid_map::GridMap& grid_map, const std::string& layer);
    void applyMorphologicalFilter(grid_map::GridMap& grid_map, const std::string& layer);
    
    // Set parameters
    void setMaxSlope(double max_slope) { max_slope_ = max_slope; }
    void setMaxRoughness(double max_roughness) { max_roughness_ = max_roughness; }
    void setSlopeWeight(double weight) { slope_weight_ = weight; }
    void setRoughnessWeight(double weight) { roughness_weight_ = weight; }
    void setElevationWeight(double weight) { elevation_weight_ = weight; }
    
private:
    // Parameters
    double max_slope_;
    double max_roughness_;
    double min_elevation_;
    double max_elevation_;
    double slope_weight_;
    double roughness_weight_;
    double elevation_weight_;
    
    // Analysis parameters
    int kernel_size_;
    double gaussian_sigma_;
    int median_kernel_size_;
    int morphological_kernel_size_;
    
    // Utility methods
    double calculateSlopeAtPoint(const grid_map::GridMap& grid_map, const grid_map::Index& index);
    double calculateRoughnessAtPoint(const grid_map::GridMap& grid_map, const grid_map::Index& index);
    double calculateLocalVariance(const grid_map::GridMap& grid_map, const grid_map::Index& index, int radius);
    bool isValidIndex(const grid_map::GridMap& grid_map, const grid_map::Index& index);
    
    // OpenCV operations
    cv::Mat gridMapToCvMat(const grid_map::GridMap& grid_map, const std::string& layer);
    void cvMatToGridMap(const cv::Mat& cv_mat, grid_map::GridMap& grid_map, const std::string& layer);
};

} // namespace gridmap_costmap

#endif // TERRAIN_ANALYZER_HPP


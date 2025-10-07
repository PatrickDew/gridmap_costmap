# Grid Map Costmap Converter

A ROS2 package for converting grid maps from Gazebo simulation into costmaps for navigation. This package is particularly useful for Mars rover navigation to avoid rough terrain and obstacles.

<!-- ## Features

- **Grid Map to Costmap Conversion**: Converts elevation grid maps to navigation costmaps
- **Terrain Analysis**: Calculates slope, roughness, and traversability metrics
- **Costmap Generation**: Creates both occupancy grids and 2D costmaps
- **Gazebo Integration**: Works with Gazebo simulation environments
- **RViz Visualization**: Real-time visualization of terrain and costmaps
- **Configurable Parameters**: Customizable terrain analysis and cost calculation -->

## Dependencies

### Required ROS2 Packages
- `grid_map_core`
- `grid_map_ros`
- `grid_map_cv`
- `grid_map_costmap_2d`
- `grid_map_filters`
- `nav_msgs`
- `sensor_msgs`
- `geometry_msgs`
- `tf2_ros`
- `cv_bridge`
- `opencv2`
- `pcl_ros`

### System Dependencies
- ROS2 Humble
- Gazebo (for simulation)
- RViz2 (for visualization)
- OpenCV
- PCL

<!-- ## Installation

1. **Clone the repository:**
   ```bash
   cd ~/ros2_ws/src
   git clone <repository-url> gridmap_costmap
   ```

2. **Install dependencies:**
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the package:**
   ```bash
   colcon build --packages-select gridmap_costmap
   source install/setup.bash
   ``` -->

<!-- ## Usage

### Basic Usage

1. **Start the simulation:**
   ```bash
   ros2 launch gridmap_costmap simulation.launch.py
   ```

2. **Start the grid map costmap converter:**
   ```bash
   ros2 launch gridmap_costmap gridmap_costmap.launch.py
   ```

### Advanced Usage

#### Custom Parameters

Create a custom parameter file:

```yaml
# config/custom_params.yaml
gridmap_costmap_converter:
  ros__parameters:
    resolution: 0.05
    map_width: 100.0
    map_height: 100.0
    max_slope: 0.3
    max_roughness: 0.2
    slope_weight: 0.5
    roughness_weight: 0.3
    elevation_weight: 0.2
```

Launch with custom parameters:

```bash
ros2 launch gridmap_costmap gridmap_costmap.launch.py config_file:=config/custom_params.yaml
``` -->

#### Topics

**Subscribed Topics:**
- `/grid_map` (grid_map_msgs/GridMap): Input grid map
- `/pointcloud` (sensor_msgs/PointCloud2): Point cloud data (optional)

**Published Topics:**
- `/costmap` (nav_msgs/OccupancyGrid): Generated occupancy grid
- `/costmap_2d` (nav_msgs/Costmap): Generated 2D costmap
- `/processed_grid_map` (grid_map_msgs/GridMap): Processed grid map
- `/terrain_markers` (visualization_msgs/MarkerArray): Terrain visualization markers

## Configuration

### Terrain Analysis Parameters

| Parameter | Description | Default | Range |
|-----------|-------------|---------|-------|
| `max_slope` | Maximum traversable slope (radians) | 0.5 | 0.0 - 1.57 |
| `max_roughness` | Maximum traversable roughness | 0.3 | 0.0 - 1.0 |
| `min_elevation` | Minimum safe elevation (m) | -2.0 | -10.0 - 10.0 |
| `max_elevation` | Maximum safe elevation (m) | 2.0 | -10.0 - 10.0 |

### Cost Calculation Weights

| Parameter | Description | Default | Range |
|-----------|-------------|---------|-------|
| `slope_weight` | Weight for slope cost | 0.4 | 0.0 - 1.0 |
| `roughness_weight` | Weight for roughness cost | 0.3 | 0.0 - 1.0 |
| `elevation_weight` | Weight for elevation cost | 0.3 | 0.0 - 1.0 |

### Map Parameters

| Parameter | Description | Default | Range |
|-----------|-------------|---------|-------|
| `resolution` | Map resolution (m/cell) | 0.1 | 0.01 - 1.0 |
| `map_width` | Map width (m) | 50.0 | 10.0 - 1000.0 |
| `map_height` | Map height (m) | 50.0 | 10.0 - 1000.0 |

## Algorithm Details

### Terrain Analysis

1. **Slope Calculation**: Computes terrain slope using gradient analysis
2. **Roughness Calculation**: Calculates local terrain roughness using variance
3. **Traversability Assessment**: Combines slope, roughness, and elevation costs

### Cost Calculation

The costmap generation uses a weighted combination of terrain features:

```
cost = slope_weight * slope_cost + roughness_weight * roughness_cost + elevation_weight * elevation_cost
```

### Filtering

- **Gaussian Filter**: Smooths terrain features
- **Median Filter**: Removes noise
- **Morphological Filter**: Closes small gaps

## Examples

### Mars Rover Navigation

```bash
# Start Gazebo with Mars terrain
ros2 launch gridmap_costmap simulation.launch.py world_file:=worlds/mars_terrain.world

# Start costmap converter
ros2 launch gridmap_costmap gridmap_costmap.launch.py

# Visualize in RViz
ros2 run rviz2 rviz2 -d config/simulation.rviz
```

### Custom Terrain

1. Create a custom Gazebo world file
2. Configure terrain parameters
3. Launch with custom configuration

## Troubleshooting

### Common Issues

1. **Missing Dependencies:**
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

2. **TF2 Errors:**
   - Ensure proper frame configuration
   - Check transform publishing

3. **Grid Map Issues:**
   - Verify grid map topic names
   - Check layer names in configuration

### Debug Mode

Enable debug logging:

```bash
ros2 run gridmap_costmap gridmap_costmap_node --ros-args --log-level debug
```

<!-- ## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

This package is licensed under the BSD-3-Clause license. -->

## References

- [Grid Map Library](https://github.com/ANYbotics/grid_map)
- [ROS2 Navigation](https://navigation.ros.org/)
- [Gazebo Simulation](http://gazebosim.org/)

<!-- ## Support

For issues and questions:
- Create an issue on GitHub
- Check the documentation
- Review the configuration examples -->

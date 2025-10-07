#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get the launch directory
    pkg_share = FindPackageShare(package='gridmap_costmap').find('gridmap_costmap')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world_file')
    use_rviz = LaunchConfiguration('use_rviz')
    config_file = LaunchConfiguration('config_file')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(pkg_share, 'worlds', 'smooth_mars_terrain.world'),
        description='Full path to world file to load')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RVIZ')
    
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'advanced_terrain_params.yaml'),
        description='Full path to the config file to use')
    
    # Start Gazebo
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen')
    
    # Spawn the Mars rover
    spawn_robot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'mars_rover', '-file', os.path.join(pkg_share, 'models', 'mars_rover.sdf')],
        output='screen')
    
    # Advanced costmap converter node with enhanced parameters
    advanced_costmap_converter_cmd = Node(
        package='gridmap_costmap',
        executable='gridmap_costmap_advanced_node',
        name='advanced_costmap_converter',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('velodyne_points', '/velodyne_points'),
            ('scan', '/scan'),
            ('camera/image_raw', '/camera/image_raw'),
        ])
    
    # Static transform publisher for map frame
    static_tf_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='screen')
    
    # RViz node with enhanced configuration
    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'advanced_simulation.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_config_file_cmd)
    
    # Add the actions to the launch description
    ld.add_action(start_gazebo_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(static_tf_cmd)
    ld.add_action(advanced_costmap_converter_cmd)
    ld.add_action(rviz_cmd)
    
    return ld

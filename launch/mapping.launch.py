#!/usr/bin/env python3
"""
Launch file for SLAM mapping with the forklift robot.

This launch file starts:
1. Robot simulation (MVSIM)
2. SLAM Toolbox for creating a map
3. RViz for visualization

Usage:
    ros2 launch smart_warehouse mapping.launch.py

To save the map when done:
    ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/smart_warehouse/src/smart_warehouse/maps/mundo_map --ros-args -p use_sim_time:=true
    
Or use the provided script:
    ros2 run smart_warehouse save_map.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_smart_warehouse = get_package_share_directory('smart_warehouse')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Paths - Use warehouse.world.xml as the current world
    world_file = os.path.join(pkg_smart_warehouse, 'worlds', 'warehouse_numerated.world.xml')
    rviz_config = os.path.join(pkg_smart_warehouse, 'rviz_configs', 'mapping.rviz')
    slam_params_file = os.path.join(pkg_smart_warehouse, 'config', 'slam_params.yaml')

    # ==================== 1. SIMULATION ====================
    simulation = Node(
        package='mvsim',
        executable='mvsim_node',
        name='mvsim',
        output='screen',
        parameters=[
            {'world_file': world_file},
            {'headless': False}
        ]
    )

    # ==================== 2. SLAM TOOLBOX ====================
    # Use custom params file if exists
    slam_launch_args = {'use_sim_time': 'true'}
    if os.path.exists(slam_params_file):
        slam_launch_args['slam_params_file'] = slam_params_file
    
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments=slam_launch_args.items()
    )

    # ==================== 3. RVIZ ====================
    rviz_args = []
    if os.path.exists(rviz_config):
        rviz_args = ['-d', rviz_config]
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=rviz_args,
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        
        # Launch nodes
        simulation,
        
        # Delay SLAM start to ensure simulation is ready
        TimerAction(
            period=3.0,
            actions=[slam]
        ),
        
        # Delay RViz start
        TimerAction(
            period=5.0,
            actions=[rviz]
        ),
    ])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_smart_warehouse = get_package_share_directory('smart_warehouse')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Paths
    simulation_launch = os.path.join(pkg_smart_warehouse, 'launch', 'simulation.launch.py')
    nav2_launch_file = os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')
    map_file = os.path.join(pkg_smart_warehouse, 'maps', 'warehouse_numerated_map.yaml')
    params_file = os.path.join(pkg_smart_warehouse, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_smart_warehouse, 'rviz_configs', 'mapping.rviz')
    world_file = os.path.join(pkg_smart_warehouse, 'worlds', 'warehouse_numerated.world.xml')

    # 1. Simulation
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch),
        launch_arguments={'world_file': world_file}.items()
    )

    # 2. Nav2 Bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': 'true',
            'autostart': 'true'
        }.items()
    )

    # 3. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # 4. Graph Visualizer
    graph_viz_node = Node(
        package='smart_warehouse',
        executable='graph_visualizer',
        name='graph_visualizer',
        output='screen'
    )

    return LaunchDescription([
        simulation,
        nav2_bringup,
        rviz_node,
        graph_viz_node
    ])

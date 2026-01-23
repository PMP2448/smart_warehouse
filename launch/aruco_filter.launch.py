#!/usr/bin/env python3
"""
Launch file para iniciar el filtro de outliers del ArUco
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Directorio del paquete
    pkg_dir = get_package_share_directory('smart_warehouse')
    config_file = os.path.join(pkg_dir, 'config', 'aruco_filter_params.yaml')
    
    # Argumentos de launch
    enable_tf_arg = DeclareLaunchArgument(
        'enable_tf',
        default_value='true',
        description='Habilitar publicación de transforms filtrados'
    )
    
    # Nodo del filtro
    filter_node = Node(
        package='smart_warehouse',
        executable='aruco_pose_filter',
        name='aruco_pose_filter',
        output='screen',
        parameters=[config_file, {'enable_tf_broadcast': LaunchConfiguration('enable_tf')}]
    )
    
    return LaunchDescription([
        enable_tf_arg,
        filter_node,
    ])

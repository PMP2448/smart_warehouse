import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_smart_warehouse = get_package_share_directory('smart_warehouse')
    pkg_mvsim = get_package_share_directory('mvsim')

    # Configurable arguments
    run_docking_arg = DeclareLaunchArgument(
        'run_docking',
        default_value='false',
        description='Launch the ArUco docking logic node'
    )

    # Paths
    aruco_world_file = os.path.join(pkg_smart_warehouse, 'worlds', 'aruco_test_world_v2.xml')
    mvsim_launch = os.path.join(pkg_mvsim, 'launch', 'launch_world.launch.py')
    rviz_config_file = os.path.join(pkg_smart_warehouse, 'rviz_configs', 'aruco_test.rviz')
    filter_params_file = os.path.join(pkg_smart_warehouse, 'config', 'aruco_filter_params.yaml')

    # 1. MVSIM World
    mvsim_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mvsim_launch),
        launch_arguments={'world_file': aruco_world_file, 'use_rviz': 'false'}.items()
    )

    # 2. ArUco Detector (aruco_ros single)
    # Based on notes.txt, adapted to Launch syntax
    aruco_detector_node = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        parameters=[{
            'marker_id': 0,
            'marker_size': 0.3333,
            'reference_frame': 'base_link',
            'marker_frame': 'aruco_detectado',
            'image_is_rectified': False,
            'camera_frame': 'aruco_cam',
            'use_sim_time': True
        }],
        remappings=[
            ('/image', '/aruco_cam/image_raw'),
            ('/camera_info', '/aruco_cam/camera_info')
        ],
        output='screen'
    )

    # 3. ArUco Pose Filter
    aruco_filter_node = Node(
        package='smart_warehouse',
        executable='aruco_pose_filter',
        name='aruco_pose_filter',
        parameters=[filter_params_file],
        output='screen'
    )

    # 4. Docking Logic (Optional)
    docking_node = Node(
        package='smart_warehouse',
        executable='docking_aruco',
        name='aruco_docking',
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_docking'))
    )

    # 5. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        run_docking_arg,
        mvsim_node,
        aruco_detector_node,
        aruco_filter_node,
        docking_node,
        rviz_node
    ])

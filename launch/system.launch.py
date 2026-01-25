import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_smart_warehouse = get_package_share_directory('smart_warehouse')

    # Paths
    navigation_launch = os.path.join(pkg_smart_warehouse, 'launch', 'navigation.launch.py')
    graph_file = os.path.join(pkg_smart_warehouse, 'graphs', 'warehouse_graph.geojson')

    # 1. Navigation (Sim + Nav2 + RViz)
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch)
    )

    waypoint_follower = Node(
        package='smart_warehouse',
        executable='waypoint_follower',
        name='graph_navigator',
        output='screen',
        parameters=[{'graph_file': graph_file},
                    {'use_sim_time': True}]
    )

    # 3. Interface Node (GUI)
    interface = Node(
        package='smart_warehouse',
        executable='interface_node',
        name='interface_node',
        output='screen'
    )

    # 4. Lift Controller
    lift_controller = Node(
        package='smart_warehouse',
        executable='lift_controller',
        name='lift_controller',
        output='screen'
    )

    # 5. Aruco Approach (Integrated Manager)
    aruco_approach = Node(
        package='smart_warehouse',
        executable='aruco_approach',
        name='aruco_approach',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 6. Aruco Pose Filter
    aruco_pose_filter = Node(
        package='smart_warehouse',
        executable='aruco_pose_filter',
        name='aruco_pose_filter',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        navigation,
        waypoint_follower,
        interface,
        lift_controller,
        aruco_approach,
        aruco_pose_filter
    ])

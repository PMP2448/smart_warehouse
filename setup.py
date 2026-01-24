import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'smart_warehouse'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.xml')),
        (os.path.join('share', package_name, 'definitions'), glob('definitions/*.xml')),
        (os.path.join('share', package_name, 'rviz_configs'), glob('rviz_configs/*.rviz')),
        (os.path.join('share', package_name, 'aruco_textures'), glob('aruco_textures/*.png')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'templates'), glob('templates/*')),
        (os.path.join('share', package_name, 'graphs'), glob('graphs/*.geojson')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pmp',
    maintainer_email='pmp@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aruco_generator = smart_warehouse.aruco_generator:main',
            'monitor_aruco = smart_warehouse.monitor_aruco:main',
            'aruco_pose_filter = smart_warehouse.aruco_pose_filter:main',
            'docking_aruco = smart_warehouse.docking_aruco:main',
            'graph_editor = smart_warehouse.graph_editor:main',
            'graph_visualizer = smart_warehouse.graph_visualizer:main',
            'interface_node = smart_warehouse.interface_node:main',
            'lift_controller = smart_warehouse.lift_controller:main',
            'waypoint_follower = smart_warehouse.waypoint_follower:main',
            'pallet_visual_servoing = smart_warehouse.pallet_visual_servoing:main'
        ],
    },
)

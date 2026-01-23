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
            'docking_aruco_v2 = smart_warehouse.docking_aruco_v2:main',
            'docking_aruco_v3 = smart_warehouse.docking_aruco_v3:main',
            'docking_aruco_v4 = smart_warehouse.docking_aruco_v4:main',
            'docking_aruco_v4_filtered = smart_warehouse.docking_aruco_v4_filtered:main',
        ],
    },
)

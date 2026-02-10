import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_uav_slam = get_package_share_directory('uav_slam')
    config_path = os.path.join(pkg_uav_slam, 'config', 'velodyne_16.yaml')

    # PointCloud Timestamp Injector - adds per-point 'time' field
    # Required for FAST-LIO motion undistortion in Gazebo simulation
    timestamp_injector = Node(
        package='uav_slam',
        executable='pointcloud_timestamp_injector.py',
        name='pointcloud_timestamp_injector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'scan_period': 0.1  # 10Hz LiDAR
        }]
    )

    # FAST-LIO SLAM node
    fast_lio = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[
            config_path,
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        timestamp_injector,
        fast_lio
    ])


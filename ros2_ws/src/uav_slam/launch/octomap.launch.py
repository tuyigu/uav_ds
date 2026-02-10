import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_uav_slam = get_package_share_directory('uav_slam')
    config_path = os.path.join(pkg_uav_slam, 'config', 'octomap.yaml')

    return LaunchDescription([
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[
                config_path,
                {'use_sim_time': True}
            ],
            remappings=[
                ('cloud_in', '/cloud_registered')
            ]
        )
    ])

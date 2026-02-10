import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('uav_navigation'),
        'config', 'planner_params.yaml'
    )

    path_planner = Node(
        package='uav_navigation',
        executable='path_planner.py',
        name='path_planner',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription([
        path_planner
    ])

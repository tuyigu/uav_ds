import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_uav_simulation = get_package_share_directory('uav_simulation')

    # Bridge ROS topics
    # Map GZ topics to ROS topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {'config_file': os.path.join(pkg_uav_simulation, 'launch', 'bridge.yaml')},
            {'qos_overrides./tf_static.publisher.durability': 'transient_local'},
        ],
        output='screen'
    )

    return LaunchDescription([
        bridge
    ])

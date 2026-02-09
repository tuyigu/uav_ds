from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    uav_bt_agent_share = get_package_share_directory('uav_bt_agent')
    tree_xml_path = os.path.join(uav_bt_agent_share, 'config', 'simple_flight_tree.xml')

    return LaunchDescription([
        # 1. Flight Core (UDP connection to PX4)
        Node(
            package='flight_core',
            executable='flight_core_node',
            name='flight_core',
            output='screen'
        ),

        # 2. Web Agent (WebSocket server + Bridge)
        Node(
            package='uav_web_agent',
            executable='uav_web_agent_node',
            name='uav_web_agent',
            output='screen'
        ),

        # 3. Behavior Tree Agent (Logic)
        Node(
            package='uav_bt_agent',
            executable='uav_bt_agent_node',
            name='uav_bt_agent',
            output='screen',
            arguments=[tree_xml_path]  # Pass the full path to the XML
        ),
    ])

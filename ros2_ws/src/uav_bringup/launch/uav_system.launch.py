import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Path to the default BT XML file
    uav_bt_agent_share = get_package_share_directory('uav_bt_agent')
    default_bt_xml_path = os.path.join(uav_bt_agent_share, 'config', 'simple_flight_tree.xml')

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
            parameters=[
                {'bt_xml_path': default_bt_xml_path}
            ]
        ),

        # 4. ROSBridge Server (WebSocket Interface for Web Backend)
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port': 9090}]
        ),
    ])

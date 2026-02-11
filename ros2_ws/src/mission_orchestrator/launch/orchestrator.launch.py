from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mission_orchestrator',
            executable='orchestrator_node',
            name='mission_orchestrator',
            output='screen',
            parameters=[{
                'home_x': 0.0,
                'home_y': 0.0,
                'home_z': 0.0,
                'cruise_alt': 30.0,
                'approach_alt': 15.0,
                'search_alt': 5.0,
                'takeoff_height': 5.0,
                'loop_rate_hz': 2.0,
            }],
        ),
    ])

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Launch Simulation (Gazebo + Bridge)
    # Note: bridge is launched inside simulation.launch.py
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uav_simulation'),
                'launch',
                'simulation.launch.py'
            ])
        ])
    )

    # 2. Launch TF Infrastructure (RSP + TF Broadcaster)
    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uav_bringup'),
                'launch',
                'tf_setup.launch.py'
            ])
        ]),
        # Disable PX4 TFs if we are using SLAM (SLAM will provide map->base_link)
        launch_arguments={
            'use_sim_time': 'true',
            'enable_tf': 'false' 
        }.items()
    )

    # 3. Launch SLAM (FAST-LIO)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uav_slam'),
                'launch',
                'fast_lio.launch.py'
            ])
        ])
    )

    delayed_slam = TimerAction(
        period=10.0,
        actions=[slam_launch]
    )

    # 4. Launch OctoMap (3D Occupancy Mapping)
    octomap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uav_slam'),
                'launch',
                'octomap.launch.py'
            ])
        ])
    )

    delayed_octomap = TimerAction(
        period=15.0,
        actions=[octomap_launch]
    )

    # 5. Launch Perception (ArUco Detector)
    # Wait 5 seconds for simulation and TF to stabilize
    perception_node = Node(
        package='uav_perception',
        executable='aruco_detector',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    delayed_perception = TimerAction(
        period=5.0,
        actions=[perception_node]
    )

    # 6. Launch Path Planner (after OctoMap is ready)
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uav_navigation'),
                'launch',
                'path_planner.launch.py'
            ])
        ])
    )

    delayed_nav = TimerAction(
        period=20.0,
        actions=[nav_launch]
    )

    return LaunchDescription([
        sim_launch,
        tf_launch,
        delayed_slam,
        delayed_octomap,
        delayed_perception,
        delayed_nav
    ])

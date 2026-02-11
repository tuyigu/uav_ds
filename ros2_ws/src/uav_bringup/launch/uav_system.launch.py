import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

# ── Configuration ──────────────────────────────────────────────
# PX4 default home position (Matches urban_delivery.sdf)
HOME_LAT = 47.3979711
HOME_LON = 8.5461637
YOLO_MODEL = '/home/wss/Dev/Robotics/workspaces/uav_ds/ros2_ws/src/uav_perception/config/aruco_yolov8n.pt'


def generate_launch_description():

    # ================================================================
    # Layer 1: Simulation + Sensor Infrastructure
    # ================================================================

    # 1. Gazebo simulation + ROS-GZ bridge
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uav_simulation'),
                'launch',
                'simulation.launch.py'
            ])
        ])
    )

    # 2. TF Infrastructure (RSP + TF Broadcaster)
    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uav_bringup'),
                'launch',
                'tf_setup.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'enable_tf': 'false'
        }.items()
    )

    # ================================================================
    # Layer 2: SLAM + Mapping (delayed for sim stabilization)
    # ================================================================

    # 3. FAST-LIO SLAM
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uav_slam'),
                'launch',
                'fast_lio.launch.py'
            ])
        ])
    )
    delayed_slam = TimerAction(period=10.0, actions=[slam_launch])

    # 4. OctoMap 3D occupancy grid
    octomap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uav_slam'),
                'launch',
                'octomap.launch.py'
            ])
        ])
    )
    delayed_octomap = TimerAction(period=15.0, actions=[octomap_launch])

    # ================================================================
    # Layer 3: Perception (camera-based detection)
    # ================================================================

    # 5. ArUco detector (precise, low altitude)
    aruco_node = Node(
        package='uav_perception',
        executable='aruco_detector',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    delayed_aruco = TimerAction(period=5.0, actions=[aruco_node])

    # 6. YOLO detector (coarse, high altitude)
    yolo_node = Node(
        package='uav_perception',
        executable='yolo_detector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'model_path': YOLO_MODEL,
            'target_class': 'armor',
            'confidence_threshold': 0.5,
            'max_fps': 10.0,
        }]
    )
    delayed_yolo = TimerAction(period=8.0, actions=[yolo_node])

    # 7. Marker fuser (height-aware YOLO↔ArUco switch)
    fuser_node = Node(
        package='uav_perception',
        executable='marker_fuser',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'switch_altitude': 5.0,
            'target_class': 'armor',
            'min_yolo_confidence': 0.4,
        }]
    )
    delayed_fuser = TimerAction(period=8.0, actions=[fuser_node])

    # ================================================================
    # Layer 4: Navigation + Flight Control
    # ================================================================

    # 8. Path planner (A* with OctoMap)
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uav_navigation'),
                'launch',
                'path_planner.launch.py'
            ])
        ])
    )
    delayed_nav = TimerAction(period=20.0, actions=[nav_launch])

    # 9. FlightCore FSM (action servers: takeoff/land/move_to)
    flight_core_node = Node(
        package='flight_core',
        executable='flight_core_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    delayed_flight = TimerAction(period=5.0, actions=[flight_core_node])

    # ================================================================
    # Layer 5: Decision + Communication
    # ================================================================

    # 10. rosbridge server (WebSocket for web backend)
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': 9090,
            'address': '',
        }]
    )
    delayed_rosbridge = TimerAction(period=2.0, actions=[rosbridge_node])

    # 11. Web Agent (ROS↔Web topic bridge)
    web_agent_node = Node(
        package='uav_web_agent',
        executable='uav_web_agent_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    delayed_web_agent = TimerAction(period=3.0, actions=[web_agent_node])

    # 12. Mission Orchestrator (decision layer)
    orchestrator_node = Node(
        package='mission_orchestrator',
        executable='orchestrator_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'home_x': 0.0,
            'home_y': 0.0,
            'home_z': 0.0,
            'home_lat': HOME_LAT,
            'home_lon': HOME_LON,
            'cruise_alt': 30.0,
            'approach_alt': 15.0,
            'search_alt': 5.0,
            'takeoff_height': 5.0,
            'loop_rate_hz': 2.0,
        }]
    )
    delayed_orchestrator = TimerAction(period=10.0, actions=[orchestrator_node])

    # ================================================================
    # Layer 6: BT Agent (behavior tree executor — starts last)
    # ================================================================

    # 13. BT Agent
    bt_agent_node = Node(
        package='uav_bt_agent',
        executable='bt_agent_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    delayed_bt = TimerAction(period=12.0, actions=[bt_agent_node])

    # ================================================================
    # Launch all (ordered by dependency + delay)
    # ================================================================
    return LaunchDescription([
        # Layer 1: Simulation
        sim_launch,
        tf_launch,
        # Layer 2: SLAM
        delayed_slam,
        delayed_octomap,
        # Layer 3: Perception
        delayed_aruco,
        delayed_yolo,
        delayed_fuser,
        # Layer 4: Navigation + Flight
        delayed_nav,
        delayed_flight,
        # Layer 5: Communication + Decision
        delayed_rosbridge,
        delayed_web_agent,
        delayed_orchestrator,
        # Layer 6: Execution
        delayed_bt,
    ])

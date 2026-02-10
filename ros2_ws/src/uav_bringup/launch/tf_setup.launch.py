from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare(package='uav_bringup').find('uav_bringup')
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'x500_delivery.urdf'])

    enable_tf_arg = DeclareLaunchArgument(
        'enable_tf',
        default_value='true',
        description='Whether to enable PX4 TF broadcaster'
    )

    # Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['cat ', urdf_file]), 'use_sim_time': True}]
    )

    # TF Broadcaster (Dynamic transforms from PX4)
    tf_broadcaster = Node(
        package='uav_bringup',
        executable='px4_tf_broadcaster',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'enable_tf': LaunchConfiguration('enable_tf')
        }]
    )

    # Static TF: Gazebo Sensor Frame -> URDF Camera Link
    # Gazebo frame: x500_0/camera_down_link/camera_down_sensor
    # URDF link: camera_down_link
    # We bridge them with identity transform so lookups work
    gazebo_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'camera_down_link', '--child-frame-id', 'x500_0/camera_down_link/camera_down_sensor', '--ros-args', '-p', 'use_sim_time:=true']
    )

    return LaunchDescription([
        enable_tf_arg,
        rsp,
        tf_broadcaster,
        gazebo_camera_tf
    ])

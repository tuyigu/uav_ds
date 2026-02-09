from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # 🌟 必须开启 Sim Time，否则时间戳对不上
    SIM_TIME_SETTING = True

    urdf_content = """
    <robot name="x500_depth">
      <link name="base_link"/>
      <link name="camera_link"/>
      <link name="camera_link_optical"/>
      
      <joint name="camera_joint" type="fixed">
        <parent link="base_link"/><child link="camera_link"/>
        <origin xyz="0.12 0.03 0.242" rpy="0 0 0"/>
      </joint>
      
      <joint name="camera_optical_joint" type="fixed">
        <parent link="camera_link"/><child link="camera_link_optical"/>
        <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
      </joint>
    </robot>
    """

    rtabmap_launch_path = PathJoinSubstitution(
        [FindPackageShare('rtabmap_launch'), 'launch', 'rtabmap.launch.py']
    )

    return LaunchDescription([
        # A. 静态 TF 发布者
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': urdf_content,
                'use_sim_time': SIM_TIME_SETTING,
                'publish_frequency': 30.0
            }]
        ),

        # B. 动态 TF & Odom 桥接器
        Node(
            package='px4_tf_broadcaster',
            executable='px4_tf_broadcaster_node',
            name='px4_odom_bridge',
            output='screen',
            parameters=[{'use_sim_time': SIM_TIME_SETTING}]
        ),

        # C. Sensor Bridge (核心修改：适配 walls 环境)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='sensor_bridge',
            output='screen',
            arguments=[
                # 1. 时钟 (列表里有 /clock，直接桥接)
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

                # 2. RGB 图像 (前缀改成了 /world/walls/...)
                '/world/walls/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image',

                # 3. 深度图像 (保持不变)
                '/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image',

                # 4. 相机内参 (前缀改成了 /world/walls/...)
                '/world/walls/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

                '--ros-args',
                '-r', '/world/walls/model/x500_depth_0/link/camera_link/sensor/IMX214/image:=/camera/image_raw',
                '-r', '/depth_camera:=/camera/depth/image_raw',
                '-r', '/world/walls/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info:=/camera/camera_info'
            ],
            parameters=[{'use_sim_time': SIM_TIME_SETTING}]
        ),

        # D. RTAB-Map SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch_path),
            launch_arguments={
                'rtabmap_args': '--delete_db_on_start',
                'use_sim_time': str(SIM_TIME_SETTING),
                'frame_id': 'base_link',
                'visual_odometry': 'False',
                'odom_topic': '/odom',
                'rgb_topic': '/camera/image_raw',
                'depth_topic': '/camera/depth/image_raw',
                'camera_info_topic': '/camera/camera_info',
                'approx_sync': 'True',
                'queue_size': '30',
                'qos': '2',
                'rviz': 'True'
            }.items()
        )
    ])
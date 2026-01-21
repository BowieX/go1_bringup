import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ---------------- 获取路径 ----------------
    unitree_pkg = get_package_share_directory('unitree_ros')
    livox_pkg = get_package_share_directory('livox_ros_driver2')
    fast_lio_pkg = get_package_share_directory('fast_lio')
    bringup_pkg = get_package_share_directory('go1_bringup')
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')

    # ---------------- 配置文件路径 ----------------
    # 确保刚才 setup.py 修改生效后，这里能找到文件
    pc2scan_config = os.path.join(bringup_pkg, 'config', 'pointcloud_to_laserscan_params.yaml')
    slam_config = os.path.join(bringup_pkg, 'config', 'slam_params.yaml')

    # ---------------- 1. 硬件驱动 ----------------
    unitree_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(unitree_pkg, 'launch', 'unitree_driver_launch.py')
        )
    )

    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_pkg, 'launch', 'msg_MID360_launch.py')
        ),
        launch_arguments={
            'rviz_enable': 'false',
            'publish_freq': '10.0',
            'msg_frame_id': 'livox_frame'
        }.items()
    )

    # ---------------- 2. TF 变换 ----------------
    # body -> livox_frame
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_lidar_tf',
        arguments=['0.1', '0.0', '0.15', '0.0', '0.0', '0.0', 'body', 'livox_frame']
    )

    # camera_init -> odom_lio (兼容性TF)
    map_identity_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_init_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_init', 'odom_lio']
    )

    # ---------------- 3. 定位算法 (FAST_LIO) ----------------
    fast_lio_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_pkg, 'launch', 'mapping.launch.py')
        )
    )

    # ---------------- 4. 数据转换 (3D -> 2D) ----------------
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/cloud_registered'),
            ('scan', '/scan')
        ],
        parameters=[pc2scan_config] # 加载参数文件
    )

    # ---------------- 5. 2D 建图 (SLAM Toolbox) ----------------
    # 直接调用 SLAM Toolbox 的 launch，并传入 params_file
    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_pkg, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'slam_params_file': slam_config}.items()
    )

    return LaunchDescription([
        unitree_driver,
        livox_driver,
        lidar_tf,
        map_identity_tf,
        fast_lio_node,
        pointcloud_to_laserscan_node,
        slam_toolbox_node
    ])

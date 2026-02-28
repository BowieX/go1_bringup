# 文件路径: ~/go1_ws/src/go1_bringup/launch/go1_base.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # ---------------- 路径获取 ----------------
    unitree_pkg = get_package_share_directory('unitree_ros')
    livox_pkg = get_package_share_directory('livox_ros_driver2')
    fast_lio_pkg = get_package_share_directory('fast_lio')
    bringup_pkg = get_package_share_directory('go1_bringup')

    # ---------------- 配置文件 ----------------
    # 3D转2D的参数配置
    pc2scan_config = os.path.join(bringup_pkg, 'config', 'pointcloud_to_laserscan_params.yaml')

    # ---------------- 1. 硬件驱动 ----------------
    # Unitree Go1 驱动 (底层运动控制)
    unitree_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(unitree_pkg, 'launch', 'unitree_driver_launch.py')
        )
    )

    # Livox MID360 驱动
    # 注意：msg_MID360_launch.py 不接受 launch_arguments
    # 如需修改参数，请直接编辑 livox_ros_driver2/launch/msg_MID360_launch.py
    # 默认配置：frame_id='livox_frame', publish_freq=10.0, xfer_format=1 (CustomMsg)
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_pkg, 'launch', 'msg_MID360_launch.py')
        )
    )

    # ---------------- 2. TF 变换 ----------------
    # 静态 TF: 告诉 ROS 雷达安装在机器狗的什么位置
    # 假设雷达安装在狗背部上方 15cm, 前方 10cm
    # body 是机器狗中心 (FAST_LIO输出), livox_frame 是雷达数据坐标系
    # ROS 2 Humble 推荐使用 --frame-id 和 --child-frame-id 格式
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_lidar_tf',
        arguments=[
            '--x', '0.1',
            '--y', '0.0', 
            '--z', '0.15',
            '--yaw', '0.0',
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'body',
            '--child-frame-id', 'livox_frame'
        ]
    )

    # ---------------- 3. 里程计算法 (FAST_LIO) ----------------
    # FAST_LIO 负责发布 /Odometry 和 TF (camera_init -> body)
    fast_lio_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_pkg, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'rviz': 'false',          # 基础启动时不看 Rviz
            'config_file': 'mid360.yaml' # 确保这里对应你的 yaml 文件名
        }.items()
    )

    # ---------------- 4. 数据转换 (3D -> 2D) ----------------
    # 将 FAST_LIO 输出的去畸变 3D 点云 (/cloud_registered) 压扁成 2D 激光 (/scan)
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/cloud_registered'), # FAST_LIO 的输出话题
            ('scan', '/scan')
        ],
        parameters=[pc2scan_config]
    )

    return LaunchDescription([
        unitree_driver,
        livox_driver,
        lidar_tf,
        fast_lio_node,
        pointcloud_to_laserscan_node
    ])

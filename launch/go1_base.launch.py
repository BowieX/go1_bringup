# 文件路径: ~/go1_ws/src/go1_bringup/launch/go1_base.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # ---------------- Launch 参数 ----------------
    use_odom_fusion = LaunchConfiguration('use_odom_fusion')
    declare_use_odom_fusion = DeclareLaunchArgument(
        'use_odom_fusion',
        default_value='true',
        description='是否启动 robot_localization EKF 融合节点 (false=基础模式，不依赖 robot_localization)')

    # ---------------- 路径获取 ----------------
    unitree_pkg = get_package_share_directory('unitree_ros')
    livox_pkg = get_package_share_directory('livox_ros_driver2')
    fast_lio_pkg = get_package_share_directory('fast_lio')
    bringup_pkg = get_package_share_directory('go1_bringup')

    # ---------------- 配置文件 ----------------
    # 3D转2D的参数配置
    pc2scan_config = os.path.join(bringup_pkg, 'config', 'pointcloud_to_laserscan_params.yaml')
    # EKF融合里程计配置 (robot_localization)
    ekf_config = os.path.join(bringup_pkg, 'config', 'ekf_odom_fusion.yaml')

    # ---------------- 1. 硬件驱动 ----------------
    # Unitree Go1 驱动 (底层运动控制)
    unitree_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(unitree_pkg, 'launch', 'unitree_driver_launch.py')
        )
    )

    # Livox MID360S 驱动
    # 注意：msg_MID360s_launch.py 不接受 launch_arguments
    # 如需修改参数，请直接编辑 livox_ros_driver2/launch_ROS2/msg_MID360s_launch.py
    # 默认配置：frame_id='livox_frame', publish_freq=10.0, xfer_format=1 (CustomMsg)
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_pkg, 'launch', 'msg_MID360s_launch.py')
        )
    )

    # ---------------- 2. TF 变换 ----------------
    # 静态 TF: body -> livox_frame (LiDAR 相对机体的安装外参)
    #
    # ⚠️ 实机部署必须重新测量以下 6 个参数！ (见实验手册 §4.5 LiDAR 外参测量)
    #    body      = 机器狗腰部几何中心 (四腿投影中心)，FAST-LIO 位姿输出帧
    #    livox_frame = MID-360S 外壳几何中心，LiDAR 数据帧
    #    坐标系: X 前、Y 左、Z 上 (ROS REP-103)
    #    x 正值 = LiDAR 在机体前方；y 正值 = 在左侧；z 正值 = 在上方
    #    精度要求: x/y 误差 <2cm，z 误差 <1cm
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_lidar_tf',
        arguments=[
            '--x', '0.10',     # TODO(实机): LiDAR 相对 body 的前后偏移 (m)
            '--y', '0.00',     # TODO(实机): 左右偏移 (m)
            '--z', '0.15',     # TODO(实机): 上下偏移 (m)
            '--yaw',   '0.0',  # TODO(实机): 安装偏角 (rad, 绕 Z)
            '--pitch', '0.0',  # TODO(实机): 安装俯仰 (rad, 绕 Y，向下倾为负)
            '--roll',  '0.0',  # TODO(实机): 安装滚转 (rad, 绕 X)
            '--frame-id', 'body',
            '--child-frame-id', 'livox_frame'
        ]
    )

    # 静态 TF: 帧桥接 unitree_base -> body (单位变换)
    # unitree_ros 发布的里程计使用 unitree_odom -> unitree_base 帧
    # robot_localization 需要在 body 帧中工作，因此需要桥接
    # 仅在启用里程计融合时需要
    unitree_base_to_body_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='unitree_base_to_body_tf',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--yaw', '0.0',
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'unitree_base',
            '--child-frame-id', 'body'
        ],
        condition=IfCondition(use_odom_fusion)
    )

    # ---------------- 3. 里程计算法 (FAST_LIO) ----------------
    # FAST_LIO 负责发布 /Odometry 和 TF (camera_init -> body)
    fast_lio_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_pkg, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'rviz': 'false',          # 基础启动时不看 Rviz
            'config_file': 'mid360.yaml' # FAST-LIO 配置 (MID-360 和 MID-360S 通用)
        }.items()
    )

    # ---------------- 4. EKF 里程计融合 (robot_localization) ----------------
    # 融合 Go1 腿部里程计 (/odom) 和机体 IMU (/imu)，输出平滑里程计 (/odometry/filtered)
    # 该融合结果作为先验信息，供改进后的 FAST-LIO2 在几何退化场景下使用
    # 仅在启用里程计融合时启动 (use_odom_fusion:=true)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('odometry/filtered', '/odometry/filtered')
        ],
        condition=IfCondition(use_odom_fusion)
    )

    # ---------------- 5. 数据转换 (3D -> 2D) ----------------
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
        declare_use_odom_fusion,
        unitree_driver,
        livox_driver,
        lidar_tf,
        unitree_base_to_body_tf,
        fast_lio_node,
        ekf_node,
        pointcloud_to_laserscan_node
    ])

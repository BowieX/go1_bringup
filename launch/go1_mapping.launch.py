# 文件路径: ~/go1_ws/src/go1_bringup/launch/go1_mapping.launch.py
#
# 路线 B 建图方案 (2026-04 调整):
#   FAST-LIO 在线建图 + 累积 PCD, 不再启动 slam_toolbox.
#   退出后用 scripts/pcd_to_map.py 将 FAST_LIO/PCD/scans.pcd 切片为 Nav2 静态地图.
#
# 动机: slam_toolbox 2D 扫描匹配在 Go1 颠簸 + 长廊退化场景下累计角度误差,
#       而 FAST-LIO 3D 建图 (ikd-Tree 增量更新) 质量明显更高.
#       离线切片避开 2D SLAM 弱点, Nav2 全栈保持原样.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('go1_bringup')

    # Launch 参数
    use_odom_fusion = LaunchConfiguration('use_odom_fusion')
    declare_use_odom_fusion = DeclareLaunchArgument(
        'use_odom_fusion',
        default_value='true',
        description='是否启动 robot_localization EKF 融合节点')

    record_bag = LaunchConfiguration('record_bag')
    declare_record_bag = DeclareLaunchArgument(
        'record_bag',
        default_value='true',
        description='是否自动 rosbag 录制本次建图 (转发给 go1_base)')

    bag_dir = LaunchConfiguration('bag_dir')
    declare_bag_dir = DeclareLaunchArgument(
        'bag_dir',
        default_value=os.path.join(os.path.expanduser('~'), 'go1_ws', 'bags', 'mapping'),
        description='bag 输出根目录 (建议与 nav 分开, 默认 ~/go1_ws/bags/mapping)')

    rviz_config = os.path.join(bringup_pkg, 'config', 'go1_mapping.rviz')

    # 1. 启动底层 (驱动 + FAST-LIO + pointcloud_to_laserscan + 可选 EKF + rosbag)
    #    FAST-LIO 会在节点退出时把累积的 3D 点云写到 FAST_LIO/PCD/scans.pcd
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'go1_base.launch.py')
        ),
        launch_arguments={
            'use_odom_fusion': use_odom_fusion,
            'record_bag': record_bag,
            'bag_dir': bag_dir,
        }.items()
    )

    # 2. RViz2 (建图可视化: 看 FAST-LIO /cloud_registered + /Odometry)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_mapping',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        declare_use_odom_fusion,
        declare_record_bag,
        declare_bag_dir,
        base_launch,
        rviz_node,
    ])

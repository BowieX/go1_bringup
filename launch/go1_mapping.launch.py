# 文件路径: ~/go1_ws/src/go1_bringup/launch/go1_mapping.launch.py
#
# 路线 B 建图方案 (2026-04 调整):
#   FAST-LIO 在线建图 + 累积 PCD, 不再启动 slam_toolbox.
#   退出后用 `ros2 run go1_bringup pcd_to_map` 把 FAST_LIO/PCD/scans.pcd 切片为
#   Nav2 静态地图.
#
# 动机: slam_toolbox 2D 扫描匹配在 Go1 颠簸 + 长廊退化场景下累计角度误差,
#       而 FAST-LIO 3D 建图 (ikd-Tree 增量更新) 质量明显更高.
#       离线切片避开 2D SLAM 弱点, Nav2 全栈保持原样.
#
# 2026-05-02: 接入归档守护进程 (auto_archive 参数, 默认 true)
#   launch 启动时拉起 archive_on_shutdown, 退出信号到来后等待 scans.pcd
#   写入稳定, 再调用 archive_map 复制 PCD 并生成 4 套过滤变体 + 论文对比图.
#   防止下次建图覆盖, 同时一次性产出对比素材.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
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

    enable_odom_constraint = LaunchConfiguration('enable_odom_constraint')
    declare_enable_odom_constraint = DeclareLaunchArgument(
        'enable_odom_constraint',
        default_value='false',
        description='是否让 FAST-LIO 使用 /odometry/filtered 作为退化场景位置约束 '
                    '(需要 use_odom_fusion:=true)')

    force_odom_degraded = LaunchConfiguration('force_odom_degraded')
    declare_force_odom_degraded = DeclareLaunchArgument(
        'force_odom_degraded',
        default_value='false',
        description='是否强制所有里程计约束帧按退化高权重处理 '
                    '(仅用于常开强约束消融, 正式建图保持 false)')

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

    auto_archive = LaunchConfiguration('auto_archive')
    declare_auto_archive = DeclareLaunchArgument(
        'auto_archive',
        default_value='true',
        description='Ctrl+C 退出后是否自动归档 PCD + 生成 4 套过滤变体 + 对比图 '
                    '(false 时只保留 FAST_LIO/PCD/scans.pcd, 用户需手动跑 archive_map.py)')

    archive_label = LaunchConfiguration('archive_label')
    declare_archive_label = DeclareLaunchArgument(
        'archive_label',
        default_value='',
        description='归档目录后缀 (例如 lab_run1), 时间戳后追加, 方便辨识会话')

    rviz_config = os.path.join(bringup_pkg, 'config', 'go1_mapping.rviz')

    # 1. 启动底层 (驱动 + FAST-LIO + pointcloud_to_laserscan + 可选 EKF + rosbag)
    #    FAST-LIO 会在节点退出时把累积的 3D 点云写到 FAST_LIO/PCD/scans.pcd
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'go1_base.launch.py')
        ),
        launch_arguments={
            'use_odom_fusion': use_odom_fusion,
            'enable_odom_constraint': enable_odom_constraint,
            'force_odom_degraded': force_odom_degraded,
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

    # 3. 自动归档守护进程
    #
    # FAST-LIO 在节点析构时才把累积点云写到 scans.pcd。此前用 OnShutdown
    # 临时启动 archive_map, 实机 Ctrl+C 时容易被 launch 关闭流程取消, 导致
    # 没有 sessions 目录。这里在 launch 启动时先拉起轻量 watcher, 退出信号
    # 到来后由 watcher 等 PCD 稳定再归档, 比 shutdown 阶段新建进程更可靠。
    archive_watcher = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'go1_bringup', 'archive_on_shutdown',
            '--label', archive_label,
            '--wait-seconds', '30',
        ],
        output='screen',
        condition=IfCondition(auto_archive),
        sigterm_timeout='240',
        sigkill_timeout='260',
    )

    return LaunchDescription([
        declare_use_odom_fusion,
        declare_enable_odom_constraint,
        declare_force_odom_degraded,
        declare_record_bag,
        declare_bag_dir,
        declare_auto_archive,
        declare_archive_label,
        archive_watcher,
        base_launch,
        rviz_node,
    ])

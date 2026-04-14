# 文件路径: ~/go1_ws/src/go1_bringup/launch/go1_replay.launch.py
#
# 离线 rosbag 回放专用 launch 文件 — 用于 SLAM 消融实验
#
# 功能: 不启动硬件驱动，仅启动算法节点，配合 rosbag 回放进行离线对比实验。
#       通过 odom_constraint 参数切换基线 (无融合) 和改进版 (有融合) 模式。
#
# 使用方式:
#   终端1 (算法节点):
#     ros2 launch go1_bringup go1_replay.launch.py odom_constraint:=false
#   终端2 (轨迹记录):
#     python3 ~/go1_ws/src/go1_bringup/scripts/record_trajectory.py \
#       --ros-args -p use_sim_time:=true -p output_dir:=<path> -p record_mode:=fastlio
#   终端3 (回放数据):
#     ros2 bag play <bag_path> --clock --topics /livox/lidar /livox/imu /odom /imu
#
# 注意: 回放时只播放原始传感器话题，避免与算法输出话题冲突

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    fast_lio_pkg = get_package_share_directory('fast_lio')
    bringup_pkg = get_package_share_directory('go1_bringup')

    # ---------------- Launch 参数 ----------------
    odom_constraint = LaunchConfiguration('odom_constraint')
    declare_odom_constraint = DeclareLaunchArgument(
        'odom_constraint',
        default_value='false',
        description='是否启用里程计融合约束 (false=基线, true=改进版)')

    # ---------------- 配置文件路径 ----------------
    fast_lio_config = os.path.join(fast_lio_pkg, 'config', 'mid360.yaml')
    ekf_config = os.path.join(bringup_pkg, 'config', 'ekf_odom_fusion.yaml')

    # ---------------- 静态 TF ----------------
    # body -> livox_frame (始终需要)
    # ⚠️ 外参必须与 go1_base.launch.py 保持一致！修改时两处同步 (见实验手册 §4.5)
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_lidar_tf',
        arguments=[
            '--x', '0.10',
            '--y', '0.00',
            '--z', '0.15',
            '--yaw',   '0.0',
            '--pitch', '0.0',
            '--roll',  '0.0',
            '--frame-id', 'body',
            '--child-frame-id', 'livox_frame'
        ],
        parameters=[{'use_sim_time': True}]
    )

    # unitree_base -> body (仅融合模式需要，为 robot_localization 桥接帧)
    unitree_base_to_body_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='unitree_base_to_body_tf',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--frame-id', 'unitree_base',
            '--child-frame-id', 'body'
        ],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(odom_constraint)
    )

    # ---------------- FAST-LIO2 节点 (互斥启动) ----------------
    # 基线模式: 关闭里程计约束
    fast_lio_baseline = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        parameters=[
            fast_lio_config,
            {'use_sim_time': True,
             'odom_constraint.enable': False}
        ],
        output='screen',
        condition=UnlessCondition(odom_constraint)
    )

    # 改进模式: 开启里程计约束
    fast_lio_improved = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        parameters=[
            fast_lio_config,
            {'use_sim_time': True,
             'odom_constraint.enable': True}
        ],
        output='screen',
        condition=IfCondition(odom_constraint)
    )

    # ---------------- robot_localization EKF (仅改进模式) ----------------
    # 融合 /odom + /imu → /odometry/filtered，供 FAST-LIO2 里程计约束使用
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
        remappings=[
            ('odometry/filtered', '/odometry/filtered')
        ],
        condition=IfCondition(odom_constraint)
    )

    return LaunchDescription([
        declare_odom_constraint,
        lidar_tf,
        unitree_base_to_body_tf,
        fast_lio_baseline,
        fast_lio_improved,
        ekf_node
    ])

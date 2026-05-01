# 文件路径: ~/go1_ws/src/go1_bringup/launch/go1_replay.launch.py
#
# 离线 rosbag 回放专用 launch 文件 — 用于 SLAM 消融实验
#
# 功能: 不启动硬件驱动，仅启动算法节点，配合 rosbag 回放进行离线对比实验。
#       通过 odom_constraint 参数切换基线 (无融合) 和改进版 (有融合) 模式。
#       force_degraded:=true 可把改进版切换为"常开强约束"机制消融。
#
# 使用方式:
#   终端1 (算法节点):
#     ros2 launch go1_bringup go1_replay.launch.py odom_constraint:=false
#     ros2 launch go1_bringup go1_replay.launch.py odom_constraint:=true \
#       degradation_feat_threshold:=400 degradation_residual_threshold:=0.15
#   终端2 (轨迹记录):
#     ros2 run go1_bringup record_trajectory --ros-args \
#       -p use_sim_time:=true -p output_dir:=<path> -p record_mode:=fastlio
#   终端3 (回放数据):
#     ros2 bag play <bag_path> --clock --topics /livox/lidar /livox/imu /odom /imu
#
# 注意: 回放时只播放原始传感器话题，避免与算法输出话题冲突

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_lidar_extrinsics(share_dir):
    """读 config/lidar_extrinsics.yaml, 与 go1_base.launch.py 共用同一份外参."""
    with open(os.path.join(share_dir, 'config', 'lidar_extrinsics.yaml')) as f:
        cfg = yaml.safe_load(f)['lidar_extrinsics']
    return (
        str(cfg['x']), str(cfg['y']), str(cfg['z']),
        str(cfg['roll']), str(cfg['pitch']), str(cfg['yaw']),
        cfg['parent_frame'], cfg['child_frame'],
    )


def generate_launch_description():
    fast_lio_pkg = get_package_share_directory('fast_lio')
    bringup_pkg = get_package_share_directory('go1_bringup')

    # ---------------- Launch 参数 ----------------
    odom_constraint = LaunchConfiguration('odom_constraint')
    force_degraded = LaunchConfiguration('force_degraded')
    degradation_feat_threshold = LaunchConfiguration('degradation_feat_threshold')
    degradation_residual_threshold = LaunchConfiguration('degradation_residual_threshold')
    declare_odom_constraint = DeclareLaunchArgument(
        'odom_constraint',
        default_value='false',
        description='是否启用里程计融合约束 (false=基线, true=改进版)')
    declare_force_degraded = DeclareLaunchArgument(
        'force_degraded',
        default_value='false',
        description='是否强制所有里程计约束帧按退化高权重处理 (仅用于常开强约束消融)')
    declare_degradation_feat_threshold = DeclareLaunchArgument(
        'degradation_feat_threshold',
        default_value='200',
        description='退化判据: 有效特征点数低于该阈值时触发强里程计约束')
    declare_degradation_residual_threshold = DeclareLaunchArgument(
        'degradation_residual_threshold',
        default_value='0.15',
        description='退化判据: 平均残差高于该阈值时触发强里程计约束')

    # ---------------- 配置文件路径 ----------------
    fast_lio_config = os.path.join(fast_lio_pkg, 'config', 'mid360.yaml')
    ekf_config = os.path.join(bringup_pkg, 'config', 'ekf_odom_fusion.yaml')

    # ---------------- 静态 TF ----------------
    # body -> livox_frame: 与 go1_base.launch.py 共用 config/lidar_extrinsics.yaml,
    # 修改外参只改 yaml 一处.
    x, y, z, roll, pitch, yaw, parent, child = _load_lidar_extrinsics(bringup_pkg)
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_lidar_tf',
        arguments=[
            '--x', x, '--y', y, '--z', z,
            '--yaw', yaw, '--pitch', pitch, '--roll', roll,
            '--frame-id', parent, '--child-frame-id', child,
        ],
        parameters=[{'use_sim_time': True}]
    )

    # unitree_base -> imu (身份变换, 仅融合模式需要)
    # 让 robot_localization 能把 /imu (frame_id="imu") 变换到 base_link_frame=unitree_base.
    # 不再用 unitree_base -> body 桥接以免造成 body 双父 TF 冲突, 详见 go1_base.launch.py 注释.
    unitree_base_to_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='unitree_base_to_imu_tf',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--yaw', '0.0',
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'unitree_base',
            '--child-frame-id', 'imu'
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
             'odom_constraint.enable': True,
             'odom_constraint.force_degraded': ParameterValue(
                 force_degraded, value_type=bool),
             'odom_constraint.degradation_feat_threshold': ParameterValue(
                 degradation_feat_threshold, value_type=int),
             'odom_constraint.degradation_residual_threshold': ParameterValue(
                 degradation_residual_threshold, value_type=float)}
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
        declare_force_degraded,
        declare_degradation_feat_threshold,
        declare_degradation_residual_threshold,
        lidar_tf,
        unitree_base_to_imu_tf,
        fast_lio_baseline,
        fast_lio_improved,
        ekf_node
    ])

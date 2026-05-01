# 文件路径: ~/go1_ws/src/go1_bringup/launch/go1_base.launch.py

import os
from datetime import datetime

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node


def _load_lidar_extrinsics(share_dir):
    """读 config/lidar_extrinsics.yaml, 返回 (x, y, z, roll, pitch, yaw, parent, child)."""
    with open(os.path.join(share_dir, 'config', 'lidar_extrinsics.yaml')) as f:
        cfg = yaml.safe_load(f)['lidar_extrinsics']
    return (
        str(cfg['x']), str(cfg['y']), str(cfg['z']),
        str(cfg['roll']), str(cfg['pitch']), str(cfg['yaw']),
        cfg['parent_frame'], cfg['child_frame'],
    )


def _resolve_livox_mid360s_launch(livox_share_dir):
    """优先使用已安装的 MID360s launch, install 过期时回退到源码 launch_ROS2."""
    installed = os.path.join(livox_share_dir, 'launch', 'msg_MID360s_launch.py')
    if os.path.exists(installed):
        return installed

    # 当前工程采用 ~/go1_ws/src 作为 colcon 工作区根。旧 install 里可能只装了
    # msg_MID360_launch.py, 但源码 launch_ROS2 下已经有 MID360s 版本; 为了让
    # go1_base 在重编译 livox_ros_driver2 前也能给出可执行路径, 这里做一次回退。
    workspace_root = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.dirname(livox_share_dir))))
    source_launch = os.path.join(
        workspace_root, 'livox_ros_driver2', 'launch_ROS2', 'msg_MID360s_launch.py')
    if os.path.exists(source_launch):
        return source_launch

    raise FileNotFoundError(
        '未找到 Livox MID360s launch。请重新编译 livox_ros_driver2, 或确认 '
        'livox_ros_driver2/launch_ROS2/msg_MID360s_launch.py 存在。')


def generate_launch_description():
    # ---------------- Launch 参数 ----------------
    use_odom_fusion = LaunchConfiguration('use_odom_fusion')
    declare_use_odom_fusion = DeclareLaunchArgument(
        'use_odom_fusion',
        default_value='true',
        description='是否启动 robot_localization EKF 融合节点 (false=基础模式，不依赖 robot_localization)')

    enable_odom_constraint = LaunchConfiguration('enable_odom_constraint')
    declare_enable_odom_constraint = DeclareLaunchArgument(
        'enable_odom_constraint',
        default_value='false',
        description='是否让 FAST-LIO 使用 /odometry/filtered 作为退化场景位置约束 '
                    '(需要 use_odom_fusion:=true 或已有 /odometry/filtered 发布者)')

    force_odom_degraded = LaunchConfiguration('force_odom_degraded')
    declare_force_odom_degraded = DeclareLaunchArgument(
        'force_odom_degraded',
        default_value='false',
        description='是否强制 FAST-LIO 将所有里程计约束帧视为退化帧 '
                    '(仅用于常开强约束消融实验, 正式建图/导航保持 false)')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时钟 /clock (离线回放时设为 true)')

    # ---------- rosbag 自动录制参数 ----------
    # 每次启动实验都会在 bag_dir/<时间戳>/ 下生成一份 bag, 供离线回放与消融实验复用.
    # 离线回放 (go1_replay.launch.py / use_sim_time:=true) 时必须手动关掉, 否则会递归录制旧数据.
    record_bag = LaunchConfiguration('record_bag')
    declare_record_bag = DeclareLaunchArgument(
        'record_bag',
        default_value='true',
        description='是否自动 ros2 bag record 全部关键话题 (离线回放时建议 false)')

    bag_dir = LaunchConfiguration('bag_dir')
    declare_bag_dir = DeclareLaunchArgument(
        'bag_dir',
        default_value=os.path.join(os.path.expanduser('~'), 'go1_ws', 'bags'),
        description='bag 输出根目录, 实际保存路径为 <bag_dir>/<YYYY-MM-DD_HH-MM-SS>')

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
    # Unitree 参数由 go1_bringup 提供一份项目内固定配置，避免当前环境误解析到
    # /opt/ros/humble/share/unitree_ros 后使用 odom/base_link 默认帧。
    unitree_config = os.path.join(bringup_pkg, 'config', 'unitree_go1_params.yaml')
    livox_launch = _resolve_livox_mid360s_launch(livox_pkg)

    # ---------------- 1. 硬件驱动 ----------------
    # Unitree Go1 驱动 (底层运动控制)
    unitree_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(unitree_pkg, 'launch', 'unitree_driver_launch.py')
        ),
        launch_arguments={
            'params_file': unitree_config,
        }.items()
    )

    # Livox MID360S 驱动 (注意是 MID360s 不是 MID360, 且 launch 不接受 launch_arguments)
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            livox_launch
        ),
    )

    # ---------------- 2. TF 变换 ----------------
    # 静态 TF: body -> livox_frame (LiDAR 相对机体的安装外参)
    # 数值由 config/lidar_extrinsics.yaml 提供, go1_replay.launch.py 读同一份文件,
    # 修改外参只改 yaml 一个地方即可.
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
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 静态 TF: unitree_base -> imu (身份变换)
    #
    # 背景: unitree_ros 发布 /imu 时 header.frame_id="imu", 但没有发布 imu 帧到
    # 其他帧的 TF. robot_localization EKF 需要把 IMU 数据从 imu 帧变换到
    # base_link_frame (unitree_base), 因此必须补这条静态 TF.
    #
    # 偏移为何取 0: Go1 机体 IMU 装在四足几何中心, 相对 unitree_base 几乎无偏移
    # (毫米级, 对 EKF 姿态融合可忽略).
    #
    # 为何不直接把 unitree_base 和 body 桥接: 那样 body 会同时有 camera_init
    # (FAST-LIO) 和 unitree_base 两个父节点, tf2 会报 TF_REPEATED_DATA 警告并
    # 让位姿查询结果时序相关. 现行方案让 FAST-LIO 侧 (camera_init -> body) 与
    # unitree 侧 (unitree_odom -> unitree_base -> imu) 两棵子树互不相交, EKF
    # 只在 unitree 子树中工作, 其输出 /odometry/filtered 由 FAST-LIO 的
    # odom_constraint 按消息内容读取 (不依赖 TF): 首帧记录 unitree_odom 与
    # camera_init 的 SE(2) 初始关系, 后续只把腿部里程计平面位移投到
    # FAST-LIO 的 camera_init 平面, 不在 TF 树里桥接 body 与 unitree_base.
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
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_odom_fusion)
    )

    # ---------------- 3. 里程计算法 (FAST_LIO) ----------------
    # FAST_LIO 负责发布 /Odometry 和 TF (camera_init -> body)
    fast_lio_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_pkg, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'rviz': 'false',
            'config_file': 'mid360.yaml',
            'use_sim_time': use_sim_time,
            'odom_constraint_enable': enable_odom_constraint,
            'odom_constraint_force_degraded': force_odom_degraded,
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
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
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
            ('cloud_in', '/cloud_registered'),  # FAST_LIO 的输出话题
            ('scan', '/scan')
        ],
        parameters=[pc2scan_config, {'use_sim_time': use_sim_time}]
    )

    # ---------------- 6. rosbag 自动录制 ----------------
    # 关键原始话题全量录制, 供后续消融实验回放 (见 go1_replay.launch.py).
    # 话题选择依据: FAST-LIO 需要 /livox/{lidar,imu}; robot_localization 需要 /odom /imu;
    #              /tf_static 让外参在离线重建时可复用; /cmd_vel + goal/status 用于导航 trial 分析;
    #              /Odometry 录制算法输出, 便于无需重跑 FAST-LIO 即可核对轨迹.
    # 注: 不录 /tf (动态 TF 在回放时会由 FAST-LIO 重新生成, 录了也会冲突).
    bag_stamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    bag_output = PathJoinSubstitution([bag_dir, bag_stamp])
    bag_topics = [
        '/livox/lidar', '/livox/imu',
        '/odom', '/imu',
        '/tf_static',
        '/cmd_vel',
        '/goal_pose',
        '/navigate_to_pose/_action/status',
        '/Odometry',
    ]
    bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_output, *bag_topics],
        output='screen',
        condition=IfCondition(record_bag),
    )
    bag_log = LogInfo(
        msg=['[go1_base] rosbag recording to: ', bag_output],
        condition=IfCondition(record_bag),
    )

    return LaunchDescription([
        declare_use_odom_fusion,
        declare_enable_odom_constraint,
        declare_force_odom_degraded,
        declare_use_sim_time,
        declare_record_bag,
        declare_bag_dir,
        LogInfo(msg=['[go1_base] unitree_ros share: ', unitree_pkg]),
        LogInfo(msg=['[go1_base] Unitree params: ', unitree_config]),
        LogInfo(msg=['[go1_base] Livox launch: ', livox_launch]),
        bag_log,
        unitree_driver,
        livox_driver,
        lidar_tf,
        unitree_base_to_imu_tf,
        fast_lio_node,
        ekf_node,
        pointcloud_to_laserscan_node,
        bag_record,
    ])

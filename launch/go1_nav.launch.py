# 文件路径: ~/go1_ws/src/go1_bringup/launch/go1_nav.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    go1_bringup_pkg = get_package_share_directory('go1_bringup')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    # 默认地图: my_lab.yaml (由 PCD 离线切片生成, 见实验手册 §5)
    # 首次部署前必须完成建图; 否则 launch 时显式 map:=.../test_map.yaml 回退到占位地图
    default_map = os.path.join(go1_bringup_pkg, 'maps', 'my_lab.yaml')
    nav_params = os.path.join(go1_bringup_pkg, 'config', 'go1_nav_params.yaml')

    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_odom_fusion = LaunchConfiguration('use_odom_fusion')
    enable_odom_constraint = LaunchConfiguration('enable_odom_constraint')
    record_bag = LaunchConfiguration('record_bag')
    bag_dir = LaunchConfiguration('bag_dir')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml file to load')

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=nav_params,
        description='Full path to the ROS2 parameters file to use')

    declare_use_odom_fusion = DeclareLaunchArgument(
        'use_odom_fusion',
        default_value='false',
        description='是否启动 robot_localization EKF 融合节点 (默认 false: nav 阶段 '
                    'FAST-LIO odom_constraint 默认关闭, AMCL 也不订阅 /odometry/filtered, '
                    '启 EKF 也无人消费, 只浪费 CPU; 在线改进模式需与 '
                    'enable_odom_constraint:=true 同时打开)')

    declare_enable_odom_constraint = DeclareLaunchArgument(
        'enable_odom_constraint',
        default_value='false',
        description='是否让 FAST-LIO 使用 /odometry/filtered 作为退化场景位置约束 '
                    '(需要 use_odom_fusion:=true)')

    declare_record_bag = DeclareLaunchArgument(
        'record_bag',
        default_value='true',
        description='是否自动 rosbag 录制本次实验 (转发给 go1_base)')

    declare_bag_dir = DeclareLaunchArgument(
        'bag_dir',
        default_value=os.path.join(os.path.expanduser('~'), 'go1_ws', 'bags', 'nav'),
        description='bag 输出根目录 (建议与 mapping 分开, 默认 ~/go1_ws/bags/nav)')

    # 1. 启动底层 (驱动 + 里程计) - 注意：这里不带 SLAM
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go1_bringup_pkg, 'launch', 'go1_base.launch.py')
        ),
        launch_arguments={
            'use_odom_fusion': use_odom_fusion,
            'enable_odom_constraint': enable_odom_constraint,
            'record_bag': record_bag,
            'bag_dir': bag_dir,
        }.items()
    )

    # 2. 启动 Nav2 (定位 + 规划 + 控制)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'params_file': params_file,
            'use_sim_time': 'False',
            'slam': 'False',       # 关键点：禁用 Nav2 自带的 SLAM，使用 AMCL
            'autostart': 'True',   # 自动让 Nav2 进入 Active 状态
            'log_level': 'info'
        }.items()
    )

    # 3. Rviz (Nav2 视角)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(nav2_bringup_pkg, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        declare_map,
        declare_params,
        declare_use_odom_fusion,
        declare_enable_odom_constraint,
        declare_record_bag,
        declare_bag_dir,
        base_launch,
        nav2_launch,
        rviz_node
    ])

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

    # 默认参数
    # 假设你建图后保存的地图叫 my_map.yaml，放在 maps 文件夹下
    # 如果没有 maps 文件夹，请先创建: mkdir -p ~/go1_ws/src/go1_bringup/maps
    default_map = os.path.join(go1_bringup_pkg, 'maps', 'my_lab.yaml')
    nav_params = os.path.join(go1_bringup_pkg, 'config', 'go1_nav_params.yaml')

    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_odom_fusion = LaunchConfiguration('use_odom_fusion')
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
        default_value='true',
        description='是否启动 robot_localization EKF 融合节点')

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
        declare_record_bag,
        declare_bag_dir,
        base_launch,
        nav2_launch,
        rviz_node
    ])

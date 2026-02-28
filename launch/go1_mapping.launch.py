# 文件路径: ~/go1_ws/src/go1_bringup/launch/go1_mapping.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    bringup_pkg = get_package_share_directory('go1_bringup')
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')

    # 配置文件
    slam_config = os.path.join(bringup_pkg, 'config', 'slam_params.yaml')

    # 1. 启动底层 (驱动 + 里程计 + 数据转换)
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'go1_base.launch.py')
        )
    )

    # 2. 启动 SLAM Toolbox (建图模式)
    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_pkg, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'slam_params_file': slam_config}.items()
    )

    # 3. 启动 Rviz (可选，这里不强制，可以手动启动)
    # 建议手动运行: ros2 run rviz2 rviz2 -d <your_rviz_config>

    return LaunchDescription([
        base_launch,
        slam_toolbox_node
    ])

# 文件路径: ~/go1_ws/src/go1_bringup/launch/go1_sim_test.launch.py
#
# AMCL + Nav2 仿真测试 launch 文件 — 无需真实硬件，无需 Gazebo
#
# 原理:
#   - robot_state_publisher 发布 TF (map -> odom -> base_footprint)
#   - fake_robot_node.py 在地图中模拟机器人位置，发布 /scan + /odom + /tf
#   - Nav2 bringup (AMCL + A* + TEB) 正常启动，订阅上述话题
#   - RViz 可视化：粒子云、扫描点、规划路径
#
# 启动方式:
#   ros2 launch go1_bringup go1_sim_test.launch.py
#
# 测试脚本 (另开终端):
#   python3 ~/go1_ws/src/go1_bringup/scripts/test_amcl_convergence.py
#   python3 ~/go1_ws/src/go1_bringup/scripts/test_nav2_navigation.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('go1_bringup')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    sim_map = os.path.join(bringup_pkg, 'maps', 'sim_test_map.yaml')
    nav_params = os.path.join(bringup_pkg, 'config', 'go1_sim_nav_params.yaml')
    fake_robot_script = os.path.join(bringup_pkg, 'scripts', 'fake_robot_node.py')

    # ---------------- Launch 参数 ----------------
    map_yaml = LaunchConfiguration('map')
    declare_map = DeclareLaunchArgument(
        'map', default_value=sim_map,
        description='仿真地图 yaml 路径')

    # ---------------- 最简机器人 URDF ----------------
    # 只有一个 base_footprint link，TEB/AMCL 需要这个帧
    robot_description = (
        '<?xml version="1.0"?>'
        '<robot name="go1_sim">'
        '  <link name="base_footprint"/>'
        '  <link name="base_link"/>'
        '  <joint name="base_joint" type="fixed">'
        '    <parent link="base_footprint"/>'
        '    <child link="base_link"/>'
        '    <origin xyz="0 0 0" rpy="0 0 0"/>'
        '  </joint>'
        '  <link name="laser_link"/>'
        '  <joint name="laser_joint" type="fixed">'
        '    <parent link="base_link"/>'
        '    <child link="laser_link"/>'
        '    <origin xyz="0 0 0.1" rpy="0 0 0"/>'
        '  </joint>'
        '</robot>'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )

    # ---------------- 虚假机器人节点 ----------------
    # 发布: /scan (LaserScan), /odom (Odometry), TF odom->base_footprint
    # 接收: /cmd_vel (Twist) 驱动模拟位置更新
    # 使用 ExecuteProcess 直接运行 Python 脚本（避免 entry_point 配置）
    fake_robot = ExecuteProcess(
        cmd=[
            'python3', fake_robot_script,
            '--ros-args',
            '-p', f'map_yaml:={sim_map}',
            '-p', 'init_x:=0.0',
            '-p', 'init_y:=0.0',
            '-p', 'init_yaw:=0.0',
            '-p', 'scan_topic:=/scan',
            '-p', 'odom_topic:=/odom',
        ],
        output='screen'
    )

    # ---------------- Nav2 bringup ----------------
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'params_file': nav_params,
            'use_sim_time': 'False',
            'slam': 'False',
            'autostart': 'True',
        }.items()
    )

    # ---------------- RViz ----------------
    rviz_config = os.path.join(bringup_pkg, 'config', 'go1_sim_test.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    return LaunchDescription([
        declare_map,
        robot_state_publisher,
        fake_robot,
        nav2_launch,
        rviz,
    ])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    unitree_pkg = get_package_share_directory('unitree_ros')
    livox_pkg = get_package_share_directory('livox_ros_driver2')

    unitree_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(unitree_pkg, 'launch', 'unitree_driver_launch.py')
        )
    )

    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_pkg, 'launch', 'rviz_MID360_launch.py')
        ),
        launch_arguments={'rviz_enable': 'false', 'msg_frame_id': 'livox_frame'}.items()
    )

    #  --x --y --z --yaw --pitch --roll frame_id child_frame_id
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf_publisher',
        arguments=['--x', '0.1', '--y', '0.0', '--z', '0.15', '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0', '--frame-id', 'base_link', '--child-frame-id', 'livox_frame']
    )

    return LaunchDescription([
        unitree_driver,
        lidar_tf,
        livox_driver
    ])

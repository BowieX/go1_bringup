#!/usr/bin/env python3
"""
轨迹记录节点 - 用于 evo 评估工具

功能:
  订阅 FAST-LIO2 的 /Odometry 和 robot_localization 的 /odometry/filtered，
  将位姿数据以 TUM 格式保存到文件，供 evo 工具进行轨迹精度评估。

TUM 格式: timestamp tx ty tz qx qy qz qw

使用方式:
  ros2 run go1_bringup record_trajectory.py --ros-args \
    -p output_dir:=/path/to/save \
    -p record_mode:=both
"""

import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')

        # 参数: 输出目录和记录模式
        self.declare_parameter('output_dir', os.path.expanduser('~/go1_ws/trajectories'))
        self.declare_parameter('record_mode', 'both')  # 'fastlio', 'fused', 'both'

        output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.record_mode = self.get_parameter('record_mode').get_parameter_value().string_value

        # 创建输出目录
        os.makedirs(output_dir, exist_ok=True)

        # 打开输出文件
        self.file_fastlio = None
        self.file_fused = None

        if self.record_mode in ('fastlio', 'both'):
            filepath = os.path.join(output_dir, 'traj_fastlio.txt')
            self.file_fastlio = open(filepath, 'w')
            self.get_logger().info(f'Recording FAST-LIO2 trajectory to: {filepath}')
            # 订阅 FAST-LIO2 里程计
            self.sub_fastlio = self.create_subscription(
                Odometry, '/Odometry', self.fastlio_callback, 50)

        if self.record_mode in ('fused', 'both'):
            filepath = os.path.join(output_dir, 'traj_fused_odom.txt')
            self.file_fused = open(filepath, 'w')
            self.get_logger().info(f'Recording fused odom trajectory to: {filepath}')
            # 订阅融合里程计
            self.sub_fused = self.create_subscription(
                Odometry, '/odometry/filtered', self.fused_callback, 50)

        self.fastlio_count = 0
        self.fused_count = 0

        # 每10秒输出一次统计
        self.timer = self.create_timer(10.0, self.print_stats)

    def odom_to_tum_line(self, msg: Odometry) -> str:
        """将 Odometry 消息转换为 TUM 格式的一行"""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        return f'{t:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} {q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n'

    def fastlio_callback(self, msg: Odometry):
        if self.file_fastlio:
            self.file_fastlio.write(self.odom_to_tum_line(msg))
            self.fastlio_count += 1

    def fused_callback(self, msg: Odometry):
        if self.file_fused:
            self.file_fused.write(self.odom_to_tum_line(msg))
            self.fused_count += 1

    def print_stats(self):
        self.get_logger().info(
            f'Recorded: FAST-LIO2={self.fastlio_count}, Fused={self.fused_count} poses')
        # 刷新文件确保数据写入磁盘
        if self.file_fastlio:
            self.file_fastlio.flush()
        if self.file_fused:
            self.file_fused.flush()

    def destroy_node(self):
        """节点销毁时关闭文件"""
        if self.file_fastlio:
            self.file_fastlio.close()
            self.get_logger().info(f'FAST-LIO2 trajectory saved ({self.fastlio_count} poses)')
        if self.file_fused:
            self.file_fused.close()
            self.get_logger().info(f'Fused odom trajectory saved ({self.fused_count} poses)')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

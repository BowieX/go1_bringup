#!/usr/bin/env python3
"""
轨迹记录节点 - 用于 evo 评估工具.

功能:
  订阅 FAST-LIO2 的 /Odometry 和 robot_localization 的 /odometry/filtered，
  将位姿数据以 TUM 格式保存到文件，供 evo 工具进行轨迹精度评估。

TUM 格式: timestamp tx ty tz qx qy qz qw

record_mode 选项:
  - 'fastlio': 仅记录 /Odometry (FAST-LIO2)
  - 'fused':   仅记录 /odometry/filtered (EKF 融合)
  - 'both':    记录 fastlio + fused
  - 'all':     记录 fastlio + fused + /odom (原始腿部里程计)

experiment_label 参数 (可选):
  设置后输出文件名会包含此标签，方便 evaluate_slam.sh 直接使用:
    - 'baseline'  → traj_fastlio_baseline.txt
    - 'improved'  → traj_fastlio_improved.txt
    - ''          → traj_fastlio.txt (默认，不加标签)

使用方式 (在线):
  python3 record_trajectory.py --ros-args -p output_dir:=/path -p record_mode:=both

使用方式 (消融实验回放):
  python3 record_trajectory.py --ros-args -p use_sim_time:=true \
    -p output_dir:=$HOME/go1_ws/trajectories \
    -p record_mode:=fastlio -p experiment_label:=baseline
"""

import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')

        # 参数: 输出目录、记录模式、实验标签
        self.declare_parameter('output_dir', os.path.expanduser('~/go1_ws/trajectories'))
        self.declare_parameter('record_mode', 'both')  # 'fastlio', 'fused', 'both', 'all'
        self.declare_parameter('experiment_label', '')   # 'baseline'/'improved'/''

        output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.record_mode = self.get_parameter('record_mode').get_parameter_value().string_value
        label = self.get_parameter('experiment_label').get_parameter_value().string_value
        # 文件名后缀: '' → 无后缀, 'baseline' → '_baseline'
        self.label_suffix = f'_{label}' if label else ''

        # 创建输出目录
        os.makedirs(output_dir, exist_ok=True)

        # 打开输出文件
        self.file_fastlio = None
        self.file_fused = None
        self.file_odom = None

        if self.record_mode in ('fastlio', 'both', 'all'):
            filepath = os.path.join(output_dir, f'traj_fastlio{self.label_suffix}.txt')
            self.file_fastlio = open(filepath, 'w')
            self.get_logger().info(f'Recording FAST-LIO2 trajectory to: {filepath}')
            self.sub_fastlio = self.create_subscription(
                Odometry, '/Odometry', self.fastlio_callback, 50)

        if self.record_mode in ('fused', 'both', 'all'):
            filepath = os.path.join(output_dir, f'traj_fused_odom{self.label_suffix}.txt')
            self.file_fused = open(filepath, 'w')
            self.get_logger().info(f'Recording fused odom trajectory to: {filepath}')
            self.sub_fused = self.create_subscription(
                Odometry, '/odometry/filtered', self.fused_callback, 50)

        if self.record_mode == 'all':
            filepath = os.path.join(output_dir, f'traj_leg_odom{self.label_suffix}.txt')
            self.file_odom = open(filepath, 'w')
            self.get_logger().info(f'Recording raw leg odom trajectory to: {filepath}')
            self.sub_odom = self.create_subscription(
                Odometry, '/odom', self.odom_callback, 50)

        self.fastlio_count = 0
        self.fused_count = 0
        self.odom_count = 0

        # 每10秒输出一次统计
        self.timer = self.create_timer(10.0, self.print_stats)

    def odom_to_tum_line(self, msg: Odometry) -> str:
        """将 Odometry 消息转换为 TUM 格式的一行."""
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

    def odom_callback(self, msg: Odometry):
        if self.file_odom:
            self.file_odom.write(self.odom_to_tum_line(msg))
            self.odom_count += 1

    def print_stats(self):
        parts = [f'FAST-LIO2={self.fastlio_count}']
        if self.file_fused:
            parts.append(f'Fused={self.fused_count}')
        if self.file_odom:
            parts.append(f'LegOdom={self.odom_count}')
        self.get_logger().info(f'Recorded: {", ".join(parts)} poses')
        # 刷新文件确保数据写入磁盘
        if self.file_fastlio:
            self.file_fastlio.flush()
        if self.file_fused:
            self.file_fused.flush()
        if self.file_odom:
            self.file_odom.flush()

    def destroy_node(self):
        """节点销毁时关闭文件."""
        if self.file_fastlio:
            self.file_fastlio.close()
            self.get_logger().info(f'FAST-LIO2 trajectory saved ({self.fastlio_count} poses)')
        if self.file_fused:
            self.file_fused.close()
            self.get_logger().info(f'Fused odom trajectory saved ({self.fused_count} poses)')
        if self.file_odom:
            self.file_odom.close()
            self.get_logger().info(f'Raw leg odom trajectory saved ({self.odom_count} poses)')
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

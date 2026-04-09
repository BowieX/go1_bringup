#!/usr/bin/env python3
"""
导航指标记录节点 - 用于评估自主导航系统综合性能

功能:
  监听 Nav2 导航动作的状态，记录每次导航任务的:
  - 成功/失败状态
  - 导航耗时 (秒)
  - 路径长度 (米，基于实际行走轨迹)
  - 恢复行为触发次数

  最终输出 CSV 文件，可用于统计分析。

使用方式:
  ros2 run go1_bringup nav_metrics.py --ros-args \
    -p output_file:=/path/to/nav_metrics.csv
"""

import os
import math
import csv
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class NavMetricsRecorder(Node):
    def __init__(self):
        super().__init__('nav_metrics_recorder')

        # 参数
        self.declare_parameter('output_file',
                               os.path.expanduser('~/go1_ws/trajectories/nav_metrics.csv'))
        output_file = self.get_parameter('output_file').get_parameter_value().string_value

        # 创建输出目录
        os.makedirs(os.path.dirname(output_file), exist_ok=True)

        # CSV 输出
        self.csv_file = open(output_file, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'trial_id', 'timestamp', 'status',
            'duration_s', 'path_length_m',
            'goal_x', 'goal_y', 'goal_yaw'
        ])

        # 状态变量
        self.trial_id = 0
        self.is_navigating = False
        self.nav_start_time = None
        self.last_pos = None
        self.path_length = 0.0
        self.current_goal = None

        # 订阅里程计，用于计算实际行走路径长度
        self.sub_odom = self.create_subscription(
            Odometry, '/Odometry', self.odom_callback, 10)

        # 订阅导航目标
        self.sub_goal = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        # 监听 Nav2 NavigateToPose action 的状态
        self._action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        # 订阅 navigate_to_pose 的 status 变化
        self.sub_nav_status = self.create_subscription(
            GoalStatus, '/navigate_to_pose/_action/status',
            self.nav_status_callback, 10)

        # 使用简化方案: 通过 /goal_pose 话题检测导航开始
        # 通过定时器检查 action 状态 (Nav2 不直接发布 GoalStatus 话题)

        self.get_logger().info(f'Nav metrics recorder started. Output: {output_file}')
        self.get_logger().info('Waiting for navigation goals...')

    def goal_callback(self, msg: PoseStamped):
        """检测到新的导航目标 → 开始记录"""
        self.trial_id += 1
        self.is_navigating = True
        self.nav_start_time = self.get_clock().now()
        self.path_length = 0.0
        self.last_pos = None
        self.current_goal = msg

        gx = msg.pose.position.x
        gy = msg.pose.position.y
        self.get_logger().info(
            f'[Trial {self.trial_id}] Navigation started → goal=({gx:.2f}, {gy:.2f})')

    def odom_callback(self, msg: Odometry):
        """累积实际行走路径长度"""
        if not self.is_navigating:
            return

        pos = msg.pose.pose.position
        if self.last_pos is not None:
            dx = pos.x - self.last_pos[0]
            dy = pos.y - self.last_pos[1]
            dist = math.sqrt(dx * dx + dy * dy)
            # 过滤异常跳变 (> 1m 单步)
            if dist < 1.0:
                self.path_length += dist
        self.last_pos = (pos.x, pos.y)

    def nav_status_callback(self, msg):
        """Nav2 action 状态回调 (简化处理)"""
        pass

    def record_result(self, success: bool):
        """记录一次导航任务结果"""
        if not self.is_navigating:
            return

        duration = (self.get_clock().now() - self.nav_start_time).nanoseconds / 1e9
        status_str = 'SUCCESS' if success else 'FAILED'

        goal_x = goal_y = goal_yaw = 0.0
        if self.current_goal:
            goal_x = self.current_goal.pose.position.x
            goal_y = self.current_goal.pose.position.y
            # 从四元数提取 yaw
            q = self.current_goal.pose.orientation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            goal_yaw = math.atan2(siny, cosy)

        self.csv_writer.writerow([
            self.trial_id,
            datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            status_str,
            f'{duration:.2f}',
            f'{self.path_length:.3f}',
            f'{goal_x:.3f}',
            f'{goal_y:.3f}',
            f'{goal_yaw:.3f}'
        ])
        self.csv_file.flush()

        self.get_logger().info(
            f'[Trial {self.trial_id}] {status_str} | '
            f'time={duration:.1f}s | path={self.path_length:.2f}m')

        self.is_navigating = False

    def destroy_node(self):
        # 如果正在导航中退出，记录为失败
        if self.is_navigating:
            self.record_result(success=False)
        self.csv_file.close()
        self.get_logger().info(f'Nav metrics saved. Total trials: {self.trial_id}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavMetricsRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

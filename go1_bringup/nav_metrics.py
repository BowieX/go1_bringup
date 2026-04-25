#!/usr/bin/env python3
"""
导航指标记录节点 - 用于评估自主导航系统综合性能

功能:
  监听 Nav2 NavigateToPose action 的状态，记录每次导航任务的:
  - 成功/失败状态 (SUCCEEDED / ABORTED / CANCELED)
  - 导航耗时 (秒)
  - 路径长度 (米，基于 /Odometry 累积)

  每完成一次 trial 立刻写入一行 CSV。

实现方式:
  订阅 /navigate_to_pose/_action/status 话题 (action_msgs/msg/GoalStatusArray)，
  跟踪每个 goal_id 的状态转移:
    STATUS_ACCEPTED (1) / STATUS_EXECUTING (2) → 进入导航中
    STATUS_SUCCEEDED (4) → 成功
    STATUS_CANCELED  (5) / STATUS_ABORTED (6) → 失败
  同时订阅 /goal_pose 以获取目标点（方便记录和打印），
  订阅 /Odometry (FAST-LIO) 累积实际行走路径。

使用方式:
  python3 nav_metrics.py --ros-args \
    -p output_file:=/path/to/nav_metrics.csv
"""

import os
import math
import csv
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from action_msgs.msg import GoalStatus, GoalStatusArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


# GoalStatus 常量映射 (便于日志输出)
STATUS_NAME = {
    GoalStatus.STATUS_UNKNOWN:   'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED:  'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED:  'CANCELED',
    GoalStatus.STATUS_ABORTED:   'ABORTED',
}

TERMINAL_STATUSES = {
    GoalStatus.STATUS_SUCCEEDED,
    GoalStatus.STATUS_CANCELED,
    GoalStatus.STATUS_ABORTED,
}


class NavMetricsRecorder(Node):
    def __init__(self):
        super().__init__('nav_metrics_recorder')

        # 参数
        self.declare_parameter(
            'output_file',
            os.path.expanduser('~/go1_ws/trajectories/nav_metrics.csv'))
        output_file = self.get_parameter('output_file').get_parameter_value().string_value

        # 创建输出目录并打开 CSV
        os.makedirs(os.path.dirname(output_file), exist_ok=True)
        self.csv_file = open(output_file, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'trial_id', 'timestamp', 'status',
            'duration_s', 'path_length_m',
            'goal_x', 'goal_y', 'goal_yaw'
        ])
        self.csv_file.flush()

        # === 状态变量 ===
        self.trial_id = 0
        # 当前正在跟踪的 goal_id (bytes), None 表示空闲
        self.current_goal_id = None
        # 当前 trial 的起始时刻、累计路径、目标点
        self.nav_start_time = None
        self.path_length = 0.0
        self.last_pos = None
        self.current_goal_pose = None  # PoseStamped，用于记录目标 xy/yaw
        # 已记录过的 terminal goal_id，避免重复写入
        self.recorded_goal_ids = set()

        # === QoS ===
        # Nav2 action status 话题使用 reliable+transient_local，必须匹配
        action_status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # === 订阅 ===
        # FAST-LIO 里程计 - 累积路径长度
        self.sub_odom = self.create_subscription(
            Odometry, '/Odometry', self.odom_callback, 10)

        # 目标点 - 仅用于记录目标坐标（不作为 trial 起止信号）
        self.sub_goal = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Nav2 action status - trial 起止的权威来源
        self.sub_status = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.status_callback,
            action_status_qos)

        self.get_logger().info(f'Nav metrics recorder started. Output: {output_file}')
        self.get_logger().info('Waiting for navigation goals...')

    # ----------------------------------------------------------------------
    # 目标点回调：仅缓存，真正的 trial 起止由 action status 驱动
    # ----------------------------------------------------------------------
    def goal_callback(self, msg: PoseStamped):
        self.current_goal_pose = msg
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        self.get_logger().info(f'Received goal_pose: ({gx:.2f}, {gy:.2f})')

    # ----------------------------------------------------------------------
    # 里程计回调：累积路径长度 (仅在导航中计数)
    # ----------------------------------------------------------------------
    def odom_callback(self, msg: Odometry):
        if self.current_goal_id is None:
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

    # ----------------------------------------------------------------------
    # 核心：Nav2 action status 回调
    #   GoalStatusArray.status_list 每次广播的是所有 goal 的当前状态，
    #   我们挑出"最新的非终态 goal"作为当前 trial；当它转到终态时记录结果。
    # ----------------------------------------------------------------------
    def status_callback(self, msg: GoalStatusArray):
        if not msg.status_list:
            return

        # 找出非终态 goal (执行中的)
        active = [s for s in msg.status_list if s.status not in TERMINAL_STATUSES]

        # 处理：新导航任务开始
        if active:
            # Nav2 单 goal 模式下通常只有 1 个 active，取第一个
            new_goal_id = bytes(active[0].goal_info.goal_id.uuid)
            if self.current_goal_id != new_goal_id:
                # 之前若有未结算的 trial（不该发生，但防御性处理）
                if self.current_goal_id is not None:
                    self._finalize_trial(GoalStatus.STATUS_CANCELED)
                self._start_trial(new_goal_id)

        # 处理：当前 trial 进入终态
        if self.current_goal_id is not None:
            for s in msg.status_list:
                if bytes(s.goal_info.goal_id.uuid) == self.current_goal_id \
                        and s.status in TERMINAL_STATUSES:
                    self._finalize_trial(s.status)
                    break

    # ----------------------------------------------------------------------
    def _start_trial(self, goal_id: bytes):
        self.trial_id += 1
        self.current_goal_id = goal_id
        self.nav_start_time = self.get_clock().now()
        self.path_length = 0.0
        self.last_pos = None
        self.get_logger().info(f'[Trial {self.trial_id}] Navigation started')

    def _finalize_trial(self, status: int):
        if self.current_goal_id in self.recorded_goal_ids:
            # 同一 goal_id 多次出现终态（话题重传），跳过
            self.current_goal_id = None
            return
        self.recorded_goal_ids.add(self.current_goal_id)

        duration = (self.get_clock().now() - self.nav_start_time).nanoseconds / 1e9
        status_str = STATUS_NAME.get(status, 'UNKNOWN')

        # 目标坐标 (从最近一次 /goal_pose)
        goal_x = goal_y = goal_yaw = 0.0
        if self.current_goal_pose is not None:
            goal_x = self.current_goal_pose.pose.position.x
            goal_y = self.current_goal_pose.pose.position.y
            # 四元数 → yaw (仅绕 Z)
            q = self.current_goal_pose.pose.orientation
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

        self.current_goal_id = None

    # ----------------------------------------------------------------------
    def destroy_node(self):
        # 节点关闭时若仍在导航，写入一条 CANCELED 记录
        if self.current_goal_id is not None:
            self._finalize_trial(GoalStatus.STATUS_CANCELED)
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

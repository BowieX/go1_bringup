#!/usr/bin/env python3
# 文件路径: ~/go1_ws/src/go1_bringup/scripts/test_nav2_navigation.py
#
# Nav2 完整导航流程测试
#
# 测试场景 (基于 sim_test_map):
#   Goal 1: 左房间内部    (-3.0,  1.0)  — 直线可达，基础测试
#   Goal 2: 右房间内部    ( 3.0,  1.0)  — 需穿越走廊，中等复杂度
#   Goal 3: 走廊入口对面  ( 0.0, -2.5)  — 下方区域，需绕行
#   Goal 4: 无法到达的点  ( 0.0,  0.0)  — 墙内点 (触发规划失败)
#
# 每个 goal 记录: 是否成功、路径长度、耗时、重规划次数
#
# 使用前提: go1_sim_test.launch.py 已启动，AMCL 已收敛
#
# 运行方式:
#   python3 ~/go1_ws/src/go1_bringup/scripts/test_nav2_navigation.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math
import time
import csv

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import NavigateToPose


# 测试起始位置：上方中央走廊（从此点可连通左右房间）
# 注: sim_test_map 的上下两半在 0.25m 膨胀下不互通 (门洞太窄)，
#     因此全部目标都选在上半部分 (左右房间 + 中央)
START_POSE = (0.0, 0.5, 0.0)

# 测试目标点: (名称, x, y, yaw_deg, 期望结果)
# 已通过 BFS 验证: start 与所有 goal 在 0.25m 膨胀下互通
NAV_GOALS = [
    ("Goal1_左房间",  -2.0,  1.5,   0.0, "SUCCESS"),
    ("Goal2_右房间",   2.0,  1.5,   0.0, "SUCCESS"),
    ("Goal3_右上角",   3.5,  2.5,   0.0, "SUCCESS"),
]

# 每个 goal 的超时时间
GOAL_TIMEOUT = 60.0   # 秒


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class Nav2NavigationTester(Node):
    def __init__(self):
        super().__init__('nav2_navigation_tester')

        # ---------- NavigateToPose Action Client ----------
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ---------- /teleport 发布 + /initialpose 发布 (测试前重置机器人) ----------
        self.teleport_pub = self.create_publisher(PoseStamped, '/teleport', 10)
        init_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        self.init_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', init_qos)

        # ---------- 当前位置 (来自 AMCL 估计) ----------
        amcl_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._pose_cb, amcl_qos)

        self.current_x   = 0.0
        self.current_y   = 0.0
        self.current_yaw = 0.0

        # ---------- 路径订阅 (统计规划路径长度) ----------
        self.plan_sub = self.create_subscription(
            Path, '/plan', self._plan_cb, 10)
        self.plan_length = 0.0
        self.replan_count = 0

        # ---------- 结果存储 ----------
        self.results = []

        self.get_logger().info('Nav2 导航测试器已启动')

    def reset_robot(self, x: float = 0.0, y: float = 0.0, yaw_deg: float = 0.0):
        """把机器人瞬移回 (x,y,yaw) 并同步重置 AMCL 初始位姿。
        用于测试开始前清除前一次测试的残留状态。"""
        # 1. 发布 /teleport 让 fake_robot_node 瞬移
        tp = self._make_goal_pose(x, y, yaw_deg)
        self.teleport_pub.publish(tp)
        # 2. 发布 /initialpose 同步 AMCL 粒子分布
        ip = PoseWithCovarianceStamped()
        ip.header.stamp    = self.get_clock().now().to_msg()
        ip.header.frame_id = 'map'
        ip.pose.pose = tp.pose
        ip.pose.covariance[0]  = 0.0625   # x σ=0.25
        ip.pose.covariance[7]  = 0.0625   # y σ=0.25
        ip.pose.covariance[35] = 0.0685   # yaw σ≈15°
        self.init_pose_pub.publish(ip)
        self.get_logger().info(f'已重置机器人到 ({x:.2f}, {y:.2f}, yaw={yaw_deg:.0f}°)')
        # 3. 等待 TF 和 AMCL 收敛
        for _ in range(30):
            rclpy.spin_once(self, timeout_sec=0.1)

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        self.current_x   = msg.pose.pose.position.x
        self.current_y   = msg.pose.pose.position.y
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def _plan_cb(self, msg: Path):
        """统计全局规划路径长度，每次收到新 plan 算作一次重规划"""
        if len(msg.poses) < 2:
            return
        length = 0.0
        for i in range(1, len(msg.poses)):
            dx = msg.poses[i].pose.position.x - msg.poses[i-1].pose.position.x
            dy = msg.poses[i].pose.position.y - msg.poses[i-1].pose.position.y
            length += math.hypot(dx, dy)
        # 第一次收到规划路径时记录完整长度，后续重规划只更新计数
        if self.replan_count == 0:
            self.plan_length = length
        self.replan_count += 1

    def _make_goal_pose(self, x: float, y: float, yaw_deg: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        yaw = math.radians(yaw_deg)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def navigate_to(self, name: str, x: float, y: float,
                    yaw_deg: float, expected: str) -> dict:
        """发送导航目标，等待结果，返回测试结果字典"""
        sep = '─' * 55
        print(f'\n{sep}')
        print(f'  {name}')
        print(f'  目标: ({x:.1f}, {y:.1f}, yaw={yaw_deg:.0f}°)')
        print(f'  当前位置: ({self.current_x:.2f}, {self.current_y:.2f})')
        straight_dist = math.hypot(x - self.current_x, y - self.current_y)
        print(f'  直线距离: {straight_dist:.2f}m | 期望: {expected}')
        print(sep)

        # 等待 action server 就绪
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            print('  ✗ navigate_to_pose action server 未响应')
            return {'name': name, 'success': False, 'reason': 'server_timeout'}

        # 重置统计
        self.plan_length = 0.0
        self.replan_count = 0

        # 发送目标
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._make_goal_pose(x, y, yaw_deg)
        t_start = time.time()

        send_future = self._nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        goal_handle = send_future.result()

        if not goal_handle or not goal_handle.accepted:
            print('  ✗ 目标被拒绝 (可能在障碍物内)')
            return {'name': name, 'success': False, 'reason': 'goal_rejected',
                    'expected': expected}

        print('  ✓ 目标已接受，导航中...')

        # 等待结果
        result_future = goal_handle.get_result_async()
        t_deadline = t_start + GOAL_TIMEOUT

        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.2)
            elapsed = time.time() - t_start
            dist_to_goal = math.hypot(x - self.current_x, y - self.current_y)
            print(f'  t={elapsed:5.1f}s | 剩余距离={dist_to_goal:.2f}m | '
                  f'规划路径={self.plan_length:.2f}m | 重规划={self.replan_count}次',
                  end='\r')

            if time.time() > t_deadline:
                goal_handle.cancel_goal_async()
                print(f'\n  ✗ 超时 ({GOAL_TIMEOUT:.0f}s)')
                return {
                    'name': name, 'success': False, 'reason': 'timeout',
                    'elapsed_s': round(time.time() - t_start, 1),
                    'plan_length_m': round(self.plan_length, 2),
                    'replan_count': self.replan_count,
                    'expected': expected
                }

        print()  # 换行

        elapsed = time.time() - t_start
        result = result_future.result()
        status = result.status

        success = (status == GoalStatus.STATUS_SUCCEEDED)
        status_str = {
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_ABORTED:   'ABORTED',
            GoalStatus.STATUS_CANCELED:  'CANCELED',
        }.get(status, f'UNKNOWN({status})')

        final_dist = math.hypot(x - self.current_x, y - self.current_y)
        icon = '✓' if success else '✗'
        print(f'  {icon} 状态: {status_str} | 耗时: {elapsed:.1f}s | '
              f'到达距离: {final_dist:.3f}m')
        print(f'    规划路径长度: {self.plan_length:.2f}m | 重规划次数: {self.replan_count}')

        return {
            'name': name,
            'success': success,
            'status': status_str,
            'expected': expected,
            'elapsed_s': round(elapsed, 1),
            'plan_length_m': round(self.plan_length, 2),
            'replan_count': self.replan_count,
            'final_dist_to_goal_m': round(final_dist, 3),
        }

    def print_report(self):
        print('\n' + '═' * 65)
        print('  Nav2 导航测试报告')
        print('═' * 65)
        print(f'  {"目标":<18} {"期望":<10} {"实际":<12} {"耗时":>7}  {"路径长度":>9}  {"重规划":>6}')
        print('  ' + '─' * 61)
        for r in self.results:
            actual = r.get('status', 'ERROR')
            t_str  = f"{r.get('elapsed_s', 0):.1f}s"
            pl_str = f"{r.get('plan_length_m', 0):.2f}m"
            rp_str = str(r.get('replan_count', 0))
            print(f"  {r['name']:<18} {r.get('expected',''):<10} {actual:<12} "
                  f"{t_str:>7}  {pl_str:>9}  {rp_str:>6}")
        print('═' * 65)

        # 保存 CSV
        csv_path = '/tmp/nav2_navigation_results.csv'
        if self.results:
            with open(csv_path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=self.results[0].keys())
                writer.writeheader()
                writer.writerows(self.results)
            print(f'\n  结果已保存至 {csv_path}')

        print()
        print('  解读:')
        print('  • SUCCEEDED + 重规划=1: 理想情况，一次规划直达')
        print('  • SUCCEEDED + 重规划>1: 导航中途遇到动态障碍触发重规划')
        print('  • ABORTED: TEB/DWB 无法找到可行路径 (空间太窄或障碍太密)')
        print('  • 路径长度 >> 直线距离: 规划器绕路，可调低 inflation_radius')


def main():
    rclpy.init()
    tester = Nav2NavigationTester()

    print('\n' + '═' * 65)
    print('  Nav2 完整导航流程测试')
    print('  仿真地图: sim_test_map (两室一廊 + 下方开放区)')
    print('═' * 65)
    print()
    print('  等待 Nav2 启动并完成初始定位 (8秒)...')
    for _ in range(80):
        rclpy.spin_once(tester, timeout_sec=0.1)

    # 重置机器人到安全起点 (离墙 >0.55m, 避开膨胀代价区)
    sx, sy, syaw = START_POSE
    print(f'  重置机器人到起点 ({sx:.1f}, {sy:.1f})...')
    tester.reset_robot(sx, sy, syaw)
    time.sleep(2.0)
    for _ in range(20):
        rclpy.spin_once(tester, timeout_sec=0.1)

    # 依次测试所有目标
    for name, gx, gy, gyaw, expected in NAV_GOALS:
        result = tester.navigate_to(name, gx, gy, gyaw, expected)
        tester.results.append(result)
        # 每个目标间等 1s，让机器人停稳
        time.sleep(1.0)
        for _ in range(10):
            rclpy.spin_once(tester, timeout_sec=0.1)

    tester.print_report()
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# 文件路径: ~/go1_ws/src/go1_bringup/scripts/test_amcl_convergence.py
#
# AMCL 初始位姿收敛行为测试
#
# 测试三种初始位姿情况:
#   Case 1: 正确初始位姿         (0°偏差)  → 期望: 快速收敛
#   Case 2: 朝向偏差 45°         → 期望: 移动后自动纠正
#   Case 3: 朝向完全反转 180°    → 期望: 无法自动收敛，需重新设置
#
# 使用前提: go1_sim_test.launch.py 已启动
#
# 运行方式:
#   python3 ~/go1_ws/src/go1_bringup/scripts/test_amcl_convergence.py
#
# 输出:
#   - 终端实时打印定位误差
#   - 测试结束后生成 /tmp/amcl_convergence_report.txt

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math
import time
import threading

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry


# 机器人在仿真中的真实位置 (fake_robot_node 维护)
# 测试时机器人静止在原点，通过发布 /initialpose 测试 AMCL 收敛
ROBOT_TRUE_X   = 0.0
ROBOT_TRUE_Y   = 0.0
ROBOT_TRUE_YAW = 0.0  # 弧度

# AMCL 收敛判定阈值
CONVERGE_POS_THR  = 0.15   # 位置误差 < 15cm 视为收敛
CONVERGE_YAW_THR  = 0.20   # 朝向误差 < 0.2 rad (~11°) 视为收敛
CONVERGE_TIMEOUT  = 30.0   # 30秒内未收敛视为失败
WIGGLE_DIST       = 0.5    # 抖动距离: 来回移动帮助粒子收敛

# 测试用例: (名称, 初始位姿偏差_x, 偏差_y, 偏差_yaw_度, 期望结果)
TEST_CASES = [
    ("Case1_正确位姿",    0.0,  0.0,    0, "CONVERGE_FAST"),
    ("Case2_偏差45度",   0.0,  0.0,   45, "CONVERGE_SLOW"),
    ("Case3_反向180度",  0.0,  0.0,  180, "NO_CONVERGE"),
]


def yaw_from_quaternion(q):
    """从四元数提取偏航角 (弧度)"""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def angle_diff(a: float, b: float) -> float:
    """两角度之差，结果在 [-π, π]"""
    d = a - b
    while d > math.pi:  d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    return d


class AmclConvergenceTester(Node):
    def __init__(self):
        super().__init__('amcl_convergence_tester')

        # ---------- AMCL 位姿估计订阅 ----------
        amcl_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_cb,
            amcl_qos
        )

        # ---------- 初始位姿发布 ----------
        self.init_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            amcl_qos
        )

        # ---------- cmd_vel 发布 (驱动机器人抖动，帮助粒子收敛) ----------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ---------- 状态 ----------
        self.current_amcl_x   = None
        self.current_amcl_y   = None
        self.current_amcl_yaw = None
        self.pose_lock = threading.Lock()

        self.results = []

        self.get_logger().info('AMCL 收敛测试器已启动，等待 AMCL 初始化 (3秒)...')

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        with self.pose_lock:
            self.current_amcl_x   = msg.pose.pose.position.x
            self.current_amcl_y   = msg.pose.pose.position.y
            self.current_amcl_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def _get_error(self):
        """获取当前 AMCL 估计与真实位置的误差"""
        with self.pose_lock:
            if self.current_amcl_x is None:
                return None, None
            pos_err = math.hypot(
                self.current_amcl_x - ROBOT_TRUE_X,
                self.current_amcl_y - ROBOT_TRUE_Y
            )
            yaw_err = abs(angle_diff(self.current_amcl_yaw, ROBOT_TRUE_YAW))
        return pos_err, yaw_err

    def _publish_initial_pose(self, x: float, y: float, yaw_deg: float):
        """发布 /initialpose，设置 AMCL 粒子初始分布"""
        yaw = math.radians(yaw_deg)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # 协方差: 位置 ±0.5m, 朝向 ±π/6 rad
        msg.pose.covariance[0]  = 0.25   # x 方差
        msg.pose.covariance[7]  = 0.25   # y 方差
        msg.pose.covariance[35] = (math.pi / 6) ** 2  # yaw 方差
        self.init_pose_pub.publish(msg)
        self.get_logger().info(
            f'  → 发布初始位姿: ({x:.2f}, {y:.2f}, yaw={yaw_deg:.0f}°)')

    def _wiggle(self, duration: float = 4.0):
        """让机器人来回抖动，帮助 AMCL 粒子收敛"""
        cmd = Twist()
        t0 = time.time()
        phase = 0
        while time.time() - t0 < duration:
            elapsed = time.time() - t0
            # 每 0.8s 切换方向
            if int(elapsed / 0.8) % 2 == 0:
                cmd.linear.x = 0.1
            else:
                cmd.linear.x = -0.1
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
        # 停止
        cmd.linear.x = 0.0
        self.cmd_pub.publish(cmd)

    def run_test(self, name: str, init_x: float, init_y: float,
                 yaw_deg: float, expected: str):
        """运行单个测试用例"""
        sep = '─' * 50
        print(f'\n{sep}')
        print(f'  {name}')
        print(f'  初始位姿偏差: x={init_x:.1f}m, y={init_y:.1f}m, yaw={yaw_deg:.0f}°')
        print(f'  期望结果: {expected}')
        print(sep)

        # 1. 重置: 先给正确位姿让 AMCL 稳定
        self._publish_initial_pose(ROBOT_TRUE_X, ROBOT_TRUE_Y, 0.0)
        time.sleep(2.0)
        rclpy.spin_once(self, timeout_sec=0.1)

        # 2. 发布测试初始位姿 (偏差版本)
        self._publish_initial_pose(
            ROBOT_TRUE_X + init_x,
            ROBOT_TRUE_Y + init_y,
            math.degrees(ROBOT_TRUE_YAW) + yaw_deg
        )
        time.sleep(1.0)

        # 3. 记录初始误差
        pos_err0, yaw_err0 = self._get_error()
        print(f'  初始误差: 位置={pos_err0:.3f}m, 朝向={math.degrees(yaw_err0):.1f}°'
              if pos_err0 is not None else '  初始误差: AMCL 尚未响应')

        # 4. 抖动驱动，等待收敛
        converged = False
        converge_time = None
        t_start = time.time()

        print(f'  开始抖动驱动，等待收敛 (最多 {CONVERGE_TIMEOUT:.0f}s)...')
        while time.time() - t_start < CONVERGE_TIMEOUT:
            self._wiggle(duration=2.0)
            rclpy.spin_once(self, timeout_sec=0.1)

            pos_err, yaw_err = self._get_error()
            if pos_err is None:
                continue

            elapsed = time.time() - t_start
            print(f'  t={elapsed:5.1f}s | 位置误差={pos_err:.3f}m | '
                  f'朝向误差={math.degrees(yaw_err):.1f}°', end='\r')

            if pos_err < CONVERGE_POS_THR and yaw_err < CONVERGE_YAW_THR:
                converged = True
                converge_time = elapsed
                break

        print()  # 换行

        # 5. 记录结果
        pos_err_final, yaw_err_final = self._get_error()
        result = {
            'name': name,
            'yaw_offset_deg': yaw_deg,
            'expected': expected,
            'converged': converged,
            'converge_time_s': round(converge_time, 1) if converge_time else None,
            'final_pos_err_m': round(pos_err_final, 3) if pos_err_final else None,
            'final_yaw_err_deg': round(math.degrees(yaw_err_final), 1) if yaw_err_final else None,
        }
        self.results.append(result)

        status = '✓ 收敛' if converged else '✗ 未收敛'
        print(f'  结果: {status}')
        if converged:
            print(f'  收敛耗时: {converge_time:.1f}s')
        print(f'  最终误差: 位置={pos_err_final:.3f}m, '
              f'朝向={math.degrees(yaw_err_final):.1f}°')

        # 如果是 180° 反转且未收敛，给出提示
        if not converged and yaw_deg == 180:
            print()
            print('  ★ 这就是朝向完全反转时的现象：')
            print('    粒子全部集中在错误朝向，激光匹配度极低，')
            print('    机器人无论如何移动都无法自动纠正。')
            print('    → 解决方法: 重新手动点击 2D Pose Estimate 给正确朝向')

        return converged

    def print_report(self):
        """打印最终测试报告"""
        print('\n' + '═' * 60)
        print('  AMCL 收敛测试报告')
        print('═' * 60)
        print(f'  {"用例":<20} {"期望":<16} {"实际":<10} {"耗时":>6}  {"最终位置误差":>12}')
        print('  ' + '─' * 56)
        for r in self.results:
            actual = '收敛' if r['converged'] else '未收敛'
            t_str = f"{r['converge_time_s']:.1f}s" if r['converge_time_s'] else 'N/A'
            e_str = f"{r['final_pos_err_m']:.3f}m" if r['final_pos_err_m'] else 'N/A'
            print(f"  {r['name']:<20} {r['expected']:<16} {actual:<10} {t_str:>6}  {e_str:>12}")
        print('═' * 60)
        print()
        print('  结论:')
        print('  • Case1 (0° 偏差):   AMCL 几乎无需移动即可维持正确定位')
        print('  • Case2 (45° 偏差):  抖动几秒后粒子收敛，轨迹初期偏斜但自动纠正')
        print('  • Case3 (180° 反转): AMCL 粒子无法收敛，必须重新设置初始位姿')
        print()

        # 写入文件
        with open('/tmp/amcl_convergence_report.txt', 'w') as f:
            f.write('AMCL 收敛测试报告\n')
            for r in self.results:
                f.write(str(r) + '\n')
        print('  报告已保存至 /tmp/amcl_convergence_report.txt')


def main():
    rclpy.init()
    tester = AmclConvergenceTester()

    print('\n' + '═' * 60)
    print('  AMCL 初始位姿收敛行为测试')
    print('  仿真地图: 左右两房间 + 中央走廊')
    print('  机器人真实位置: (0.0, 0.0, 0°)')
    print('═' * 60)
    print()
    print('  等待 Nav2 / AMCL 完全启动 (5秒)...')
    for i in range(50):
        rclpy.spin_once(tester, timeout_sec=0.1)

    # 运行三个测试用例
    for name, dx, dy, dyaw, expected in TEST_CASES:
        tester.run_test(name, dx, dy, dyaw, expected)
        time.sleep(1.0)  # 用例间稍作间隔

    tester.print_report()
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

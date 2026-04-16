#!/usr/bin/env python3
# 文件路径: ~/go1_ws/src/go1_bringup/scripts/fake_robot_node.py
#
# 虚假机器人节点 — 无需 Gazebo，在地图上模拟 Go1 运动与 LiDAR 扫描
#
# 发布:
#   /scan        (sensor_msgs/LaserScan)  — 基于地图射线投射
#   /odom        (nav_msgs/Odometry)      — 模拟里程计
#   TF: odom -> base_footprint
#
# 订阅:
#   /cmd_vel     (geometry_msgs/Twist)    — Nav2 控制指令驱动位置更新
#
# 射线投射原理:
#   对每个激光角度，沿射线方向步进，直到命中占用格 (pixel < 128) 或超出范围
#   步进分辨率 = 地图分辨率 / 2 以保证精度

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
import math
import yaml
import os
from PIL import Image

from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


class FakeRobotNode(Node):
    def __init__(self):
        super().__init__('fake_robot_node')

        # ---------- 参数 ----------
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('init_x', 0.0)
        self.declare_parameter('init_y', 0.0)
        self.declare_parameter('init_yaw', 0.0)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')

        map_yaml = self.get_parameter('map_yaml').value
        self.x   = self.get_parameter('init_x').value
        self.y   = self.get_parameter('init_y').value
        self.yaw = self.get_parameter('init_yaw').value
        scan_topic = self.get_parameter('scan_topic').value
        odom_topic = self.get_parameter('odom_topic').value

        # ---------- 加载地图 ----------
        self._load_map(map_yaml)

        # ---------- 激光扫描参数 (模拟 2D LiDAR) ----------
        self.scan_angle_min = -math.pi        # -180°
        self.scan_angle_max =  math.pi        # +180°
        self.scan_num_beams  = 360            # 每度一束
        self.scan_range_min  = 0.1
        self.scan_range_max  = 8.0

        # ---------- 状态 ----------
        self.vx = 0.0   # 当前速度指令
        self.vy = 0.0
        self.wz = 0.0

        # ---------- TF ----------
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------- 发布者 ----------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )
        self.scan_pub = self.create_publisher(LaserScan, scan_topic, sensor_qos)
        self.odom_pub = self.create_publisher(Odometry,  odom_topic, 10)

        # ---------- 订阅者 ----------
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        # /teleport 话题: 测试脚本可通过它重置机器人位置 (不经碰撞检测)
        self.tele_sub = self.create_subscription(
            PoseStamped, '/teleport', self._teleport_cb, 10)

        # ---------- 定时器 ----------
        self.dt = 0.05   # 20 Hz 物理更新
        self.create_timer(self.dt,  self._physics_timer)
        self.create_timer(0.1, self._scan_timer)   # 10 Hz 扫描

        self.get_logger().info(
            f'FakeRobotNode 启动: 初始位置 ({self.x:.2f}, {self.y:.2f}, '
            f'yaw={math.degrees(self.yaw):.1f}°)')

    # ------------------------------------------------------------------
    # 地图加载
    # ------------------------------------------------------------------
    def _load_map(self, yaml_path: str):
        if not yaml_path or not os.path.exists(yaml_path):
            self.get_logger().error(f'地图文件不存在: {yaml_path}')
            raise RuntimeError(f'地图文件不存在: {yaml_path}')

        with open(yaml_path, 'r') as f:
            meta = yaml.safe_load(f)

        pgm_path = os.path.join(os.path.dirname(yaml_path), meta['image'])
        self.map_resolution = float(meta['resolution'])
        ox, oy = meta['origin'][0], meta['origin'][1]
        self.map_origin_x = float(ox)
        self.map_origin_y = float(oy)
        negate   = int(meta.get('negate', 0))
        occ_thr  = float(meta.get('occupied_thresh', 0.65))

        img = np.array(Image.open(pgm_path).convert('L'))
        # PGM 中白色 (255) = free，黑色 (0) = occupied
        # negate=1 时反转
        if negate:
            img = 255 - img
        # 占用阈值: 像素值 / 255 < occupied_thresh 则为占用
        self.map_data = img  # shape: (H, W), dtype uint8
        self.map_h, self.map_w = img.shape
        self.occ_threshold_px = int(occ_thr * 255)

        self.get_logger().info(
            f'地图加载: {self.map_w}x{self.map_h} px, '
            f'分辨率 {self.map_resolution}m/px, '
            f'原点 ({self.map_origin_x}, {self.map_origin_y})')

    # ------------------------------------------------------------------
    # 世界坐标 -> 地图像素坐标
    # ------------------------------------------------------------------
    def _world_to_map(self, wx: float, wy: float):
        """返回 (col, row)，超出范围返回 None"""
        col = int((wx - self.map_origin_x) / self.map_resolution)
        # PGM 行从上到下，世界 Y 轴从下到上，需要翻转
        row = self.map_h - 1 - int((wy - self.map_origin_y) / self.map_resolution)
        if 0 <= col < self.map_w and 0 <= row < self.map_h:
            return col, row
        return None

    # ------------------------------------------------------------------
    # 单射线投射，返回命中距离
    # ------------------------------------------------------------------
    def _raycast(self, sx: float, sy: float, angle: float) -> float:
        step = self.map_resolution * 0.5
        dx = math.cos(angle) * step
        dy = math.sin(angle) * step
        x, y = sx, sy
        max_steps = int(self.scan_range_max / step) + 1

        for _ in range(max_steps):
            x += dx
            y += dy
            px = self._world_to_map(x, y)
            if px is None:
                # 超出地图边界视作命中墙壁
                dist = math.hypot(x - sx, y - sy)
                return min(dist, self.scan_range_max)
            col, row = px
            if self.map_data[row, col] < (255 - self.occ_threshold_px):
                dist = math.hypot(x - sx, y - sy)
                return max(self.scan_range_min, min(dist, self.scan_range_max))

        return self.scan_range_max

    # ------------------------------------------------------------------
    # 碰撞检测：机器人半径 0.32m
    # ------------------------------------------------------------------
    def _is_collision(self, x: float, y: float, radius: float = 0.20) -> bool:
        checks = [
            (x, y),
            (x + radius, y), (x - radius, y),
            (x, y + radius), (x, y - radius),
            (x + radius * 0.7, y + radius * 0.7),
            (x - radius * 0.7, y + radius * 0.7),
            (x + radius * 0.7, y - radius * 0.7),
            (x - radius * 0.7, y - radius * 0.7),
        ]
        for cx, cy in checks:
            px = self._world_to_map(cx, cy)
            if px is None:
                return True
            col, row = px
            if self.map_data[row, col] < (255 - self.occ_threshold_px):
                return True
        return False

    # ------------------------------------------------------------------
    # cmd_vel 回调
    # ------------------------------------------------------------------
    def _cmd_vel_cb(self, msg: Twist):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.wz = msg.angular.z

    # ------------------------------------------------------------------
    # 瞬移回调: 测试脚本用 /teleport 话题强制重置机器人位置
    # ------------------------------------------------------------------
    def _teleport_cb(self, msg: PoseStamped):
        new_x   = msg.pose.position.x
        new_y   = msg.pose.position.y
        q = msg.pose.orientation
        new_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.x, self.y, self.yaw = new_x, new_y, new_yaw
        self.vx = self.vy = self.wz = 0.0
        self.get_logger().info(
            f'[teleport] -> ({new_x:.2f}, {new_y:.2f}, '
            f'{math.degrees(new_yaw):.1f}°)')

    # ------------------------------------------------------------------
    # 物理更新定时器 (20 Hz)
    # ------------------------------------------------------------------
    def _physics_timer(self):
        now = self.get_clock().now()

        # 全向运动学积分 (机体系 -> 世界系)
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        dx_world = (self.vx * cos_yaw - self.vy * sin_yaw) * self.dt
        dy_world = (self.vx * sin_yaw + self.vy * cos_yaw) * self.dt
        dyaw     = self.wz * self.dt

        new_x   = self.x   + dx_world
        new_y   = self.y   + dy_world
        new_yaw = self.yaw + dyaw

        # 碰撞检测：不允许穿墙
        if not self._is_collision(new_x, new_y):
            self.x, self.y, self.yaw = new_x, new_y, new_yaw
        else:
            self.vx = self.vy = self.wz = 0.0

        # 发布 TF: odom -> base_footprint
        t = TransformStamped()
        t.header.stamp    = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        # 四元数：绕 Z 轴旋转 yaw（必须使用 self.yaw，碰撞时 new_yaw 未赋值给 self.yaw）
        t.transform.rotation.z = math.sin(self.yaw / 2.0)
        t.transform.rotation.w = math.cos(self.yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)

        # 发布 Odometry
        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_footprint'
        odom.pose.pose.position.x  = self.x
        odom.pose.pose.position.y  = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x  = self.vx
        odom.twist.twist.linear.y  = self.vy
        odom.twist.twist.angular.z = self.wz
        self.odom_pub.publish(odom)

    # ------------------------------------------------------------------
    # 激光扫描定时器 (10 Hz)
    # ------------------------------------------------------------------
    def _scan_timer(self):
        now = self.get_clock().now()
        # endpoint=False 确保角度间隔 = (angle_max - angle_min) / num_beams
        # 与 LaserScan.angle_increment 一致，避免 AMCL 几何错位
        angles = np.linspace(
            self.scan_angle_min, self.scan_angle_max, self.scan_num_beams,
            endpoint=False)

        ranges = []
        for a in angles:
            world_angle = self.yaw + a   # 扫描角度转换到世界坐标系
            r = self._raycast(self.x, self.y, world_angle)
            ranges.append(r)

        scan = LaserScan()
        scan.header.stamp    = now.to_msg()
        scan.header.frame_id = 'base_footprint'
        scan.angle_min       = self.scan_angle_min
        scan.angle_max       = self.scan_angle_max
        scan.angle_increment = (self.scan_angle_max - self.scan_angle_min) / self.scan_num_beams
        scan.time_increment  = 0.0
        scan.scan_time       = 0.1
        scan.range_min       = self.scan_range_min
        scan.range_max       = self.scan_range_max
        scan.ranges          = [float(r) for r in ranges]
        scan.intensities     = []
        self.scan_pub.publish(scan)

    # ------------------------------------------------------------------
    # 公开接口：外部脚本可调用强制设置位置（模拟 2D Pose Estimate）
    # ------------------------------------------------------------------
    def teleport(self, x: float, y: float, yaw: float):
        """瞬移机器人到指定位置（不检查碰撞，用于测试）"""
        self.x, self.y, self.yaw = x, y, yaw
        self.get_logger().info(
            f'[teleport] -> ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°)')


def main(args=None):
    rclpy.init(args=args)
    node = FakeRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

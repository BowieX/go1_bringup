#!/usr/bin/env python3
"""诊断脚本: 监控 AMCL 的 TF、scan、particlecloud 状态"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.msg import ParticleCloud
import time

class AmclDiagnostics(Node):
    def __init__(self):
        super().__init__('amcl_diagnostics')
        
        self.scan_count = 0
        self.particle_count = 0
        self.amcl_pose_count = 0
        self.tf_ok_count = 0
        self.tf_fail_count = 0
        self.start_time = time.time()
        self.last_report = time.time()
        
        # 订阅 /scan (BEST_EFFORT)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, depth=5)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, sensor_qos)
        
        # 订阅 /particlecloud (BEST_EFFORT)
        self.create_subscription(ParticleCloud, '/particlecloud', self._particle_cb, sensor_qos)
        
        # 订阅 /amcl_pose (RELIABLE + TRANSIENT_LOCAL)
        amcl_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl_pose_cb, amcl_qos)
        
        # TF buffer
        from tf2_ros import Buffer, TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 定时报告 (每2秒)
        self.create_timer(2.0, self._report)
        self.get_logger().info('AMCL 诊断工具启动')
    
    def _scan_cb(self, msg):
        self.scan_count += 1
        # 每收到一条 scan, 检查 TF
        try:
            # 检查 odom -> base_footprint
            tf = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
            self.tf_ok_count += 1
        except Exception as e:
            self.tf_fail_count += 1
            if self.tf_fail_count <= 3:
                self.get_logger().warn(f'TF odom->base_footprint 失败: {e}')
    
    def _particle_cb(self, msg):
        self.particle_count += 1
        n = len(msg.particles)
        self.get_logger().info(f'收到 particlecloud! 粒子数={n}')
    
    def _amcl_pose_cb(self, msg):
        self.amcl_pose_count += 1
        p = msg.pose.pose
        self.get_logger().info(
            f'收到 amcl_pose: ({p.position.x:.3f}, {p.position.y:.3f})')
    
    def _report(self):
        elapsed = time.time() - self.start_time
        
        # 检查 map -> odom TF
        map_odom_ok = False
        try:
            tf = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            map_odom_ok = True
            x = tf.transform.translation.x
            y = tf.transform.translation.y
        except Exception:
            pass
        
        # 检查所有帧
        frames = self.tf_buffer.all_frames_as_string()
        frame_list = [line.split('Frame')[1].split(' ')[0] if 'Frame' in line else '' 
                      for line in frames.split('\n') if 'Frame' in line]
        
        self.get_logger().info(
            f'[{elapsed:.0f}s] scan={self.scan_count} | '
            f'particlecloud={self.particle_count} | '
            f'amcl_pose={self.amcl_pose_count} | '
            f'TF_ok={self.tf_ok_count} TF_fail={self.tf_fail_count} | '
            f'map→odom={"✅" if map_odom_ok else "❌"} | '
            f'frames={len(frame_list)}')
        
        if not map_odom_ok and elapsed > 10:
            self.get_logger().error(
                f'map→odom TF 不存在! 可用帧: {frames[:500]}')

def main():
    rclpy.init()
    node = AmclDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
已知几何距离误差测量工具.

用法:
  python3 measure_geometry.py <traj_file> <constraints_csv> [--output results.csv]

功能:
  读取 TUM 格式轨迹文件和已知几何约束 CSV，对每组约束:
  1. 在轨迹中找到距离起点/终点坐标最近的点
  2. 计算这两个轨迹点之间的距离
  3. 与卷尺实测距离对比，输出绝对误差和百分比误差

geometry_constraints.csv 格式 (第一行可选为注释):
  # 标签, 实测距离(m), 起点x, 起点y, 终点x, 终点y
  走廊长度,30.5,0.0,0.0,30.2,1.1
  房间宽度,6.0,0.0,0.0,0.0,5.9

  起点/终点是你在实验场地中标记的位置的 **大致坐标**（从 SLAM 地图或
  evo_traj 俯视图中读取），脚本会自动在轨迹中找距离这些坐标最近的点。

  如何获取坐标:
  1. 跑完 SLAM 后用 evo_traj 画俯视图，读出关键点的大致 (x, y)
  2. 或者在 RViz 中将鼠标悬停在轨迹上查看坐标
  3. 用卷尺量对应的实际距离

适用场景:
  走廊长度、房间对角线、两柱间距、门口宽度等任何可以用卷尺量的距离。
  建议至少准备 3-5 组约束，覆盖正常区域和退化区域。
"""

import argparse
import csv
import math
import sys


def load_trajectory(filepath):
    """读取 TUM 格式轨迹，返回 [(timestamp, x, y, z), ...]."""
    poses = []
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) >= 4:
                poses.append((
                    float(parts[0]),
                    float(parts[1]),
                    float(parts[2]),
                    float(parts[3]),
                ))
    return poses


def load_constraints(filepath):
    """读取几何约束 CSV，返回 [(name, measured_dist, sx, sy, ex, ey), ...]."""
    constraints = []
    with open(filepath) as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) < 6:
                continue
            name = row[0].strip()
            if name.startswith('#'):
                continue
            constraints.append((
                name,
                float(row[1]),
                float(row[2]), float(row[3]),
                float(row[4]), float(row[5]),
            ))
    return constraints


def find_nearest_pose(poses, target_x, target_y):
    """在轨迹中找距离 (target_x, target_y) 最近的点."""
    best_dist = float('inf')
    best_pose = None
    best_idx = 0
    for i, (t, x, y, z) in enumerate(poses):
        d = math.sqrt((x - target_x)**2 + (y - target_y)**2)
        if d < best_dist:
            best_dist = d
            best_pose = (t, x, y, z)
            best_idx = i
    return best_pose, best_dist, best_idx


def evaluate(traj_file, constraints_file, output_file=None):
    poses = load_trajectory(traj_file)
    if not poses:
        print(f"错误: 轨迹文件为空或格式错误: {traj_file}", file=sys.stderr)
        return []

    constraints = load_constraints(constraints_file)
    if not constraints:
        print(f"错误: 约束文件为空或格式错误: {constraints_file}", file=sys.stderr)
        return []

    results = []
    print(f"\n轨迹: {traj_file} ({len(poses)} 个位姿点)")
    print(f"约束: {constraints_file} ({len(constraints)} 组)\n")
    print(f"{'标签':<16s} {'实测(m)':>8s} {'SLAM(m)':>8s} {'误差(m)':>8s} {'误差%':>7s} {'匹配偏差':>10s}")
    print("-" * 65)

    for name, measured, sx, sy, ex, ey in constraints:
        p1, d1, _ = find_nearest_pose(poses, sx, sy)
        p2, d2, _ = find_nearest_pose(poses, ex, ey)

        slam_dist = math.sqrt((p2[1] - p1[1])**2 + (p2[2] - p1[2])**2)
        error = abs(slam_dist - measured)
        pct = (error / measured * 100) if measured > 0 else 0

        # 匹配偏差: 指定坐标与实际匹配点之间的距离，偏差过大说明坐标估计不准
        match_dev = max(d1, d2)
        warn = " !" if match_dev > 1.0 else ""

        print(
            f"{name:<16s} {measured:>8.2f} {slam_dist:>8.2f} "
            f"{error:>8.3f} {pct:>6.1f}% {match_dev:>8.2f}m{warn}"
        )

        results.append({
            'name': name,
            'measured': measured,
            'slam_dist': slam_dist,
            'error': error,
            'error_pct': pct,
            'match_dev': match_dev,
        })

    if results:
        errors = [r['error'] for r in results]
        pcts = [r['error_pct'] for r in results]
        mean_err = sum(errors) / len(errors)
        mean_pct = sum(pcts) / len(pcts)
        print("-" * 65)
        print(f"{'平均':<16s} {'':>8s} {'':>8s} {mean_err:>8.3f} {mean_pct:>6.1f}%")

    # 输出到 CSV
    if output_file and results:
        with open(output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['标签', '实测距离(m)', 'SLAM距离(m)', '绝对误差(m)', '百分比误差(%)'])
            for r in results:
                writer.writerow([
                    r['name'],
                    f"{r['measured']:.2f}",
                    f"{r['slam_dist']:.2f}",
                    f"{r['error']:.3f}",
                    f"{r['error_pct']:.1f}",
                ])
        print(f"\n结果已保存到: {output_file}")

    return results


def main():
    parser = argparse.ArgumentParser(
        description='已知几何距离误差测量 — 对比 SLAM 轨迹与卷尺实测距离')
    parser.add_argument('traj_file', help='TUM 格式轨迹文件')
    parser.add_argument('constraints_csv', help='已知几何约束 CSV')
    parser.add_argument('--output', '-o', default=None,
                        help='输出结果 CSV 文件路径 (可选)')
    args = parser.parse_args()

    evaluate(args.traj_file, args.constraints_csv, args.output)


if __name__ == '__main__':
    main()

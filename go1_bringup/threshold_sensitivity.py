#!/usr/bin/env python3
"""阈值敏感性扫描结果聚合工具 (实验手册 §N2)

用法:
    ros2 run go1_bringup threshold_sensitivity <sweep_root>

<sweep_root> 期望布局 (扫描脚本/手工各跑一次后归档):
    sweep_root/
        feat100_res0.10/
            traj_fastlio_improved.txt
            fastlio_improved.log
            geometry_constraints.csv     # 可选, 各子目录共享
        feat100_res0.15/
            ...
        feat200_res0.10/
            ...
        ...

子目录名必须形如 ``featXXX_resY.YY`` (大小写不敏感, 下划线分隔)。

输出:
    <sweep_root>/sensitivity_summary.csv  逐组 (feat, res, 触发率, 闭环漂移, 轨迹长度)
    <sweep_root>/sensitivity_summary.md   3x3 markdown 表格 (论文直接用)

不会调用 evaluate_slam.sh。本脚本只做轻量解析: 闭环漂移取 traj 首尾 XY, 触发率
取日志中最后一条 ``[OdomStat]`` 行。完整对比 / 已知几何误差仍走 evaluate_slam.sh。
"""

from __future__ import annotations

import csv
import math
import os
import re
import sys
from pathlib import Path

NAME_RE = re.compile(r"feat(\d+)_res([\d.]+)", re.IGNORECASE)
ODOMSTAT_RE = re.compile(r"\[OdomStat\].*degraded=(\d+)/(\d+)\s*\(([\d.]+)%\)")


def loop_closure_drift(traj: Path) -> tuple[float | None, float]:
    """返回 (平面闭环漂移 m, 轨迹长度 m); 缺失时返回 (None, 0)."""
    if not traj.is_file():
        return None, 0.0
    xs, ys = [], []
    with traj.open() as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 3:
                continue
            xs.append(float(parts[1]))
            ys.append(float(parts[2]))
    if len(xs) < 2:
        return None, 0.0
    drift = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
    length = 0.0
    for i in range(1, len(xs)):
        d = math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
        if d < 1.0:  # 与 evaluate_slam.sh 一致的跳变过滤
            length += d
    return drift, length


def trigger_rate(log: Path) -> float | None:
    if not log.is_file():
        return None
    last = None
    with log.open(errors='ignore') as f:
        for line in f:
            m = ODOMSTAT_RE.search(line)
            if m:
                last = m
    if last is None:
        return None
    return float(last.group(3))


def main() -> int:
    argv = sys.argv[1:]
    if not argv or argv[0] in ('-h', '--help'):
        print(__doc__)
        return 0
    root = Path(argv[0]).expanduser().resolve()
    if not root.is_dir():
        print(f"[ERROR] 目录不存在: {root}", file=sys.stderr)
        return 1

    rows: list[dict] = []
    for sub in sorted(root.iterdir()):
        if not sub.is_dir():
            continue
        m = NAME_RE.match(sub.name)
        if not m:
            continue
        feat = int(m.group(1))
        res = float(m.group(2))
        drift, length = loop_closure_drift(sub / 'traj_fastlio_improved.txt')
        rate = trigger_rate(sub / 'fastlio_improved.log')
        rows.append({
            'feat': feat,
            'res': res,
            'trigger_rate_pct': rate,
            'drift_m': drift,
            'traj_len_m': length,
            'subdir': sub.name,
        })

    if not rows:
        print(f"[ERROR] 未在 {root} 下找到 featXXX_resY.YY 子目录", file=sys.stderr)
        return 1

    rows.sort(key=lambda r: (r['feat'], r['res']))

    csv_path = root / 'sensitivity_summary.csv'
    with csv_path.open('w', newline='') as f:
        w = csv.DictWriter(
            f,
            fieldnames=['subdir', 'feat', 'res', 'trigger_rate_pct', 'drift_m', 'traj_len_m'],
        )
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print(f"[OK] CSV: {csv_path}")

    feats = sorted({r['feat'] for r in rows})
    reses = sorted({r['res'] for r in rows})

    def cell(r: dict) -> str:
        rate = '—' if r['trigger_rate_pct'] is None else f"{r['trigger_rate_pct']:.1f}%"
        drift = '—' if r['drift_m'] is None else f"{r['drift_m']:.3f}m"
        return f"{rate} / {drift}"

    by_key = {(r['feat'], r['res']): r for r in rows}

    md_path = root / 'sensitivity_summary.md'
    with md_path.open('w') as f:
        f.write('# 退化阈值敏感性 (触发率 / 闭环漂移)\n\n')
        f.write(f'扫描根目录: `{root}`\n\n')
        header = '| feat \\\\ res |' + ''.join(f' {r:.2f} |' for r in reses)
        sep = '|' + '---|' * (len(reses) + 1)
        f.write(header + '\n' + sep + '\n')
        for ft in feats:
            line = f'| {ft} |'
            for rs in reses:
                r = by_key.get((ft, rs))
                line += f' {cell(r) if r else "—"} |'
            f.write(line + '\n')
        f.write('\n说明: 单元格为 "退化触发率 / 平面闭环漂移". '
                '触发率应在 20%-60% 区间且漂移随约束启动稳定下降, '
                '才说明阈值组合可用。\n')
    print(f"[OK] Markdown: {md_path}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

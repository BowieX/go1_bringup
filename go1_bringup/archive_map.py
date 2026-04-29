#!/usr/bin/env python3
"""
archive_map.py — 建图会话归档: 复制 PCD + 4 变体栅格地图 + 论文对比图.

设计动机:
    FAST-LIO 每次启动都会覆盖 scans.pcd. 若不及时转存, 下一次建图就会
    丢掉上次的原始数据. 本脚本在每次建图结束后自动把 scans.pcd 复制到
    maps/sessions/<时间戳>/ 下并一次性生成 4 套过滤变体 (raw / sor_only /
    cc_only / both) + matplotlib 2x2 对比图, 既能防覆盖又能直接作为论文图源.

输出目录结构:
    <out_root>/<YYYY-MM-DD_HH-MM-SS>[_label]/
        scans.pcd              # 原始点云副本
        raw.{pgm,yaml}         # 无过滤 baseline
        sor_only.{pgm,yaml}    # 仅 3D SOR
        cc_only.{pgm,yaml}     # 仅 2D 连通域过滤
        both.{pgm,yaml}        # 双级 (默认/最终版)
        compare.png            # 四宫格对比图 (论文图)
        summary.txt            # 每个变体的统计 (占据/空闲/剔除数)

用法:
    # 手动归档最近一次建图
    python3 archive_map.py --label lab_run1

    # 自动从 launch OnShutdown 触发 (见 go1_mapping.launch.py auto_archive 参数)

    # 把归档里 both.* 晋升为 maps/my_lab.* (Nav2 默认加载的那份)
    python3 archive_map.py --promote
"""
from __future__ import annotations

import argparse
import shutil
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

# 复用 pcd_to_map.py 的函数 (避免多次 fork 子进程各自重复加载 2+ GB PCD)
from go1_bringup.pcd_to_map import (
    filter_small_blobs,
    load_pcd_xyz,
    save_pgm,
    save_yaml,
    slice_and_rasterize,
    statistical_outlier_removal,
)


DEFAULT_PCD = Path.home() / "go1_ws" / "src" / "FAST_LIO" / "PCD" / "scans.pcd"
DEFAULT_OUT_ROOT = Path.home() / "go1_ws" / "src" / "go1_bringup" / "maps" / "sessions"

# 4 个对比变体 (名字, 过滤参数)
VARIANTS = [
    ("raw",      {"sor_k": 0,  "sor_std": 2.0, "min_blob": 0}),
    ("sor_only", {"sor_k": 20, "sor_std": 2.0, "min_blob": 0}),
    ("cc_only",  {"sor_k": 0,  "sor_std": 2.0, "min_blob": 5}),
    ("both",     {"sor_k": 20, "sor_std": 2.0, "min_blob": 5}),
]


# ----------------------------------------------------------------------------

def wait_for_pcd(pcd_path: Path, timeout: float) -> bool:
    """
    轮询等待 PCD 文件写完 (FAST-LIO 在析构时才落盘, 可能滞后几秒).

    判定稳定的依据: 连续两次采样大小相同且 > 0.
    """
    t0 = time.time()
    last = -1
    while time.time() - t0 < timeout:
        if pcd_path.is_file():
            cur = pcd_path.stat().st_size
            if cur > 0 and cur == last:
                return True
            last = cur
        time.sleep(1.0)
    return pcd_path.is_file() and pcd_path.stat().st_size > 0


def read_pgm(path: Path) -> np.ndarray:
    """读 P5 PGM 为 uint8 numpy array (含跳过注释行)."""
    with open(path, "rb") as f:
        magic = f.readline().strip()
        if magic != b"P5":
            raise ValueError(f"{path} 不是 P5 PGM")
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()
        return np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)


def make_compare_figure(out_dir: Path, stats: list[dict]) -> None:
    """Matplotlib 2x2 对比图. 用英文标签避免 CJK 字体依赖."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(13, 12))
    for ax, st in zip(axes.flat, stats):
        img = read_pgm(out_dir / f"{st['name']}.pgm")
        ax.imshow(img, cmap="gray", vmin=0, vmax=255)
        cfg = st["cfg"]
        parts = []
        if cfg["sor_k"] > 0:
            parts.append(f"SOR(k={cfg['sor_k']}, std={cfg['sor_std']})")
        if cfg["min_blob"] > 1:
            parts.append(f"CC(>={cfg['min_blob']})")
        desc = " + ".join(parts) if parts else "no filter"
        h, w = st["shape"]
        ax.set_title(
            f"{st['name']}  [{desc}]\n"
            f"{w} x {h}, occ={st['n_occ']:,}, free={st['n_free']:,}",
            fontsize=11,
        )
        ax.set_xticks([])
        ax.set_yticks([])
    fig.suptitle(f"PCD -> 2D grid filter comparison  ({out_dir.name})", fontsize=13)
    fig.tight_layout()
    fig.savefig(out_dir / "compare.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def write_summary(out_dir: Path, pcd_path: Path, n_pts: int, stats: list[dict]) -> None:
    lines = []
    lines.append(f"session: {out_dir.name}")
    lines.append(f"source:  {pcd_path}")
    lines.append(f"points:  {n_pts:,}")
    lines.append("")
    lines.append(f"{'variant':<10} {'SOR removed':>12} {'occ_before_cc':>14} "
                 f"{'occ_final':>10} {'free':>10} {'size':>13}")
    for st in stats:
        h, w = st["shape"]
        lines.append(
            f"{st['name']:<10} {st['n_sor_removed']:>12,} "
            f"{st['n_occ_before_cc']:>14,} {st['n_occ']:>10,} "
            f"{st['n_free']:>10,} {f'{w}x{h}':>13}"
        )
    (out_dir / "summary.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")


# ----------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--pcd-path", type=Path, default=DEFAULT_PCD,
                    help=f"源 PCD (默认 {DEFAULT_PCD})")
    ap.add_argument("--out-root", type=Path, default=DEFAULT_OUT_ROOT,
                    help=f"归档根目录 (默认 {DEFAULT_OUT_ROOT})")
    ap.add_argument("--label", type=str, default="",
                    help="会话标签, 附在时间戳后, 例如 lab_run1")
    ap.add_argument("--no-compare", action="store_true",
                    help="跳过 matplotlib 对比图 (仅保留 pgm/yaml, 省 ~10 秒)")
    ap.add_argument("--promote", action="store_true",
                    help="同时把 both.* 复制为 maps/my_lab.* (Nav2 默认加载). "
                         "注意: --promote 会用本次传入的 --z-min/--z-max/--resolution/"
                         "--hit-threshold/--padding 重新跑切片, 与之前归档的 both.* 可能不同; "
                         "若想完全复用某次归档, 直接 cp sessions/<时间戳>/both.{pgm,yaml} "
                         "到 maps/my_lab.* 并把 yaml 里 image: 字段改成 my_lab.pgm")
    ap.add_argument("--wait-seconds", type=float, default=10.0,
                    help="等待 PCD 文件稳定的最长秒数 (默认 10, FAST-LIO 析构落盘需时间)")
    # pcd_to_map.py 透传参数 (切片 + 栅格化)
    ap.add_argument("--z-min", type=float, default=0.05)
    ap.add_argument("--z-max", type=float, default=0.25)
    ap.add_argument("--resolution", type=float, default=0.05)
    ap.add_argument("--hit-threshold", type=int, default=2)
    ap.add_argument("--padding", type=float, default=0.5)
    args = ap.parse_args()

    # ---- 1. 等 PCD 稳定 ----
    print(f"[archive] 等待 PCD 就绪: {args.pcd_path}")
    if not wait_for_pcd(args.pcd_path, args.wait_seconds):
        print("[archive][ERROR] PCD 未就绪或为空, 跳过归档.", file=sys.stderr)
        return 1

    # ---- 2. 建目录 ----
    ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    if args.label:
        safe = "".join(c if c.isalnum() or c in "-_" else "_" for c in args.label)
        dirname = f"{ts}_{safe}"
    else:
        dirname = ts
    out_dir = args.out_root / dirname
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"[archive] 归档到: {out_dir}")

    # ---- 3. 复制原始 PCD (防下次建图覆盖) ----
    pcd_copy = out_dir / "scans.pcd"
    print(f"[archive] 复制 scans.pcd ({args.pcd_path.stat().st_size / 1e6:.1f} MB) ...")
    shutil.copy2(args.pcd_path, pcd_copy)

    # ---- 4. 加载一次, 4 个变体共用 ----
    print("[archive] 加载点云 ...")
    xyz = load_pcd_xyz(args.pcd_path)
    print(f"[archive]   {len(xyz):,} 个点, Z 范围 [{xyz[:,2].min():.2f}, {xyz[:,2].max():.2f}]")

    # ---- 5. 跑 4 变体 (SOR 只做一次, 给 sor_only+both 共用, 省一半时间) ----
    # 4 个变体里有 2 个开 SOR 2 个关 SOR. 预计算 xyz_sor 后直接复用, 避免重复跑
    # scipy.cKDTree (占总耗时 70%+)
    sor_keys = {(cfg["sor_k"], cfg["sor_std"]) for _, cfg in VARIANTS if cfg["sor_k"] > 0}
    xyz_cache = {None: xyz}  # key: None=原始, (k, std)=SOR 参数
    for k, std in sor_keys:
        print(f"[archive] 预计算 SOR k={k} std={std} ...")
        xyz_cache[(k, std)] = statistical_outlier_removal(xyz, k, std)
        print(f"[archive]   剔除 {len(xyz) - len(xyz_cache[(k, std)]):,} 点")

    stats = []
    for name, cfg in VARIANTS:
        print(f"[archive] 变体 {name} (SOR k={cfg['sor_k']}, min_blob={cfg['min_blob']}) ...")
        key = (cfg["sor_k"], cfg["sor_std"]) if cfg["sor_k"] > 0 else None
        xyz_in = xyz_cache[key]
        n_sor_removed = len(xyz) - len(xyz_in)
        grid, ox, oy = slice_and_rasterize(
            xyz_in, args.z_min, args.z_max, args.resolution,
            args.hit_threshold, args.padding,
        )
        n_occ_before_cc = int((grid == 2).sum())
        if cfg["min_blob"] > 1:
            grid = filter_small_blobs(grid, cfg["min_blob"])
        n_occ = int((grid == 2).sum())
        n_free = int((grid == 1).sum())
        save_pgm(out_dir / f"{name}.pgm", grid)
        save_yaml(out_dir / f"{name}.yaml", f"{name}.pgm", args.resolution, ox, oy)
        st = {
            "name": name, "cfg": cfg,
            "n_sor_removed": n_sor_removed,
            "n_occ_before_cc": n_occ_before_cc,
            "n_occ": n_occ, "n_free": n_free,
            "shape": grid.shape, "origin": (ox, oy),
        }
        stats.append(st)
        h, w = st["shape"]
        print(f"[archive]   -> {w}x{h}, 占据 {n_occ:,} (SOR 剔 {n_sor_removed:,} 点, "
              f"CC 剔 {n_occ_before_cc - n_occ:,} 格)")

    # ---- 6. 对比图 ----
    if not args.no_compare:
        print("[archive] 生成 compare.png ...")
        try:
            make_compare_figure(out_dir, stats)
        except ImportError as e:
            print(f"[archive][WARN] matplotlib 不可用, 跳过对比图: {e}", file=sys.stderr)

    # ---- 7. 文本摘要 ----
    write_summary(out_dir, args.pcd_path, len(xyz), stats)

    # ---- 8. 可选: 晋升 both.* 为 maps/my_lab.* ----
    if args.promote:
        maps_root = args.out_root.parent  # .../maps
        both_stat = next(s for s in stats if s["name"] == "both")
        ox, oy = both_stat["origin"]
        shutil.copy2(out_dir / "both.pgm", maps_root / "my_lab.pgm")
        # yaml 里的 image 字段要指向 my_lab.pgm 而不是 both.pgm, 重写
        save_yaml(maps_root / "my_lab.yaml", "my_lab.pgm",
                  args.resolution, ox, oy)
        print(f"[archive] 已同步 {maps_root / 'my_lab.pgm'} / my_lab.yaml")

    print(f"\n[archive][OK] 完成: {out_dir}")
    print(f"  对比图: {out_dir / 'compare.png'}")
    print(f"  摘要:   {out_dir / 'summary.txt'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

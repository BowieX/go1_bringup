#!/usr/bin/env python3
"""
    pcd_to_map.py — 将 FAST-LIO 累积的 3D 点云切片为 Nav2 可用的 2D 占据栅格地图.

用法:
    python3 pcd_to_map.py <input.pcd> <output_dir> <map_name> \
        [--z-min 0.05] [--z-max 0.25] [--resolution 0.05] \
        [--hit-threshold 2] [--padding 0.5] \
        [--sor-k 20] [--sor-std 2.0] [--min-blob-size 5]

输出:
    <output_dir>/<map_name>.pgm   # P5 灰度图, 0=障碍, 254=空闲, 205=未知
    <output_dir>/<map_name>.yaml  # Nav2 map_server 标准格式

设计动机:
    FAST-LIO 3D 建图 (带回环一致的 ikd-Tree 子地图) 质量极高，
    但 Go1 颠簸 + 长廊退化场景下 2D 扫描匹配易累计角度误差 (项目早期试过
    slam_toolbox, 8 字路径会出现重影), 故改为直接离线切片 PCD 生成静态
    栅格地图, 避开 2D SLAM 弱点。

杂点过滤 (两段式):
    1) 3D 统计离群点去除 (SOR): 对每个点算其 k 近邻平均距离, 分布外 (> μ+σ·std)
       的点视为噪声丢弃. 对应 PCL StatisticalOutlierRemoval, 能干净去除飞点/玻璃反射.
    2) 2D 连通域过滤: 栅格化之后, 把占据格的八连通块小于 min_blob_size 的整块置为空闲.
       去除切片平面上孤立的毛刺, 同时保留长廊墙面 (大连通块).

依赖:
    numpy, scipy (Jetson 上 `sudo apt install python3-scipy` 即可).
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from scipy.ndimage import label as cc_label
from scipy.spatial import cKDTree


# ============================================================================
# PCD 二进制解析器 (PCL 官方格式, 见 https://pointclouds.org/documentation/tutorials/pcd_file_format.html)
# ============================================================================

def parse_pcd_header(f) -> dict:
    """读取 PCD 文本 header, 返回字段字典. 文件游标会停在 DATA 行的下一字节."""
    header: dict = {}
    while True:
        line = f.readline()
        if not line:
            raise ValueError("PCD header 结束前文件就 EOF")
        line = line.decode("ascii", errors="replace").strip()
        if not line or line.startswith("#"):
            continue
        tokens = line.split()
        key = tokens[0].upper()
        vals = tokens[1:]
        if key == "FIELDS":
            header["fields"] = vals
        elif key == "SIZE":
            header["size"] = [int(v) for v in vals]
        elif key == "TYPE":
            header["type"] = vals
        elif key == "COUNT":
            header["count"] = [int(v) for v in vals]
        elif key == "WIDTH":
            header["width"] = int(vals[0])
        elif key == "HEIGHT":
            header["height"] = int(vals[0])
        elif key == "POINTS":
            header["points"] = int(vals[0])
        elif key == "DATA":
            header["data"] = vals[0].lower()
            break
    return header


def pcd_type_to_numpy(t: str, size: int) -> str:
    """PCD TYPE/SIZE 转 numpy dtype 字符串."""
    m = {"F": "f", "U": "u", "I": "i"}
    if t not in m:
        raise ValueError(f"未知的 PCD TYPE: {t}")
    return f"<{m[t]}{size}"


def load_pcd_xyz(path: Path) -> np.ndarray:
    """
    加载 PCD 文件, 只返回 (N, 3) 的 xyz float32 数组.

    支持 DATA binary; ASCII 未实现 (FAST-LIO 始终输出 binary, 无需浪费代码).
    """
    with open(path, "rb") as f:
        header = parse_pcd_header(f)
        if header["data"] != "binary":
            raise NotImplementedError(f"只支持 DATA binary, 当前是 {header['data']}")
        required_fields = {"x", "y", "z"}
        if not required_fields.issubset(header["fields"]):
            raise ValueError(f"PCD 字段缺失 xyz: {header['fields']}")
        # 构造 structured dtype, 精确匹配每个字段的 type/size/count
        dtype_fields = []
        field_specs = zip(
            header["fields"], header["size"], header["type"], header["count"],
        )
        for name, sz, tp, cnt in field_specs:
            base = pcd_type_to_numpy(tp, sz)
            if cnt == 1:
                dtype_fields.append((name, base))
            else:
                dtype_fields.append((name, base, (cnt,)))
        dtype = np.dtype(dtype_fields)
        n = header["points"]
        raw = np.frombuffer(f.read(n * dtype.itemsize), dtype=dtype, count=n)
        # 只抽取 xyz, 丢弃 intensity/normal 等, 节省内存
        xyz = np.stack([raw["x"], raw["y"], raw["z"]], axis=1).astype(np.float32, copy=False)
        return xyz


# ============================================================================
# 噪声过滤
# ============================================================================

def statistical_outlier_removal(xyz: np.ndarray, k: int, std_ratio: float) -> np.ndarray:
    """
    3D 统计离群点去除 (等价 PCL StatisticalOutlierRemoval).

    对每个点用 cKDTree 查最近 k 个邻居 (不含自身), 取 k 邻居平均距离 d_i.
    全局分布 μ = mean(d), σ = std(d); 若 d_i > μ + std_ratio·σ 则视为离群点丢弃.

    在长廊/大空间里, FAST-LIO 有时会因玻璃反射/动态物残留/IMU 积分噪声
    生成孤立的飞点, 这些点的近邻距离远大于墙面/地面密集点, 能被干净剔除.

    参数:
        k         — 近邻数, 典型 10~30. k 越大越稳, 但计算慢.
        std_ratio — 判离群的 σ 倍数, 典型 1.0~3.0. 越小越激进.
    """
    if k <= 0 or len(xyz) < k + 1:
        return xyz
    tree = cKDTree(xyz)
    # 查 k+1 个邻居 (第 0 个是点本身, 距离 0, 不计入均值)
    dists, _ = tree.query(xyz, k=k + 1, workers=-1)
    mean_d = dists[:, 1:].mean(axis=1)
    thresh = mean_d.mean() + std_ratio * mean_d.std()
    keep = mean_d <= thresh
    return xyz[keep]


def filter_small_blobs(grid: np.ndarray, min_size: int) -> np.ndarray:
    """
    八连通域过滤: 占据格的连通块若小于 min_size 个像素, 整块退回空闲.

    grid 语义沿用: 0=未知, 1=空闲, 2=占据. 原地修改并返回.

    二维切片上仍可能出现孤立的小斑点 (尤其 hit_threshold 调低时),
    CC 过滤可以清理这些"浮点", 同时大块墙面/柱子的连通分量远超 min_size, 不受影响.
    """
    if min_size <= 1:
        return grid
    occ = (grid == 2)
    # 八连通 (3x3 邻域全 1): 对角线相邻的占据格也算同一块
    structure = np.ones((3, 3), dtype=np.uint8)
    labels, n = cc_label(occ, structure=structure)
    if n == 0:
        return grid
    # 每个连通域的像素数 (bincount[0] 是背景, 跳过)
    sizes = np.bincount(labels.ravel())
    too_small = np.where(sizes < min_size)[0]
    too_small = too_small[too_small > 0]  # 去掉背景 label 0
    if too_small.size == 0:
        return grid
    small_mask = np.isin(labels, too_small)
    grid[small_mask] = 1  # 小块退回空闲
    return grid


# ============================================================================
# 切片 + 栅格化
# ============================================================================

def slice_and_rasterize(
    xyz: np.ndarray,
    z_min: float,
    z_max: float,
    resolution: float,
    hit_threshold: int,
    padding: float,
) -> tuple[np.ndarray, float, float]:
    """
    Z 切片 -> 2D 栅格化 -> 返回 (grid, origin_x, origin_y).

    grid 语义 (实际只用 1/2 两种):
      1 = 空闲 (bbox 内未被命中的格子)
      2 = 占据 (命中数 >= hit_threshold)
      0 = 保留位 (本简化版无 raytracing, 不区分 unknown, save_pgm 中也不使用)

    Z 切片基准 (重要):
      切片 Z 用的是 FAST-LIO 的 camera_init 帧, 该帧 Z=0 在启动时机器人 IMU/body
      位置 (≈ 离地 ~0.30 m, 不是地面). 因此默认值 [0.05, 0.25] 切的是 body 高度
      +5 cm 到 +25 cm (≈ 离地 0.35-0.55 m), 落在 LiDAR 安装高度 (~0.57 m) 略下方,
      能稳定捕获走廊/房间的连续墙面. 想包括低矮障碍 (椅腿/箱子) 应当下探到负值,
      例如 [-0.20, 0.05].

    核心思路:
      1. Z 切片: 只保留 z ∈ [z_min, z_max] 的点, 落在机器人碰撞高度带内.
      2. 命中计数: 在 XY 平面用 resolution 栅格累加, 超阈值则标为占据.
      3. 空闲区域: 此处采用"简化版" - 凡是在 bbox 内、未被命中的格子全部标空闲.
         真正的 raytracing (光追) 需要传感器位姿序列, 离线 PCD 里已丢失, 所以只能近似.
         对已完整覆盖的室内区域可用于 Nav2 初步规划; 若建图轨迹没有覆盖到开阔边界
         或玻璃/低矮障碍, 必须实机复核并适当裁剪地图。
    """
    # Z 切片
    z = xyz[:, 2]
    mask = (z >= z_min) & (z <= z_max)
    pts_sliced = xyz[mask]
    if pts_sliced.size == 0:
        raise RuntimeError(f"Z 切片后无点! 检查 z-min/z-max (当前 [{z_min}, {z_max}])")

    # XY bbox: 用全部切片点算范围, 四周留 padding 米
    x_min = float(pts_sliced[:, 0].min()) - padding
    x_max = float(pts_sliced[:, 0].max()) + padding
    y_min = float(pts_sliced[:, 1].min()) - padding
    y_max = float(pts_sliced[:, 1].max()) + padding

    # 栅格尺寸: Nav2 / map_server 要求 width=列数(X 方向), height=行数(Y 方向)
    # pgm 图像坐标: 第 0 行在图像最上方对应 y_max, 最后一行对应 y_min
    width = int(np.ceil((x_max - x_min) / resolution))
    height = int(np.ceil((y_max - y_min) / resolution))
    if width <= 0 or height <= 0:
        raise RuntimeError(f"栅格尺寸非法 {width}x{height}")

    # 命中计数: 把切片点的 (x,y) 转为 (col, row) 索引, 用 np.add.at 累加
    col = np.floor((pts_sliced[:, 0] - x_min) / resolution).astype(np.int32)
    # y 方向翻转: pgm 图像顶部是 y_max
    row = np.floor((y_max - pts_sliced[:, 1]) / resolution).astype(np.int32)
    # 越界保护 (bbox 已含 padding, 理论上不会出, 但防御性裁剪)
    col = np.clip(col, 0, width - 1)
    row = np.clip(row, 0, height - 1)

    hits = np.zeros((height, width), dtype=np.int32)
    np.add.at(hits, (row, col), 1)

    # 生成 occupancy grid: 1=空闲, 2=占据 (本简化版无 raytracing, 不输出 unknown)
    grid = np.zeros((height, width), dtype=np.uint8)
    # 先把 bbox 内所有非命中格子标为空闲 (简化: 不做真实 raytracing)
    grid[:] = 1  # 全部先假设空闲
    # 命中达阈值的格子标占据
    grid[hits >= hit_threshold] = 2
    # 命中数非零但不足阈值的格子: 降噪 -> 保持空闲 (即噪点不影响地图)

    # origin 遵循 Nav2 约定: 图像左下角在世界坐标 (origin_x, origin_y, 0)
    # 图像左下角对应 (x_min, y_min)
    return grid, x_min, y_min


# ============================================================================
# pgm / yaml 输出 (Nav2 map_server 兼容)
# ============================================================================

def save_pgm(path: Path, grid: np.ndarray) -> None:
    """
    P5 灰度 pgm, Nav2 trinary 像素值映射: 0=occupied, 254=free, 205=unknown.

    本简化版 grid 只有 1/2 两种 (无 raytracing, 不区分 unknown), 因此
    实际输出的像素只有 0 (occupied) 和 254 (free), 205 仅作为 grid==0
    的兜底, 当前流程不会触发.
    """
    h, w = grid.shape
    # 像素值映射
    img = np.full_like(grid, 205, dtype=np.uint8)  # unknown 兜底 (本流程不触发)
    img[grid == 1] = 254  # free
    img[grid == 2] = 0    # occupied
    header = f"P5\n{w} {h}\n255\n".encode("ascii")
    with open(path, "wb") as f:
        f.write(header)
        f.write(img.tobytes())


def save_yaml(
    path: Path,
    pgm_name: str,
    resolution: float,
    origin_x: float,
    origin_y: float,
) -> None:
    """map_server yaml 格式, 与 Nav2 默认 nav2_map_server 兼容."""
    text = (
        f"image: {pgm_name}\n"
        f"mode: trinary\n"
        f"resolution: {resolution}\n"
        f"origin: [{origin_x:.6f}, {origin_y:.6f}, 0.0]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.25\n"
    )
    path.write_text(text, encoding="utf-8")


# ============================================================================
# main
# ============================================================================

def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("pcd", type=Path, help="输入 PCD 文件 (FAST-LIO 输出的 scans.pcd)")
    ap.add_argument("out_dir", type=Path, help="输出目录 (会创建 <map_name>.pgm/.yaml)")
    ap.add_argument("map_name", type=str, help="地图文件名前缀, 例如 my_lab")
    ap.add_argument("--z-min", type=float, default=0.05,
                    help="Z 切片下界 (米, FAST-LIO camera_init 帧, Z=0≈ body 启动位置 ≈ 离地 0.30 m). "
                         "默认 0.05 ≈ 离地 0.35 m, 略低于 LiDAR (≈0.57 m)")
    ap.add_argument("--z-max", type=float, default=0.25,
                    help="Z 切片上界 (米, camera_init 帧). 默认 0.25 ≈ 离地 0.55 m, "
                         "[0.05, 0.25] 切胸-肩高度墙面带, 走廊/房间的连续墙体可被稳定捕获")
    ap.add_argument("--resolution", type=float, default=0.05,
                    help="栅格分辨率 (米/格, 默认 0.05 与 Nav2 costmap 一致)")
    ap.add_argument("--hit-threshold", type=int, default=2,
                    help="单格命中阈值: 超过此值才标为占据 (降噪, 默认 2)")
    ap.add_argument("--padding", type=float, default=0.5,
                    help="bbox 外扩 (米, 让地图边界不紧贴点云, 默认 0.5)")
    ap.add_argument("--sor-k", type=int, default=20,
                    help="SOR 近邻数 k (默认 20, 设 0 关闭 3D 离群点去除)")
    ap.add_argument("--sor-std", type=float, default=2.0,
                    help="SOR σ 倍数阈值 (默认 2.0, 越小过滤越激进)")
    ap.add_argument("--min-blob-size", type=int, default=5,
                    help="2D 连通域最小像素数 (默认 5, 小于此值的占据斑点退回空闲; 设 0 关闭)")
    args = ap.parse_args()

    if not args.pcd.is_file():
        print(f"[ERROR] PCD 文件不存在: {args.pcd}", file=sys.stderr)
        return 1
    args.out_dir.mkdir(parents=True, exist_ok=True)

    print(f"[1/5] 加载 PCD: {args.pcd} ({args.pcd.stat().st_size / 1e6:.1f} MB)")
    xyz = load_pcd_xyz(args.pcd)
    print(f"        读取 {len(xyz):,} 个点, Z 范围 [{xyz[:,2].min():.3f}, {xyz[:,2].max():.3f}]")

    if args.sor_k > 0:
        print(f"[2/5] 3D SOR 离群点去除 (k={args.sor_k}, std={args.sor_std})")
        n_before = len(xyz)
        xyz = statistical_outlier_removal(xyz, args.sor_k, args.sor_std)
        n_removed = n_before - len(xyz)
        print(f"        剔除 {n_removed:,} 点 ({n_removed / n_before * 100:.2f}%), 剩余 {len(xyz):,}")
    else:
        print("[2/5] 跳过 3D SOR (--sor-k=0)")

    print(f"[3/5] Z 切片 [{args.z_min}, {args.z_max}] + 栅格化 (res={args.resolution} m)")
    grid, ox, oy = slice_and_rasterize(
        xyz, args.z_min, args.z_max, args.resolution, args.hit_threshold, args.padding,
    )
    n_occ_raw = int((grid == 2).sum())

    if args.min_blob_size > 1:
        grid = filter_small_blobs(grid, args.min_blob_size)
        n_occ = int((grid == 2).sum())
        print(f"[4/5] 连通域过滤 (min_blob={args.min_blob_size}): 占据 {n_occ_raw:,} -> {n_occ:,} "
              f"(剔 {n_occ_raw - n_occ:,} 毛刺格)")
    else:
        n_occ = n_occ_raw
        print("[4/5] 跳过连通域过滤 (--min-blob-size<=1)")
    n_free = int((grid == 1).sum())
    print(f"        最终尺寸 {grid.shape[1]}x{grid.shape[0]}, 占据 {n_occ:,} 格, 空闲 {n_free:,} 格")
    print(f"        origin (世界坐标, 左下角): ({ox:.3f}, {oy:.3f})")

    pgm_path = args.out_dir / f"{args.map_name}.pgm"
    yaml_path = args.out_dir / f"{args.map_name}.yaml"
    print(f"[5/5] 写入 PGM: {pgm_path}")
    save_pgm(pgm_path, grid)
    print(f"       写入 YAML: {yaml_path}")
    save_yaml(yaml_path, f"{args.map_name}.pgm", args.resolution, ox, oy)

    print("\n[OK] 完成. Nav2 可直接加载:")
    print(f"    ros2 launch go1_bringup go1_nav.launch.py map:={yaml_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

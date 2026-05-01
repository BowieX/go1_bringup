#!/bin/bash
# =============================================================================
# SLAM 消融实验评估脚本
#
# 评估指标 (均不依赖外部动捕系统):
#   1. 平面闭环漂移 — 闭合回路首尾 XY 位移差 (m)，主指标
#   2. 已知几何误差 — SLAM 轨迹中两点距离 vs 卷尺实测距离 (m)
#   3. 轨迹对比图 — evo_traj 俯视图，定性对比
#   4. 轨迹一致性 RPE — 基线 vs 改进版的相对位姿误差 (m)
#   5. 退化触发率 — 改进版 [OdomStat] 末次汇报的退化帧占比
#   6. 退化触发空间分布 — 把 [OdomFrame] 解析为退化点散点叠加到改进版 XY 轨迹
#
# 注意: 本脚本不计算 APE (Absolute Pose Error)。
#       APE 需要 mm 级绝对真值 (动捕/RTK)，本项目不具备此条件。
#       用 baseline 当 "参考" 算出来的 APE 不是真正的绝对精度，论文中不应使用。
#
# 前置条件:
#   pip3 install evo --upgrade
#
# 使用方式:
#   ./evaluate_slam.sh <trajectories_dir>
#   例如: ./evaluate_slam.sh ~/go1_ws/trajectories
#
# 输入文件 (TUM 格式，由 record_trajectory.py 生成):
#   <dir>/traj_fastlio_baseline.txt   — 基线 FAST-LIO2 (关闭 odom 约束)
#   <dir>/traj_fastlio_improved.txt   — 改进 FAST-LIO2 (开启 odom 约束)
#   <dir>/traj_fastlio_always.txt     — 常开强约束消融 (可选, force_degraded:=true)
#   <dir>/geometry_constraints.csv    — 已知几何距离 (可选, 见下方格式说明)
#   <dir>/fastlio_improved.log        — 改进版 FAST-LIO 终端日志 (可选, 用于退化触发率统计)
#                                       生成方法: 在 §7.4 回放时 `2>&1 | tee <dir>/fastlio_improved.log`
#   <dir>/fastlio_always.log          — 常开强约束 FAST-LIO 日志 (可选)
#
# geometry_constraints.csv 格式 (无表头):
#   标签, 实测距离(m), 起点x, 起点y, 终点x, 终点y
#   例如:
#     走廊长度,30.5,0.0,0.0,30.2,1.1
#     房间宽度,6.0,0.0,0.0,0.0,5.9
#   起点/终点坐标是你在 SLAM 轨迹中能辨认的两个位置的 **大致坐标**，
#   脚本会自动找轨迹中距离这些坐标最近的点来计算距离。
#
# 输出:
#   <dir>/results/ 目录下的对比图表、CSV 数据、汇总报告
# =============================================================================

set -e

# 参数检查
TRAJ_DIR="${1:-$HOME/go1_ws/trajectories}"
RESULTS_DIR="${TRAJ_DIR}/results"
mkdir -p "${RESULTS_DIR}"

BASELINE="${TRAJ_DIR}/traj_fastlio_baseline.txt"
IMPROVED="${TRAJ_DIR}/traj_fastlio_improved.txt"
ALWAYS="${TRAJ_DIR}/traj_fastlio_always.txt"
GEOMETRY="${TRAJ_DIR}/geometry_constraints.csv"
IMPROVED_LOG="${TRAJ_DIR}/fastlio_improved.log"
ALWAYS_LOG="${TRAJ_DIR}/fastlio_always.log"
HAS_ALWAYS=false
if [ -f "${ALWAYS}" ]; then
    HAS_ALWAYS=true
fi

echo "=================================================="
echo " SLAM 消融实验评估 (无动捕真值方案)"
echo "=================================================="
echo " 轨迹目录: ${TRAJ_DIR}"
echo " 结果输出: ${RESULTS_DIR}"
echo ""

# 检查文件存在性
if [ ! -f "${BASELINE}" ] || [ ! -f "${IMPROVED}" ]; then
    echo "[ERROR] 缺少轨迹文件！请确保以下文件存在:"
    echo "  ${BASELINE}"
    echo "  ${IMPROVED}"
    echo ""
    echo "生成方法 (rosbag 消融实验):"
    echo "  1. 录制 rosbag:  ros2 bag record /livox/lidar /livox/imu /odom /imu -o <name>"
    echo "  2. 回放基线:     ros2 launch go1_bringup go1_replay.launch.py odom_constraint:=false"
    echo "  3. 回放改进版:   ros2 launch go1_bringup go1_replay.launch.py odom_constraint:=true"
    echo "  4. 可选常开约束: ros2 launch go1_bringup go1_replay.launch.py odom_constraint:=true force_degraded:=true"
    echo "  5. 每次回放都用 record_trajectory.py 记录轨迹 (experiment_label:=baseline/improved/always)"
    exit 1
fi

# ------------------------------------------------------------------
# 工具函数: 计算平面闭环漂移 (TUM 轨迹首尾 XY 位移差)
# ------------------------------------------------------------------
calc_loop_closure_drift() {
    local traj_file=$1
    python3 -c "
import math
with open('${traj_file}') as f:
    lines = [l.strip() for l in f if l.strip() and not l.startswith('#')]
if len(lines) < 2:
    print('N/A')
else:
    first = lines[0].split()
    last = lines[-1].split()
    dx = float(last[1]) - float(first[1])
    dy = float(last[2]) - float(first[2])
    drift = math.sqrt(dx*dx + dy*dy)
    print(f'{drift:.4f}')
"
}

# ------------------------------------------------------------------
# 工具函数: 计算轨迹总长度
# ------------------------------------------------------------------
calc_trajectory_length() {
    local traj_file=$1
    python3 -c "
import math
with open('${traj_file}') as f:
    lines = [l.strip() for l in f if l.strip() and not l.startswith('#')]
total = 0.0
for i in range(1, len(lines)):
    prev = lines[i-1].split()
    curr = lines[i].split()
    dx = float(curr[1]) - float(prev[1])
    dy = float(curr[2]) - float(prev[2])
    dist = math.sqrt(dx*dx + dy*dy)
    if dist < 1.0:  # 过滤跳变
        total += dist
print(f'{total:.2f}')
"
}

# ------------------------------------------------------------------
# 工具函数: 计算已知几何距离误差
#   输入: 轨迹文件, geometry_constraints.csv
#   对每行约束，在轨迹中找最近点，算两点间距离，与实测值对比
# ------------------------------------------------------------------
calc_geometry_errors() {
    local traj_file=$1
    local constraints_file=$2
    local label=$3
    python3 -c "
import math, csv, sys

# 读取轨迹
poses = []
with open('${traj_file}') as f:
    for line in f:
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        parts = line.split()
        poses.append((float(parts[1]), float(parts[2])))

if not poses:
    sys.exit(0)

def find_nearest(px, py):
    \"\"\"在轨迹中找距离 (px, py) 最近的点，返回 (x, y, 最近距离)\"\"\"
    best_d = float('inf')
    best = (0.0, 0.0)
    for x, y in poses:
        d = math.sqrt((x - px)**2 + (y - py)**2)
        if d < best_d:
            best_d = d
            best = (x, y)
    return best[0], best[1], best_d

# 读取约束并计算
with open('${constraints_file}') as f:
    reader = csv.reader(f)
    for row in reader:
        if len(row) < 6 or row[0].strip().startswith('#'):
            continue
        name = row[0].strip()
        measured = float(row[1])
        sx, sy = float(row[2]), float(row[3])
        ex, ey = float(row[4]), float(row[5])

        x1, y1, d1 = find_nearest(sx, sy)
        x2, y2, d2 = find_nearest(ex, ey)
        slam_dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        error = abs(slam_dist - measured)
        pct = (error / measured * 100) if measured > 0 else 0

        print(f'  {name}: 实测={measured:.2f}m, ${label}={slam_dist:.2f}m, 误差={error:.3f}m ({pct:.1f}%)')
        print(f'    匹配点: ({x1:.2f},{y1:.2f})→({x2:.2f},{y2:.2f}), 匹配偏差: {d1:.2f}m, {d2:.2f}m')
"
}

# 工具函数: 从 evo 结果 zip 中提取 RMSE
extract_evo_rmse() {
    local zip_file=$1
    python3 -c "
import json, zipfile
try:
    with zipfile.ZipFile('${zip_file}') as zf:
        with zf.open('stats.json') as f:
            stats = json.load(f)
    print(f\"{stats.get('rmse', 'N/A'):.4f}\")
except:
    print('N/A')
"
}

# ==================================================================
# 阶段 1: 平面闭环漂移 (核心指标)
# ==================================================================
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "[1/5] 平面闭环漂移 (Planar Loop Closure Drift)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

DRIFT_BASELINE=$(calc_loop_closure_drift "${BASELINE}")
DRIFT_IMPROVED=$(calc_loop_closure_drift "${IMPROVED}")
LEN_BASELINE=$(calc_trajectory_length "${BASELINE}")
LEN_IMPROVED=$(calc_trajectory_length "${IMPROVED}")
DRIFT_ALWAYS="N/A"
LEN_ALWAYS="N/A"
if ${HAS_ALWAYS}; then
    DRIFT_ALWAYS=$(calc_loop_closure_drift "${ALWAYS}")
    LEN_ALWAYS=$(calc_trajectory_length "${ALWAYS}")
fi

echo "  基线:   平面闭环漂移 = ${DRIFT_BASELINE} m  (轨迹长度 ${LEN_BASELINE} m)"
echo "  改进版: 平面闭环漂移 = ${DRIFT_IMPROVED} m  (轨迹长度 ${LEN_IMPROVED} m)"
if ${HAS_ALWAYS}; then
    echo "  常开版: 平面闭环漂移 = ${DRIFT_ALWAYS} m  (轨迹长度 ${LEN_ALWAYS} m)"
fi

# 计算漂移率 (漂移/轨迹长度 × 100%)
python3 -c "
b_drift, b_len = '${DRIFT_BASELINE}', '${LEN_BASELINE}'
i_drift, i_len = '${DRIFT_IMPROVED}', '${LEN_IMPROVED}'
a_drift, a_len = '${DRIFT_ALWAYS}', '${LEN_ALWAYS}'
if b_drift != 'N/A' and b_len != 'N/A' and float(b_len) > 0:
    print(f'  基线漂移率:   {float(b_drift)/float(b_len)*100:.2f}%')
if i_drift != 'N/A' and i_len != 'N/A' and float(i_len) > 0:
    print(f'  改进版漂移率: {float(i_drift)/float(i_len)*100:.2f}%')
if a_drift != 'N/A' and a_len != 'N/A' and float(a_len) > 0:
    print(f'  常开版漂移率: {float(a_drift)/float(a_len)*100:.2f}%')
if b_drift != 'N/A' and i_drift != 'N/A' and float(b_drift) > 0:
    reduction = (1.0 - float(i_drift)/float(b_drift)) * 100
    print(f'  漂移降低:     {reduction:.1f}%')
if i_drift != 'N/A' and a_drift != 'N/A':
    diff = float(a_drift) - float(i_drift)
    print(f'  常开-退化感知漂移差: {diff:.4f} m')
"
echo ""

# ==================================================================
# 阶段 2: 已知几何误差 (需要 geometry_constraints.csv)
# ==================================================================
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "[2/5] 已知几何距离误差 (Known Geometry Error)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if [ -f "${GEOMETRY}" ]; then
    echo ""
    echo "  --- 基线 ---"
    calc_geometry_errors "${BASELINE}" "${GEOMETRY}" "SLAM"
    echo ""
    echo "  --- 改进版 ---"
    calc_geometry_errors "${IMPROVED}" "${GEOMETRY}" "SLAM"
    if ${HAS_ALWAYS}; then
        echo ""
        echo "  --- 常开强约束 ---"
        calc_geometry_errors "${ALWAYS}" "${GEOMETRY}" "SLAM"
    fi
else
    echo "  [跳过] 未找到 ${GEOMETRY}"
    echo "  创建方法: 用卷尺量实验场地中的已知距离 (廊长/房间宽/柱间距等),"
    echo "  写入 CSV 文件, 格式:"
    echo "    标签,实测距离(m),起点x,起点y,终点x,终点y"
    echo "  例如:"
    echo "    走廊长度,30.5,0.0,0.0,30.2,1.1"
fi
echo ""

# ==================================================================
# 阶段 3: 轨迹对比图 (evo_traj)
# ==================================================================
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "[3/5] 轨迹对比图 (Trajectory Comparison)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

echo "  生成俯视轨迹对比图..."
# ROS REP-103 坐标系 (X 前, Y 左, Z 上): 俯视图是 XY 平面 (不是 XZ, XZ 是侧视图)
# 直接用 matplotlib 画, 避免 evo_traj 引入 seaborn -> pandas -> numpy ABI 冲突
# (Jetson 上 apt 的 python3-pandas 1.3.5 与 pip 的 numpy 2.x 二进制不兼容).
python3 - <<PYEOF
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

trajs = [
    ("${BASELINE}", "baseline",  "tab:blue"),
    ("${IMPROVED}", "improved",  "tab:orange"),
]
if "${HAS_ALWAYS}" == "true":
    trajs.append(("${ALWAYS}",  "always-on", "tab:green"))

fig, ax = plt.subplots(figsize=(10, 8))
for path, label, color in trajs:
    if not os.path.isfile(path):
        continue
    xs, ys = [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) >= 3:
                xs.append(float(parts[1])); ys.append(float(parts[2]))
    if xs:
        ax.plot(xs, ys, '-', color=color, label=f'{label} ({len(xs)} pts)', linewidth=1.5)
        ax.plot(xs[0], ys[0], 'o', color=color, markersize=8, markeredgecolor='black')
        ax.plot(xs[-1], ys[-1], 's', color=color, markersize=8, markeredgecolor='black')

ax.set_aspect('equal', adjustable='datalim')
ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
ax.set_title('Trajectory comparison (XY top-down, ROS REP-103)')
ax.legend(loc='best'); ax.grid(True, linestyle=':', alpha=0.5)
fig.tight_layout()
fig.savefig("${RESULTS_DIR}/trajectory_comparison.png", dpi=150)
plt.close(fig)
print(f"  保存: ${RESULTS_DIR}/trajectory_comparison.png")
PYEOF
echo ""

# ==================================================================
# 阶段 4: 轨迹一致性 RPE (合理用法: 衡量两算法局部行为差异)
# ==================================================================
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "[4/5] 轨迹一致性 RPE (Relative Pose Error)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  说明: RPE 衡量两条轨迹在局部段内的运动差异。"
echo "        在正常场景中 RPE 应很小 (两算法行为一致);"
echo "        在退化段 RPE 会增大 (baseline 漂移, improved 稳定)."
echo ""

evo_rpe tum "${BASELINE}" "${IMPROVED}" -va \
    --delta 1 --delta_unit m \
    --save_results "${RESULTS_DIR}/rpe_consistency.zip" \
    2>&1 | tee "${RESULTS_DIR}/rpe_consistency.log" || echo "  [WARN] evo_rpe 失败, 跳过"

RPE_RMSE="N/A"
if [ -f "${RESULTS_DIR}/rpe_consistency.zip" ]; then
    RPE_RMSE=$(extract_evo_rmse "${RESULTS_DIR}/rpe_consistency.zip")
fi

RPE_ALWAYS_RMSE="N/A"
if ${HAS_ALWAYS}; then
    echo ""
    echo "  额外计算: 退化感知 vs 常开强约束 的局部差异"
    evo_rpe tum "${IMPROVED}" "${ALWAYS}" -va \
        --delta 1 --delta_unit m \
        --save_results "${RESULTS_DIR}/rpe_improved_vs_always.zip" \
        2>&1 | tee "${RESULTS_DIR}/rpe_improved_vs_always.log" || echo "  [WARN] evo_rpe 常开对比失败, 跳过"
    if [ -f "${RESULTS_DIR}/rpe_improved_vs_always.zip" ]; then
        RPE_ALWAYS_RMSE=$(extract_evo_rmse "${RESULTS_DIR}/rpe_improved_vs_always.zip")
    fi
fi
echo ""

# ==================================================================
# 阶段 5: 退化触发率 (改进版必需; 常开强约束可选)
#
# 关键验证指标: 若触发率为 0, 说明退化阈值过严(feat_threshold 太低 /
# residual_threshold 太高), 改进版实际上从未激活里程计强约束, 此时
# 与基线的漂移差距几乎全部来自"常开弱约束", 论文立论会站不住脚.
# 期望值: 走廊消融实验中 20%-60% 触发率为合理区间 (与 algorithm_design.md 一致).
# ==================================================================
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "[5/6] 退化触发率 (Degradation Trigger Rate)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

TRIGGER_RATE="N/A"
DEGRADED_FRAMES="N/A"
TOTAL_FRAMES="N/A"
ALWAYS_TRIGGER_RATE="N/A"
ALWAYS_DEGRADED_FRAMES="N/A"
ALWAYS_TOTAL_FRAMES="N/A"
if [ -f "${IMPROVED_LOG}" ]; then
    # 抓取最后一条 [OdomStat] 汇报行, 形如:
    #   [OdomStat] degraded=123/456 (27.0%), feat_num=..., res_mean=...
    LAST_STAT=$(grep '\[OdomStat\]' "${IMPROVED_LOG}" | tail -n 1)
    if [ -n "${LAST_STAT}" ]; then
        DEGRADED_FRAMES=$(echo "${LAST_STAT}" | sed -n 's/.*degraded=\([0-9]*\)\/[0-9]*.*/\1/p')
        TOTAL_FRAMES=$(echo   "${LAST_STAT}" | sed -n 's/.*degraded=[0-9]*\/\([0-9]*\).*/\1/p')
        TRIGGER_RATE=$(echo   "${LAST_STAT}" | sed -n 's/.*(\([0-9.]*\)%).*/\1/p')
        echo "  最后一次汇报: ${LAST_STAT}"
        echo "  退化帧 / 总约束帧 = ${DEGRADED_FRAMES} / ${TOTAL_FRAMES}"
        echo "  触发率           = ${TRIGGER_RATE}%"
        # 异常区间提示 (区间与 algorithm_design.md / 实验手册 §7.6 保持一致)
        python3 -c "
rate = float('${TRIGGER_RATE}')
if rate < 20.0:
    print('  [WARN] 触发率过低 (<20%): 退化判据偏严, 创新只在很短窗口生效')
    print('         → 建议提高 feat_threshold 或降低 residual_threshold 后重跑')
elif rate > 60.0:
    print('  [WARN] 触发率过高 (>60%): 退化判据过于敏感, 接近全程强约束, 失去"仅退化时增强"语义')
    print('         → 建议降低 feat_threshold 或提高 residual_threshold 后重跑')
else:
    print('  [OK]   触发率在合理区间 (20-60%)')
" 2>/dev/null || true
    else
        echo "  [WARN] 日志中未找到 [OdomStat] 行"
        echo "         请确认改进版用 odom_constraint:=true 启动, 且日志包含 RCLCPP_INFO 输出"
    fi
else
    echo "  [跳过] 未找到 ${IMPROVED_LOG}"
    echo "  生成方法: 在 §7.4 改进版回放时, 给 FAST-LIO 终端加 tee:"
    echo "    ros2 launch go1_bringup go1_replay.launch.py odom_constraint:=true \\"
    echo "        2>&1 | tee ${IMPROVED_LOG}"
fi
if ${HAS_ALWAYS}; then
    echo ""
    echo "  --- 常开强约束日志 ---"
    if [ -f "${ALWAYS_LOG}" ]; then
        LAST_ALWAYS_STAT=$(grep '\[OdomStat\]' "${ALWAYS_LOG}" | tail -n 1)
        if [ -n "${LAST_ALWAYS_STAT}" ]; then
            ALWAYS_DEGRADED_FRAMES=$(echo "${LAST_ALWAYS_STAT}" | sed -n 's/.*degraded=\([0-9]*\)\/[0-9]*.*/\1/p')
            ALWAYS_TOTAL_FRAMES=$(echo   "${LAST_ALWAYS_STAT}" | sed -n 's/.*degraded=[0-9]*\/\([0-9]*\).*/\1/p')
            ALWAYS_TRIGGER_RATE=$(echo   "${LAST_ALWAYS_STAT}" | sed -n 's/.*(\([0-9.]*\)%).*/\1/p')
            echo "  最后一次汇报: ${LAST_ALWAYS_STAT}"
            echo "  常开版触发率   = ${ALWAYS_TRIGGER_RATE}%"
            echo "  说明: force_degraded:=true 时该值应接近 100%, 用来证明常开强约束实验配置生效。"
        else
            echo "  [WARN] 常开日志中未找到 [OdomStat] 行"
        fi
    else
        echo "  [跳过] 未找到 ${ALWAYS_LOG}"
        echo "  生成方法: ros2 launch go1_bringup go1_replay.launch.py odom_constraint:=true force_degraded:=true \\"
        echo "        2>&1 | tee ${ALWAYS_LOG}"
    fi
fi
echo ""

# ==================================================================
# 阶段 6: 退化触发空间分布图 (改进版 / 常开版)
#
# 关键验证: 触发率 X% 可能集中在退化走廊段, 也可能全程随机抖动.
# 论文需要证明前者. 本阶段把 [OdomFrame] 行解析为 (x, y, deg) 散点,
# 退化点红色叠加在 XY 轨迹上, 期望看到红点聚集在长廊段.
# ==================================================================
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "[6/6] 退化触发空间分布 (Degradation Spatial Distribution)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

plot_degradation_map() {
    local log_file=$1
    local traj_file=$2
    local label=$3
    local out_png=$4
    if [ ! -f "${log_file}" ]; then
        echo "  [跳过] ${label}: 未找到 ${log_file}"
        return
    fi
    python3 - <<PYEOF
import re, sys, os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

log_path = "${log_file}"
traj_path = "${traj_file}"
out_path  = "${out_png}"
label     = "${label}"

pat = re.compile(r"\[OdomFrame\]\s+t=([\-0-9.]+)\s+deg=([01])\s+x=([\-0-9.]+)\s+y=([\-0-9.]+)")
deg_xy, ok_xy = [], []
with open(log_path, errors='ignore') as f:
    for line in f:
        m = pat.search(line)
        if not m:
            continue
        x, y = float(m.group(3)), float(m.group(4))
        if m.group(2) == '1':
            deg_xy.append((x, y))
        else:
            ok_xy.append((x, y))

n_deg, n_ok = len(deg_xy), len(ok_xy)
if n_deg + n_ok == 0:
    print(f"  [WARN] {label}: 日志中未找到 [OdomFrame] 行 (需重编译 FAST_LIO)")
    sys.exit(0)

# 轨迹 (TUM: t x y z qx qy qz qw)
tx, ty = [], []
if os.path.isfile(traj_path):
    with open(traj_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) >= 3:
                tx.append(float(parts[1])); ty.append(float(parts[2]))

fig, ax = plt.subplots(figsize=(8, 8))
if tx:
    ax.plot(tx, ty, '-', color='gray', linewidth=1.0, label='trajectory', alpha=0.7)
if ok_xy:
    xs, ys = zip(*ok_xy)
    ax.scatter(xs, ys, s=4, c='tab:blue', alpha=0.4, label=f'normal ({n_ok})')
if deg_xy:
    xs, ys = zip(*deg_xy)
    ax.scatter(xs, ys, s=10, c='tab:red', alpha=0.85, label=f'degraded ({n_deg})')

rate = 100.0 * n_deg / max(1, n_deg + n_ok)
ax.set_aspect('equal', adjustable='datalim')
ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
ax.set_title(f'{label}: degradation spatial distribution ({rate:.1f}% triggered)')
ax.legend(loc='best'); ax.grid(True, linestyle=':', alpha=0.5)
fig.tight_layout()
fig.savefig(out_path, dpi=150)
plt.close(fig)
print(f"  {label}: deg={n_deg}/{n_deg+n_ok} ({rate:.1f}%), 图: {out_path}")
PYEOF
}

plot_degradation_map "${IMPROVED_LOG}" "${IMPROVED}" "improved" \
    "${RESULTS_DIR}/degradation_spatial_improved.png"
if ${HAS_ALWAYS}; then
    plot_degradation_map "${ALWAYS_LOG}" "${ALWAYS}" "always-on" \
        "${RESULTS_DIR}/degradation_spatial_always.png"
fi
echo ""

# ==================================================================
# 汇总报告 (适合直接粘贴到论文)
# ==================================================================
echo ""
echo "╔══════════════════════════════════════════════════╗"
echo "║          消融实验评估汇总 (单次实验)             ║"
echo "╚══════════════════════════════════════════════════╝"
echo ""
if ${HAS_ALWAYS}; then
    printf "%-24s | %-14s | %-14s | %-14s\n" "指标" "基线(无融合)" "退化感知" "常开强约束"
    printf "%-24s-+-%-14s-+-%-14s-+-%-14s\n" "------------------------" "--------------" "--------------" "--------------"
    printf "%-24s | %-14s | %-14s | %-14s\n" "平面闭环漂移 (m)" "${DRIFT_BASELINE}" "${DRIFT_IMPROVED}" "${DRIFT_ALWAYS}"
    printf "%-24s | %-14s | %-14s | %-14s\n" "轨迹长度 (m)" "${LEN_BASELINE}" "${LEN_IMPROVED}" "${LEN_ALWAYS}"
else
    printf "%-24s | %-14s | %-14s\n" "指标" "基线(无融合)" "改进版(有融合)"
    printf "%-24s-+-%-14s-+-%-14s\n" "------------------------" "--------------" "--------------"
    printf "%-24s | %-14s | %-14s\n" "平面闭环漂移 (m)" "${DRIFT_BASELINE}" "${DRIFT_IMPROVED}"
    printf "%-24s | %-14s | %-14s\n" "轨迹长度 (m)" "${LEN_BASELINE}" "${LEN_IMPROVED}"
fi

# 漂移率
python3 -c "
b_drift, b_len = '${DRIFT_BASELINE}', '${LEN_BASELINE}'
i_drift, i_len = '${DRIFT_IMPROVED}', '${LEN_IMPROVED}'
a_drift, a_len = '${DRIFT_ALWAYS}', '${LEN_ALWAYS}'
has_always = '${HAS_ALWAYS}' == 'true'
b_rate = f'{float(b_drift)/float(b_len)*100:.2f}%' if b_drift != 'N/A' and b_len != 'N/A' and float(b_len)>0 else 'N/A'
i_rate = f'{float(i_drift)/float(i_len)*100:.2f}%' if i_drift != 'N/A' and i_len != 'N/A' and float(i_len)>0 else 'N/A'
a_rate = f'{float(a_drift)/float(a_len)*100:.2f}%' if a_drift != 'N/A' and a_len != 'N/A' and float(a_len)>0 else 'N/A'
if has_always:
    print(f\"{'平面漂移率 (漂移/总长)':<24s} | {b_rate:<14s} | {i_rate:<14s} | {a_rate:<14s}\")
else:
    print(f\"{'平面漂移率 (漂移/总长)':<24s} | {b_rate:<14s} | {i_rate:<14s}\")
"
printf "%-24s | %-14s\n" "轨迹一致性 RPE RMSE (m)" "${RPE_RMSE}"
if ${HAS_ALWAYS}; then
    printf "%-24s | %-14s\n" "退化感知-常开 RPE (m)" "${RPE_ALWAYS_RMSE}"
fi
printf "%-24s | %-14s\n" "退化触发率 (改进版)"    "${TRIGGER_RATE}%"
printf "%-24s | %-14s\n" "退化帧/总帧 (改进版)"  "${DEGRADED_FRAMES}/${TOTAL_FRAMES}"
if ${HAS_ALWAYS}; then
    printf "%-24s | %-14s\n" "常开触发率" "${ALWAYS_TRIGGER_RATE}%"
    printf "%-24s | %-14s\n" "常开退化帧/总帧" "${ALWAYS_DEGRADED_FRAMES}/${ALWAYS_TOTAL_FRAMES}"
fi

echo ""
echo "说明:"
echo "  - 平面闭环漂移: 闭合回路首尾 XY 位移差, 越小越好"
echo "  - 漂移率: 平面闭环漂移/轨迹总长, 反映累积漂移速率"
echo "  - RPE: 两条轨迹局部段运动差异, 退化段越大说明两种方法差异越大"
echo "  - 退化触发率: 改进版运行期间 feat_num<阈值 或 res_mean>阈值 的帧占比"
echo "                触发率=0% 意味创新未激活, 必须排查阈值"
echo ""

# 保存汇总到文件
SUMMARY_FILE="${RESULTS_DIR}/summary.txt"
{
    echo "SLAM 消融实验评估汇总"
    echo "日期: $(date '+%Y-%m-%d %H:%M')"
    echo "轨迹目录: ${TRAJ_DIR}"
    echo ""
    if ${HAS_ALWAYS}; then
        printf "%-24s | %-14s | %-14s | %-14s\n" "指标" "基线(无融合)" "退化感知" "常开强约束"
        printf "%-24s-+-%-14s-+-%-14s-+-%-14s\n" "------------------------" "--------------" "--------------" "--------------"
        printf "%-24s | %-14s | %-14s | %-14s\n" "平面闭环漂移 (m)" "${DRIFT_BASELINE}" "${DRIFT_IMPROVED}" "${DRIFT_ALWAYS}"
        printf "%-24s | %-14s | %-14s | %-14s\n" "轨迹长度 (m)" "${LEN_BASELINE}" "${LEN_IMPROVED}" "${LEN_ALWAYS}"
    else
        printf "%-24s | %-14s | %-14s\n" "指标" "基线(无融合)" "改进版(有融合)"
        printf "%-24s-+-%-14s-+-%-14s\n" "------------------------" "--------------" "--------------"
        printf "%-24s | %-14s | %-14s\n" "平面闭环漂移 (m)" "${DRIFT_BASELINE}" "${DRIFT_IMPROVED}"
        printf "%-24s | %-14s | %-14s\n" "轨迹长度 (m)" "${LEN_BASELINE}" "${LEN_IMPROVED}"
    fi
    printf "%-24s | %-14s\n" "轨迹一致性 RPE RMSE (m)" "${RPE_RMSE}"
    if ${HAS_ALWAYS}; then
        printf "%-24s | %-14s\n" "退化感知-常开 RPE (m)" "${RPE_ALWAYS_RMSE}"
    fi
    printf "%-24s | %-14s\n" "退化触发率 (改进版)"    "${TRIGGER_RATE}%"
    printf "%-24s | %-14s\n" "退化帧/总帧 (改进版)"  "${DEGRADED_FRAMES}/${TOTAL_FRAMES}"
    if ${HAS_ALWAYS}; then
        printf "%-24s | %-14s\n" "常开触发率" "${ALWAYS_TRIGGER_RATE}%"
        printf "%-24s | %-14s\n" "常开退化帧/总帧" "${ALWAYS_DEGRADED_FRAMES}/${ALWAYS_TOTAL_FRAMES}"
    fi
} > "${SUMMARY_FILE}"

echo "=================================================="
echo " 评估完成！结果保存在: ${RESULTS_DIR}/"
echo " 汇总报告: ${SUMMARY_FILE}"
echo "=================================================="
echo ""
echo "多次实验汇总: 对 exp1/exp2/exp3 分别跑本脚本, 手动算均值±标准差填入论文表格。"

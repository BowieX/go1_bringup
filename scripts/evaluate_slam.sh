#!/bin/bash
# =============================================================================
# SLAM 消融实验评估脚本
#
# 评估指标 (均不依赖外部动捕系统):
#   1. 闭环漂移 — 闭合回路首尾位移差 (m)，主指标
#   2. 已知几何误差 — SLAM 轨迹中两点距离 vs 卷尺实测距离 (m)
#   3. 轨迹对比图 — evo_traj 俯视图，定性对比
#   4. 轨迹一致性 RPE — 基线 vs 改进版的相对位姿误差 (m)
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
#   <dir>/geometry_constraints.csv    — 已知几何距离 (可选, 见下方格式说明)
#   <dir>/fastlio_improved.log        — 改进版 FAST-LIO 终端日志 (可选, 用于退化触发率统计)
#                                       生成方法: 在 §7.4 回放时 `2>&1 | tee <dir>/fastlio_improved.log`
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
GEOMETRY="${TRAJ_DIR}/geometry_constraints.csv"
IMPROVED_LOG="${TRAJ_DIR}/fastlio_improved.log"

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
    echo "  4. 两次回放都用 record_trajectory.py 记录轨迹 (加 -p experiment_label:=baseline/improved)"
    exit 1
fi

# ------------------------------------------------------------------
# 工具函数: 计算闭环漂移 (TUM 轨迹首尾位置欧氏距离)
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
    dz = float(last[3]) - float(first[3])
    drift = math.sqrt(dx*dx + dy*dy + dz*dz)
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
# 阶段 1: 闭环漂移 (核心指标)
# ==================================================================
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "[1/5] 闭环漂移 (Loop Closure Drift)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

DRIFT_BASELINE=$(calc_loop_closure_drift "${BASELINE}")
DRIFT_IMPROVED=$(calc_loop_closure_drift "${IMPROVED}")
LEN_BASELINE=$(calc_trajectory_length "${BASELINE}")
LEN_IMPROVED=$(calc_trajectory_length "${IMPROVED}")

echo "  基线:   闭环漂移 = ${DRIFT_BASELINE} m  (轨迹长度 ${LEN_BASELINE} m)"
echo "  改进版: 闭环漂移 = ${DRIFT_IMPROVED} m  (轨迹长度 ${LEN_IMPROVED} m)"

# 计算漂移率 (漂移/轨迹长度 × 100%)
python3 -c "
b_drift, b_len = '${DRIFT_BASELINE}', '${LEN_BASELINE}'
i_drift, i_len = '${DRIFT_IMPROVED}', '${LEN_IMPROVED}'
if b_drift != 'N/A' and b_len != 'N/A' and float(b_len) > 0:
    print(f'  基线漂移率:   {float(b_drift)/float(b_len)*100:.2f}%')
if i_drift != 'N/A' and i_len != 'N/A' and float(i_len) > 0:
    print(f'  改进版漂移率: {float(i_drift)/float(i_len)*100:.2f}%')
if b_drift != 'N/A' and i_drift != 'N/A' and float(b_drift) > 0:
    reduction = (1.0 - float(i_drift)/float(b_drift)) * 100
    print(f'  漂移降低:     {reduction:.1f}%')
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
evo_traj tum "${BASELINE}" "${IMPROVED}" \
    --plot_mode xz \
    --save_plot "${RESULTS_DIR}/trajectory_comparison.png" \
    2>&1 | tee "${RESULTS_DIR}/traj_comparison.log" || echo "  [WARN] evo_traj 失败, 跳过"
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
echo ""

# ==================================================================
# 阶段 5: 退化触发率 (仅改进版; 需要 fastlio_improved.log)
#
# 关键验证指标: 若触发率为 0, 说明退化阈值过严(feat_threshold 太低 /
# residual_threshold 太高), 改进版实际上从未激活里程计强约束, 此时
# 与基线的漂移差距几乎全部来自"常开弱约束", 论文立论会站不住脚.
# 期望值: 走廊消融实验中 20%-60% 触发率为合理区间.
# ==================================================================
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "[5/5] 退化触发率 (Degradation Trigger Rate)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

TRIGGER_RATE="N/A"
DEGRADED_FRAMES="N/A"
TOTAL_FRAMES="N/A"
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
        # 异常区间提示
        python3 -c "
rate = float('${TRIGGER_RATE}')
if rate < 5.0:
    print('  [WARN] 触发率过低 (<5%): 退化判据几乎没触发, 创新未生效')
    print('         → 建议降低 feat_threshold 或 residual_threshold 后重跑')
elif rate > 80.0:
    print('  [WARN] 触发率过高 (>80%): 退化判据过于敏感, 几乎全程强约束')
    print('         → 建议提高 feat_threshold 或 residual_threshold 后重跑')
else:
    print('  [OK]   触发率在合理区间 (5-80%)')
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
echo ""

# ==================================================================
# 汇总报告 (适合直接粘贴到论文)
# ==================================================================
echo ""
echo "╔══════════════════════════════════════════════════╗"
echo "║          消融实验评估汇总 (单次实验)             ║"
echo "╚══════════════════════════════════════════════════╝"
echo ""
printf "%-24s | %-14s | %-14s\n" "指标" "基线(无融合)" "改进版(有融合)"
printf "%-24s-+-%-14s-+-%-14s\n" "------------------------" "--------------" "--------------"
printf "%-24s | %-14s | %-14s\n" "闭环漂移 (m)" "${DRIFT_BASELINE}" "${DRIFT_IMPROVED}"
printf "%-24s | %-14s | %-14s\n" "轨迹长度 (m)" "${LEN_BASELINE}" "${LEN_IMPROVED}"

# 漂移率
python3 -c "
b_drift, b_len = '${DRIFT_BASELINE}', '${LEN_BASELINE}'
i_drift, i_len = '${DRIFT_IMPROVED}', '${LEN_IMPROVED}'
b_rate = f'{float(b_drift)/float(b_len)*100:.2f}%' if b_drift != 'N/A' and b_len != 'N/A' and float(b_len)>0 else 'N/A'
i_rate = f'{float(i_drift)/float(i_len)*100:.2f}%' if i_drift != 'N/A' and i_len != 'N/A' and float(i_len)>0 else 'N/A'
print(f\"{'漂移率 (漂移/总长)':<24s} | {b_rate:<14s} | {i_rate:<14s}\")
"
printf "%-24s | %-14s\n" "轨迹一致性 RPE RMSE (m)" "${RPE_RMSE}"
printf "%-24s | %-14s\n" "退化触发率 (改进版)"    "${TRIGGER_RATE}%"
printf "%-24s | %-14s\n" "退化帧/总帧 (改进版)"  "${DEGRADED_FRAMES}/${TOTAL_FRAMES}"

echo ""
echo "说明:"
echo "  - 闭环漂移: 闭合回路首尾欧氏距离, 越小越好"
echo "  - 漂移率: 闭环漂移/轨迹总长, 反映累积漂移速率"
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
    printf "%-24s | %-14s | %-14s\n" "指标" "基线(无融合)" "改进版(有融合)"
    printf "%-24s-+-%-14s-+-%-14s\n" "------------------------" "--------------" "--------------"
    printf "%-24s | %-14s | %-14s\n" "闭环漂移 (m)" "${DRIFT_BASELINE}" "${DRIFT_IMPROVED}"
    printf "%-24s | %-14s | %-14s\n" "轨迹长度 (m)" "${LEN_BASELINE}" "${LEN_IMPROVED}"
    printf "%-24s | %-14s\n" "轨迹一致性 RPE RMSE (m)" "${RPE_RMSE}"
    printf "%-24s | %-14s\n" "退化触发率 (改进版)"    "${TRIGGER_RATE}%"
    printf "%-24s | %-14s\n" "退化帧/总帧 (改进版)"  "${DEGRADED_FRAMES}/${TOTAL_FRAMES}"
} > "${SUMMARY_FILE}"

echo "=================================================="
echo " 评估完成！结果保存在: ${RESULTS_DIR}/"
echo " 汇总报告: ${SUMMARY_FILE}"
echo "=================================================="
echo ""
echo "多次实验汇总: 对 exp1/exp2/exp3 分别跑本脚本, 手动算均值±标准差填入论文表格。"

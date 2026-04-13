#!/bin/bash
# =============================================================================
# SLAM 轨迹精度评估脚本 (消融实验版)
# 用于对比 FAST-LIO2 基线 (无融合) 与改进版 (融合里程计约束) 的轨迹精度
#
# 前置条件:
#   pip3 install evo --upgrade
#
# 使用方式:
#   ./evaluate_slam.sh <trajectories_dir>
#   例如: ./evaluate_slam.sh ~/go1_ws/trajectories
#
# 输入文件 (TUM 格式，由 record_trajectory.py 生成):
#   <dir>/traj_fastlio_baseline.txt   — 基线 FAST-LIO2 (关闭odom约束)
#   <dir>/traj_fastlio_improved.txt   — 改进 FAST-LIO2 (开启odom约束)
#   <dir>/traj_reference.txt          — 参考轨迹 (可选，如果有外部真值)
#
# 输出:
#   <dir>/results/ 目录下的对比图表、CSV 数据、闭环漂移报告
# =============================================================================

set -e

# 参数检查
TRAJ_DIR="${1:-$HOME/go1_ws/trajectories}"
RESULTS_DIR="${TRAJ_DIR}/results"
mkdir -p "${RESULTS_DIR}"

BASELINE="${TRAJ_DIR}/traj_fastlio_baseline.txt"
IMPROVED="${TRAJ_DIR}/traj_fastlio_improved.txt"
REFERENCE="${TRAJ_DIR}/traj_reference.txt"

echo "=================================================="
echo " SLAM 轨迹评估 (消融实验)"
echo "=================================================="
echo " 轨迹目录: ${TRAJ_DIR}"
echo " 结果输出: ${RESULTS_DIR}"
echo ""

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

# 工具函数: 从 evo 结果 zip 中提取 RMSE 和 Max
extract_evo_stats() {
    local zip_file=$1
    python3 -c "
import json, zipfile, sys
try:
    with zipfile.ZipFile('${zip_file}') as zf:
        with zf.open('stats.json') as f:
            stats = json.load(f)
    print(f\"{stats.get('rmse', 'N/A'):.4f} {stats.get('max', 'N/A'):.4f} {stats.get('mean', 'N/A'):.4f}\")
except Exception as e:
    print('N/A N/A N/A')
"
}

# ------------------------------------------------------------------
# 阶段 0: 闭环漂移计算 (不需要 ground truth)
# ------------------------------------------------------------------
echo "[0/4] 计算闭环漂移..."
DRIFT_BASELINE="N/A"
DRIFT_IMPROVED="N/A"

if [ -f "${BASELINE}" ]; then
    DRIFT_BASELINE=$(calc_loop_closure_drift "${BASELINE}")
    echo "  基线闭环漂移:   ${DRIFT_BASELINE} m"
fi

if [ -f "${IMPROVED}" ]; then
    DRIFT_IMPROVED=$(calc_loop_closure_drift "${IMPROVED}")
    echo "  改进版闭环漂移: ${DRIFT_IMPROVED} m"
fi
echo ""

# ------------------------------------------------------------------
# 模式1: 有外部参考轨迹 (ground truth) — 计算绝对/相对位姿误差
# ------------------------------------------------------------------
if [ -f "${REFERENCE}" ]; then
    echo "[1/4] 计算基线 APE (绝对位姿误差)..."
    evo_ape tum "${REFERENCE}" "${BASELINE}" -va \
        --plot_mode xz \
        --save_results "${RESULTS_DIR}/ape_baseline.zip" \
        --save_plot "${RESULTS_DIR}/ape_baseline.png" \
        2>&1 | tee "${RESULTS_DIR}/ape_baseline.log"

    echo ""
    echo "[2/4] 计算改进版 APE..."
    evo_ape tum "${REFERENCE}" "${IMPROVED}" -va \
        --plot_mode xz \
        --save_results "${RESULTS_DIR}/ape_improved.zip" \
        --save_plot "${RESULTS_DIR}/ape_improved.png" \
        2>&1 | tee "${RESULTS_DIR}/ape_improved.log"

    echo ""
    echo "[3/4] 计算 RPE (相对位姿误差, delta=1m)..."
    evo_rpe tum "${REFERENCE}" "${BASELINE}" -va \
        --delta 1 --delta_unit m \
        --save_results "${RESULTS_DIR}/rpe_baseline.zip" \
        2>&1 | tee "${RESULTS_DIR}/rpe_baseline.log"

    evo_rpe tum "${REFERENCE}" "${IMPROVED}" -va \
        --delta 1 --delta_unit m \
        --save_results "${RESULTS_DIR}/rpe_improved.zip" \
        2>&1 | tee "${RESULTS_DIR}/rpe_improved.log"

    echo ""
    echo "[4/4] 对比分析..."
    evo_res "${RESULTS_DIR}/ape_baseline.zip" "${RESULTS_DIR}/ape_improved.zip" -p \
        --save_table "${RESULTS_DIR}/comparison_ape.csv" \
        --save_plot "${RESULTS_DIR}/comparison_ape.png" \
        2>&1 | tee "${RESULTS_DIR}/comparison.log"

# ------------------------------------------------------------------
# 模式2: 无参考轨迹 — 基线与改进版直接对比
# ------------------------------------------------------------------
else
    echo "[INFO] 未找到参考轨迹 (${REFERENCE})，将进行基线与改进版的直接对比"
    echo ""

    if [ -f "${BASELINE}" ] && [ -f "${IMPROVED}" ]; then
        echo "[1/3] 绘制轨迹对比图..."
        evo_traj tum "${BASELINE}" "${IMPROVED}" -p \
            --plot_mode xz \
            --ref "${BASELINE}" \
            --save_plot "${RESULTS_DIR}/trajectory_comparison.png" \
            2>&1 | tee "${RESULTS_DIR}/traj_comparison.log"

        echo ""
        echo "[2/3] 计算改进版相对于基线的 APE..."
        evo_ape tum "${BASELINE}" "${IMPROVED}" -va \
            --plot_mode xz \
            --save_results "${RESULTS_DIR}/ape_improved_vs_baseline.zip" \
            --save_plot "${RESULTS_DIR}/ape_improved_vs_baseline.png" \
            2>&1 | tee "${RESULTS_DIR}/ape_vs_baseline.log"

        echo ""
        echo "[3/3] 计算 RPE (相对位姿误差, delta=1m)..."
        evo_rpe tum "${BASELINE}" "${IMPROVED}" -va \
            --delta 1 --delta_unit m \
            --save_results "${RESULTS_DIR}/rpe_improved_vs_baseline.zip" \
            2>&1 | tee "${RESULTS_DIR}/rpe_vs_baseline.log"
    else
        echo "[ERROR] 缺少轨迹文件！请确保以下文件存在:"
        echo "  ${BASELINE}"
        echo "  ${IMPROVED}"
        echo ""
        echo "生成方法 (rosbag 消融实验):"
        echo "  1. 录制 rosbag: ros2 bag record /livox/lidar /livox/imu /odom /imu -o <name>"
        echo "  2. 回放基线:   ros2 launch go1_bringup go1_replay.launch.py odom_constraint:=false"
        echo "  3. 回放改进版: ros2 launch go1_bringup go1_replay.launch.py odom_constraint:=true"
        echo "  4. 两次回放都用 record_trajectory.py 记录轨迹"
        echo "  5. 重命名为 traj_fastlio_baseline.txt / traj_fastlio_improved.txt"
        exit 1
    fi
fi

# ------------------------------------------------------------------
# 汇总报告 (适合直接粘贴到论文)
# ------------------------------------------------------------------
echo ""
echo "=================================================="
echo " 消融实验评估汇总"
echo "=================================================="
echo ""

# 尝试提取 evo 统计数据
APE_B_STATS="N/A N/A N/A"
APE_I_STATS="N/A N/A N/A"
RPE_B_STATS="N/A N/A N/A"
RPE_I_STATS="N/A N/A N/A"

if [ -f "${RESULTS_DIR}/ape_baseline.zip" ]; then
    APE_B_STATS=$(extract_evo_stats "${RESULTS_DIR}/ape_baseline.zip")
fi
if [ -f "${RESULTS_DIR}/ape_improved.zip" ]; then
    APE_I_STATS=$(extract_evo_stats "${RESULTS_DIR}/ape_improved.zip")
fi
if [ -f "${RESULTS_DIR}/ape_improved_vs_baseline.zip" ]; then
    APE_I_STATS=$(extract_evo_stats "${RESULTS_DIR}/ape_improved_vs_baseline.zip")
fi
if [ -f "${RESULTS_DIR}/rpe_baseline.zip" ]; then
    RPE_B_STATS=$(extract_evo_stats "${RESULTS_DIR}/rpe_baseline.zip")
fi
if [ -f "${RESULTS_DIR}/rpe_improved.zip" ]; then
    RPE_I_STATS=$(extract_evo_stats "${RESULTS_DIR}/rpe_improved.zip")
fi
if [ -f "${RESULTS_DIR}/rpe_improved_vs_baseline.zip" ]; then
    RPE_I_STATS=$(extract_evo_stats "${RESULTS_DIR}/rpe_improved_vs_baseline.zip")
fi

# 输出汇总表
printf "%-20s | %-12s | %-12s\n" "指标" "基线" "改进版"
printf "%-20s-+-%-12s-+-%-12s\n" "--------------------" "------------" "------------"
printf "%-20s | %-12s | %-12s\n" "闭环漂移 (m)" "${DRIFT_BASELINE}" "${DRIFT_IMPROVED}"
printf "%-20s | %-12s | %-12s\n" "APE RMSE (m)" "$(echo ${APE_B_STATS} | awk '{print $1}')" "$(echo ${APE_I_STATS} | awk '{print $1}')"
printf "%-20s | %-12s | %-12s\n" "APE Max (m)" "$(echo ${APE_B_STATS} | awk '{print $2}')" "$(echo ${APE_I_STATS} | awk '{print $2}')"
printf "%-20s | %-12s | %-12s\n" "APE Mean (m)" "$(echo ${APE_B_STATS} | awk '{print $3}')" "$(echo ${APE_I_STATS} | awk '{print $3}')"

# 保存汇总到文件
SUMMARY_FILE="${RESULTS_DIR}/summary.txt"
{
    echo "SLAM 消融实验评估汇总"
    echo "日期: $(date '+%Y-%m-%d %H:%M')"
    echo "轨迹目录: ${TRAJ_DIR}"
    echo ""
    printf "%-20s | %-12s | %-12s\n" "指标" "基线" "改进版"
    printf "%-20s-+-%-12s-+-%-12s\n" "--------------------" "------------" "------------"
    printf "%-20s | %-12s | %-12s\n" "闭环漂移 (m)" "${DRIFT_BASELINE}" "${DRIFT_IMPROVED}"
    printf "%-20s | %-12s | %-12s\n" "APE RMSE (m)" "$(echo ${APE_B_STATS} | awk '{print $1}')" "$(echo ${APE_I_STATS} | awk '{print $1}')"
    printf "%-20s | %-12s | %-12s\n" "APE Max (m)" "$(echo ${APE_B_STATS} | awk '{print $2}')" "$(echo ${APE_I_STATS} | awk '{print $2}')"
    printf "%-20s | %-12s | %-12s\n" "APE Mean (m)" "$(echo ${APE_B_STATS} | awk '{print $3}')" "$(echo ${APE_I_STATS} | awk '{print $3}')"
} > "${SUMMARY_FILE}"

echo ""
echo "=================================================="
echo " 评估完成！结果保存在: ${RESULTS_DIR}/"
echo " 汇总报告: ${SUMMARY_FILE}"
echo "=================================================="

#!/bin/bash
# =============================================================================
# SLAM 轨迹精度评估脚本
# 使用 evo 工具对比 FAST-LIO2 基线与改进版 (融合里程计约束) 的轨迹精度
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
#   <dir>/results/ 目录下的对比图表和 CSV 数据
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
echo " SLAM 轨迹评估 (evo toolkit)"
echo "=================================================="
echo " 轨迹目录: ${TRAJ_DIR}"
echo " 结果输出: ${RESULTS_DIR}"
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
# 模式2: 无参考轨迹 — 仅做基线与改进版的相对对比
# ------------------------------------------------------------------
else
    echo "[INFO] 未找到参考轨迹 (${REFERENCE})，将进行基线与改进版的直接对比"
    echo ""

    if [ -f "${BASELINE}" ] && [ -f "${IMPROVED}" ]; then
        echo "[1/2] 绘制轨迹对比图..."
        evo_traj tum "${BASELINE}" "${IMPROVED}" -p \
            --plot_mode xz \
            --ref "${BASELINE}" \
            --save_plot "${RESULTS_DIR}/trajectory_comparison.png" \
            2>&1 | tee "${RESULTS_DIR}/traj_comparison.log"

        echo ""
        echo "[2/2] 计算改进版相对于基线的 APE..."
        evo_ape tum "${BASELINE}" "${IMPROVED}" -va \
            --plot_mode xz \
            --save_results "${RESULTS_DIR}/ape_improved_vs_baseline.zip" \
            --save_plot "${RESULTS_DIR}/ape_improved_vs_baseline.png" \
            2>&1 | tee "${RESULTS_DIR}/ape_vs_baseline.log"
    else
        echo "[ERROR] 缺少轨迹文件！请确保以下文件存在:"
        echo "  ${BASELINE}"
        echo "  ${IMPROVED}"
        echo ""
        echo "生成方法:"
        echo "  1. 关闭 odom_constraint，运行机器人，记录基线轨迹"
        echo "  2. 开启 odom_constraint，运行机器人，记录改进轨迹"
        echo "  3. 将文件分别重命名为 traj_fastlio_baseline.txt / traj_fastlio_improved.txt"
        exit 1
    fi
fi

echo ""
echo "=================================================="
echo " 评估完成！结果保存在: ${RESULTS_DIR}/"
echo "=================================================="

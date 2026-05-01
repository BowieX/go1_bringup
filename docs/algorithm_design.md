# 算法设计与数学原理

> 本文档服务于毕业论文"方法"章节，说明本项目在 FAST-LIO2 基础上的**核心创新**：
> 基于退化感知的里程计位置约束 (Degradation-aware Odometry Position Constraint)。
>
> 读者: 论文审稿人 / 答辩评委 / 后续工作继承人。
> 不包含: 使用方法（见 [实验手册](实验手册.md)）/ 调参（见 §10.1）/ 链路验证（见 [实机前联调检查清单](实机前联调检查清单.md)）。
>
> **当前状态（2026-04-30）**: 尚未完成实机实验。本文档是算法设计与实验计划；默认阈值为首次实机验证初始值，论文"实验结果"章节必须用后续实机采集数据填充。

---

## 1. 问题背景与动机

### 1.1 FAST-LIO2 的几何退化漏洞

FAST-LIO2 采用 **迭代误差状态卡尔曼滤波 (IESEKF)** 做 LiDAR 点云 ↔ 局部地图的点面 ICP 配准。
其量测雅可比:

```
H_i = ∂(n·(Rp + t))/∂x   (n: 点面法向量; p: 点云位置)
```

在几何退化场景（长走廊、空旷大厅、圆柱通道）下:

- **走廊纵向**（沿廊前进方向）的所有点面法向量近似垂直于行进方向 → `H_i` 沿纵向的分量近零。
- 观测矩阵 `H^T R^{-1} H` 在该方向奇异值趋零 → **不可观性**。
- 滤波器在该方向只靠 IMU 预测 + bias 估计，数秒内累积 0.3-1.0 m 漂移。

### 1.2 宇树 Go1 的互补资源

Go1 提供 50 Hz 的**足端里程计** `/odom`（基于关节编码器 + 足底接触检测的运动学积分）:

- 走廊纵向**可观**（腿的前后摆动能直接测速度）。
- 绝对精度差（脚打滑累计漂移 1-3%/m 常见）。
- 与 LiDAR **误差源正交**: LiDAR 怕环境几何退化、不怕脚打滑; 里程计怕脚打滑、不怕几何退化。

**核心思路**: **仅在 LiDAR 发生退化时** 把里程计位置作为强观测注入 ESEKF, 正常场景保持 LiDAR 主导。

---

## 2. 系统架构

```
┌──────────────┐    /odom      ┌──────────────────────┐   /odometry/filtered  ┌──────────────────────┐
│ unitree_ros  │──50 Hz───────▶│ robot_localization   │──50 Hz───────────────▶│  FAST-LIO2 节点      │
│ (腿部里程计) │               │ EKF (odom + IMU 融合)│                       │  (本项目修改版)       │
└──────────────┘               └──────────────────────┘                       │                      │
┌──────────────┐    /imu                                                      │  ① 点面 ESEKF 更新   │
│ unitree IMU  │──200 Hz──────▶                                               │  ② 退化检测          │
└──────────────┘                                                              │  ③ odom 位置约束更新 │
┌──────────────┐    /livox/lidar + /livox/imu                                 │                      │
│ Livox MID360 │──10 Hz + 200 Hz─────────────────────────────────────────────▶│                      │
└──────────────┘                                                              └──────────┬───────────┘
                                                                                         │
                                                                                         ▼ /Odometry + TF
                                                                                   camera_init → body
```

**为什么用 `robot_localization` 先融合一次?**
- Go1 原生 `/odom` 仅 X/Y 有效（腿式 Z 抖动大），姿态来自 IMU 而非里程计积分。
- `robot_localization` EKF 把 `/odom` + `/imu` 融成一个带**完整 3D pose** 与**连续时间戳**的 `/odometry/filtered`。
- 输出帧保持在 `unitree_odom → unitree_base`（`publish_tf: false`，避免与 FAST-LIO 的
  `camera_init → body` TF 冲突，也不再引入第三个里程计根坐标系）。

---

## 3. 量测模型

### 3.1 状态定义 (FAST-LIO2 原生, 23-DOF on manifold)

| DOF | 符号 | 含义 | 流形 |
|-----|------|------|------|
| 0-2  | `pos`       | 位置 (world)               | ℝ³ |
| 3-5  | `rot`       | 姿态 (world ← imu)         | SO(3) |
| 6-8  | `offset_R`  | LiDAR→IMU 旋转外参         | SO(3) |
| 9-11 | `offset_T`  | LiDAR→IMU 平移外参         | ℝ³ |
| 12-14 | `vel`      | 速度 (world)               | ℝ³ |
| 15-17 | `b_g`      | 陀螺仪零偏                 | ℝ³ |
| 18-20 | `b_a`      | 加计零偏                   | ℝ³ |
| 21-22 | `grav`     | 重力 (world 下的 2-DOF 流形)| S² |

### 3.2 里程计位置观测模型

**观测量**:
```
z = p_odom_xy ∈ ℝ²   (来自 /odometry/filtered 的 X/Y, 单位: 米)
```

**量测函数**:
```
h(x) = [x.pos_x, x.pos_y]^T   (将状态中的平面位置作为预测)
```

**量测雅可比 H (2×23)**:
```
H = [I₂×₂  |  0₂×₂₁]
```

即 XY 位置块为单位阵，其余 21 DOF 均为零 — 该观测**仅约束平面位置**。
Z 不参与约束，因为 Go1 腿部里程计本身不融合 Z，Z 方向主要体现步态颠簸与 IMU
积分误差；走廊退化最主要表现为平面漂移，约束 X/Y 更稳。

> **为何不约束姿态?**
> FAST-LIO 的预积分使用 MID-360S 内置 IMU (`/livox/imu`), `robot_localization`
> 的姿态主要来自 Go1 机体 IMU (`/imu`). 两套 IMU 的安装外参、时间同步与协方差
> 未做严格标定, 若把 EKF 姿态直接作为 FAST-LIO 姿态量测, 反而会引入难以解释的
> 姿态偏置和过度自信。本文只把腿部里程计/EKF 的**平面位置增量**作为几何退化时的
> 外部约束; yaw 仅用于首帧坐标系对齐, 不作为持续姿态观测注入。

### 3.3 量测噪声 R 的切换

```
R = σ² · I₂    其中 σ = { σ_normal  = 10.0   若 非退化
                          σ_degraded = 0.1    若  退化 }
```

实际 Kalman 增益大小:
```
K = P H^T (H P H^T + R)^{-1}   (位置块)
  = P_pos (P_pos + σ²)^{-1}

  σ=10:   K ≈ P_pos / 100      → dx ≈ 0.01·z, 几乎不修正
  σ=0.1:  K ≈ P_pos / (P_pos+0.01) ≈ 0.5-0.9 (在 P_pos = 0.01-0.1 范围内)
                                → dx ≈ 0.5·z, 强力拉回 odom 位置
```

---

## 4. 退化检测

### 4.1 判据（实现版）

```
is_degraded = (effct_feat_num < N_thr)  OR  (res_mean_last > r_thr)

默认: N_thr = 200 帧有效点, r_thr = 0.15 m
```

- **effct_feat_num**: 当前 LiDAR 帧中通过 `esti_plane` 平面度检验 (`s > 0.9`)
  并与地图近邻平面对齐的点数，衡量**可用观测的数量**。
- **res_mean_last**: 这些有效点的平均点面距残差, 衡量**配准质量**。

### 4.2 方案权衡

| 判据 | 理论严谨度 | 实现代价 | 本项目选择 |
|------|:---:|:---:|:---:|
| 奇异值分析 `min σ(H^T R^{-1} H)` | ★★★ | 高 (每帧 23×23 SVD, Jetson 单核吃紧) | 否 |
| 条件数阈值 `cond(H^T H)` | ★★ | 中 (需存 H 矩阵) | 否 |
| **effct_feat_num + res_mean** | ★ | **低 (已有变量, 免费)** | **是** |

本项目创新**不在于退化判据本身**（采用 FAST-LIO 已有诊断量）, 而在于**将判据与里程计约束结合**。
判据的简化会导致**边界行为不精确**（见 §7 局限性），但足以在毕设尺度上验证融合方法的可行性。

---

## 5. ESEKF 位置观测更新

### 5.1 坐标系对齐

FAST-LIO 的位置在 `camera_init` 帧 (其原点为开机时机器人位置, X 轴为 IMU 初始朝向)。
`robot_localization` 输出的位置在 `unitree_odom` 帧 (与原始 `/odom` 一致)。

两者**原点不同**, 初始 yaw 也可能因两套 IMU/驱动定义存在小偏差。当前实现采用
首帧 **SE(2) 初始对齐**: 记录两条轨迹的初始平面位置与 yaw, 后续只使用
`unitree_odom` 下的平面位移增量, 通过初始 yaw 差旋转到 `camera_init` 平面:

```cpp
if (!odom_init_alignment_set) {
    odom_init_pos = p_odom_raw;          // unitree_odom 下的初始位置
    lio_init_pos  = x.pos;               // camera_init 下的初始 FAST-LIO 位置
    odom_init_yaw = yaw(p_odom_raw.q);   // EKF/Unitree 初始 yaw
    lio_init_yaw  = yaw(x.rot);          // FAST-LIO 初始 yaw
    odom_init_alignment_set = true;
    return;   // 首帧不做更新, 仅设置 SE(2) 对齐
}

delta_odom = p_odom_raw.xy - odom_init_pos.xy;
R0 = Rz(lio_init_yaw - odom_init_yaw);
p_odom_aligned.xy = lio_init_pos.xy + R0 * delta_odom;
```

> **启动假设**: 机器人上电后**保持静止 ≥ 1 秒** 直到 FAST-LIO 与 EKF 都出稳定位姿再开始运动。
> 若上电时机器人就在移动或漂移, 初始位置/yaw 对齐会捕获瞬时噪声, 后续约束全程带偏置。
> 详见 §7.5 (含量级估计与缓解方案); 实机前联调检查清单已在 B 阶段明确要求此启动流程。

### 5.2 Kalman 更新方程

**创新协方差**:
```
S = H P Hᵀ + R = P_xy + σ²·I₂            (2×2)
```

**Kalman 增益**:
```
K = P Hᵀ S⁻¹   (23×2)
```

**状态残差与修正**:
```
z_tilde = p_odom_xy_aligned - x.pos_xy     (2×1, 预测残差)
dx      = K · z_tilde                      (23×1, 状态增量)
x_new   = x ⊞ dx                           (流形 boxplus)
```

`boxplus` 运算: 对位置/速度/bias 等 ℝ³ 子空间是普通相加; 对 SO(3) 是
`R_new = exp(dx[3:6]) · R_old`; 对 S² (重力) 是沿切空间扰动。

**协方差更新 (Joseph form, 数值稳定)**:
```
P_new = (I − K H) P (I − K H)ᵀ + K R Kᵀ
```

> **为何用 Joseph form 而非 `P = (I − KH) P`?**
> 后者在数值计算中 K 略有误差时会让 P 失去对称性/正定性, 几次更新后滤波器发散。
> Joseph form 保证数学对称, 即使 K 非最优仍得正定 P — 是工业级滤波器标配。

### 5.3 集成点与顺序

```cpp
kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);  // 点面 ESEKF (原生)

if (odom_constraint_en) {
    bool is_degraded = ...;
    apply_odom_position_constraint(kf, is_degraded, now, timeout);      // 本项目创新
}
```

**先点面, 后里程计** 的理由:
- 点面更新代表 LiDAR 对**局部几何**的最佳估计, 是"主量测"。
- 里程计是"补量测", 只在 LiDAR 能力不足时介入。
- 若反过来, 退化场景里 LiDAR 更新会把 odom 好不容易拉回的位置再次带偏。

---

## 6. 工程防护机制

| 机制 | 代码位置 | 目的 |
|------|----------|------|
| NaN/Inf 过滤 | `odom_cbk` 开头 | EKF 发散时会发 NaN, 直接丢弃不污染缓存 |
| mutex 保护 | `mtx_odom` | 50Hz 回调线程 vs 10Hz LIO 更新线程的数据竞争 |
| Odom 超时 | `age > timeout_sec` | EKF 崩溃后不再用 stale 位置拖偏滤波器 |
| 首帧 SE(2) 对齐 one-shot | `odom_init_alignment_set` | 固定 `unitree_odom` 到 `camera_init` 的初始平面变换, 防止重复触发导致漂移 |
| 触发率统计 | `g_total_constraint_frames`, `g_degraded_frames` | 消融实验可观测性 — 仅统计成功应用 odom 约束的帧, 验证退化判据真的触发了 |

---

## 7. 已知局限与论文讨论要点

### 7.1 退化判据的简化

- 当前判据不区分**退化方向**: 走廊通常主要在纵向退化, 横向仍可能有观测,
  但 XY 约束采用各向同性 `R = σ²I₂`, 这意味着 X/Y 会按同一权重被 odom 拉动。
- **缓解**: 已不再约束 Z; X/Y 方向中, Go1 腿式里程计在低速室内实验的短时漂移
  明显小于长廊 LiDAR 退化漂移, 可接受。
- **论文建议**: 承认此简化, 讨论时提 "进一步工作可用 H^T H 特征分解做方向性约束"。

### 7.2 坐标系 yaw 对齐的局限

- 当前实现已在首帧记录 `lio_init_yaw - odom_init_yaw`, 把腿部里程计的 XY 位移投到
  `camera_init` 平面, 避免了"只平移补偿"在两套 IMU 初始 yaw 不一致时的系统偏差。
- 该 yaw 对齐是**一次性初始对齐**, 不持续使用 EKF/Unitree 的姿态校正 FAST-LIO。
  若长时间运行后 FAST-LIO 或腿部里程计自身发生 yaw/尺度漂移, 两个平面坐标系仍会逐步偏离。
- **量级估计**: 30 m 闭环 + 正常 LiDAR yaw 漂移 ~1° → 位置误差 30·sin(1°) ≈ 0.5 m, 接近漂移总量。
- **论文建议**: 可在讨论章节说明"本文采用初始 SE(2) 对齐, 不进行持续 yaw 观测更新";
  毕设尺度内 (单次实验 < 2 分钟、低速 0.2-0.3 m/s) 影响有限。

### 7.3 时间戳对齐

- `odom_latest_pos` 取最新 EKF 输出, 与当前 LiDAR 帧末时间可能错开 ±20 ms。
- 机器人 0.3 m/s → 6 mm 位置差, 远小于 σ_degraded=0.1 m, 可忽略。
- 若未来速度上调至 1-2 m/s, 需要在 odom_cbk 中维护一个时间戳插值缓冲。

### 7.4 无绝对真值

- 本实验无动捕/RTK 真值, 所有评估指标基于**相对**标准（平面闭环漂移 + 卷尺测已知几何距离）。
- **论文建议**: 明确声明此前提, 避免审稿人质疑; APE 绝不以 baseline 作参考计算。

### 7.5 初始 SE(2) 对齐的"静止启动"假设

- `apply_odom_position_constraint` 在首次收到 odom 时, 用
  `odom_init_pos / lio_init_pos / odom_init_yaw / lio_init_yaw`
  一次性记录 `camera_init` 与 `unitree_odom` 之间的初始 SE(2) 平面关系, 之后所有
  退化时刻的位置观测都按 `lio_init + R0 * (odom - odom_init)` 套到 `camera_init` 系下。
  这隐含了一个假设: **首帧采样时机器人静止**。
- 风险来源是两条滤波器的启动节奏不同步:
  - FAST-LIO IMU 静态初始化阶段会跳过若干帧 (`flg_first_scan` / `flg_EKF_inited` 双门控);
  - `robot_localization` EKF 启动只需第一条 `/odom` 与 `/imu`, 通常更快进入稳态。
  若在 EKF 稳定 → FAST-LIO 进入主循环 这段时间窗内 (实测 0.5–1.5 s) 机器人已经移动,
  那么 FAST-LIO 与 EKF 的"初始点"并不对应同一物理位置, 后续 `odom - odom_init`
  的位移增量会带入这段启动偏置, 体现为系统性偏置 (而非随机噪声)。
- **量级估计**: Go1 平稳遥控启动加速度 ~0.3 m/s², 1 s 偏移量约 0.15 m;
  在 σ_degraded = 0.1 的强约束下, 退化段会被这个偏置稳定拖偏 ~0.1–0.15 m。
  这与"无融合时退化漂移 0.5–1.0 m"相比量级小得多, 但属于**可消除**的系统性误差。
- **现行缓解措施**:
  1. **代码端: 静止启动守卫** (已实现, `apply_odom_position_constraint` 内 "静止启动守卫" 注释段):
     设置初始 SE(2) 对齐之前同时检查 FAST-LIO **平面速度** (`sqrt(vel_x² + vel_y²)`)
     与 odom **平面速度** 都 `< 0.05 m/s`, 任一速度仍 ≥ 5 cm/s 则推迟到下一帧。
     不取 `vel.norm()` (3D) 是因为 Go1 步态会让 Z 速度即使在"完全站立"时也短时震荡到
     0.1-0.2 m/s 量级, 用 3D 范数会让守卫长期挂在"还在动"分支永不对齐;
     而初始 SE(2) 对齐本身只关心平面位姿。
     该守卫把"是否静止"的判断从人工流程下沉到代码, 即使操作者忘记静止 1 s 启动,
     算法本身也会等 LIO/EKF 平面速度都收敛到 < 5 cm/s 后才设置初始对齐;
     如果连续 5 s 仍未收敛 (例如 IMU 线松/EKF 崩溃), 守卫会节流打印诊断警告,
     而不会静默挂死。
  2. **流程保证**: 启动 `go1_base.launch.py use_odom_fusion:=true` 后, 仍建议让机器狗站立静止
     ≥ 1 s 再发 `/cmd_vel`; 守卫只是 fallback, 流程保证可以让首帧对齐发生在 FAST-LIO 完全
     收敛后, 而不是 LIO 速度刚降到 5 cm/s 的边缘瞬间。
- **进一步工作**:
  - **多点对齐**: 使用启动后短时间静止窗口内多帧均值估计初始位置和 yaw,
    比单帧对齐更抗 IMU/odom 瞬时噪声。
  - **更稳健的静止检测**: 引入多帧速度均值或零速检测, 避免单帧速度刚好低于阈值的边缘情况。
- **论文建议**: 在"实现细节 / 系统假设"小节列明此约束并说明已用速度门控缓解,
  与 §7.2 (yaw 对齐局限) 并列; 实验流程章节明确写"采集启动后静止 ≥ 1 s 再开始遥控"。

---

## 8. 工程贡献陈述 (论文摘要可用)

> **定位**: 本节描述本毕业设计在已有开源工作 (FAST-LIO2 / robot_localization) 之上的
> **工程贡献与系统集成**, 不主张全新算法。各点配套数据见实验五/六, 论文撰写时建议
> 与 §4.2 (退化判据选型权衡) 和 §7 (已知局限) 对照, 避免审稿人质疑。

1. **退化感知的腿部里程计 XY 位置约束集成**:
   在 FAST-LIO2 的 IESEKF 框架下, 首帧完成 `unitree_odom` 到 `camera_init` 的 SE(2)
   初始平面对齐, 点面迭代收敛后追加一次基于外部融合里程计平面位移的 XY 位置观测更新
   (量测矩阵 H = [I₂, 0₂×₂₁], Joseph form 协方差更新). 该集成方式在 FAST-LIO2 上游
   未提供, 是本项目针对四足平台几何退化场景的工程改造; 数学公式本身是标准 Kalman 量测更新。
2. **退化判据与噪声切换的联动机制**:
   退化判据复用 FAST-LIO 已有诊断量 (`effct_feat_num` / `res_mean_last`),
   不主张此判据的发明权 (见 §4.2 选型对比); 本项目的工程贡献是把该判据与量测噪声
   `R = σ²·I₂` 的双档切换 (σ_normal=10.0 ↔ σ_degraded=0.1) 联动, 实现"正常场景几乎不干预、
   退化场景强约束"的选择性增强, 并以触发率 (20%-60% 期望区间) 作为可观测验证手段。
3. **与 robot_localization EKF + FAST-LIO2 的无缝集成**:
   两套子模块均为现有开源工作, 本项目对它们的耦合改动 ≈ 100 行 C++ (主要在
   `laserMapping.cpp` 的 odom 回调 + `apply_odom_position_constraint` + 主循环集成点),
   保留了 FAST-LIO2 原有的 ikd-Tree 增量建图与 IMU 预积分能力, 不需重新推导 Jacobian。
4. **面向宇树 Go1 四足平台的完整自主导航系统部署**:
   含 Livox MID-360S 驱动适配、`pointcloud_to_laserscan` 跟随者过滤、PCD → 2D 栅格离线切片
   建图工具链、Nav2 (AMCL + A* + TEB) 全栈集成、消融实验回放与评估脚本; 代码与实验流程开源。

---

## 9. 进一步工作

- **方向性退化判据**: 用 `H^T R^{-1} H` 特征分解, 仅对退化方向注入 odom 约束, 避免对良好方向的"无谓拖动"。
- **Odom 质量自适应**: 根据 Go1 足底接触状态 (`/foot_force`) 动态调整 `odom_noise_degraded` —
  脚打滑时降低里程计权重。
- **与 LIO-SAM 的对比实验**: LIO-SAM 通过闭环检测后处理抑制累积漂移, 本方法为实时在线抑制,
  互补关系值得论文讨论。
- **动捕场地标定**: 若条件允许, 在学校有动捕的实验室补一组 APE 数据作为绝对精度验证。

---

## 参考

- FAST-LIO2 原论文: Xu et al., "FAST-LIO2: Fast Direct LiDAR-Inertial Odometry", IEEE T-RO 2022.
- robot_localization: Moore & Stouch, "A Generalized Extended Kalman Filter Implementation for the ROS", IAS 2014.
- 实现细节: [FAST_LIO/src/laserMapping.cpp](../../FAST_LIO/src/laserMapping.cpp) — 用编辑器搜 `// ====== 里程计位置约束相关变量 ======` 起始的全局变量段、`odom_cbk` 回调、`apply_odom_position_constraint` 核心函数; 主循环集成点搜 `// ==================== 里程计位置约束 (几何退化场景增强) ====================` 注释。(避免使用行号引用, 因为代码迭代会让行号漂移)
- 消融实验流程: [实验手册 §7](实验手册.md)
- 调参闭环: [实验手册 §10.1](实验手册.md)

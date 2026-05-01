# 长廊 rosbag 现场录制流程

> 目标：先稳定录下实验四所需的长廊原始 bag，后续分析、回放、消融在实验室离线完成。
>
> 适用场景：当前优先保证 bag 录制成功，同时在 RViz 里观察 FAST-LIO 实时建图效果；不在现场调试 `robot_localization` / `odom_constraint`。
>
> 录制模式：基础模式 `use_odom_fusion:=false`
>
> 最新结论：带门框、柱子、间隔凸起或明显装饰物的长廊可能并不产生退化。正式录 3 组前，先录 1 组 pilot bag，离线验证默认阈值或候选阈值确实能触发退化，再继续录完整重复数据。

---

## 1. 使用原则

- 启动 `go1_mapping.launch.py` 后，**会自动开始 rosbag 录制，并自动打开 RViz**。
- 机器人启动后**可能自动起立**，这是正常现象。
- **不要一启动就立刻走**；建议机器人站稳后原地静止 `5-10` 秒，再开始正式路线。
- 这几秒静止数据**不需要后续手动剔除**，保留即可。
- 用胶带标出**起点圆点 + 朝向箭头**，终点尽量回到同一位置和朝向，便于闭环漂移解释。
- 现场记录走廊几何和环境特征：长度、宽度、墙面材质、门框/柱子/凸起/玻璃/海报等。
- 正式实验建议先运行 `experiment_log init` 建立会话目录；它只负责建档，不会启动机器人，也不会自动开始录包。
- 每次录完后，**必须立刻执行 `ros2 bag info` 验包**，确认关键话题都录到了。
- 若本次目标只是录 bag，不想在退出时额外等待 PCD 归档，可传 `auto_archive:=false`。

---

## 2. 先判断这条长廊是否适合作为退化主实验

强退化路线优先选择：

- 走廊段尽量长，建议 `>= 30 m`。
- 两侧墙面尽量平整、重复、少门框、少柱子、少凸起。
- 地面和墙面纹理少，避免密集海报、桌椅、堆放物。
- 尽量避开上次 `exp5/6/7` 那类带间隔凸起、局部特征丰富的长廊。
- 能形成闭环：从起点出发，走完整条长廊，到尽头后原路返回起点。

现场不要只凭“很长”判断退化。长廊如果有周期凸起、门洞、消防箱、柱子等，FAST-LIO 可能仍然稳定，默认阈值触发率会接近 0。

推荐决策：

1. 先录 `degradation_pilot_<地点>` 一组完整往返 bag。
2. 回实验室立即离线跑一次 baseline / improved / always-on。
3. 若默认 `feat=200,res=0.15` 触发率仍为 0，可再跑 `feat=400,res=0.15` 阈值扫描。
4. 只有当触发集中在长廊段、且改进版不明显拉坏轨迹时，再录同一路线 3 组正式重复数据。
5. 如果 `feat=400` 能触发但轨迹变差，说明该路线更适合作为“弱退化/稳定场景反证”，不适合作为主退化提升实验。

---

## 3. 现场最小流程

### Step 0：创建实验会话记录

正式采集建议先创建会话目录。pilot 可用 `label:=degradation_pilot_<地点>`，正式重复数据再用 `degradation_expX`。

这一步的作用是创建 `~/go1_ws/experiments/.../metadata.yaml`、`geometry_constraints.csv` 和 `COMMANDS.md`，方便论文实验归档；它**不等于录包**，真正的 rosbag 仍由下一步 `go1_mapping.launch.py` 自动录制。

```bash
source /opt/ros/humble/setup.bash
source ~/go1_ws/install/setup.bash

ros2 run go1_bringup experiment_log init \
  --kind slam_degradation \
  --label degradation_pilot_corridor_new \
  --location "实验楼X层走廊" \
  --route "起点-长廊尽头-原路返回起点"
```

然后把现场测量的走廊长度、宽度、显著特征、人流情况和实际 bag 路径写进该会话目录的 `metadata.yaml` 或现场笔记。

如果现场来不及执行这一步，可以采集后补跑同样命令，再把已录好的 bag 路径、采集时间和备注补进 `metadata.yaml`。但正式论文实验建议采集前先建档，避免后面忘记路线和场地条件。

### Step 1：启动底层节点、自动录包并打开 RViz

```bash
source /opt/ros/humble/setup.bash
source ~/go1_ws/install/setup.bash

EXP_NAME=degradation_pilot_corridor_new
BAG_ROOT=$HOME/go1_ws/bags/$EXP_NAME

mkdir -p "$BAG_ROOT"

ros2 launch go1_bringup go1_mapping.launch.py \
  use_odom_fusion:=false \
  bag_dir:="$BAG_ROOT" \
  auto_archive:=false
```

说明：
- 启动后会自动拉起底层节点、开始 rosbag 录制，并打开 RViz。
- 当前阶段只录原始数据，不启用融合，不做现场消融。
- `auto_archive:=false` 用于避免退出时自动跑 PCD 归档，减少现场等待时间；如果你还想顺便保存本次 3D 地图会话，可改回 `true`。

---

### Step 2：机器人起立后，原地静止 5-10 秒

建议动作：
- 等机器人自动起立。
- 等其身体姿态稳定、不再继续微调。
- **原地静止 `5-10` 秒**，给后续离线回放的初始对齐留稳定窗口。

> 这段静止数据保留在 bag 中即可，通常不需要裁掉。

---

### Step 3：检查关键话题是否正常

新开一个终端，执行：

```bash
source /opt/ros/humble/setup.bash
source ~/go1_ws/install/setup.bash

ros2 topic hz /livox/lidar
```

观察几秒后 `Ctrl+C`，再执行：

```bash
ros2 topic hz /livox/imu
```

```bash
ros2 topic hz /odom
```

```bash
ros2 topic hz /imu
```

```bash
ros2 topic hz /joint_states
```

最小通过判据：
- `/livox/lidar` 有稳定输出
- `/livox/imu` 有稳定输出
- `/odom` 有稳定输出
- `/imu` 有稳定输出
- `/joint_states` 有稳定输出（用于关节角度/速度等诊断记录）

> 如果这一步失败，不要开始正式走长廊，先排查链路。

---

### Step 3.5：在 RViz 里观察实时建图效果

`go1_mapping.launch.py` 会自动打开 RViz，并加载项目自带配置。现场主要看这几项：

- `RegisteredCloud`：对应 `/cloud_registered`，这是 FAST-LIO 的主要实时点云
- `Odometry`：对应 `/Odometry`，用于观察轨迹是否连续
- `TF`：确认 `camera_init -> body -> livox_frame` 链路存在
- `LaserScan`：对应 `/scan`，可辅助确认 3D→2D 投影正常

建议观察要点：
- 墙面是否连续、平直，是否有明显重影
- 转身或原地小幅移动时，点云是否整体稳定跟随
- 轨迹是否连续，无明显跳变

> 如果 RViz 没有打开，手动执行 `rviz2 -d ~/go1_ws/install/go1_bringup/share/go1_bringup/config/go1_mapping.rviz` 也可以。

---

### Step 4：激活遥控器

在另一个终端执行：

```bash
source /opt/ros/humble/setup.bash
source ~/go1_ws/install/setup.bash

ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

说明：
- 该命令用于把驱动模式切到可遥控状态。
- 每次重新启动底层驱动后，通常都需要执行一次。

---

### Step 5：开始正式长廊闭环录制

建议路线：
- 起点按胶带箭头朝向站好，原地静止 `1-2` 秒
- 遥控进入长廊
- 走完整个退化走廊段
- 到达尽头/目标点
- 原路返回
- 尽量回到起点附近
- 终点再次静止 `1-2` 秒

建议采集要求：
- 线速度尽量保持平稳，不要猛冲
- 长廊段尽量足够长（建议 `>= 30 m`）
- 起点和终点尽量重合，方便后续闭环漂移评估
- 每段 bag 录完后记一下标签、路线和备注
- 若现场看到大量门框、凸起、柱子等特征，备注中明确写出来；这类路线可能会被归为弱退化反证。

---

### Step 6：结束录制并立刻验包

回到启动 `ros2 launch` 的终端，按 `Ctrl+C`。

等待 launch **完全退出**后，执行：

```bash
source /opt/ros/humble/setup.bash
source ~/go1_ws/install/setup.bash

EXP_NAME=degradation_pilot_corridor_new
BAG=$(ls -td "$HOME/go1_ws/bags/$EXP_NAME"/*/ | head -1)

echo "$BAG"
ros2 bag info "$BAG"
```

最小成功判据：
- `ros2 bag info` 中能看到以下话题，且消息数大于 0：
  - `/livox/lidar`
  - `/livox/imu`
  - `/odom`
  - `/imu`
  - `/joint_states`

建议同时检查是否出现这些辅助诊断话题：
- `/bms_state`
- `/sensor_ranges`
- `/tf_static`
- `/Odometry`

若旧 bag 缺少 `/joint_states`，仍可用于 SLAM 消融，因为核心回放只依赖 `/livox/lidar`、`/livox/imu`、`/odom`、`/imu`；但后续新采数据建议保留关节状态，方便排查步态或接触异常。

---

### Step 7：可选的快速回放抽查

如果时间允许，建议现场再抽查一次：

```bash
source /opt/ros/humble/setup.bash
source ~/go1_ws/install/setup.bash

EXP_NAME=degradation_pilot_corridor_new
BAG=$(ls -td "$HOME/go1_ws/bags/$EXP_NAME"/*/ | head -1)

ros2 bag play "$BAG" --clock --topics /livox/lidar /livox/imu /odom /imu
```

若能正常回放，再离场。

---

## 4. 何时连续录制 3 组正式数据

不要一上来就把同一路线录满 3 组。建议按下面顺序：

1. `degradation_pilot_<地点>`：先录 1 组 pilot。
2. 离线验证：
   - `ros2 bag info` 话题齐全；
   - baseline / improved / always-on 能正常回放；
   - improved 触发率不是 0；
   - 红点集中在长廊段；
   - improved 没有明显比 baseline 更差。
3. 通过后，再同一路线连续录 3 组正式重复数据。

正式重复数据的命名建议避开已用的 `exp2-7`，例如：

```bash
EXP_NAME=degradation_exp8
```

```bash
EXP_NAME=degradation_exp9
```

```bash
EXP_NAME=degradation_exp10
```

如果需要重新从论文表格编号，也可以用更语义化的目录：

```bash
EXP_NAME=degradation_strong_corridor_1
```

```bash
EXP_NAME=degradation_strong_corridor_2
```

```bash
EXP_NAME=degradation_strong_corridor_3
```

建议目录形如：

```bash
~/go1_ws/bags/degradation_strong_corridor_1/<timestamp>/
~/go1_ws/bags/degradation_strong_corridor_2/<timestamp>/
~/go1_ws/bags/degradation_strong_corridor_3/<timestamp>/
```

---

## 5. 如果现场时间充足：采 1 组强退化 pilot + 3 组正式重复

推荐现场目标：

- 第一次：`degradation_pilot_<地点>`，用于回实验室快速验证场地是否真的退化。
- 若 pilot 已经在现场/实验室验证过，则正式录三组：
  - `degradation_strong_corridor_1`
  - `degradation_strong_corridor_2`
  - `degradation_strong_corridor_3`

每组都按完整闭环路线走一次，不要把同一次长时间 bag 后切成三段，因为三组重复实验需要独立启动、独立初始状态和独立人为控制误差。

---

## 6. 现场只记住这几件事

- 先找真正弱特征长廊：少门框、少凸起、少杂物。
- 正式录 3 组前，先录 1 组 pilot 并离线验证触发率。
- 启动后 **bag 会自动开始录，RViz 也会自动打开**。
- 机器人起立后 **先静止 5-10 秒**，不要马上走。
- 起点用胶带标圆点和朝向箭头，返程尽量回到同一点同朝向。
- 正式出发前先确认 `/livox/lidar`、`/odom` 正常。
- 走之前看一眼 RViz，确认 FAST-LIO 点云和轨迹是正常的。
- 发一次空 `/cmd_vel` 激活遥控。
- 录完立刻 `ros2 bag info` 验包。
- **先保证 bag 是好的，后处理都可以回实验室再做。**

---

## 7. 当前阶段不必现场完成的事

以下工作都可以在实验室离线完成，不必阻塞本次长廊采集：

- `go1_replay.launch.py` 的基线 / 改进版 / always-on 三组回放
- `record_trajectory` 轨迹记录
- `evaluate_slam.sh` 评估
- 退化触发率统计
- 阈值敏感性扫描
- 轨迹图、RPE、空间分布图、几何误差表生成

结论：**当前现场最重要的目标有两个：拿到可用的原始长廊 bag，并确保该长廊确实有希望成为强退化样本。**

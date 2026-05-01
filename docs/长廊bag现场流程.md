# 长廊 rosbag 现场录制流程

> 目标：先稳定录下实验四所需的长廊原始 bag，后续分析、回放、消融在实验室离线完成。
>
> 适用场景：当前优先保证 bag 录制成功，同时在 RViz 里观察 FAST-LIO 实时建图效果；不在现场调试 `robot_localization` / `odom_constraint`。
>
> 录制模式：基础模式 `use_odom_fusion:=false`

---

## 1. 使用原则

- 启动 `go1_mapping.launch.py` 后，**会自动开始 rosbag 录制，并自动打开 RViz**。
- 机器人启动后**可能自动起立**，这是正常现象。
- **不要一启动就立刻走**；建议机器人站稳后原地静止 `5-10` 秒，再开始正式路线。
- 这几秒静止数据**不需要后续手动剔除**，保留即可。
- 每次录完后，**必须立刻执行 `ros2 bag info` 验包**，确认关键话题都录到了。
- 若本次目标只是录 bag，不想在退出时额外等待 PCD 归档，可传 `auto_archive:=false`。

---

## 2. 现场最小流程

### Step 1：启动底层节点、自动录包并打开 RViz

```bash
source /opt/ros/humble/setup.bash
source ~/go1_ws/src/install/setup.bash

EXP_NAME=degradation_exp1
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
source ~/go1_ws/src/install/setup.bash

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

最小通过判据：
- `/livox/lidar` 有稳定输出
- `/livox/imu` 有稳定输出
- `/odom` 有稳定输出
- `/imu` 有稳定输出

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

> 如果 RViz 没有打开，手动执行 `rviz2 -d ~/go1_ws/src/install/go1_bringup/share/go1_bringup/config/go1_mapping.rviz` 也可以。

---

### Step 4：激活遥控器

在另一个终端执行：

```bash
source /opt/ros/humble/setup.bash
source ~/go1_ws/src/install/setup.bash

ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

说明：
- 该命令用于把驱动模式切到可遥控状态。
- 每次重新启动底层驱动后，通常都需要执行一次。

---

### Step 5：开始正式长廊闭环录制

建议路线：
- 起点原地静止 `1-2` 秒
- 遥控进入长廊
- 走完整个退化走廊段
- 到达尽头/目标点
- 原路返回
- 尽量回到起点附近
- 终点再次静止 `1-2` 秒

建议采集要求：
- 线速度尽量保持平稳，不要猛冲
- 长廊段尽量足够长（建议 `>= 20-30 m`）
- 起点和终点尽量重合，方便后续闭环漂移评估
- 每段 bag 录完后记一下标签、路线和备注

---

### Step 6：结束录制并立刻验包

回到启动 `ros2 launch` 的终端，按 `Ctrl+C`。

等待 launch **完全退出**后，执行：

```bash
source /opt/ros/humble/setup.bash
source ~/go1_ws/src/install/setup.bash

BAG=$(ls -td "$HOME/go1_ws/bags/degradation_exp1"/*/ | head -1)

echo "$BAG"
ros2 bag info "$BAG"
```

最小成功判据：
- `ros2 bag info` 中能看到以下话题，且消息数大于 0：
  - `/livox/lidar`
  - `/livox/imu`
  - `/odom`
  - `/imu`

---

### Step 7：可选的快速回放抽查

如果时间允许，建议现场再抽查一次：

```bash
source /opt/ros/humble/setup.bash
source ~/go1_ws/src/install/setup.bash

BAG=$(ls -td "$HOME/go1_ws/bags/degradation_exp1"/*/ | head -1)

ros2 bag play "$BAG" --clock --topics /livox/lidar /livox/imu /odom /imu
```

若能正常回放，再离场。

---

## 3. 连续录制 3 组长廊数据

重复上述流程三次，只改实验名：

```bash
EXP_NAME=degradation_exp1
```

```bash
EXP_NAME=degradation_exp2
```

```bash
EXP_NAME=degradation_exp3
```

建议目录形如：

```bash
~/go1_ws/bags/degradation_exp1/<timestamp>/
~/go1_ws/bags/degradation_exp2/<timestamp>/
~/go1_ws/bags/degradation_exp3/<timestamp>/
```

---

## 4. 现场只记住这几件事

- 启动后 **bag 会自动开始录，RViz 也会自动打开**。
- 机器人起立后 **先静止 5-10 秒**，不要马上走。
- 正式出发前先确认 `/livox/lidar`、`/odom` 正常。
- 走之前看一眼 RViz，确认 FAST-LIO 点云和轨迹是正常的。
- 发一次空 `/cmd_vel` 激活遥控。
- 录完立刻 `ros2 bag info` 验包。
- **先保证 bag 是好的，后处理都可以回实验室再做。**

---

## 5. 当前阶段不必现场完成的事

以下工作都可以在实验室离线完成，不必阻塞本次长廊采集：

- `go1_replay.launch.py` 的基线 / 改进版 / always-on 三组回放
- `record_trajectory` 轨迹记录
- `evaluate_slam.sh` 评估
- 退化触发率统计
- 阈值敏感性扫描
- 轨迹图、RPE、空间分布图、几何误差表生成

结论：**当前现场最重要的目标只有一个：拿到可用的原始长廊 bag。**

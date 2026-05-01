# 地图文件存放目录

当前架构: FAST-LIO 在线累积 3D 点云 → 离线切片生成 2D 栅格地图 (Nav2 可直接加载).
**不再使用 slam_toolbox**.

典型流程 (详见 [实验手册 §5](../docs/实验手册.md)):

```bash
# 1. 在线建图 (FAST-LIO 会在 Ctrl+C 退出时把累积点云写到 FAST_LIO/PCD/scans.pcd,
#    launch 的 auto_archive 还会同时把会话归档到 maps/sessions/<时间戳>/)
ros2 launch go1_bringup go1_mapping.launch.py use_odom_fusion:=false

# 2. 离线切片 (或使用 archive_map --promote 用最新 scans.pcd 重切并晋升为 my_lab.*;
#    --promote 不能晋升历史会话目录里的 both.*, 详见 实验手册 §5.7)
ros2 run go1_bringup pcd_to_map \
    ~/go1_ws/src/FAST_LIO/PCD/scans.pcd \
    ~/go1_ws/src/go1_bringup/maps \
    my_lab
```

生成:
- `my_lab.pgm` — Nav2 map_server 标准栅格图 (0=占据, 254=空闲, 205=未知)
- `my_lab.yaml` — 地图元数据

导航 launch (`go1_nav.launch.py`) 默认加载 `my_lab.yaml`.

`maps/sessions/` 是自动归档目录, 体积很大且不会随 `go1_bringup` 安装; 需要用于论文的
`compare.png` 或历史地图时, 从对应会话目录手动挑选。

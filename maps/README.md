# 地图文件存放目录

在完成建图后，使用以下命令保存地图：

```bash
# 保存地图 (slam_toolbox 方式)
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/ziggy/go1_ws/src/go1_bringup/maps/my_lab'}}"

# 或者使用 nav2 map_saver (需要先启动 map_saver_server)
ros2 run nav2_map_server map_saver_cli -f /home/ziggy/go1_ws/src/go1_bringup/maps/my_lab
```

保存后会生成：
- `my_lab.pgm` - 地图图像文件
- `my_lab.yaml` - 地图元数据文件

导航时 launch 文件会自动加载 `my_lab.yaml`。

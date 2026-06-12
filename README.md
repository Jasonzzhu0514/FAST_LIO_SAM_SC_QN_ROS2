# fast_lio_sam_sc_qn2 ROS 2 工作空间

FAST-LIO-SAM-SC-QN 的 ROS 2 工作空间，主要用于 Livox MID360/MID360s。工作空间内包含 ROS 2 版 FAST-LIO 前端和 `fast_lio_sam_sc_qn2` 后端：前端输出里程计和配准点云，后端完成关键帧、回环检测、配准、GTSAM 位姿图优化、地图发布和结果保存。`src/web_mapping` 作为子模块提供浏览器可视化界面，`fast_lio_web_broker` 负责把本算法的话题、状态、地图路径和建图控制对齐到通用 `/web_mapping/*` 接口。

默认输入配置位于 `src/fast_lio_sam_sc_qn2/config/mid360.yaml`：

- 里程计：`Odometry_loc`
- 配准点云：`cloud_registered_1`
- 地图坐标系：`camera_init`
- 机体坐标系：`body`

## 编译

```bash
cd ~/Documents/fast_lio_sam_sc_qn_ros2
source /opt/ros/<distro>/setup.bash
./build.sh
source install/setup.bash
```

常见 ROS 依赖：

```bash
sudo apt install ros-<distro>-pcl-ros ros-<distro>-pcl-conversions ros-<distro>-rosbag2-cpp
```

后端还需要 GTSAM、TEASER++、PCL、OpenMP、TBB。若编译本仓库内的 `livox_ros_driver2`，还需要 Livox SDK 2。

不要混用 ROS 1 和 ROS 2 环境。编译前确认：

```bash
echo $ROS_VERSION
echo $ROS_DISTRO
```

`ROS_VERSION` 应为 `2`。

## 雷达配置

MID360s 配置文件：

```text
src/livox_ros_driver2/config/MID360s_config.json
```

配置示例：

- 主机网卡 IP：`192.168.1.5`
- 雷达 IP：`192.168.1.122`

如果你的电脑或雷达 IP 不同，需要改 `host_ip` 和 `lidar_configs[].ip`。可用下面命令确认本机网卡：

```bash
ip -4 addr
ip route get <雷达IP>
```

如果日志出现：

```text
found lidar not defined in the user-defined config
```

通常是 `lidar_configs[].ip` 没写成实际雷达 IP。

## 单独启动雷达

先只启动 Livox 驱动，确认雷达正常：

```bash
cd ~/Documents/fast_lio_sam_sc_qn_ros2
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

如果是 MID360：

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

正常日志应包含类似内容：

```text
successfully change work mode
successfully enable Livox Lidar imu
```

## 测试雷达是否正常

驱动默认使用 `multi_topic=0`，点云和 IMU 话题通常是：

- `/livox/lidar`
- `/livox/imu`

检查话题是否存在：

```bash
ros2 topic list | grep livox
```

检查点云是否持续发布：

```bash
ros2 topic hz /livox/lidar
```

检查 IMU 是否持续发布：

```bash
ros2 topic hz /livox/imu
```

查看消息类型：

```bash
ros2 topic info /livox/lidar
ros2 topic info /livox/imu
```

如果要 `echo` Livox 自定义消息，当前终端也必须加载本工作空间：

```bash
cd ~/Documents/fast_lio_sam_sc_qn_ros2
source /opt/ros/<distro>/setup.bash
source install/setup.bash
ros2 topic echo /livox/lidar
```

当前 `msg_MID360s_launch.py` 默认 `xfer_format=1`，所以 `/livox/lidar` 类型通常是：

```text
livox_ros_driver2/msg/CustomMsg
```

如果需要 RViz 直接查看 `sensor_msgs/msg/PointCloud2`，把 `src/livox_ros_driver2/launch_ROS2/msg_MID360s_launch.py` 里的：

```python
xfer_format = 1
```

改为：

```python
xfer_format = 0
```

然后重新启动雷达驱动。

## 启动建图

推荐一条命令拉起 Livox 驱动、FAST-LIO 前端和后端。

```bash
cd ~/Documents/fast_lio_sam_sc_qn_ros2
source /opt/ros/<distro>/setup.bash
source install/setup.bash
ros2 launch fast_lio_sam_sc_qn2 mid360_mapping.launch.py start_livox_driver:=true use_mid360s:=true start_fast_lio_frontend:=true
```

如果是 MID360，不是 MID360s：

```bash
ros2 launch fast_lio_sam_sc_qn2 mid360_mapping.launch.py start_livox_driver:=true start_fast_lio_frontend:=true
```

如果 Livox 驱动已经单独启动，只拉起前端和后端：

```bash
ros2 launch fast_lio_sam_sc_qn2 mid360_mapping.launch.py start_fast_lio_frontend:=true
```

如果 FAST-LIO 前端也已经启动，只拉起后端：

```bash
ros2 launch fast_lio_sam_sc_qn2 mid360_mapping.launch.py
```

启动后检查链路：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic list | grep -E 'Odometry_loc|cloud_registered_1|corrected_current_pcd|pose_stamped'
ros2 topic list | grep web_mapping
```

`src/fast_lio_sam_sc_qn2/third_party/FAST_LIO` 是 ROS 2 版 FAST-LIO 前端源码，作为普通目录随工作空间编译，不作为 submodule。

## Web Mapping

`src/web_mapping` 是独立 Web Mapping 子模块。`fast_lio_web_broker` 是本项目的适配层，负责把 FAST-LIO-SAM-SC-QN 的话题、状态、控制命令和地图保存路径对齐到通用 `/web_mapping/*` 接口。

如果建图已经由 `mid360_mapping.launch.py` 启动，默认会同时启动 broker。另开终端启动 Web UI：

```bash
cd ~/Documents/fast_lio_sam_sc_qn_ros2
source /opt/ros/<distro>/setup.bash
source install/setup.bash
ros2 launch web_mapping web_mapping.launch.py
```

如果希望由网页按钮控制开始/停止建图，启动控制入口：

```bash
cd ~/Documents/fast_lio_sam_sc_qn_ros2
source /opt/ros/<distro>/setup.bash
source install/setup.bash
ros2 launch fast_lio_sam_sc_qn2 web_mapping_control.launch.py
```

浏览器打开：

```text
http://127.0.0.1:8765
```

broker 发布的标准接口：

- `/web_mapping/raw_cloud`：由 `/livox/lidar` 原始 Livox CustomMsg 转成 PointCloud2
- `/web_mapping/current_frame`
- `/web_mapping/global_map`
- `/web_mapping/pose`
- `/web_mapping/raw_path`
- `/web_mapping/optimized_path`
- `/web_mapping/imu`
- `/web_mapping/lidar_status`
- `/web_mapping/status`

Web 前端的开始、停止、保存命令会发送到 `/web_mapping/command`。broker 会把这些通用命令转成当前算法的实际控制逻辑：

- 附着模式：`mid360_mapping.launch.py` 已经启动建图，Web 只负责显示、停止状态和触发 `save_dir`
- 控制模式：`web_mapping_control.launch.py` 先启动 Web 与 broker，网页点击“开始建图”后再拉起 Livox、FAST-LIO 前端和后端

历史地图读取路径由 broker 通过 `/web_mapping/status` 暴露，默认读取本工作空间的 `maps/`，不是 `web_mapping` 子模块内的目录。保存地图时，后端仍按 FAST-LIO-SAM-SC-QN 的规则写入：

```text
<workspace>/maps/<session>/
```

如果需要改地图目录：

```bash
ros2 launch fast_lio_sam_sc_qn2 web_mapping_control.launch.py map_history_root:=/data/slam/maps
```

这时 Web 会读取 `/data/slam/maps`，broker 会让后端保存到 `/data/slam/maps/<session>/`。

## 自定义配置

复制默认配置后修改话题、坐标系、保存路径或回环参数：

```bash
cp src/fast_lio_sam_sc_qn2/config/mid360.yaml ~/mid360_my_robot.yaml
ros2 launch fast_lio_sam_sc_qn2 mid360_mapping.launch.py backend_config_file:=~/mid360_my_robot.yaml
```

常改参数：

- `input.odom_topic`
- `input.cloud_topic`
- `basic.map_frame`
- `basic.robot_frame`
- `result.save_directory`
- `result.save_map_pcd`
- `result.save_map_bag`

## 保存结果

节点退出时会自动保存已有关键帧。运行中也可以手动保存：

```bash
ros2 topic pub --once save_dir std_msgs/msg/String "{data: '/tmp/fast_lio_sam_sc_qn2'}"
```

默认输出目录结构：

```text
maps/
  sequence_YYYYmmdd_HHMMSS/
    map_optimized.pcd
    map_raw.pcd
    poses_kitti.txt
    poses_matrix.txt
    poses_tum.txt
    result_bag/
      metadata.yaml
      *.db3
    scans/
      000000.pcd
      000001.pcd
```

`result.session_name` 可以指定固定 session 目录名。

## 输出话题

默认输出话题：

- `corrected_odom`
- `corrected_path`
- `ori_odom`
- `ori_path`
- `corrected_map`
- `corrected_current_pcd`
- `loop_detection`
- `pose_stamped`
- `src`
- `dst`
- `coarse_aligned_quatro`
- `fine_aligned_nano_gicp`

## 常见问题

缺少 `pcl_ros`：

```bash
sudo apt install ros-<distro>-pcl-ros
```

`livox_ros_driver2` 编译时出现下面 warning 通常不影响使用：

```text
io features related to pcap/png/libusb will be disabled
```

如需消除：

```bash
sudo apt install libpcap-dev libpng-dev libusb-1.0-0-dev
```

第三方源码在 `src/fast_lio_sam_sc_qn2/third_party/`。其中 `nano_gicp`、`quatro`、`scancontext_tro` 由本包 CMake 构建；`FAST_LIO` 是独立 ROS 2 前端包，会由 `build.sh` 纳入 colcon 编译。

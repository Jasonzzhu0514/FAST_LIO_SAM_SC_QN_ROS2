# fast_lio_sam_sc_qn2 ROS 2 工作空间

FAST-LIO-SAM-SC-QN 后端的 ROS 2 移植版本，主要用于 Livox MID360/MID360s。它不负责前端里程计计算，只订阅 FAST-LIO 前端输出的里程计和配准点云，完成关键帧、回环检测、配准、GTSAM 位姿图优化、地图发布和结果保存。

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

建图后端需要 FAST-LIO 前端已经发布：

- `Odometry_loc`
- `cloud_registered_1`

启动前检查：

```bash
ros2 topic list | grep -E 'Odometry_loc|cloud_registered_1'
```

### 方式一：雷达和前端已启动，只启动后端

```bash
cd ~/Documents/fast_lio_sam_sc_qn_ros2
source install/setup.bash
ros2 launch fast_lio_sam_sc_qn2 mid360_mapping.launch.py
```

这是推荐流程：先单独确认雷达正常，再启动 FAST-LIO 前端，最后启动本后端。

### 方式二：由本 launch 同时启动 Livox 驱动和后端

MID360s：

```bash
ros2 launch fast_lio_sam_sc_qn2 mid360_mapping.launch.py start_livox_driver:=true use_mid360s:=true
```

MID360：

```bash
ros2 launch fast_lio_sam_sc_qn2 mid360_mapping.launch.py start_livox_driver:=true
```

默认情况下，这个 launch 只会额外启动 Livox 驱动，不会启动 FAST-LIO 前端。

如果本工作空间内已经有 `fast_lio` 包，也可以显式让同一个 launch 拉起前端：

```bash
ros2 launch fast_lio_sam_sc_qn2 mid360_mapping.launch.py start_fast_lio_frontend:=true
```

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

第三方源码在 `src/fast_lio_sam_sc_qn2/third_party/`，由本包 CMake 统一构建，不需要单独作为 colcon 包编译。

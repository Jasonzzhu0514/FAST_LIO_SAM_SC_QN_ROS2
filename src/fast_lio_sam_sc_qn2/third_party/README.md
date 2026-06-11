# 第三方源码

这个目录存放 `fast_lio_sam_sc_qn2` 使用或保留的第三方源码。

- `nano_gicp`: Nano-GICP 配准实现。
- `quatro`: Quatro 粗配准实现。
- `scancontext_tro`: Scan Context 回环候选检测实现。
- `FAST_LIO`: ROS 2 版 FAST-LIO 前端源码，是普通目录，不是 submodule。

`nano_gicp`、`quatro`、`scancontext_tro` 不需要作为独立 colcon 包编译，各自目录内的 `COLCON_IGNORE` 会阻止 colcon 继续扫描。

当前包的 `CMakeLists.txt` 会构建：

- `nano_gicp_vendor`
- `quatro_vendor`
- `scancontext_vendor`

`FAST_LIO` 作为独立 ROS 2 包参与 colcon 编译，提供 `fastlio_mapping` 前端节点，发布后端需要的 `Odometry_loc` 和 `cloud_registered_1`。

本地改动：

- `quatro/src/matcher.cc` 使用 `QUATRO_FLANN_CORES` 编译定义控制 FLANN 线程数，不再写死固定线程数。

# fast_lio_sam_sc_qn2 ROS 2 Workspace

ROS 2 port of the FAST-LIO-SAM-SC-QN mapping back-end. This workspace is
currently scoped for Livox MID360/MID360s and is intended to run behind a ROS 2
FAST-LIO front-end such as the one in `sunray_slam`.

## What This Package Does

- Subscribes to FAST-LIO odometry and registered point cloud output.
- Adds keyframes, Scan Context loop candidate detection, Quatro coarse
  registration, Nano-GICP fine registration, and GTSAM pose graph optimization.
- Publishes corrected odometry/path/map/debug clouds.
- Saves optimized keyframes, poses, and a corrected PCD map.

Default MID360 profile:

- Odometry topic: `Odometry_loc`
- Registered cloud topic: `cloud_registered_1`
- Map frame: `camera_init`
- Robot frame: `body`

These are defaults in `src/fast_lio_sam_sc_qn2/config/mid360.yaml`; change that
file or pass another config file if your FAST-LIO front-end uses different names.

## Workspace Layout

```text
fast_lio_sam_sc_qn_ros2/
  build.sh
  src/
    fast_lio_sam_sc_qn2/
    livox_ros_driver2/
    fast_lio/                  # optional; usually provided by sunray_slam
```

`livox_ros_driver2` is optional for the back-end itself. Keep it in this
workspace if you want this workspace to build/start the official MID360 driver;
remove it or add `COLCON_IGNORE` if your robot stack already starts the Livox
driver from another workspace such as `sunray_slam`.

The local driver is a git submodule fetched from:

```text
https://github.com/Livox-SDK/livox_ros_driver2
```

The upstream driver keeps `package_ROS2.xml` instead of a tracked `package.xml`,
so this workspace generates `src/livox_ros_driver2/package.xml` from
`package_ROS2.xml` before building.

After cloning this workspace, initialize submodules if needed:

```bash
cd fast_lio_sam_sc_qn_ros2
git submodule update --init --recursive
```

## Dependencies

Source your ROS 2 environment first:

```bash
source /opt/ros/<distro>/setup.bash
echo $ROS_VERSION
```

`ROS_VERSION` must be `2`. Do not build this workspace from a ROS 1 environment
such as `noetic`.

Install ROS/package dependencies where available:

```bash
cd fast_lio_sam_sc_qn_ros2
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

You still need the non-ROS libraries required by the mapping back-end:

- GTSAM
- TEASER++
- PCL
- OpenMP
- TBB

If you keep `src/livox_ros_driver2` in this workspace, also install Livox SDK 2.
If the Livox driver is supplied by `sunray_slam` or another workspace, the
back-end package itself does not need Livox SDK 2.

If `rosdep` cannot resolve GTSAM or TEASER++ for your distro, install those
libraries manually before building.

## Build

Build this workspace after all system dependencies are installed:

```bash
cd fast_lio_sam_sc_qn_ros2
source /opt/ros/<distro>/setup.bash
./build.sh
source install/setup.bash
```

`build.sh` runs:

```bash
colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=${ROS_DISTRO}
```

The script refuses to run unless `ROS_VERSION=2` is present in the environment.
It also prepares `livox_ros_driver2/package.xml` for colcon package discovery.
`ROS_EDITION=ROS2` and `DISTRO_ROS=${ROS_DISTRO}` are needed because
`livox_ros_driver2` keeps ROS 1 and ROS 2 build logic in one `CMakeLists.txt`.
They are harmless if only `fast_lio_sam_sc_qn2` is built.

The original ROS 1 project builds `nano_gicp` first, then `quatro`, then the
main FAST-LIO-SAM-SC-QN package. This ROS 2 port keeps that order inside this
package CMake:

```text
nano_gicp_vendor -> quatro_vendor -> scancontext_vendor -> fast_lio_sam_sc_qn2_core
```

They are source vendors under `third_party/`; do not build them as separate
colcon packages.

## Start Mapping

You need two data sources:

- Livox driver publishing MID360 data.
- FAST-LIO front-end publishing odometry and registered cloud topics.

This package is the loop-closure/pose-graph mapping back-end. It does not run
the FAST-LIO front-end by itself.

Before starting the back-end, check that FAST-LIO is publishing the configured
input topics:

```bash
ros2 topic list | grep -E 'Odometry_loc|cloud_registered_1'
```

### Option 1: Run Back-End Only

Use this when `livox_ros_driver2` and FAST-LIO are already running, for example
from `sunray_slam`:

```bash
cd fast_lio_sam_sc_qn_ros2
source install/setup.bash
ros2 launch fast_lio_sam_sc_qn2 mid360_mapping.launch.py
```

In this mode the launch file does not require `livox_ros_driver2` to be
installed.

### Option 2: Also Start Livox MID360 Driver

This starts the official MID360 driver launch from `livox_ros_driver2`, but you
still need a FAST-LIO front-end running to produce `Odometry_loc` and
`cloud_registered_1`.

```bash
cd fast_lio_sam_sc_qn_ros2
source install/setup.bash
ros2 launch fast_lio_sam_sc_qn2 mid360_mapping.launch.py start_livox_driver:=true
```

For MID360s:

```bash
ros2 launch fast_lio_sam_sc_qn2 mid360_mapping.launch.py start_livox_driver:=true use_mid360s:=true
```

If this command reports that `livox_ros_driver2` cannot be found, source the
workspace that contains the driver or keep/build the local `src/livox_ros_driver2`
copy.

### Use A Custom Config

Copy and edit the default config if your FAST-LIO topic names, frame names, save
directory, or loop parameters differ:

```bash
cp src/fast_lio_sam_sc_qn2/config/mid360.yaml ~/mid360_my_robot.yaml
ros2 launch fast_lio_sam_sc_qn2 mid360_mapping.launch.py config_file:=~/mid360_my_robot.yaml
```

Useful fields:

- `input.odom_topic`
- `input.cloud_topic`
- `input.cloud_is_in_world_frame`
- `basic.map_frame`
- `basic.robot_frame`
- `result.save_map_pcd`
- `result.save_map_bag`
- `result.save_in_kitti_format`
- `result.save_directory`
- `result.maps_directory_name`
- `result.seq_name`
- `result.session_name`
- `result.session_timestamp_format`

## Save Mapping Results

Results are saved automatically on shutdown if keyframes exist. You can also
trigger a save while the node is running:

```bash
ros2 topic pub --once save_dir std_msgs/msg/String "{data: '/tmp/fast_lio_sam_sc_qn2'}"
```

If the message data is empty, the node uses `result.save_directory`. If
`result.save_directory` is empty, it uses the current working directory of the
node process. Results are written under `result.maps_directory_name`, which
defaults to `maps`. If you launch the node in a ROS namespace, publish to the
namespaced `save_dir` topic. Without a namespace, `save_dir` resolves to
`/save_dir`.

Each node run uses one timestamped mapping session directory. Manual save and
shutdown save write to the same session. With default settings and a launch from
the workspace root, the layout is:


```text
fast_lio_sam_sc_qn_ros2/
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
        ...
```

If you publish a save root explicitly, for example `/tmp/fast_lio_sam_sc_qn2`,
the same session directory is created under:

```text
/tmp/fast_lio_sam_sc_qn2/maps/sequence_YYYYmmdd_HHMMSS/
```

Session directory contents:

```text
map_optimized.pcd   # keyframes accumulated with optimized poses
map_raw.pcd         # keyframes accumulated with original FAST-LIO poses
poses_kitti.txt
poses_matrix.txt
poses_tum.txt
result_bag/
  metadata.yaml
  *.db3
scans/
  000000.pcd
  000001.pcd
  ...
```

Set `result.session_name` if you want a fixed session folder name instead of the
automatic timestamp suffix generated when the node starts. `result_bag/` is the
ROS 2 rosbag2 equivalent of the original `result.bag`.

## Main Published Topics

Default output topics are relative names from `mid360.yaml`:

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

Because they are relative topic names, ROS namespaces can be applied normally.

## Third-Party Source Vendors

`src/fast_lio_sam_sc_qn2/third_party` is intentionally a source-vendor directory,
not a set of independent ROS packages:

- `nano_gicp`
- `quatro`
- `scancontext_tro`

The package-level CMake builds them as vendor libraries. See
`src/fast_lio_sam_sc_qn2/third_party/README.md` for notes about local changes.

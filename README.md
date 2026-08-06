# [voxel3d 5Vx ros2](https://github.com/5VOXEL/voxel3d_5Vx_ros2)
##### Copyright (c) 2026 5Voxel Co., Ltd.

</br>

A camera driver node that wraps the 5Voxel `voxel3d` SDK for **ROS2 (rclcpp)**. ROS1 isn't supported.

## Features

- Drives a 5Voxel depth camera via the `voxel3d` SDK: ToF (depth + confidence + point cloud), RGB (MJPG, decoded via `cv::imdecode`), FLIR/Lepton3 thermal, and embedded IMU
- Dedicated capture thread + process thread pipeline frame acquisition, so the steady-state per-frame period is roughly `max(capture, process)` instead of the sum of both
- Runtime-configurable thermal display range (`voxel3d.thermal_min_temp` / `voxel3d.thermal_max_temp`) via `ParamHelper`, read once at startup
- Falls back to the last valid RGB / thermal frame on a failed sensor query or decode, instead of publishing a black frame
- Publishes a `map → voxel3d_frame` TF every frame
- Custom RViz2 panel (`voxel3d_panel` / `Voxel3dRvizPanel`) showing live RGB/Depth/IR/Thermal previews, IMU accelerometer/gyro readouts, per-stream resolution/FOV/FPS, and device identity/firmware info
- Per-device log tag (`[ INFO. ][ 5HiRab <SN> ]`) in the startup scan/init logs, laying groundwork for multi-sensor support

## Directory layout

```
voxel3d/
├── inc/                          # voxel3d.h (SDK header)
├── lib/                          # libvoxel3d.so
├── Models/
│   └── SharedData.h              # SensorData / SensorInfo / hiRabSensorInfo structs
├── Helpers/
│   ├── ParamHelper.hpp           # flat ROS1/ROS2 parameter wrapper
│   └── FrameTimer.hpp
└── node/
    ├── CMakeLists.txt
    └── ros2/
        └── voxel3d_node/
            ├── CMakeLists.txt
            ├── package.xml
            ├── rviz_common_plugins.xml(.in)
            ├── config/
            │   ├── voxel3d_params_ros1.yaml
            │   └── voxel3d_params_ros2.yaml
            ├── inc/voxel3d_node/
            │   └── voxel3d_panel.hpp
            └── src/
                ├── voxel3d_node.cpp
                └── voxel3d_panel.cpp
```

## Prerequisites

- Ubuntu 22.04 LTS
- ROS2 Humble installed and set up
- CMake > 3.24
- OpenCV 4.5.4
- Qt5 (Core, Widgets) — for the RViz panel
- 5Voxel `voxel3d` SDK — expected at the repo root (`voxel3d/inc/voxel3d.h`, `voxel3d/lib/libvoxel3d.so`), three directories up from `node/ros2/voxel3d_node` (override with `-DVOXEL3D_ROOT_DIR=` if you move it)

## Dependencies

**ROS2 packages**
- rclcpp, sensor_msgs, cv_bridge, geometry_msgs, tf2_ros, std_msgs
- If the RViz panel is enabled (always, currently): rviz_common, rviz2, pluginlib, Qt5
- `nav_msgs` and `visualization_msgs` are also declared in `package.xml`/`CMakeLists.txt` but aren't currently used by any code in `voxel3d_node.cpp` — likely left over from copying the sibling `acaas_node` package's build files. Safe to install but not required by anything voxel3d_node actually does today.

```bash
sudo apt install \
  ros-humble-cv-bridge \
  ros-humble-sensor-msgs \
  ros-humble-tf2-ros \
  ros-humble-rviz2 \
  ros-humble-rviz-common
```

## Set up your environment

`source` tells your shell where your ROS2 workspace is. It affects:
- `ros2 run` — being able to find your node
- `rviz2` — loading your plugin/panel
- `ros2 topic list` — being able to find your msg types

Without sourcing, ROS2 doesn't know your package exists.

This needs to be run in every new terminal, so it's recommended to add the system one to `~/.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

The workspace overlay should be sourced after every build:

```bash
source ~/ros2/examples/ros-5voxel/install/setup.bash
```

## Building

```bash
colcon build --packages-select voxel3d_node
source install/setup.bash
```

## Running

There's no launch file yet (unlike the `acaas_node` sibling package), so run the executable directly:

```bash
ros2 run voxel3d_node voxel3d_node
```
To load `config/voxel3d_params_ros2.yaml` instead of the built-in defaults:
```bash
ros2 run voxel3d_node voxel3d_node --ros-args --params-file $(ros2 pkg prefix voxel3d_node)/share/voxel3d_node/config/voxel3d_params_ros2.yaml
```

## Visualizing in RViz

Once `voxel3d_node` is running (`ros2 run` / `rosrun`), open RViz separately and add the following:

**1. Point cloud**
- Displays panel → `Add` → By topic → `camera/frame_pc_ir`, `camera/frame_pc_rgb`, or `camera/frame_pc_thermal` (`sensor_msgs/PointCloud2`) — same XYZ geometry, each colored from a different source. Add more than one and toggle their checkboxes to compare, or just add the one you want.
- Set **Fixed Frame** (top of the Displays panel) to `voxel3d_frame` (or `map`), otherwise the point cloud won't render.

**2. Camera images**
- Displays panel → `Add` → By topic → `camera/frame_depth`, `camera/frame_ir`, `camera/frame_rgb`, or `camera/frame_thermal` (`sensor_msgs/Image`)

**3. Voxel3dRvizPanel (custom RViz panel)**
- Menu bar → **Panels** → **Add New Panel** → look for **`voxel3d_node/Voxel3dRvizPanel`** ("A rviz2 panel to display data from multiple 5Voxel devices.")
- If it doesn't show up in the list, double-check that `voxel3d_node` is sourced (`source install/setup.bash` / `source devel/setup.bash`) in the same terminal you launched `rviz2` from — RViz discovers plugins through the same package index as `ros2 run`.
- Shows, per expandable section: live RGB/IR/Depth/Thermal previews, IMU acceleration/angular velocity/timestamp, per-stream (RGB/Depth/Thermal) resolution/FOV/FPS, and device name/S/N/firmware version/build date/library version.

## Runtime Parameters

| Key | Type | Default | Description |
|---|---|---|---|
| `voxel3d.fusion_mode` | int | 0 | Depth/RGB/FLIR alignment mode passed to `voxel3d_set_rectifyType()`: `0`=NONE, `1`=RGB2TOF, `2`=FLIR2TOF, `3`=TOF2RGB |
| `voxel3d.thermal_min_temp` | double | 20.0 | Lower bound (°C) of the thermal-to-ironbow display mapping |
| `voxel3d.thermal_max_temp` | double | 50.0 | Upper bound (°C) of the thermal-to-ironbow display mapping |
| `voxel3d.depth_max_range` | double | 10000.0 | Fixed real-world depth range (mm) that the depth image is scaled against before JET colorization — 0mm maps to color 0, `depth_max_range` mm maps to color 255, so the mapping stays consistent frame-to-frame. Not yet exposed in either yaml config. |

> Note: `config/voxel3d_params_ros2.yaml` (nested under `ros__parameters`) is the supported config file.

## Published Topics

| Topic | Type | Description |
|---|---|---|
| `camera/frame_pc_ir` | `sensor_msgs/PointCloud2` | Point cloud, colored from the confidence/IR image |
| `camera/frame_pc_rgb` | `sensor_msgs/PointCloud2` | Same point cloud, colored from RGB |
| `camera/frame_pc_thermal` | `sensor_msgs/PointCloud2` | Same point cloud, colored from the thermal (ironbow) image |
| `camera/frame_depth` | `sensor_msgs/Image` | Depth, colorized with the JET colormap |
| `camera/frame_ir` | `sensor_msgs/Image` | Confidence map (stands in for IR), grayscale→BGR |
| `camera/frame_rgb` | `sensor_msgs/Image` | RGB, decoded from the sensor's MJPG stream |
| `camera/frame_thermal` | `sensor_msgs/Image` | FLIR/Lepton3 thermal, colorized with an ironbow LUT between `thermal_min_temp`/`thermal_max_temp` |
| `imu/accel` | `sensor_msgs/Imu` | Accelerometer |
| `imu/gyro` | `sensor_msgs/Imu` | Gyroscope |
| `voxel3d/name`, `voxel3d/sn`, `voxel3d/fw_version`, `voxel3d/fw_build_date`, `voxel3d/lib_version` | `std_msgs/String` | Device identity / firmware info |
| `voxel3d/info_rgb_res`, `voxel3d/info_rgb_fov`, `voxel3d/info_rgb_fps` | `std_msgs/String` | RGB stream resolution / FOV / FPS (FPS pre-formatted to 1 decimal) |
| `voxel3d/info_depth_res`, `voxel3d/info_depth_fov`, `voxel3d/info_depth_fps` | `std_msgs/String` | Depth (ToF) stream resolution / FOV / FPS (FPS pre-formatted to 1 decimal) |
| `voxel3d/info_thermal_res`, `voxel3d/info_thermal_fov`, `voxel3d/info_thermal_fps` | `std_msgs/String` | Thermal stream resolution / FOV / FPS (FPS pre-formatted to 1 decimal) |

TF: `map → voxel3d_frame`, published every frame in `publish_frames()`.

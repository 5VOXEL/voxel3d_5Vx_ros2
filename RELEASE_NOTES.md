# Release Notes

## v1.0.0 — 20260806

### Added
- Point cloud now published on three separate topics — `camera/frame_pc_ir`, `camera/frame_pc_rgb`, `camera/frame_pc_thermal` — each carrying the same XYZ geometry colored from a different source. Switch which color you're looking at by toggling the display's checkbox in RViz's own Displays panel.
- `voxel3d.fusion_mode` param (`0`=NONE, `1`=RGB2TOF, `2`=FLIR2TOF, `3`=TOF2RGB), configurable from yaml instead of always defaulting to `RectifyType::NONE`.
- Sensor is now released on shutdown: a new `~CameraNode()` destructor joins the capture/process threads and calls `voxel3d_release()` so Ctrl+C no longer leaves the device "open" as far as the SDK/driver is concerned.

### Fixed
- Point cloud publish crashed under ROS1 (`ros::Publisher` has no `operator->`) — now branches `.publish()`/`->publish()` correctly like the image topics already did.
- Depth/confidence colorization was saturating almost the whole frame to one color; depth now scales against a fixed real-world range (`voxel3d.depth_max_range`, default 10000mm) and confidence auto-normalizes per frame.
- `config/voxel3d_params_ros1.yaml` was nested, which ROS1 can't flatten (`/` is its only namespace separator, not `.`), so every param in it silently fell back to its hardcoded default. Rewritten as flat keys.

### Changed
- `voxel3d/info_*_fps` topics switched from `std_msgs/Float64` to `std_msgs/String`, pre-formatted to 1 decimal place.

### Known issues / not in scope for this release
- Same as v0.1.0 below, still open.

## v0.1.0 — 20260727

### Added
- Dual ROS1 (`roscpp`) / ROS2 (`rclcpp`) `voxel3d_node` built from a single source file (`voxel3d_node.cpp`); CMake auto-detects the environment the same way as the sibling `acaas_node` package.
- Drives ToF (depth + confidence + point cloud), RGB (MJPG, decoded via `cv::imdecode`), FLIR/Lepton3 thermal, and embedded IMU via the `voxel3d` SDK; publishes on `camera/frame_depth`, `camera/frame_ir`, `camera/frame_rgb`, `camera/frame_thermal`, `camera/frame_pc`, `imu/accel`, `imu/gyro`.
- Pipelined capture/process threads (`captureLoop()` / `processLoop()`), handed off through a single-slot "latest wins" buffer, so the steady-state per-frame period is roughly `max(capture, process)` instead of the two running back-to-back.
- Runtime-configurable thermal display range via `voxel3d.thermal_min_temp` / `voxel3d.thermal_max_temp` params (`ParamHelper`), read once at startup on both ROS1 and ROS2.
- Publishes a `map → voxel3d_frame` TF every frame.
- Custom RViz2 panel (`voxel3d_panel` / `Voxel3dRvizPanel`) with live RGB/Depth/IR/Thermal previews, IMU accel/gyro/timestamp, per-stream resolution/FOV/FPS, and device identity/firmware info.
- Per-device log tag (`[ INFO. ][ 5HiRab <SN> ]`) in `sensorScan()`/`sensorInit()`'s startup logs — falls back to plain `5HiRab` today (single/auto-selected device), automatically picks up a real S/N once a multi-sensor scan loop is added.

### Fixed
- FLIR crash: `cv::cvtColor` `!_src.empty()` assertion, caused by `lastValidFlir.copyTo(_sensorData->flir)` running even when `lastValidFlir` was still empty (first FLIR query failed before ever succeeding once) — `Mat::copyTo()` from an empty source collapses the destination to empty too. Fixed with an `!lastValidFlir.empty()` guard.
- RGB resize crash: `cv::resize` `inv_scale_x > 0` assertion from resizing into `captured->rgbResized` while it was still an empty/unsized `Mat` — replaced with a straight `.clone()`.
- RGB "flicker": a failed `voxel3d_rgb_queryframe()` or `cv::imdecode()` previously left the published RGB frame at the constructor's all-black default. Now falls back to the last successfully decoded frame (`lastValidRgb`), mirroring the existing FLIR fallback pattern.
- 9 missing publisher initializations (`info_rgb_res/fov/fps`, `info_depth_*`, `info_thermal_*`) — declared and published to in `publish_frames()`, but never actually created via `create_publisher`/`advertise`, so every publish was a null-`shared_ptr` dereference.
- Two RViz panel wild-pointer segfaults from off-by-one `QLabel`-creation loop bounds (sensor-info loop, IMU loop): each left one `QLabel` (`lblSensorLibVersion`, `lblImuAngVelZValue`) never `new`'d, then dereferenced every 33 ms by `updateLabels()`.
- `dev_name`/`product_sn` comparisons using `==` against fixed `char[]` arrays (always false, pointer comparison) — switched to `strcmp(...) == 0`.
- A batch of ROS1-branch typos/type mismatches in `voxel3d_panel` that had gone unbuilt/unverified (`SubDephtImage`, broken `SubThermalImage` subscribe syntax, `SubInfoTheramlFov`, `callbackSensorFwBuildData`, `SubsensorLibVersion`, `std_msge::Float64`, `Voxel3dRvizPnael`, `std_msgs::string` → `std_msgs::String`, wrong `callbackSensorName` param type, stray `;;`).
- ROS2 subscription callbacks changed to take their message `SharedPtr` by value instead of `const SharedPtr&` — `rclcpp`'s `AnySubscriptionCallback` variant only accepts the by-value form for a non-const message type, so the by-const-ref signatures failed to compile once the RViz panel was actually built under ROS2.

### Changed
- Unified all of `voxel3d_node.cpp`'s startup `printf` logging under a single `[ INFO. ][ 5HiRab ]` tag format (previously a mix of `[ INFO. ]` and untagged lines).
- Refactored `voxel3d_panel.hpp`/`.cpp` to remove ROS1/ROS2 duplication: shared message-type aliases (`ImageMsg`/`ImuMsg`/`StringMsg`/`Float64Msg` + their `Ptr`/`Sub` variants) and a `VOXEL3D_SUBSCRIBE(...)` macro collapse each subscription from a duplicated pair down to one line (header 335→289 lines, source 556→473 lines).
- Applied the same collapsing pattern (`VOXEL3D_ADVERTISE(...)`) to `voxel3d_node.cpp`'s constructor for its 18 publisher setups.

### Known issues / not in scope for this release
- `config/voxel3d_params_ros1.yaml` still nests its keys under `acaas:` (`acaas.thermal_min_temp` / `acaas.thermal_max_temp`), copied over from the `acaas_node` sibling package's config. `voxel3d_node.cpp` actually reads the flat parameter names `voxel3d.thermal_min_temp` / `voxel3d.thermal_max_temp`, so loading this yaml as-is via `rosparam load` doesn't apply — the node silently falls back to its built-in defaults (20.0 / 50.0 °C). Needs the yaml's keys corrected to match.
- `voxel3d.depth_max_range` isn't present in either yaml config file yet (defaults to `10000.0` mm in code).
- No launch files yet (unlike `acaas_node`) — run via `ros2 run` / `rosrun` directly; pass `--ros-args --params-file` (ROS2) to override yaml defaults.
- `nav_msgs` and `visualization_msgs` are declared as build dependencies (`package.xml`/`CMakeLists.txt`) but aren't used by any code in `voxel3d_node.cpp` today — likely leftover from copying `acaas_node`'s build files.

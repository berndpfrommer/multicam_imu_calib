# (WORK IN PROGRESS, NOT FUNCTIONAL): Multi-camera and IMU calibration

This repository provides ROS2 support for intrinsic and extrinsic
calibration of multiple synchronized cameras as well as camera-to-IMU extrinsic calibration.

## Supported platforms

Should work under ROS2 versions starting with Foxy.

## How to build
Create a workspace (``~/ws``), and clone this repo:

```bash
pkg=multicam_imu_calib
mkdir -p ~/$pkg/src
cd ~/ws
git clone https://github.com/berndpfrommer/${pkg}.git src/${pkg}
```
Install all system packages that this package depends on:
```bash
rosdep install --from-paths src --ignore-src
```

```bash
cd ~/ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo  # (optionally add -DCMAKE_EXPORT_COMPILE_COMMANDS=1)
```

## How to use

TODO.

## License

This software is issued under the Apache License Version 2.0.

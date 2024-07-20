# Simulating test data for multicam_imu_calib

## How to run simulations for generating test data


1) Start simulator
```
gz sim -r -s --headless-render ./camera.sdf
```

2) Start GUI
```
gz sim -g
```

3) Start bridge
```
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=./bridge.yaml
```

4) Start recording

```
ros2 bag record /camera/image_raw /camera/camera_info /imu /camera/odom
```

5) Make moves for simulation

```
python3 ./control_sim.py
```

# How to run simulations for generating test data


## Start simulator
```
gz sim -r -s --headless-render ./camera.sdf
```

### Start GUI
```
gz sim -g
```

### Start bridge
```
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=./bridge.yaml
```

### Start recording

```
ros2 bag record /camera/image_raw /camera/camera_info /imu /camera/odom
```

### Send simulation targets:

```
python3 ./control_sim.py
```

# Simulating test data for multicam_imu_calib

## How to run simulations for generating test data


1) Start simulator
```
gz sim -r -s --headless-render ./world.sdf
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
ros2 bag record --use-sim-time /camera/image_raw /camera/camera_info /imu /camera/odom /tf /clock
```

5) Make moves for simulation

```
python3 ./control_sim.py
```

## How to play back:

DO NOT use play back with the ``--clock`` option as rosbag2 will then ignore the ``/clock`` messages in the bag.

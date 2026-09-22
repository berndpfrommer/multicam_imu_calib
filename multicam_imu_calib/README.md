# Multi-camera and IMU calibration

This repository provides ROS2 support for intrinsic and extrinsic
calibration of multiple synchronized cameras as well as camera-to-IMU extrinsic calibration.

At the moment, the code base is still very experimental and not well tested! Tread carefully!
The IMU calibration is virtually untested and should NOT BE USED.

## Supported platforms

Should work under ROS2 versions starting with Humble.

## How to build

Set the following shell variables:

```bash
repo=multicam_imu_calib
url=https://github.com/berndpfrommer/${repo}.git
```

and follow the [instructions here](https://github.com/ros-misc-utilities/.github/blob/master/docs/build_ros_repository.md)

## Features

- intrinsic camera calibration with equidistant (fish eye) and standard radial-tangential models
- extrinsic camera calibration
- not really, but somewhat working: extrinsic camera/imu calibration
- multi-target support (you can use multiple calibration boards, so long as their respective pose does not change)
- supports UMich and MIT Apriltag detectors

## Differences to Kalibr

- first-class support for ROS2
- maybe faster in some situations
- built on top of GTSAM
- supports calibration with multiple targets (must be static!)
- not as robust
- not as well documented
- not as well tested
- no support for checker boards
- no corner refinement
- no nice diagnostics
- requires more elaborate input file with starting guesses for intrinsics

## How to use

### Input file

You need:

- The images in a standard rosbag. For stereo cameras, the topics must be synchronized, i.e. the header time stamps of the cameras have to match.
- A calibration input file


Here is an example input file:

```yaml
targets:
  - name: "apriltag board mit"
    type: "apriltag_board"
    detector: "mit"
    family: "tf36h11"
    border_width: 2
    starting_tag_id: 0
    tag_size: 0.1         # tag edge length in meters
    rows: 9
    distance_rows: 0.13   # in meters!
    columns: 7
    distance_columns: 0.13
  
cameras:
  - name: eb_0
    image_topic: /event_camera/events/image
    image_transport: raw
    pixel_noise: 1.0
    intrinsics:
      fx: 1200.0
      fy: 1200.0
      cx: 640.0
      cy: 360.0
    resolution: [1280, 720]
    distortion_model:
      type: radtan
      coefficients:
        - -0.23
        - 0.25
        - 0.0
        - 0.0
```

The calibration board specification is similar to Kalibr, but the distance between rows and columns is given directly in meters (e.g. the distance between left tag edge of a column and the left tag edge of the next column)

### Calibration from a rosbag

```bash
ros2 run multicam_imu_calib calibrate_from_bag --ros-args -p config_file:=./config/sim.yaml -p in_bag:=$path_to_input_bag -p out_bag:=$path_to_output_bag
```

This will produce calibration output results in the output directory:

- ``calibrations.yaml``: calibration output result with error estimates (diagonals of covariance matrices)
- ``${path_to_output_bag}``: ROS messages with pose of the camera rig, transforms, apriltag detections, and debug images (with tags drawn onto)
- ``projections.txt``: ROS header timestamps (first column), observed image points (2nd + 3rd) and projected image points (4th and 5th)

Hint: when debugging outlier points (found in ``projections.txt``) make sure to look at the header stamp of the corresponding image in
the rosbag. You can view the header stamp with ``rqt_bag`` by looking at ``raw`` data alongside the image.

After the calibration has finished, the results can be displayed with the diagnostics python script:

```bash
ros2 run multicam_imu_calib diagnostics.py -f ./results/projections.txt
```

### Online calibration

Not working yet.

## License

This software is issued under the Apache License Version 2.0.

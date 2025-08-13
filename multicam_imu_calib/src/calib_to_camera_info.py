#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Copyright 2025 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#

import yaml
import argparse
import numpy as np


def read_yaml(filename):
    with open(filename, "r") as y:
        try:
            return yaml.safe_load(y)
        except yaml.YAMLError as e:
            print(e)


def write_yaml(ydict, fname):
    with open(fname, "w") as f:
        try:
            yaml.dump(ydict, f)
        except yaml.YAMLError as y:
            print("Error:", y)


def to_camerainfo(k, camera_input_name, camera_output_name):
    y = {}
    intr = k["intrinsics"]
    # make K-matrix from intrinsics
    intrinsics = np.array(
        [[intr["fx"], 0, intr["cx"]], [0, intr["fy"], intr["cy"]], [0, 0, 1]]
    )

    distortion_model = k["distortion_model"]["type"]
    if distortion_model == "radtan":
        distortion_model = "plumb_bob"
    elif distortion_model == "equidistant":
        distortion_model = "fisheye"
    y["image_width"] = 1920
    y["image_height"] = 1024
    y["camera_name"] = (
        k["image_topic"] if camera_input_name is None else camera_input_name
    )
    y["camera_matrix"] = {"rows": 3, "cols": 3, "data": intrinsics.flatten().tolist()}
    y["distortion_model"] = distortion_model
    dc = k["distortion_model"]["coefficients"]
    y["distortion_coefficients"] = {"rows": 1, "cols": len(dc), "data": dc}
    y["rectification_matrix"] = {
        "rows": 3,
        "cols": 3,
        "data": np.eye(3).flatten().tolist(),
    }
    y["projection_matrix"] = {
        "rows": 3,
        "cols": 4,
        "data": np.hstack([intrinsics, np.array([[0], [0], [0]])]).flatten().tolist(),
    }
    return y


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-i", "--input", required=True, help="input file in multicam_imu format"
    )
    parser.add_argument(
        "-o", "--output", required=True, help="output file in opencv/camerainfo format"
    )
    parser.add_argument(
        "-c", "--camera", default="camera_0", help="name of camera to extract"
    )
    parser.add_argument(
        "-n", "--camera_name", default=None, help="name of camera in camerainfo file"
    )
    args = parser.parse_args()

    calib_dict = read_yaml(args.input)

    for cam in calib_dict["cameras"]:
        if cam["name"] == args.camera:
            camerainfo_dict = to_camerainfo(cam, args.camera, args.camera_name)
            write_yaml(camerainfo_dict, args.output)


if __name__ == "__main__":
    main()

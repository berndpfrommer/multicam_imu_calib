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
import tf_transformations


def inline_ndarray_representer(dumper: yaml.Dumper, array: np.ndarray) -> yaml.Node:
    node = dumper.represent_list(array.tolist())
    node.flow_style = True
    return node


yaml.add_representer(np.ndarray, inline_ndarray_representer)


def read_yaml(filename):
    with open(filename, "r") as y:
        try:
            return yaml.safe_load(y)
        except yaml.YAMLError as e:
            print(e)


def write_yaml(ydict, fname):
    with open(fname, "w") as f:
        try:
            yaml.dump(ydict, f, sort_keys=False, default_flow_style=False)
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
    y["image_width"] = k["resolution"][0]
    y["image_height"] = k["resolution"][1]
    y["camera_name"] = camera_output_name
    y["camera_matrix"] = {"rows": 3, "cols": 3, "data": intrinsics.flatten()}
    y["distortion_model"] = distortion_model
    dc = k["distortion_model"]["coefficients"]
    y["distortion_coefficients"] = {"rows": 1, "cols": len(dc), "data": np.array(dc)}
    y["rectification_matrix"] = {
        "rows": 3,
        "cols": 3,
        "data": np.eye(3).flatten(),
    }
    y["projection_matrix"] = {
        "rows": 3,
        "cols": 4,
        "data": np.hstack([intrinsics, np.array([[0], [0], [0]])]).flatten(),
    }
    return y


def tf_to_yaml(tf):
    return {
        "rows": 3,
        "cols": 4,
        "data": tf.flatten(),
    }


def get_camera(calib_dict, name):
    for cam in calib_dict["cameras"]:
        if cam["name"] == name:
            return cam
    return None


def read_poses(calib_dict, camera_names):
    poses = []
    for cam_name in camera_names:
        cam = get_camera(calib_dict, cam_name)
        if cam is None:
            raise Exception(f"camera not found: {cam_name}")
        pose = cam["pose"]
        pq = pose["orientation"]
        q = np.array((pq["x"], pq["y"], pq["z"], pq["w"]))
        p = pose["position"]
        t = np.array((p["x"], p["y"], p["z"]))
        se3_matrix = tf_transformations.quaternion_matrix(q)
        se3_matrix[0:3, 3] = t
        poses.append(se3_matrix)
    return poses


def compute_transforms(calib_dict, camera_names):
    poses = read_poses(calib_dict, camera_names)
    if len(poses) != len(camera_names):
        raise Exception("need poses for all cameras!")
    if len(poses) < 2:
        return [tf_transformations.identity_matrix()]

    rel_transforms = []
    for i in range(1, len(poses)):
        T_w_a = poses[i - 1]
        T_w_b = poses[i]
        T_b_w = tf_transformations.inverse_matrix(T_w_b)
        T_b_a = np.dot(T_b_w, T_w_a)
        if i == 1:  # append two times for the first transform
            rel_transforms.append(T_b_a)
        rel_transforms.append(T_b_a)

    return rel_transforms


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-i", "--input", required=True, help="input file in multicam_imu format"
    )
    parser.add_argument(
        "-c",
        "--camera_names",
        nargs="+",
        type=str,
        help="space-separated list of camera names to extract",
    )
    parser.add_argument(
        "-o",
        "--output_names",
        nargs="+",
        default=None,
        help="space-separated list of output camera names",
    )
    args = parser.parse_args()

    out_names = args.camera_names if args.output_names is None else args.output_names
    if len(out_names) != len(args.camera_names):
        raise Exception("number of output_names must match camera_names!")

    calib_dict = read_yaml(args.input)

    transforms = compute_transforms(calib_dict, args.camera_names)

    for i, cam_name in enumerate(args.camera_names):
        cam = get_camera(calib_dict, cam_name)
        if cam is not None:
            camerainfo_dict = to_camerainfo(cam, args.camera_names[i], out_names[i])
            tf_idx = 1 if i == 0 else i
            tf_sym = "T_" + out_names[tf_idx] + "_" + out_names[tf_idx - 1]
            camerainfo_dict[tf_sym] = tf_to_yaml(transforms[tf_idx])
            T_b_c = tf_to_yaml(tf_transformations.identity_matrix())
            camerainfo_dict["T_b_c"] = tf_to_yaml(tf_transformations.identity_matrix())
            write_yaml(camerainfo_dict, out_names[i] + ".yaml")
        else:
            print(f("WARNING: camera with name {cam_name} not found in file!"))


if __name__ == "__main__":
    main()

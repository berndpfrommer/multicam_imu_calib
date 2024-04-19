#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

import argparse

import matplotlib.pyplot as plt
import numpy as np


def main(fname):
    a = np.loadtxt(fname)
    cam_idx = np.unique(a[:, 0]).astype(np.uint32)
    cutoff = 5
    for i in cam_idx:
        pts = a[a[:, 0].astype(np.uint32) == i, 2:]
        dx = pts[:, 0] - pts[:, 2]
        dy = pts[:, 1] - pts[:, 3]
        hx, bx = np.histogram(dx[np.abs(dx) < np.std(dx) * cutoff], bins=100, density=True)
        plt.plot(0.5 * (bx[:-1] + bx[1:]), hx, label='error in x')
        hy, by = np.histogram(dy[np.abs(dy) < np.std(dy) * cutoff], bins=100, density=True)
        plt.plot(0.5 * (by[:-1] + by[1:]), hy, label='error in y')
        plt.legend()
        plt.title(f'error distribution for camera {i}')
        plt.show()

        plt.scatter(pts[:, 0], pts[:, 1], s=8, label='image points')
        plt.scatter(pts[:, 2], pts[:, 3], s=8, label='projected points')
        plt.title(f'projection error cam {i}')
        plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='calibration diagnostics.')
    parser.add_argument(
        '--projection_file_name',
        '-f',
        help='name of file with projected points',
        required=True,
        default=None,
    )
    args = parser.parse_args()
    main(args.projection_file_name)

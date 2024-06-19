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

import launch
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    """Create composable node."""
    container = ComposableNodeContainer(
        name='calibration_container',
        namespace=LaunchConfig('ns'),
        package='rclcpp_components',
        # prefix=['xterm -e gdb -ex run --args'],
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                name='front_end',
                package='multicam_imu_calib',
                plugin='multicam_imu_calib::FrontEndComponent',
                namespace=LaunchConfig('ns'),
                parameters=[{'config_file': LaunchConfig('config_file')}],
                remappings=[('foo', 'bar')],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )
    return [container]


def generate_launch_description():
    """Create composable node by calling opaque function."""
    return launch.LaunchDescription(
        [
            LaunchArg(
                'ns',
                default_value=['multicam_imu_calibration'],
                description='namespace for calibration',
            ),
            LaunchArg('config_file', description='path to configuration file'),
            OpaqueFunction(function=launch_setup),
        ]
    )

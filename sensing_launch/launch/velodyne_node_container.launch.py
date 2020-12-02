# Copyright 2020 Tier IV, Inc. All rights reserved.
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

import launch
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackage
from pathlib import Path
import os

context = LaunchContext()


def get_package_share_directory(package_name):
    """Return the absolute path to the share directory of the given package."""
    return os.path.join(Path(FindPackage(package_name).perform(context)), 'share', package_name)


default_values_dict = {
    'VLP16': {'calibration': 'VLP16db', 'nbeams': 16, 'min_range': 0.4, 'max_range': 130.0},
    '32C': {'calibration': 'VeloView-CLP-32C.yaml', 'nbeams': 32, 'min_range': 0.4,
            'max_range': 200.0},
    'VLS128': {'calibration': 'VLS-128_FS1.yaml', 'nbeams': 128, 'min_range': 0.5,
               'max_range': 250.0},
}


def invalid_intensity(nbeams: int):
    return [0] * nbeams


def generate_launch_description():
    model_param = DeclareLaunchArgument(
        'model',
        description='velodyne sensor model; one of {}'.format(default_values_dict.keys())
    ),

    default_values = default_values_dict[LaunchConfiguration('model')]

    launch_arguments = [model_param]

    def add_launch_arg_default(name: str):
        launch_arguments.append(DeclareLaunchArgument(name, default=default_values[name]))

    def add_launch_arg(name: str, default: str):
        launch_arguments.append(DeclareLaunchArgument(name, default=default))

    add_launch_arg('launch_driver', True)

    velodyne_pointcloud_prefix = get_package_share_directory('velodyne_pointcloud')
    config_file_name = os.path.join(velodyne_pointcloud_prefix,
                                    'params/{}'.format(default_values['calibration']))
    add_launch_arg('calibration', config_file_name)

    add_launch_arg('device_ip', '192.168.1.201')
    add_launch_arg('sensor_frame', 'velodyne')
    add_launch_arg('base_frame', 'base_link')
    add_launch_arg('container_name', 'velodyne_composable_node_container')
    add_launch_arg_default('min_range')
    add_launch_arg_default('max_range')
    add_launch_arg('pcap', '')
    add_launch_arg('port', 2368)
    add_launch_arg('read_fast', False)
    add_launch_arg('read_once', False)
    add_launch_arg('repeat_delay', 0.0)
    add_launch_arg('rpm', 600.0)
    add_launch_arg('laserscan_ring', -1)
    add_launch_arg('laserscan_resolution', 0.007)
    add_launch_arg('num_points_thresholds', 300)
    add_launch_arg('invalid_intensity', invalid_intensity(default_values['nbeams']))

    def create_parameter_dict(*args):
        result = {}
        for x in args:
            result[x] = LaunchConfiguration(x)
        return result

    nodes = []

    if LaunchConfiguration('launch_driver'):
        # load driver as in
        # https://github.com/ros-drivers/velodyne/blob/ros2/velodyne_driver/launch/velodyne_driver_node-VLP16-composed-launch.py
        # velodyne_driver_prefix = get_package_share_directory('velodyne_driver')
        # config_file_name = os.path.join(velodyne_driver_prefix, 'params/{}.yaml'.format(default_values['calibration']))
        nodes.append(ComposableNode(
            package='velodyne_driver',
            node_plugin='velodyne_driver::VelodyneDriver',
            node_name='velodyne_driver_node',
            parameters=[create_parameter_dict('device_ip', 'frame_id', 'model', 'pcap', 'port',
                                              'read_fast', 'read_once', 'repeat_delay', 'rpm')
                        ],
        )
        )

    # turn packets into pointcloud as in
    # https://github.com/ros-drivers/velodyne/blob/ros2/velodyne_pointcloud/launch/velodyne_convert_node-VLP16-composed-launch.py
    nodes.append(ComposableNode(
        package='velodyne_pointcloud',
        node_plugin='velodyne_pointcloud::Convert',
        node_name='velodyne_convert_node',
        parameters=[create_parameter_dict('velodyne_points', 'velodyne_points_ex', 'calibration',
                                          'min_range', 'max_range', 'num_points_thresholds',
                                          'invalid_intensity')]
    )
    )

    cropbox_parameters = create_parameter_dict('input_frame', 'output_frame')
    cropbox_parameters['negative'] = True

    cropbox_remappings = [
        ('/min_x', '/vehicle_info/min_longitudinal_offset'),
        ('/max_x', '/vehicle_info/max_longitudinal_offset'),
        ('/min_z', '/vehicle_info/min_lateral_offset'),
        ('/max_z', '/vehicle_info/max_lateral_offset'),
        ('/min_z', '/vehicle_info/min_height_offset'),
        ('/max_z', '/vehicle_info/max_height_offset'),
    ]

    nodes.append(ComposableNode(
        package='pointcloud_preprocessor',
        plugin='pointcloud_preprocessor::CropBoxFilterComponent',
        name='crop_box_filter_self',
        remappings=[('/input', 'pointcloud_raw_ex'),
                    ('/output', 'self_cropped/pointcloud_ex')
                    ] + cropbox_remappings,
        parameters=[cropbox_parameters],
        output='log',
    )
    )

    nodes.append(ComposableNode(
        package='pointcloud_preprocessor',
        plugin='pointcloud_preprocessor::CropBoxFilterComponent',
        name='crop_box_filter_mirror',
        remappings=[('/input', 'self_cropped/pointcloud_ex'),
                    ('/output', 'mirror_cropped/pointcloud_ex'),
                    ] + cropbox_remappings,
        parameters=[cropbox_parameters],
        output='log',
    )
    )

    # TODO(fred-apex-ai) Still need the distortion component
    if False:
        nodes.append(ComposableNode(
            package='TODO',
            plugin='TODO',
            name='fix_distortion',
            remappings=[
                ('velodyne_points_ex', 'mirror_cropped/pointcloud_ex'),
                ('velodyne_points_interpolate', 'rectified/pointcloud'),
                ('velodyne_points_interpolate_ex', 'rectified/pointcloud_ex'),
            ],
        )
        )

    nodes.append(ComposableNode(
        package='pointcloud_preprocessor',
        plugin='pointcloud_preprocessor::RingOutlierFilterComponent',
        name='ring_outlier_filter',
        remappings=[
            ('/input', 'rectified/pointcloud_ex'),
            ('/output', 'outlier_filtered/pointcloud')
        ],
    )
    )

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name=LaunchConfiguration('container_name'),
        namespace='pointcloud_preprocessor',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=nodes,
    )

    return launch.LaunchDescription(launch_arguments + [container])

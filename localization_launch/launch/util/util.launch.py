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
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml


def launch_setup(context, *args, **kwargs):
    # https://github.com/ros2/launch_ros/issues/156
    def load_composable_node_param(param_path):
        with open(LaunchConfiguration(param_path).perform(context), "r") as f:
            return yaml.safe_load(f)["/**"]["ros__parameters"]

    crop_box_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::CropBoxFilterComponent",
        name="crop_box_filter_measurement_range",
        remappings=[
            ("input", LaunchConfiguration("input/pointcloud")),
            ("output", "measurement_range/pointcloud"),
        ],
        parameters=[
            load_composable_node_param("crop_box_filter_measurement_range_param_path"),
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )
    voxel_grid_downsample_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::VoxelGridDownsampleFilterComponent",
        name="voxel_grid_downsample_filter",
        remappings=[
            ("input", "measurement_range/pointcloud"),
            ("output", "voxel_grid_downsample/pointcloud"),
        ],
        parameters=[load_composable_node_param("voxel_grid_downsample_filter_param_path")],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )
    random_downsample_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::RandomDownsampleFilterComponent",
        name="random_downsample_filter",
        remappings=[
            ("input", "voxel_grid_downsample/pointcloud"),
            ("output", LaunchConfiguration("output/pointcloud")),
        ],
        parameters=[load_composable_node_param("random_downsample_filter_param_path")],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    composable_nodes = [
        crop_box_component,
        voxel_grid_downsample_component,
        random_downsample_component,
    ]

    individual_container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=composable_nodes,
        condition=UnlessCondition(LaunchConfiguration("use_pointcloud_container")),
        output="screen",
    )
    pointcloud_container_loader = LoadComposableNodes(
        composable_node_descriptions=composable_nodes,
        target_container=LaunchConfiguration("container_name"),
        condition=IfCondition(LaunchConfiguration("use_pointcloud_container")),
    )
    return [individual_container, pointcloud_container_loader]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        arg = DeclareLaunchArgument(name, default_value=default_value, description=description)
        launch_arguments.append(arg)

    add_launch_arg(
        "crop_box_filter_measurement_range_param_path",
        [
            FindPackageShare("localization_launch"),
            "/config/crop_box_filter_measurement_range.param.yaml",
        ],
        "path to the parameter file of crop_box_filter_measurement_range",
    )
    add_launch_arg(
        "voxel_grid_downsample_filter_param_path",
        [FindPackageShare("localization_launch"), "/config/voxel_grid_filter.param.yaml"],
        "path to the parameter file of voxel_grid_downsample_filter",
    )
    add_launch_arg(
        "random_downsample_filter_param_path",
        [FindPackageShare("localization_launch"), "/config/random_downsample_filter.param.yaml"],
        "path to the parameter file of random_downsample_filter",
    )
    add_launch_arg("use_intra_process", "true", "use ROS2 component container communication")
    add_launch_arg("use_multithread", "False")
    add_launch_arg("use_pointcloud_container", "False")
    add_launch_arg("container_name", "pointcloud_container")

    add_launch_arg(
        "output/pointcloud",
        "downsample/pointcloud",
        "final output topic name",
    )

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return launch.LaunchDescription(
        launch_arguments
        + [set_container_executable, set_container_mt_executable]
        + [OpaqueFunction(function=launch_setup)]
    )

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


class DetectionPreProcessPipeline:
    def __init__(self, context):
        self.output_topic = (
            "/perception/object_recognition/detection/pointcloud_map_filtered/pointcloud"
        )

    def create_common_pipeline(self, output_topic):
        components = []
        downsample_topic = "/perception/obstacle_segmentation/downsampled/pointcloud"
        components.append(
            ComposableNode(
                package="pointcloud_preprocessor",
                plugin="pointcloud_preprocessor::VoxelGridDownsampleFilterComponent",
                name="voxel_grid_downsample_filter",
                remappings=[
                    ("input", LaunchConfiguration("input_topic")),
                    ("output", downsample_topic),
                ],
                parameters=[
                    {
                        "voxel_size_x": 0.1,
                        "voxel_size_y": 0.1,
                        "voxel_size_z": 0.1,
                    }
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            )
        )

        components.append(
            ComposableNode(
                package="compare_map_segmentation",
                plugin="compare_map_segmentation::VoxelBasedCompareMapFilterComponent",
                name="voxel_based_compare_map_filter",
                remappings=[
                    ("input", downsample_topic),
                    ("map", "/map/pointcloud_map"),
                    ("output", output_topic),
                ],
                parameters=[
                    {
                        "distance_threshold": 0.5,
                    }
                ],
                extra_arguments=[
                    {"use_intra_process_comms": False},
                ],
            )
        )
        return components


def launch_setup(context, *args, **kwargs):
    pipeline = DetectionPreProcessPipeline(context)

    components = []
    components.extend(
        pipeline.create_common_pipeline(
            output_topic=pipeline.output_topic,
        )
    )

    individual_container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=components,
        condition=UnlessCondition(LaunchConfiguration("use_pointcloud_container")),
        output="screen",
    )
    pointcloud_container_loader = LoadComposableNodes(
        composable_node_descriptions=components,
        target_container=LaunchConfiguration("container_name"),
        condition=IfCondition(LaunchConfiguration("use_pointcloud_container")),
    )
    return [individual_container, pointcloud_container_loader]


def generate_launch_description():

    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg("input_topic", "")
    add_launch_arg("use_multithread", "False")
    add_launch_arg("use_intra_process", "True")
    add_launch_arg("use_pointcloud_container", "False")
    add_launch_arg("container_name", "detection_preprocess_pipeline_container")

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

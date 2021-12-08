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
import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml


def get_vehicle_info(context):
    path = LaunchConfiguration("vehicle_param_file").perform(context)
    with open(path, "r") as f:
        p = yaml.safe_load(f)["/**"]["ros__parameters"]
    p["vehicle_length"] = p["front_overhang"] + p["wheel_base"] + p["rear_overhang"]
    p["vehicle_width"] = p["wheel_tread"] + p["left_overhang"] + p["right_overhang"]
    p["min_longitudinal_offset"] = -p["rear_overhang"]
    p["max_longitudinal_offset"] = p["front_overhang"] + p["wheel_base"]
    p["min_lateral_offset"] = -(p["wheel_tread"] / 2.0 + p["right_overhang"])
    p["max_lateral_offset"] = p["wheel_tread"] / 2.0 + p["left_overhang"]
    p["min_height_offset"] = 0.0
    p["max_height_offset"] = p["vehicle_height"]
    return p


def get_vehicle_mirror_info(context):
    path = LaunchConfiguration("vehicle_mirror_param_file").perform(context)
    with open(path, "r") as f:
        p = yaml.safe_load(f)
    return p


def create_additional_pipeline(vehicle_info, lidar_name, ground_segmentation_param):
    components = []
    components.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::CropBoxFilterComponent",
            name=f"{lidar_name}_crop_box_filter",
            remappings=[
                ("input", f"/sensing/lidar/{lidar_name}/outlier_filtered/pointcloud"),
                ("output", f"{lidar_name}/measurement_range_cropped/pointcloud"),
            ],
            parameters=[
                {
                    "input_frame": LaunchConfiguration("base_frame"),
                    "output_frame": LaunchConfiguration("base_frame"),
                    "min_z": vehicle_info["min_height_offset"],
                    "max_z": vehicle_info["max_height_offset"],
                },
                ground_segmentation_param[f"{lidar_name}_crop_box_filter"]["parameters"],
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    components.append(
        ComposableNode(
            package="ground_segmentation",
            plugin=ground_segmentation_param[f"{lidar_name}_ground_filter"]["plugin"],
            name=f"{lidar_name}_ground_filter",
            remappings=[
                ("input", f"{lidar_name}/measurement_range_cropped/pointcloud"),
                ("output", f"{lidar_name}/no_ground/pointcloud"),
            ],
            parameters=[ground_segmentation_param[f"{lidar_name}_ground_filter"]["parameters"]],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    return components


def create_ransac_pipeline(ground_segmentation_param):
    livox_concat_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent",
        name="livox_concatenate_data",
        remappings=[("output", "livox_concatenated/pointcloud")],
        parameters=[
            {
                "input_topics": ground_segmentation_param["ransac_input_topics"],
                "output_frame": LaunchConfiguration("base_frame"),
                "timeout_sec": 1.0,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    short_height_obstacle_detection_area_filter_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::CropBoxFilterComponent",
        name="short_height_obstacle_detection_area_filter",
        remappings=[
            ("input", "livox_concatenated/pointcloud"),
            ("output", "short_height_obstacle_detection_area/pointcloud"),
        ],
        parameters=[
            {
                "input_frame": LaunchConfiguration("base_frame"),
                "output_frame": LaunchConfiguration("base_frame"),
            },
            ground_segmentation_param["short_height_obstacle_detection_area_filter"]["parameters"],
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    vector_map_filter_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::Lanelet2MapFilterComponent",
        name="vector_map_filter",
        remappings=[
            ("input/pointcloud", "short_height_obstacle_detection_area/pointcloud"),
            ("input/vector_map", "/map/vector_map"),
            ("output", "vector_map_filtered/pointcloud"),
        ],
        parameters=[
            {
                "voxel_size_x": 0.25,
                "voxel_size_y": 0.25,
            }
        ],
        # cannot use intra process because vector map filter uses transient local.
        extra_arguments=[{"use_intra_process_comms": False}],
    )

    ransac_ground_filter_component = ComposableNode(
        package="ground_segmentation",
        plugin="ground_segmentation::RANSACGroundFilterComponent",
        name="ransac_ground_filter",
        remappings=[
            ("input", "vector_map_filtered/pointcloud"),
            ("output", "short_height/no_ground/pointcloud"),
        ],
        parameters=[ground_segmentation_param["ransac_ground_filter"]["parameters"]],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    return [
        livox_concat_component,
        short_height_obstacle_detection_area_filter_component,
        vector_map_filter_component,
        ransac_ground_filter_component,
    ]


def create_common_pipeline(vehicle_info, ground_segmentation_param, input_topic, output_topic):
    components = []
    components.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter",
            remappings=[
                ("input", input_topic),
                ("output", "measurement_range_cropped/pointcloud"),
            ],
            parameters=[
                {
                    "input_frame": LaunchConfiguration("base_frame"),
                    "output_frame": LaunchConfiguration("base_frame"),
                    "min_z": vehicle_info["min_height_offset"],
                    "max_z": vehicle_info["max_height_offset"],
                },
                ground_segmentation_param["common_crop_box_filter"]["parameters"],
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    components.append(
        ComposableNode(
            package="ground_segmentation",
            plugin=ground_segmentation_param["common_ground_filter"]["plugin"],
            name="common_ground_filter",
            remappings=[
                ("input", "measurement_range_cropped/pointcloud"),
                ("output", output_topic),
            ],
            parameters=[ground_segmentation_param["common_ground_filter"]["parameters"]],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )
    return components


def get_additional_lidars_concatenated_component(input_topics, output_topic):

    return ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent",
        name="concatenate_data",
        remappings=[("output", output_topic)],
        parameters=[
            {
                "input_topics": input_topics,
                "output_frame": LaunchConfiguration("base_frame"),
            }
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )


def get_single_frame_obstacle_segmentation_concatenated_component(input_topics, output_topic):
    return ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent",
        name="concatenate_no_ground_data",
        remappings=[("output", output_topic)],
        parameters=[
            {
                "input_topics": input_topics,
                "output_frame": LaunchConfiguration("base_frame"),
            }
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )


def create_single_frame_obstacle_segmentation_components(
    ground_segmentation_param, vehicle_info, input_topic, output_topic
):

    additional_lidars = ground_segmentation_param["additional_lidars"]
    use_ransac = bool(ground_segmentation_param["ransac_input_topics"])
    use_additional = bool(additional_lidars)
    relay_topic = "no_ground/oneshot/pointcloud"
    common_pipeline_output = "no_ground/pointcloud" if use_additional or use_ransac else output_topic

    components = create_common_pipeline(
        vehicle_info,
        ground_segmentation_param,
        input_topic=input_topic,
        output_topic=common_pipeline_output,
    )

    if use_additional:
        for lidar_name in additional_lidars:
            components.extend(create_additional_pipeline(vehicle_info, lidar_name, ground_segmentation_param))
        components.append(
            get_additional_lidars_concatenated_component(
                input_topics=[common_pipeline_output]
                + list(map(lambda x: f"{x}/no_ground/pointcloud"), additional_lidars),
                output_topic=relay_topic if use_ransac else output_topic,
            )
        )

    if use_ransac:
        components.append(create_ransac_pipeline(ground_segmentation_param))
        components.append(
            get_single_frame_obstacle_segmentation_concatenated_component(
                input_topics=[
                    "short_height/no_ground/pointcloud",
                    relay_topic if use_additional else common_pipeline_output,
                ],
                output_topic=output_topic,
            )
        )

    return components


def create_time_series_outlier_filter_components(input_topic, output_topic):
    components = []
    components.append(
        ComposableNode(
            package="occupancy_grid_map_outlier_filter",
            plugin="occupancy_grid_map_outlier_filter::OccupancyGridMapOutlierFilterComponent",
            name="occupancy_grid_map_outlier_filter",
            remappings=[
                ("~/input/occupancy_grid_map", "/perception/occupancy_grid_map/map"),
                ("~/input/pointcloud", input_topic),
                ("~/output/pointcloud", output_topic),
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    return components


def create_single_frame_outlier_filter_components(input_topic, output_topic):
    components = []
    components.append(
        ComposableNode(
            package="elevation_map_loader",
            plugin="ElevationMapLoaderNode",
            name="elevation_map_loader",
            remappings=[
                ("output/elevation_map", "elevation_map"),
                ("input/pointcloud_map", "/map/pointcloud_map"),
                ("input/vector_map", "/map/vector_map"),
            ],
            parameters=[
                {
                    "use_lane_filter": False,
                    "use_inpaint": True,
                    "inpaint_radius": 1.0,
                    "param_file_path": PathJoinSubstitution(
                        [
                            FindPackageShare("perception_launch"),
                            "config",
                            "obstacle_segmentation",
                            "ground_segmentation",
                            "elevation_map_parameters.yaml",
                        ]
                    ),
                    "elevation_map_directory": PathJoinSubstitution(
                        [FindPackageShare("elevation_map_loader"), "data", "elevation_maps"]
                    ),
                    "use_elevation_map_cloud_publisher": False,
                }
            ],
            extra_arguments=[{"use_intra_process_comms": False}],
        )
    )

    components.append(
        ComposableNode(
            package="compare_map_segmentation",
            plugin="compare_map_segmentation::CompareElevationMapFilterComponent",
            name="compare_elevation_map_filter",
            remappings=[
                ("input", input_topic),
                ("output", "map_filtered/pointcloud"),
                ("input/elevation_map", "elevation_map"),
            ],
            parameters=[
                {
                    "map_frame": "map",
                    "map_layer_name": "elevation",
                    "height_diff_thresh": 0.15,
                    "input_frame": "map",
                    "output_frame": "base_link",
                }
            ],
            extra_arguments=[{"use_intra_process_comms": False}],  # can't use this with transient_local
        )
    )

    components.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::VoxelGridDownsampleFilterComponent",
            name="voxel_grid_filter",
            remappings=[
                ("input", "map_filtered/pointcloud"),
                ("output", "voxel_grid_filtered/pointcloud"),
            ],
            parameters=[
                {
                    "input_frame": LaunchConfiguration("base_frame"),
                    "output_frame": LaunchConfiguration("base_frame"),
                    "voxel_size_x": 0.04,
                    "voxel_size_y": 0.04,
                    "voxel_size_z": 0.08,
                }
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    components.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::VoxelGridOutlierFilterComponent",
            name="voxel_grid_outlier_filter",
            remappings=[
                ("input", "voxel_grid_filtered/pointcloud"),
                ("output", output_topic),
            ],
            parameters=[
                {
                    "voxel_size_x": 0.4,
                    "voxel_size_y": 0.4,
                    "voxel_size_z": 100.0,
                    "voxel_points_threshold": 5,
                }
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    return components


def launch_setup(context, *args, **kwargs):

    vehicle_info = get_vehicle_info(context)

    ground_segmentation_param_path = os.path.join(
        get_package_share_directory("perception_launch"),
        "config/obstacle_segmentation/ground_segmentation/ground_segmentation.param.yaml",
    )
    with open(ground_segmentation_param_path, "r") as f:
        ground_segmentation_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    single_frame_obstacle_seg_output = "/perception/obstacle_segmentation/single_frame/pointcloud_raw"
    use_single_frame_filter = ground_segmentation_param["use_single_frame_filter"]
    use_time_series_filter = ground_segmentation_param["use_time_series_filter"]
    relay_topic = "single_frame/filtered/pointcloud"
    output_topic = "/perception/obstacle_segmentation/pointcloud"

    components = []
    components.extend(
        create_single_frame_obstacle_segmentation_components(
            ground_segmentation_param,
            vehicle_info,
            input_topic="/sensing/lidar/concatenated/pointcloud",
            output_topic=single_frame_obstacle_seg_output,
        )
    )
    if use_single_frame_filter:
        components.extend(
            create_single_frame_outlier_filter_components(
                input_topic=single_frame_obstacle_seg_output,
                output_topic=relay_topic if use_time_series_filter else output_topic,
            )
        )
    if use_time_series_filter:
        components.extend(
            create_time_series_outlier_filter_components(
                input_topic=relay_topic if use_single_frame_filter else single_frame_obstacle_seg_output,
                output_topic=output_topic,
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

    add_launch_arg("base_frame", "base_link")
    add_launch_arg("vehicle_param_file")
    add_launch_arg("use_multithread", "False")
    add_launch_arg("use_intra_process", "True")
    add_launch_arg("use_pointcloud_container", "False")
    add_launch_arg("container_name", "perception_pipeline_container")

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

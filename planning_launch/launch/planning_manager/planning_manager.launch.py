# Copyright 2022 Tier IV, Inc. All rights reserved.
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
from launch.actions import SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name="planning_manager_container",
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[
            ComposableNode(
                package="planning_manager",
                plugin="planning_manager::PlanningManagerNode",
                name="planning_manager_node",
                remappings=[
                    (
                        "~/input/route",
                        "/planning/mission_planning/route",
                    ),
                    ("~/input/vector_map", LaunchConfiguration("map_topic_name")),
                    ("~/input/predicted_objects", "/perception/object_recognition/objects"),
                    ("~/input/odometry", "/localization/kinematic_state"),
                    (
                        "~/input/external_approval",
                        "/planning/scenario_planning/lane_driving/behavior_planning/"
                        "behavior_path_planner/path_change_approval",
                    ),
                    (
                        "~/input/force_approval",
                        "/planning/scenario_planning/lane_driving/behavior_planning/"
                        "behavior_path_planner/path_change_force",
                    ),
                    (
                        "~/srv/behavior_path_planner/plan",
                        "/planning/scenario_planning/lane_driving/behavior_planning/"
                        "behavior_path_planner/plan",
                    ),
                    (
                        "~/srv/behavior_path_planner/validate",
                        "/planning/scenario_planning/lane_driving/behavior_planning/"
                        "behavior_path_planner/validate",
                    ),
                    (
                        "~/srv/behavior_velocity_planner/plan",
                        "/planning/scenario_planning/lane_driving/behavior_planning/"
                        "behavior_velocity_planner/plan",
                    ),
                    (
                        "~/srv/behavior_velocity_planner/validate",
                        "/planning/scenario_planning/lane_driving/behavior_planning/"
                        "behavior_velocity_planner/validate",
                    ),
                    (
                        "~/output/trajectory",
                        "/tmp"
                        "/planning/scenario_planning/trajectory",
                    ),
                ],
            ),
        ],
        output="screen",
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
    )

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument("map_topic_name", default_value="/map/vector_map"),
            set_container_mt_executable,
            container,
        ]
    )

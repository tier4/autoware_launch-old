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
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml
import uuid


def search_contains(d):
    result = []

    if not d:
        return None
    elif isinstance(d, dict):
        for k, v in d.items():
            if k == "contains":
                result.append(str(v).translate(str.maketrans({'[': '', ']': '', ':': '', '\'': ''})).lstrip(' '))
            else:
                result += search_contains(v)
    else:
        None

    return result


def create_dummy_diag_node_list(diag_config_path):
    with open(diag_config_path, "r") as f:
        diag_config = yaml.safe_load(f)["/**"]["ros__parameters"]
    diags = search_contains(diag_config)

    nodes = []
    for d in diags:
        # set concat filter as a component
        node = ComposableNode(
            package="dummy_diag_publisher",
            plugin="DummyDiagPublisherNode",
            name="dummy_diag_publisher_" + str(uuid.uuid4())[0:8],
            parameters=[
                {
                    "diag_name": d,
                    "update_rate": 10.0,
                    "is_active": True,
                    "status": 0
                }
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
        nodes.append(node)

    return nodes


def launch_setup(context, *args, **kwargs):
    nodes = []
    diag_config_path_array = LaunchConfiguration("diag_config_path_array").perform(context).replace(' ', '').split(',')
    print(diag_config_path_array)
    for p in diag_config_path_array:
        print(p)
        nodes += create_dummy_diag_node_list(p)

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name="dummy_diag_publisher_container",
        namespace="dummy_diag_publisher",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=nodes,
        output="screen",
    )

    return [container]


def generate_launch_description():

    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg("diag_config_path_array")

    return launch.LaunchDescription(
        launch_arguments
        + [OpaqueFunction(function=launch_setup)]
    )


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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

  pkg = 'pointcloud_preprocessor'

  launch_arguments = []

  def add_launch_arg(name: str, default_value=None):
    # a default_value of None is equivalent to not passing that kwarg at all
    launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

  add_launch_arg('base_frame', 'base_link')

  # set self crop box filter as a component
  cropbox_self_component = ComposableNode(
      package=pkg,
      plugin='pointcloud_preprocessor::CropBoxFilterComponent',
      name='crop_box_filter',
      remappings=[
          ('/input', 'livox/lidar'),
          ('/output', 'self_cropped/pointcloud'),
      ],
      parameters=[
          {
              'input_frame': LaunchConfiguration('base_frame'),
              'output_frame': LaunchConfiguration('base_frame'),
              'min_x': "/vehicle_info/min_longitudinal_offset",  # TODO
              'max_x': "/vehicle_info/max_longitudinal_offset",  # TODO
              'min_y': "/vehicle_info/min_lateral_offset",  # TODO
              'max_y': "/vehicle_info/max_lateral_offset",  # TODO
              'min_z': "/vehicle_info/min_height_offset",  # TODO
              'max_z': "/vehicle_info/max_height_offset",  # TODO
              'negative': True,
          }
      ]
  )

  # set mirror crop box filter as a component
  cropbox_mirror_component = ComposableNode(
      package=pkg,
      plugin='pointcloud_preprocessor::CropBoxFilterComponent',
      name='crop_box_filter',
      remappings=[
          ('/input', 'self_cropped/pointcloud'),
          ('/output', 'mirror_cropped/pointcloud'),
      ],
      parameters=[
          {
              'input_frame': LaunchConfiguration('base_frame'),
              'output_frame': LaunchConfiguration('base_frame'),
              'min_x': "/vehicle_info/mirror/min_longitudinal_offset" # TODO
              'max_x': "/vehicle_info/mirror/max_longitudinal_offset" # TODO
              'min_y': "/vehicle_info/mirror/min_lateral_offset" # TODO
              'max_y': "/vehicle_info/mirror/max_lateral_offset" # TODO
              'min_z': "/vehicle_info/mirror/min_height_offset" # TODO
              'max_z': "/vehicle_info/mirror/max_height_offset" # TODO
              'negative': True,
          }
      ]
  )

  # set container to run all required components in the same process
  container = ComposableNodeContainer(
      name='pointcloud_preprocessor_container',
      namespace='pointcloud_preprocessor',
      package='rclcpp_components',
      executable='component_container',
      composable_node_descriptions=[
          cropbox_self_component,
          cropbox_mirror_component,
      ],
      output='screen',
  )

  return launch.LaunchDescription(launch_arguments + [container])

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
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression

ns = 'pointcloud_preprocessor'
pkg = 'pointcloud_preprocessor'

# TODO these don't work anymore when not called as launch file

# # declare launch arguments
# input_points_raw_list_param = DeclareLaunchArgument(
#     'input_points_raw_list',
#     default_value="['/points_raw']",
#     description="Input pointcloud topic_name list as a string_array. "
#     "To subscribe miultiple topics, write as: \"['/points_raw0', '/points_raw1', '/points_raw2', ...]\"")
#
# output_points_raw_param = DeclareLaunchArgument(
#     'output_points_raw',
#     default_value='/points_raw/cropbox/filtered')
#
# tf_output_frame_param = DeclareLaunchArgument(
#     'tf_output_frame',
#     default_value='base_link')

def create_concatenate(input_topics, output_frame, output='points_raw/concatenated',
                       approximate_sync=True):
  return ComposableNode(
      package=pkg,
      plugin='pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent',
      name='concatenate_filter',
      remappings=[('output', output)],
      parameters=[
          {
              'input_topics': input_topics,
              'output_frame': output_frame,
              'approximate_sync': approximate_sync,
          }
      ]
  )

def create_crop_box_filter(input, output, tf_frame, min_x=-50.0, max_x=100.0, min_y=-50.0,
                           max_y=50.0,
                           min_z=-2.0, max_z=3.0, negative=False):
  # set crop box filter as a component
  return ComposableNode(
      package=pkg,
      plugin='pointcloud_preprocessor::CropBoxFilterComponent',
      name='crop_box_filter',
      remappings=[
          ('input', input),
          ('output', output),
      ],
      parameters=[
          {
              'input_frame': tf_frame,
              'output_frame': tf_frame,
              'min_x': min_x,
              'max_x': max_x,
              'min_y': min_y,
              'max_y': max_y,
              'min_z': min_z,
              'max_z': max_z,
              'negative': negative,
          }
      ]
  )

def create_passthrough_filter():
    pass

def create_ray_ground_filter():
    pass



def create_container(composable_nodes):
  # set container to run all required components in the same process
  return ComposableNodeContainer(
      name='pointcloud_preprocessor_container',
      namespace=ns,
      package='rclcpp_components',
      executable='component_container',
      composable_node_descriptions=[
              composable_nodes
      ],
      output='screen',
  )

def create_log_info():
  # check the size of input_points_raw_list
  return LogInfo(
      msg=PythonExpression(
          ["'input_points_raw_list size = ' + str(len(", LaunchConfiguration(
              'input_points_raw_list'), "))"]
      )
  )

# def create_launch_description():
#   return launch.LaunchDescription([
#       input_points_raw_list_param,
#       output_points_raw_param,
#       tf_output_frame_param,
#       create_container(),
#       create_log_info()
#   ])
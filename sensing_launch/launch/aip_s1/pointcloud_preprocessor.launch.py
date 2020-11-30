
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


context = LaunchContext()


def generate_launch_description():

  pkg = 'pointcloud_preprocessor'

  # # declare launch arguments
  # input_points_raw_list_param = DeclareLaunchArgument(
  #     'input_points_raw_list',
  #     default_value="['/points_raw']",
  #     description="Input pointcloud topic_name list as a string_array. "
  #     "To subscribe multiple topics, write as: \"['/points_raw0', '/points_raw1', '/points_raw2', ...]\"")
  #
  # output_points_raw_param = DeclareLaunchArgument(
  #     'output_points_raw',
  #     default_value='/points_raw/cropbox/filtered')
  #
  # tf_output_frame_param = DeclareLaunchArgument(
  #     'tf_output_frame',
  #     default_value='base_link')

  # set concat filter as a component
  concat_component = ComposableNode(
      package=pkg,
      plugin='pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent',
      name='concatenate_data',
      remappings=[('/output', 'concatenated/pointcloud')],
      parameters=[
          {
              'input_topics': ['/sensing/lidar/top/outlier_filtered/pointcloud',
                               '/sensing/lidar/left/outlier_filtered/pointcloud',
                               '/sensing/lidar/right/outlier_filtered/pointcloud'],
              'output_frame': 'base_link',
          }
      ]
  )

  # set crop box filter as a component
  cropbox_component = ComposableNode(
      package=pkg,
      plugin='pointcloud_preprocessor::CropBoxFilterComponent',
      name='crop_box_filter',
      remappings=[
          ('/input', 'concatenated/pointcloud'),
          ('/output', "mesurement_range_cropped/pointcloud"),
          ('/min_z', '/vehicle_info/min_height_offset'),
          ('/max_z', '/vehicle_info/max_height_offset'),
      ],
      parameters=[
          {
              'input_frame': 'base_link',
              'output_frame': 'base_link',
              'min_x': -50.0,
              'max_x': 100.0,
              'min_y': -50.0,
              'max_y': 50.0,
              'min_z': -2.0,
              'max_z': 3.0,
              'negative': False,
          }
      ]
  )

  passthrough_component = ComposableNode(
      package=pkg,
      plugin='pointcloud_preprocessor::PassThroughFilterNodelet',
      name='passthrough_filter',
      remappings=[
          ('/input', 'top/rectified/pointcloud'),
          ('/output', 'concatenated/pointcloud')
      ],
      parameters=[{
          'output_frame': 'base_link',
      }]
  )

  ground_component = ComposableNode(
      package=pkg,
      plugin='pointcloud_preprocessor::RayGroundFilterNodelet',
      name='ray_ground_filter',
      remappings=[
          ('/input', 'mesurement_range_cropped/pointcloud'),
          ('/output', 'no_ground/pointcloud')
      ],
      parameters=[{
        "general_max_slope": 10.0,
        "local_max_slope": 10.0,
        "min_height_threshold": 0.2,
      }]
  )

  relay_component = ComposableNode(
      package='topic_tools',
      plugin='topic_tools::RelayNode',
      name='relay',
      parameters=[{
        "input_topic": "/sensing/lidar/top/rectified/pointcloud",
        "output_topic": "/sensing/lidar/pointcloud",
        "type": "sensor_msgs/msg/PointCloud2",
      }],
  )

  # set container to run all required components in the same process
  container = ComposableNodeContainer(
      name='pointcloud_preprocessor_container',
      namespace='pointcloud_preprocessor',
      package='rclcpp_components',
      executable='component_container',
      composable_node_descriptions=[
          concat_component,
          cropbox_component,
          passthrough_component,
          ground_component,
          relay_component,
      ],
      output='screen',
  )

  # # check the size of input_points_raw_list
  # log_info = LogInfo(
  #     msg=PythonExpression(
  #         ["'input_points_raw_list size = ' + str(len(", LaunchConfiguration(
  #             'input_points_raw_list'), "))"]
  #     )
  # )

  return launch.LaunchDescription([
      container,
  ])

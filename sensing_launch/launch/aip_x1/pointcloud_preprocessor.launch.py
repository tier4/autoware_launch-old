
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
  add_launch_arg('use_concat_filter', 'use_concat_filter')
  add_launch_arg('use_radius_search', 'use_radius_search')

  # set concat filter as a component
  if (LaunchConfiguration('use_concat_filter')):
    concat_component = ComposableNode(
        package=pkg,
        plugin='pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent',
        name='concatenate_data',
        remappings=[('/output', 'concatenated/pointcloud')],
        parameters=[
            {
                'input_topics': ['/sensing/lidar/top/outlier_filtered/pointcloud',
                                '/sensing/lidar/front_left/mirror_cropped/pointcloud',
                                '/sensing/lidar/front_right/mirror_cropped/pointcloud',
                                '/sensing/lidar/front_center/mirror_cropped/pointcloud'],
                'output_frame': 'base_link',
            }
        ]
    )
  else:
    # set PointCloud PassThrough Filter as a component
    concat_component = ComposableNode(
        package=pkg,
        plugin='pointcloud_preprocessor::PassThroughFilterComponent',
        name='passthrough_filter',
        remappings=[
            ('/input', 'top/outlier_filtered/pointcloud'),
            ('/output', 'concatenated/pointcloud'),
        ],
        parameters=[
            {
                'output_frame': 'base_link',
                'min_z': "/vehicle_info/min_height_offset", # TODO
                'max_z': "/vehicle_info/max_height_offset", # TODO
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
          ('/output', 'mesurement_range_cropped/pointcloud'),
      ],
      parameters=[
          {
              'input_frame': LaunchConfiguration('base_frame'),
              'output_frame': LaunchConfiguration('base_frame'),
              'min_x': -50.0,
              'max_x': 100.0,
              'min_y': -50.0,
              'max_y': 50.0,
              'min_z': "/vehicle_info/min_height_offset", # TODO
              'max_z': "/vehicle_info/max_height_offset", # TODO
              'negative': False,
          }
      ]
  )

  ground_component = ComposableNode(
      package=pkg,
      plugin='pointcloud_preprocessor::RayGroundFilterComponent',
      name='ray_ground_filter',
      remappings=[
          ('/input', 'mesurement_range_cropped/pointcloud'),
          ('/output', 'no_ground/pointcloud_with_outlier'),
      ],
      parameters=[{
        "initial_max_slope": 1.0,
        "general_max_slope": 10.0,
        "local_max_slope": 10.0,
        "min_height_threshold": 0.1,
        "use_vehicle_footprint": True,
        "min_x": "/vehicle_info/min_longitudinal_offset",  # TODO get from yaml
        "max_x": "/vehicle_info/max_longitudinal_offset",  # TODO
        "min_y": "/vehicle_info/min_lateral_offset",  # TODO
        "max_y": "/vehicle_info/max_lateral_offset",  # TODO
      }]
  )

  voxel_grid_filter_component = ComposableNode(
    package=pkg,
    plugin='pointcloud_preprocessor::VoxelGridDownsampleFilterComponent',
    name='voxel_grid_filter',
    remappings=[
        ('/input', 'no_ground/pointcloud_with_outlier'),
        ('/output', 'voxel_grid_filtered/pointcloud'),
    ],
    parameters=[{
        "voxel_size_x": 0.04,
        "voxel_size_y": 0.04,
        "voxel_size_z": 0.08,
        "input_frame": "base_link", # TODO
        "output_frame": "base_link", # TODO
    }]
  )

  if (LaunchConfiguration('use_radius_search')):
    radius_search_2d_outlier_filter_component = ComposableNode(
        package=pkg,
        plugin='pointcloud_preprocessor::RadiusSearch2dOutlierFilterComponent',
        name='radius_search_2d_outlier_filter',
        remappings=[
            ('/input', 'voxel_grid_filtered/pointcloud'),
            ('/output', 'no_ground/pointcloud'),
        ],
        parameters=[{
            "search_radius": 0.2,
            "min_neighbors": 5
        }]
    )
  else:
    voxel_grid_outlier_filter_component = ComposableNode(
        package=pkg,
        plugin='pointcloud_preprocessor::VoxelGridOutlierFilterComponent',
        name='voxel_grid_filter',
        remappings=[
            ('/input', 'voxel_grid_filtered/pointcloud'),
            ('/output', 'no_ground/pointcloud'),
        ],
        parameters=[{
            "voxel_size_x": 0.4,
            "voxel_size_y": 0.4,
            "voxel_size_z": 100,
            "voxel_points_threshold": 5,
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
          ground_component,
          relay_component,
      ],
      output='screen',
  )

  return launch.LaunchDescription(launch_arguments + [container])

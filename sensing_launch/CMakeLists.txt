cmake_minimum_required(VERSION 3.5)
project(sensing_launch)

find_package(ament_cmake_auto REQUIRED)

# Force Tier IV's fork version
find_package(velodyne_driver 0.2.0 EXACT REQUIRED)
find_package(velodyne_pointcloud 0.2.0 EXACT REQUIRED)

ament_auto_package(INSTALL_TO_SHARE
  launch
  data
)

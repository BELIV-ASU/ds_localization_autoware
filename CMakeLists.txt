cmake_minimum_required(VERSION 3.14)
project(ds_localization_autoware)

find_package(autoware_cmake REQUIRED)
autoware_package()

## Find non-ROS library
find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)

set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})
find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

set(DS_LOCALIZATION_AUTOWARE_HEADERS
  include/ds_localization_autoware/ds_localization_autoware.hpp
)

ament_auto_add_library(ds_localization_autoware_node SHARED
  src/ds_localization_autoware.cpp
  ${DS_LOCALIZATION_AUTOWARE_HEADERS}
)

target_link_libraries(ds_localization_autoware_node
  Geographic
)

rclcpp_components_register_node(ds_localization_autoware_node
  PLUGIN "ds_localization_autoware::DsLocalizationAutoware"
  EXECUTABLE ds_localization_autoware
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)

cmake_minimum_required(VERSION 3.14)
project(trajectory_selector)

find_package(autoware_cmake REQUIRED)

autoware_package()

ament_auto_add_executable(trajectory_selector
  src/trajectory_selector.cpp
)

ament_auto_package(
    INSTALL_TO_SHARE 
    launch
)

cmake_minimum_required(VERSION 3.14)
project(autoware_core_api)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)

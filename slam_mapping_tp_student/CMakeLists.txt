cmake_minimum_required(VERSION 3.8)
project(slam_mapping_tp)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  worlds
  controllers
  docs
  DESTINATION share/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(ur_description_local)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package(
  CONFIG_EXTRAS cmake/ur_description_localConfig.cmake
)

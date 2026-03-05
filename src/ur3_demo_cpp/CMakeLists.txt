cmake_minimum_required(VERSION 3.8)
project(ur3_demo_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# MoveIt packages
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(std_srvs REQUIRED)

# add_executable(ur3_demo_node src/ur3_demo.cpp)


# target_include_directories(ur3_demo_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# ament_target_dependencies(ur3_demo_node
#   rclcpp
#   geometry_msgs
#   moveit_core
#   moveit_ros_planning
#   moveit_ros_planning_interface
#   moveit_visual_tools
# )

add_executable(panda_demo_node src/moveit_cpp_tutorial.cpp)

target_include_directories(panda_demo_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(panda_demo_node 
  rclcpp
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  std_srvs
  graph_msgs


)

target_link_libraries(panda_demo_node
  moveit_visual_tools::moveit_visual_tools
)

install(TARGETS
  # ur3_demo_node
  panda_demo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
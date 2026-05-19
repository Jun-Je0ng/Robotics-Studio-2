cmake_minimum_required(VERSION 3.8)
project(motion_control_subsystem)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# MoveIt packages
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nlohmann_json REQUIRED)





find_package(ament_cmake_python REQUIRED)   # Python support
find_package(rclpy REQUIRED)                # Python ROS2 client
find_package(object_msgs REQUIRED)          # your custom messages

ament_python_install_package(${PROJECT_NAME})  # makes the package importable

install(PROGRAMS                            # installs Python script as executable
  src/test_helper.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)





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

# add_executable(gripper_demo_node src/moveit_cpp_tutorial.cpp)
add_executable(gripper_demo_node src/gripper_demo_clean.cpp)
# add_executable(gripper_pick_place src/unit_test_pick_n_place.cpp)
# add_executable(gripper_pick_place src/pick_n_place_try2.cpp)
add_executable(gripper_pick_place src/pick_n_place_try2_1.cpp)


target_include_directories(gripper_demo_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(gripper_demo_node 
  rclcpp
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  std_srvs
  graph_msgs
  object_msgs
)

target_link_libraries(gripper_demo_node
  moveit_visual_tools::moveit_visual_tools
  nlohmann_json::nlohmann_json
)

ament_target_dependencies(gripper_pick_place 
  rclcpp
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  std_srvs
  graph_msgs
  object_msgs
)

target_link_libraries(gripper_pick_place
  moveit_visual_tools::moveit_visual_tools
  nlohmann_json::nlohmann_json
)

install(TARGETS
  # ur3_demo_node
  gripper_demo_node
  gripper_pick_place
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
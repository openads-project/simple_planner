cmake_minimum_required(VERSION 3.8)
project(simple_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(perception_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(route_planning_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(trajectory_interfaces REQUIRED)
find_package(carla_msgs REQUIRED)

set(TARGET_NAME simple_planner_node)

add_executable(${TARGET_NAME} src/simple_planner_node.cpp)
add_executable(simple_controller_node src/simple_controller_node.cpp)

target_include_directories(${TARGET_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(simple_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${TARGET_NAME} PUBLIC c_std_99 cxx_std_17)
target_compile_features(simple_controller_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(${TARGET_NAME}
  perception_interfaces  
  rclcpp
  route_planning_interfaces
  std_msgs
  trajectory_interfaces
)
ament_target_dependencies(simple_controller_node
  perception_interfaces  
  rclcpp
  std_msgs
  trajectory_interfaces
  carla_msgs
)

install(TARGETS ${TARGET_NAME} simple_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()

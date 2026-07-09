cmake_minimum_required(VERSION 3.8)
project(simple_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(perception_msgs_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(route_planning_msgs REQUIRED)
find_package(route_planning_msgs_utils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_perception_msgs REQUIRED)
find_package(tf2_route_planning_msgs REQUIRED)
find_package(tracetools REQUIRED)
find_package(trajectory_planning_msgs REQUIRED)
find_package(trajectory_planning_msgs_utils REQUIRED)
find_package(visualization_msgs REQUIRED)

set(TARGET_NAME simple_planner_node)

add_executable(${TARGET_NAME}
  src/simple_planner.cpp
  src/object_handling.cpp
)

target_include_directories(${TARGET_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(${TARGET_NAME} PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(${TARGET_NAME}
  diagnostic_msgs
  diagnostic_updater
  perception_msgs
  perception_msgs_utils
  rclcpp
  route_planning_msgs
  route_planning_msgs_utils
  std_msgs
  std_srvs
  tf2_perception_msgs
  tf2_route_planning_msgs
  tracetools
  trajectory_planning_msgs
  trajectory_planning_msgs_utils
  visualization_msgs
)

install(TARGETS ${TARGET_NAME}
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_clang_format_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../.vscode/format/.clang-format)
  set(ament_cmake_clang_tidy_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../.vscode/lint/.clang-tidy)
  set(ament_cmake_flake8_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../.vscode/lint/ament_flake8.ini)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

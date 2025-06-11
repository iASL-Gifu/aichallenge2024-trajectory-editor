cmake_minimum_required(VERSION 3.8)
project(editor_tool_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(editor_tool_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(autoware_auto_planning_msgs REQUIRED)

include_directories(
  include
)

add_executable(interactive_server
  src/interactive_server.cpp
)

ament_target_dependencies(interactive_server
  rclcpp
  visualization_msgs
  interactive_markers
  geometry_msgs
  editor_tool_srvs
  tf2_geometry_msgs
  std_srvs
  autoware_auto_planning_msgs
)

install(TARGETS
  interactive_server
  DESTINATION lib/${PROJECT_NAME})


ament_package()

cmake_minimum_required(VERSION 3.8)
project(rviz_editor_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

include_directories(src ${OGRE_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS}/Paging)

set(CMAKE_AUTOMOC ON)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/csv_marker_display.cpp
)

ament_target_dependencies(${PROJECT_NAME} 
  pluginlib
  rclcpp
  rviz_common
  visualization_msgs
  geometry_msgs
)

target_link_libraries(${PROJECT_NAME} Qt5::Widgets)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}  
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()

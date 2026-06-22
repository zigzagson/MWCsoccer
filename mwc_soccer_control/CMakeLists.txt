cmake_minimum_required(VERSION 3.8)
project(mwc_soccer_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_center REQUIRED)
find_package(unitree_sdk2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(soccer_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(soccer_brain_node
  src/soccer_brain_node.cpp
)

target_compile_features(soccer_brain_node PUBLIC cxx_std_17)

ament_target_dependencies(soccer_brain_node
  geometry_msgs
  message_center
  rclcpp
  rclcpp_action
  soccer_msgs
  std_msgs
)

target_link_libraries(soccer_brain_node
  unitree_sdk2
)

install(TARGETS soccer_brain_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/soccer_visualizer.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config scripts web
  DESTINATION share/${PROJECT_NAME})

ament_package()

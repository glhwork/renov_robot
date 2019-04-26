cmake_minimum_required(VERSION 2.8.3)
project(mobile_base_navigation)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mobile_base_navigation
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(lsd
  src/LSD.cpp
)

add_library(detect
  src/WallDetect.cpp
)
target_link_libraries(detect
  lsd
  ${catkin_LIBRARIES}
)

add_executable(detect_node
  src/detect_node.cpp
)
target_link_libraries(detect_node
  detect
  lsd
  ${catkin_LIBRARIES}
)


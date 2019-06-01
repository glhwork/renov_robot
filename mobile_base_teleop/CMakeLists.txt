cmake_minimum_required(VERSION 2.8.3)
project(mobile_base_teleop)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mobile_base_teleop
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  lib
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  lib
)

add_library(teleop
  src/Teleop.cpp
)

add_executable(tele_operate
  src/tele_operate.cpp
)
target_link_libraries(tele_operate
  teleop
  ${catkin_LIBRARIES}
)

add_executable(teleop_gyj
  src/teleop_gyj.cpp
)
target_link_libraries(teleop_gyj
  ${catkin_LIBRARIES}
)

cmake_minimum_required(VERSION 2.8.3)
project(sensor_startup)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  serial
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sensor_startup
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(mobile_imu
  src/MobileImu.cpp
)
target_link_libraries(mobile_imu
  ${catkin_LIBRARIES}
)

add_executable(imu_core
  src/imu_core.cpp
  src/MobileImu.cpp
)
target_link_libraries(imu_core
  mobile_imu
  ${catkin_LIBRARIES}
)




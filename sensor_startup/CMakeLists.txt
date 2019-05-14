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

find_package(Threads)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sensor_startup
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs tf
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

# define the library used in sensor communication
add_library(mobile_sensor
  src/MobileImu.cpp
  src/MobileMotor.cpp
)
target_link_libraries(mobile_sensor
  usbcan
  ${catkin_LIBRARIES}
)

# define the imu node
add_executable(imu_core
  src/imu_core.cpp
)
target_link_libraries(imu_core
  mobile_sensor
  usbcan
  ${catkin_LIBRARIES}
)

# define the motor data node
add_executable(motor_core
  src/motor_core.cpp
)
target_link_libraries(motor_core
  mobile_sensor
  usbcan
  ${catkin_LIBRARIES} 
)

add_executable(test_node
  src/test.cpp
)
target_link_libraries(test_node
  usbcan
  ${catkin_LIBRARIES}
)
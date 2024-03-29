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
find_package(yaml-cpp REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sensor_startup
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(include lib ${catkin_INCLUDE_DIRS})
link_directories(lib)

# define the library used in sensor communication
add_library(sensor_reader
  src/imu_reader.cpp
  src/motor_reader.cpp
)
target_link_libraries(sensor_reader
  yaml-cpp
  controlcan
  ${catkin_LIBRARIES}
)

# define the imu node
add_executable(imu_reader_node src/imu_reader_node.cpp)
target_link_libraries(imu_reader_node
  sensor_reader
  controlcan
  ${catkin_LIBRARIES}
)

# define the motor data node
add_executable(motor_reader_node src/motor_reader_node.cpp)
target_link_libraries(motor_reader_node
  sensor_reader
  yaml-cpp
  controlcan
  ${catkin_LIBRARIES} 
)

add_executable(abs_encoder src/test/abs_encoder.cpp)
target_link_libraries(abs_encoder controlcan ${catkin_LIBRARIES})

add_executable(stop_motor src/test/stop_motor.cpp)
target_link_libraries(stop_motor ${catkin_LIBRARIES})

add_executable(send_test src/test/send_test.cpp  )
target_link_libraries(send_test ${catkin_LIBRARIES})

add_executable(walking_demo src/test/walking_demo)
target_link_libraries(walking_demo yaml-cpp ${catkin_LIBRARIES})

add_executable(limit_switch src/limit_switch.cpp)
target_link_libraries(limit_switch ${catkin_LIBRARIES})

add_executable(remote_control src/remote_control.cpp)
target_link_libraries(remote_control
  sensor_reader
  yaml-cpp
  ${catkin_LIBRARIES}
)

add_executable(send_teleop_signal src/send_teleop_signal.cpp)
target_link_libraries(send_teleop_signal yaml-cpp ${catkin_LIBRARIES})

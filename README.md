# renov_robot

navigation stack of renovation robot

## Author

name : glh

mail : georgeglh@link.cuhk.edu.hk

## Third-party pkgs

### LiDAR driver

document : <http://wiki.ros.org/sick_tim>

clone from git : <https://github.com/uos/sick_tim>

### open source SLAM

document : <http://wiki.ros.org/gmapping>

clone from git(openslam_gmapping) : <https://github.com/ros-perception/openslam_gmapping>

clone from git(gmapping) : <https://github.com/ros-perception/slam_gmapping>

### robot_pose_ekf

document : <http://wiki.ros.org/robot_pose_ekf>

get : sudo apt-get install ros-[distro]-robot-pose-ekf

### serial port communication

get : sudo apt-get install ros-[distro]-serial

## Pkgs && Nodes

### sensor_startup

sensor drivers for applying or testing sensors
  - For motor driver with CANOpen protocol based on CAN analyst, reinstall the driver of CH340

### mobile_base_slam

launch slam node for base navigation

### mobile_base_navigation

navigation scheme including environment perception

### mobile_base_description

external paramters describing links of robot

### mobile_base_controller

path tracking scheme

### mobile_base_teleop

key-borad tele-operation of mobile base
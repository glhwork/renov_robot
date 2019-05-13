#ifndef MOBILEMOTOR_H
#define MOBILEMOTOR_H

#include <iostream>
#include <string>
#include <vector>
#include <sstream>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "serial/serial.h"

#include "controlcan.h"


namespace mobile {

class MobileMotor {
 public:
  MobileMotor();
  ~MobileMotor() {}
  void ParamInit();
  void Setup();
  void CanBusInit();
  void SetMode();

 private:
  
  /* PARAMETERS */
  // port connected with CAN-USB converter 
  std::string port;
  // topic used to publish joint states
  std::string state_topic;
  // device type of CAN-Analyst, refer to controlcan.h
  int device_type;

  // working mode of motors: 
  // 0 -> position servo
  // 1 -> velocity servo
  int steering_mode;
  int walking_mode;

  /* GLOBAL VARIABLES */
  int device_index;
  ros::NodeHandle nh;
  ros::NodeHandle n_private;
  ros::Publisher state_pub;



};  // class MobileMotor

} // namespace mobile

#endif
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

 private:
  
  /* PARAMETERS */
  std::string port;
  std::string state_topic;
  int device_type;

  
  int device_index;
  ros::NodeHandle nh;
  ros::NodeHandle n_private;
  ros::Publisher state_pub;



};  // class MobileMotor

} // namespace mobile

#endif
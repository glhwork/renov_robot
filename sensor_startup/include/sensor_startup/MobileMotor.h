#ifndef MOBILEMOTOR_H
#define MOBILEMOTOR_H

#include <iostream>
#include <string>
#include <vector>
#include <sstream>

#include "yaml-cpp/yaml.h"

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "serial/serial.h"

#include "CanAssist.h"
#include "controlcan.h"


namespace mobile {

struct IdConfig {
  int walk_chn1;
  int walk_chn2;
  int steer_chn1;
  int steer_chn2;
};


class MobileMotor {
 public:
  MobileMotor();
  virtual ~MobileMotor() {}
  void ParamInit();
  void ReadFile(const std::string& address);
  void Setup();
  void CanBusInit();
  void SetMode();

  VCI_CAN_OBJ* MobileMotor::GetVciObject(const int& obj_num, const int& chn);
  void IdCheck();

 private:
  
  /* PARAMETERS */
  // port connected with CAN-USB converter 
  std::string port;
  // topic used to publish joint states
  std::string state_topic;
  // address of yaml file containing motor driver configurations
  std::string file_address;
  

 

  /* GLOBAL VARIABLES */
  // this CAN device only has one can channel
  int device_index;
  // device type of CAN-Analyst, refer to controlcan.h
  int device_type;
  int can_index;

  // working mode of motors:
  // 0 -> position servo
  // 1 -> velocity servo
  int steering_mode;
  int walking_mode;

  unsigned int encoder_lines;
  // COBID of multiple motor drivers
  int cob_id[4];

  bool if_initial;


  ros::NodeHandle nh;
  ros::NodeHandle n_private;
  ros::Publisher state_pub;



};  // class MobileMotor

} // namespace mobile

#endif
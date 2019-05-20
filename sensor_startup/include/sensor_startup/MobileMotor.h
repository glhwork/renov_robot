#ifndef MOBILEMOTOR_H
#define MOBILEMOTOR_H

#include <iostream>
#include <string>
#include <vector>
#include <sstream>

#include "yaml-cpp/yaml.h"

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
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
  bool CanBusInit();
  bool SetMode();
  bool EnableMotor();
  void DataTransform(BYTE* data, uint8_t* cmd, const uint& len);

  VCI_CAN_OBJ* MobileMotor::GetVciObject(const int& obj_num);
  void IdCheck();
  uint SendCommand(PVCI_CAN_OBJ obj, uint len);

  void ControlCallback(const sensor_msgs::JointState& joint_state);
  void TeleopCallback(const geometry_msgs::Twist& twist);
  void FeedbackCallback(const ros::Timer&);

  private :

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
  // id_num means the quantity of motor-drivers we use
  int id_num;

  // working mode of motors:
  // 0 -> position servo
  // 1 -> velocity servo
  int steering_mode;
  int walking_mode;

  uint encoder_lines;
  // COBID of multiple motor drivers
  int cob_id[4];

  Command cmd;

  bool if_initial;


  ros::NodeHandle nh;
  ros::NodeHandle n_private;
  ros::Publisher state_pub;



};  // class MobileMotor

} // namespace mobile

#endif
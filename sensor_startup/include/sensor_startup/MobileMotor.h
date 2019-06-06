#ifndef MOBILEMOTOR_H
#define MOBILEMOTOR_H

#include <iostream>
#include <string>
#include <vector>
#include <sstream>

#include "yaml-cpp/yaml.h"

#include "ros/ros.h"
#include "std_msgs/Bool.h"
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
  virtual ~MobileMotor();
  void ParamInit();
  void ReadFile(const std::string& address);
  void Setup();
  bool CanBusInit();
  bool SetMode();
  bool EnableMotor();
  void ModeCommand(const int& id_0, const int& id_1, 
                   const int& len, const uint8_t& mode);
  void DataInitial(BYTE* data, uint8_t* cmd, const uint& len);

  VCI_CAN_OBJ* GetVciObject(const int& obj_num);
  void IdCheck();
  void SendCommand(PVCI_CAN_OBJ obj, const uint& len);

  void ControlCallback(const sensor_msgs::JointState& joint_state);
  void DataTransform(BYTE* data, uint8_t* cmd, const uint& len,
                     const uint8_t& index, const float& velo);

  void TeleopCallback(const geometry_msgs::Twist& twist);
  void FeedbackCallback(const ros::TimerEvent&);
  void StopCallback(const std_msgs::Bool& stop);
  void ControlMotor(const std::vector<float>& velocity);
   
  void PrintTest(BYTE* data, const int& len, const std::string& str);

 protected:
  // COBID of multiple motor drivers
  int cob_id[4];
  // pre-defined commands as data arrays
  Command cmd;

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
  // id_num means the quantity of motor-drivers we use
  int id_num;

  // working mode of motors:
  // 0 -> position servo
  // 1 -> velocity servo
  int steering_mode;
  int walking_mode;

  // lines of encoders
  uint encoder_lines;
  // max value of velocity when used in position mode
  uint max_velocity;
  // command sign of each motor
  int motor_sign[8];
  

  bool if_initial;


  ros::NodeHandle nh;
  ros::NodeHandle n_private;
  ros::Publisher state_pub;



};  // class MobileMotor

} // namespace mobile

#endif
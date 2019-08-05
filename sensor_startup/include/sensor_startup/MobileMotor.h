#ifndef MOBILEMOTOR_H
#define MOBILEMOTOR_H

#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <sstream>
#include <unistd.h>
#include <boost/thread.hpp>

#include "yaml-cpp/yaml.h"

#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include "serial/serial.h"

#include "CanAssist.h"
#include "controlcan.h"

#define PI 3.141592653

/* code refactoring !!!! */



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
  bool DriverInit();
  bool SetMode();
  bool EnableMotor();
  void ModeCommand(const int& id_0, const int& id_1, 
                   const int& len, const uint8_t& mode);
  void DataInitial(BYTE* data, uint8_t* cmd, const uint& len);

  VCI_CAN_OBJ* GetVciObject(const int& obj_num);
  void IdCheck();
  bool SendCommand(PVCI_CAN_OBJ obj, const uint& len);

  void ControlCallback(const sensor_msgs::JointState& joint_state);
  void DataTransform(BYTE* data, uint8_t* cmd, const uint& len,
                     const uint8_t& index, const int& velo);

  void TeleopCallback(const geometry_msgs::Twist& twist);
  void FeedbackCallback();
  void StopCallback(const std_msgs::Bool& stop);
  void ControlMotor(const std::vector<float>& raw_state);
   
  void StopMotor();
  void FeedbackReq();
  std::vector<int> CommandTransform(const std::vector<float>& raw_state);
  void PrintTest(BYTE* data, const int& len, const std::string& str);

  int  FourByteHex2Int(uint8_t* data);
  bool ReadEncoder(int* encod_data);
  void Homing();

  void Loop();
  void SubLoop();

 protected:
  // COBID of multiple motor drivers
  int cob_id[4];
  // pre-defined commands as data arrays
  Command cmd;

  // this CAN device has two can channels
  // device type of CAN-Analyst, refer to controlcan.h
  int device_type;
  int device_index;
  int can_index;
  // id_num means the quantity of motor-drivers we use
  int id_num;

  bool if_initial;
  // homing position
  int home[4];
  // lines of encoders
  uint encoder_s;
  uint encoder_w;
  uint abs_encoder;

  double pre_time;
  double cur_time;

 private:

  /* LAUNCH PARAMETERS */
  // port connected with CAN-USB converter
  std::string port;
  // topic used to publish joint states
  std::string state_topic;
  // address of yaml file containing motor driver configurations
  std::string file_address;
  // delay time used to pause between two commands
  int delay_time;
  // time to determine how long shoud it wait if buffer is empty
  int wait_time;
  // the publish period of motor states
  double state_pub_period;

  

 

  /* CONFIG PARAMETERS */
  YAML::Node param;
  

  // working mode of motors:
  // 0 -> position servo
  // 1 -> velocity servo
  int steering_mode;
  int walking_mode;

  
  // reduction ratio of steering or walking motors
  double reduc_ratio_s;
  double reduc_ratio_w;
  // max value of velocity when used in position mode
  uint max_velocity;
  // command sign of each motor
  int motor_sign[8];
  // homing value of absolute encoder
  int abs_home[4];
  // error limit to judge whether steering motor finishes homing
  int error_limit;
  // PID parameters in homing process
  double home_kp;
  double home_ki;
  double home_kd;

  
  /* GLOBAL VARIABLES */
  ros::NodeHandle nh;
  ros::NodeHandle n_private;
  ros::Publisher state_pub;

  ros::Subscriber control_sub;
  ros::Subscriber teleop_sub;
  ros::Subscriber stop_sub;

  /* THREADS */ 
  boost::thread* state_pub_thread;


};  // class MobileMotor

}  // namespace mobile

#endif
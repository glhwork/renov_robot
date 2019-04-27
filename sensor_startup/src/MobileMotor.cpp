#include "sensor_startup/MobileMotor.h"

using mobile::MobileMotor;

MobileMotor::MobileMotor() {
  n_private = ros::NodeHandle("motor_core");
  ParamInit();
  Setup();
  CanBusInit();
}

void MobileMotor::ParamInit() {

  if (!n_private.getParam("port", port)) {
    port = "/dev/ttyUISB0";
  }
  if (!n_private.getParam("state_topic", state_topic)) {
    state_topic = "motor_state";
  }
  if (!n_private.getParam("device_type", device_type)) {
    device_type = 3;
  }

}

void MobileMotor::Setup() {
  state_pub = nh.advertise<sensor_msgs::JointState>(state_topic, 100);
}

void MobileMotor::CanBusInit() {
  std::stringstream ss;
  ss << port.back();
  ss >> device_index;
  
  if (!VCI_OpenDevice(device_type, device_index, 0)) {
    ROS_ERROR("open CAN failure");
  } else {
    ROS_INFO("open CAN successfully");
  };

  VCI_INIT_CONFIG config;
  config.AccCode = 0;
  config.AccMask = 0;
  config.Filter = 8;
  config.Mode = 0;
  config.Timing0 = 0 & 0xff;
  config.Timing1 = 0 >> 8;
  
  if (!VCI_InitCAN(device_type, device_index, 0, &config)) {
    ROS_ERROR("initial failure");
  } else {
    ROS_INFO("initial successfully");
  }
}


#include <iostream>
#include <ctime>
#include "sensor_startup/controlcan.h"

#include "ros/ros.h"

int dev_type = VCI_USBCAN1;
int dev_ind = 0;
int can_ind = 0;

u_char motor[8] = {0x23, 0xff, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00};
u_char brake[8] = {0x23, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void StartDevice() {
  std::cout << "begin" << std::endl;
  if (!VCI_OpenDevice(dev_type, dev_ind, 0)) {
    std::cout << "open failure" << std::endl;
  } else {
    std::cout << "open successfully" << std::endl;
  }

  VCI_INIT_CONFIG config;
  config.AccCode = 0x00;
  config.AccMask = 0x00;
  config.Filter = 0x08;
  config.Mode = 0;
  config.Timing0 = 0x00;
  config.Timing1 = 0x1c;

  if (!VCI_InitCAN(dev_type, dev_ind, can_ind, &config)) {
    std::cout << "init failure" << std::endl;
  } else {
    std::cout << "init successfully" << std::endl;
  }

  if (!VCI_StartCAN(dev_type, dev_ind, can_ind)) {
    std::cout << "start failure" << std::endl;
  } else {
    std::cout << "start successfully" << std::endl;
  }
}

void Enable() {
  VCI_CAN_OBJ can[4];

  for (size_t i = 0; i < 4; i++) {
    can[i].ID = 0x00000601;
    can[i].SendType = 0;
    can[i].RemoteFlag = 0;
    can[i].ExternFlag = 0;
  }

  can[0].DataLen = 5;
  can[0].Data[0] = 0x2f;
  can[0].Data[1] = 0x60;
  can[0].Data[2] = 0x60;
  can[0].Data[3] = 0x00;
  can[0].Data[4] = 0x03;

  for (size_t i = 1; i < 4; i++) {
    can[i].DataLen = 6;
    can[i].Data[0] = 0x2b;
    can[i].Data[1] = 0x40;
    can[i].Data[2] = 0x60;
    can[i].Data[3] = 0x00;
    can[i].Data[5] = 0x00;
  }
  can[1].Data[4] = 0x06;
  can[2].Data[4] = 0x07;
  can[3].Data[4] = 0x0f;

  std::cout << "enable command quantity : "
            << VCI_Transmit(dev_type, dev_ind, can_ind, can, 4) << std::endl;
}

void GetCommand(u_char* data, u_char* cmd, const u_char& t) {
  for (size_t i = 0; i < 8; i++) {
    data[i] = cmd[i];
  }
  data[2] = t;
}

void Brake() {
  VCI_CAN_OBJ command[2];

  for (size_t i = 0; i < 2; i++) {
    command[i].ID = 0x00000601;
    command[i].SendType = 0;
    command[i].RemoteFlag = 0;
    command[i].ExternFlag = 0;
  }

  command[0].DataLen = 8;
  command[1].DataLen = 8;
  GetCommand(command[0].Data, brake, 0x60);
  GetCommand(command[1].Data, brake, 0x68);
  std::cout << "brake quantity : "
            << VCI_Transmit(dev_type, dev_ind, can_ind, command, 2)
            << std::endl;
  // std::cout << "brake quantity : "
  //           << VCI_Transmit(dev_type, dev_ind, can_ind, command, 2)
  //           << std::endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;
  ros::Duration dr(2.0);

  StartDevice();
  Enable();
  
  dr.sleep();

  VCI_CAN_OBJ command[2];
  
  for (size_t i = 0; i < 2; i++) {
    command[i].ID = 0x00000601;
    command[i].SendType = 0;
    command[i].RemoteFlag = 0;
    command[i].ExternFlag = 0;
  } 
  
  command[0].DataLen = 8;
  command[1].DataLen = 8;
  GetCommand(command[0].Data, motor, 0x60);
  GetCommand(command[1].Data, motor, 0x68);
  std::cout << "velo command quantity : "
            << VCI_Transmit(dev_type, dev_ind, can_ind, command, 2)
            << std::endl;
  // std::cout << "velo command quantity : "
  //           << VCI_Transmit(dev_type, dev_ind, can_ind, command, 2)
  //           << std::endl;

  dr.sleep();
 
  Brake();
  

  ros::Duration(0.2).sleep();

  if (!VCI_CloseDevice(dev_type, dev_ind)) {
    std::cout << "close device failure" << std::endl;
  } else {
    std::cout << "close device successfully" << std::endl;
  }

  return 0;
}

#include <iostream>
#include <ctime>
#include "sensor_startup/controlcan.h"

int main() {

  int dev_type = VCI_USBCAN1;
  int dev_ind = 0;
  int can_ind = 0;
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

  // DWORD port = 4001;
  // if (!VCI_SetReference(dev_type, dev_ind, can_ind, 0x060007, (PVOID)&port)) {
  //   std::cout << "set reference failure" << std::endl;
  // } else {
  //   std::cout << "set reference successfully" << std::endl;
  // }


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

  VCI_CAN_OBJ can[4];
  VCI_CAN_OBJ command;
  
  for (size_t i = 0; i < 4; i++) {
    can[i].ID = 0x00000601;
    can[i].SendType = 0;
    can[i].RemoteFlag = 0;
    can[i].ExternFlag = 0;

    command.ID = 0x00000601;
    command.SendType = 0;
    command.RemoteFlag = 0;
    command.ExternFlag = 0;
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
  

  command.DataLen = 8;
  command.Data[0] = 0x23;
  command.Data[1] = 0xff;
  command.Data[2] = 0x68;
  command.Data[3] = 0x00;
  command.Data[4] = 0xe8;
  command.Data[5] = 0x03;
  command.Data[6] = 0x00;
  command.Data[7] = 0x00;

  std::cout << "command quantity : " 
            << VCI_Transmit(dev_type, dev_ind, can_ind, can, 4) << std::endl;
  std::cout << "command quantity : "
            << VCI_Transmit(dev_type, dev_ind, can_ind, &command, 1)
            << std::endl;
  std::cout << "command quantity : "
            << VCI_Transmit(dev_type, dev_ind, can_ind, &command, 1)
            << std::endl;
  time_t t;
  time_t t_pre;
  t_pre = t = time(&t); 
  while(1) {
    t = time(&t);
    // std::cout << 1 << " ";
    if (abs(t - t_pre) > 0.5) {
      break;
    }
  }
  std::cout << std::endl;
  if (!VCI_CloseDevice(dev_type, dev_ind)) {
    std::cout << "close device failure" << std::endl;
  } else {
    std::cout << "close device successfully" << std::endl;
  }

  return 0;
}
#include <iostream>
#include <ctime>
#include "sensor_startup/controlcan.h"

int main() {
  if (!VCI_OpenDevice(VCI_USBCAN_E_U, 0, 0)) {
    std::cout << "open failure" << std::endl;
  } else {
    std::cout << "open successfully" << std::endl;
  }

  VCI_INIT_CONFIG config;
  config.AccCode = 0;
  config.AccMask = 0;
  config.Filter = 0x08;
  config.Mode = 0;
  config.Timing0 = 0x00;
  config.Timing1 = 0x1c;

  if (!VCI_InitCAN(3, 0, 0, &config)) {
    std::cout << "init failure" << std::endl;
  } else {
    std::cout << "init successfully" << std::endl;
  }

  if (!VCI_StartCAN(3, 0, 0)) {
    std::cout << "start failure" << std::endl;
  } else {
    std::cout << "start successfully" << std::endl;
  }

  VCI_CAN_OBJ can[5];
  
  for (size_t i = 0; i < 5; i++) {
    can[i].ID = 0x00000601;
    can[i].SendType = 0;
    can[i].RemoteFlag = 0;
    can[i].ExternFlag = 0;
  }
  
  can[0].DataLen = 5;
  can[0].Data[0] = 0x2f;
  can[0].Data[1] = 0x60;
  can[0].Data[0] = 0x60;
  can[0].Data[0] = 0x00;
  can[0].Data[0] = 0x03;

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

  can[4].DataLen = 8;
  can[4].Data[0] = 0x23;
  can[4].Data[1] = 0xff;
  can[4].Data[2] = 0x60;
  can[4].Data[3] = 0x00;
  can[4].Data[4] = 0xe8;
  can[4].Data[5] = 0x03;
  can[4].Data[6] = 0x00;
  can[4].Data[7] = 0x00;

  VCI_Transmit(3, 0, 0, can, 5);
  VCI_Transmit(3, 0, 0, can, 5);

  time_t t;
  time_t t_pre;
  t_pre = t = time(&t);
  while(1) {
    t = time(&t);
    if ((t - t_pre) < 5) {
      break;
    }
  }
  VCI_CloseDevice(3, 0);

  return 0;
}
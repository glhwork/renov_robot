#include <iostream>
#include <serial/serial.h>
#include <string>
#include <unistd.h>

#include "ros/ros.h"
#include "sensor_msgs/Range.h"

#define ENCODER_1 0x01
#define ENCODER_2 0x02
#define ENCODER_3 0x03
#define ENCODER_4 0x04

typedef uint8_t _u8;
typedef uint16_t _u16;
_u8 cmd[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xc4, 0x0b};

_u8 REQUEST_DATA_1[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xc4, 0x0b};
_u8 REQUEST_DATA_2[8] = {0x02, 0x03, 0x00, 0x00, 0x00, 0x02, 0xc4, 0x38};
_u8 REQUEST_DATA_3[8] = {0x03, 0x03, 0x00, 0x00, 0x00, 0x02, 0xc5, 0xe9};
_u8 REQUEST_DATA_4[8] = {0x04, 0x03, 0x00, 0x00, 0x00, 0x02, 0xc4, 0x5e};

/* set the address
 01 06 00 04 00 01 09 cb
 01 06 00 04 00 02 49 ca
 01 06 00 04 00 03 88 0a
 01 06 00 04 00 04 c9 c8
 */

/* data request
 01 03 00 00 00 02 c4 0b
 02 03 00 00 00 02 c4 38
 03 03 00 00 00 02 c5 e9
 04 03 00 00 00 02 c4 5e
 */
serial::Serial encod_ser;

void InitSerial(const std::string& port_id, const int& baudrate);
void SetAddress();
void ReadEncoder();

int main(int argc, char** argv) {
  ros::init(argc, argv, "abs_encoder");
  ros::NodeHandle n;

  std::string port;
  int baudrate;
  if (!ros::param::get("port", port)) {
    port = "/dev/ttyUSB0";
  }
  if (!ros::param::get("baudrate", baudrate)) {
    baudrate = 9600;
  }
  
  ros::Publisher pub = n.advertise<sensor_msgs::Range>("abd_impulse", 100);
  ros::Rate r(10);

  InitSerial(port, baudrate);
  if (encod_ser.isOpen()) {
    while(ros::ok()) {
      ReadEncoder();
      ros::spinOnce();
      r.sleep();
    }
  }

  return 0;

}


void InitSerial(const std::string& port_id, const int& baudrate) {
  try {
    encod_ser.setPort(port_id);
    encod_ser.setBaudrate(baudrate);
    encod_ser.setParity(serial::parity_none);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(1000);
    encod_ser.setTimeout(time_out);
    encod_ser.open();
  } catch (const serial::IOException& ex) {
    ROS_INFO("Unable to open port on %s -> [%s]", port_id, ex.what());
    return ;
  }

}

void ReadEncoder() {
  std::vector<_u8> data_1, data_2, data_3, data_4;

  encod_ser.write(REQUEST_DATA_1, 8);
  if (encod_ser.available()) {
    int size = encod_ser.available();
    encod_ser.read(data_1, size);
  }
  int result_1;
  result_1 = ((_u16)data_1[3] << 24) + ((_u16)data_1[4] << 16) +
             ((_u16)data_1[5] << 8) + (_u16)data_1[6];

  encod_ser.write(REQUEST_DATA_2, 8);
  if (encod_ser.available()) {
    int size = encod_ser.available();
    encod_ser.read(data_2, size);
  }
  int result_2;
  result_2 = ((_u16)data_2[3] << 24) + ((_u16)data_2[4] << 16) +
             ((_u16)data_2[5] << 8) + (_u16)data_2[6];

  encod_ser.write(REQUEST_DATA_3, 8);
  if (encod_ser.available()) {
    int size = encod_ser.available();
    encod_ser.read(data_3, size);
  }
  int result_3;
  result_3 = ((_u16)data_3[3] << 24) + ((_u16)data_3[4] << 16) +
             ((_u16)data_3[5] << 8) + (_u16)data_3[6];

  encod_ser.write(REQUEST_DATA_4, 8);
  if (encod_ser.available()) {
    int size = encod_ser.available();
    encod_ser.read(data_4, size);
  }
  int result_4;
  result_4 = ((_u16)data_4[3] << 24) + ((_u16)data_4[4] << 16) +
             ((_u16)data_4[5] << 8) + (_u16)data_4[6];
}



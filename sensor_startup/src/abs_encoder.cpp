#include <iostream>
#include <serial/serial.h>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/Range.h"

typedef uint8_t _u8;
_u8 cmd[6] = {0x01, 0x03, 0xaa, 0x16, 0x0e, 0xb6};
serial::Serial encod_ser;

void InitSerial(const std::string& port_id, const int& baudrate);
void Loop(ros::Publisher pub);

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
      Loop(pub);
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

void Loop(ros::Publisher pub) {
  encod_ser.write(cmd, 6);
  _u8* buffer;
  int size;
  encod_ser.read(buffer, size);

  int result;
  result = (buffer[2] << 8 + buffer[3]);
  sensor_msgs::Range impulse;
  impulse.header.frame_id = "abs_encoder";
  impulse.header.stamp = ros::Time::now();

  impulse.range = (double)result;
  pub.publish(impulse);
}

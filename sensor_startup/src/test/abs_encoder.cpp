#include <iostream>
#include <serial/serial.h>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/Range.h"

typedef uint8_t _u8;
typedef uint16_t _u16;
_u8 cmd[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xc4, 0x0b};
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
  size_t n = sizeof(cmd) / sizeof(cmd[0]);
  encod_ser.write(cmd, n);

  if (encod_ser.available()) {
    std::vector<_u8> data;
    int size = encod_ser.available();
    encod_ser.read(data, size); 
    
    for (size_t i = 0; i < size; i++) {
      std::cout << "0x" << std::hex << (_u16)data[i] << "  ";
    } 
    std::cout << std::endl;

    int result;
    result = ((_u16)data[3] << 24) + 
             ((_u16)data[4] << 16) +
             ((_u16)data[5] << 8) + 
             (_u16)data[6]; 

    sensor_msgs::Range impulse;
    impulse.header.frame_id = "abs_encoder";
    impulse.header.stamp = ros::Time::now();

    impulse.range = (double)result;
    pub.publish(impulse);
  }
  
}

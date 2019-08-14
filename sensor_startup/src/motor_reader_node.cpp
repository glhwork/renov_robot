#include "sensor_startup/motor_reader.h"

using mobile_base::MotorReader;

int main(int argc, char** argv) {
  ros::init(argc, argv, "motor_reader");

  MotorReader motor_reader;
  motor_reader.Loop();
  ros::spin();

  return 0;


}

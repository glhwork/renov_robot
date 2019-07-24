#include "sensor_startup/MobileMotor.h"

using mobile::MobileMotor;

int main(int argc, char** argv) {
  ros::init(argc, argv, "motor_core");
  ros::NodeHandle n;

  MobileMotor motor;
  motor.Loop();
  ros::spin();

  return 0;


}

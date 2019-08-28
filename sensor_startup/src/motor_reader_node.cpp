#include "sensor_startup/motor_reader.h"

using mobile_base::MotorReader;

int main(int argc, char** argv) {
  ros::init(argc, argv, "motor_reader");
  ros::NodeHandle nh;
  MotorReader motor_reader;

  ros::Subscriber control_sub, home_sub, teleop_sub, stop_sub, odom_sub;
  control_sub =
      nh.subscribe("cmd_base_joint", 10, &MotorReader::ControlCallback, &motor_reader);
  home_sub = nh.subscribe("mobile_platform_driver_position_feedback", 10,
                          &MotorReader::GetHomeCallback, &motor_reader);
  stop_sub = nh.subscribe("stop", 10, &MotorReader::StopCallback, &motor_reader);
  odom_sub = nh.subscribe("odom", 10, &MotorReader::OdomCallback, &motor_reader);

  motor_reader.Loop();
  ros::spin();

  return 0;


}

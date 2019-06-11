#include "sensor_startup/MobileMotor.h"

using mobile::MobileMotor;

int main(int argc, char** argv) {
  ros::init(argc, argv, "motor_core");
  ros::NodeHandle n;

  MobileMotor motor;
  ros::Rate r(10);
  ros::Subscriber control_pub =
      n.subscribe("cmd_base_joint", 10, &MobileMotor::ControlCallback, &motor);
  ros::Subscriber teleop_sub = 
      n.subscribe("cmd_vel", 10, &MobileMotor::TeleopCallback, &motor);
  ros::Subscriber stop_sub = 
      n.subscribe("stop", 10, &MobileMotor::StopCallback, &motor);
  ros::Timer feedback_timer = 
      n.createTimer(ros::Duration(0.1), &MobileMotor::FeedbackCallback, &motor);
  r.sleep();  // ????????
  ros::spin();

  return 0;


}

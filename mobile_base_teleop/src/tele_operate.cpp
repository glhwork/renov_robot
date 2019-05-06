#include "mobile_base_teleop/Teleop.h"

using teleop::Teleop;

int main(int argc, char** argv) {
  ros::init(argc, argv, "tele_operate");
  ros::NodeHandle n;
  ros::Rate r(10);

  Teleop tele;
  std::cout << "get class" << std::endl;
  while (ros::ok()) {
    tele.GetKey();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
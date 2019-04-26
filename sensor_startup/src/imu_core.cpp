#include "sensor_startup/MobileImu.h"

using mobile::MobileImu;

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_core");
  ros::NodeHandle n;
  
  int imu_pub_rate;
  if (!ros::param::get("imu_pub_rate", imu_pub_rate)) {
    imu_pub_rate = 10;
  }

  MobileImu imu;
  ros::Rate r(imu_pub_rate);
  while (ros::ok()) {
    imu.ReadData();
    ros::spinOnce();
    r.sleep();
  }
  return 0;

}
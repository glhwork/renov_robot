#include "sensor_startup/ImuReader.h"

using mobile_base::ImuReader; 

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_core");
  ros::NodeHandle n;
  
  int imu_pub_rate;
  if (!ros::param::get("imu_pub_rate", imu_pub_rate)) {
    imu_pub_rate = 10;
  }

  ImuReader imu_reader;
  ros::Rate r(imu_pub_rate);
  while (ros::ok()) {
    imu_reader.ReadData();
    ros::spinOnce();
    r.sleep();
  }
  return 0;

}
#include "sensor_startup/imu_reader.h"

using mobile_base::ImuReader; 

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_reader");
  ros::NodeHandle n;
  
  int imu_publish_rate;
  if (!ros::param::get("imu_publish_rate", imu_publish_rate)) {
    imu_publish_rate = 10;
  }

  ImuReader imu_reader;
  ros::Rate r(imu_publish_rate);
  while (ros::ok()) {
    imu_reader.ReadData();
    ros::spinOnce();
    r.sleep();
  }
  return 0;

}
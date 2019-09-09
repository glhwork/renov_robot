#include "sensor_startup/kinco/driver_reader_ros.h"

using mobile_base::DriverReaderROS;

int main(int argc, char** argv) {
  ros::init(argc, argv, "driver_reader_ros_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  
  DriverReaderROS driver_reader(nh, nh_private);
  ros::spin();

  return 0;
}
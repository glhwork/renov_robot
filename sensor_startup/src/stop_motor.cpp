#include "ros/ros.h"
#include "std_msgs/Bool.h"



int main(int argc, char** argv) {
  ros::init(argc, argv, "stop_motor");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<std_msgs::Bool>("stop", 100);
  ros::Rate r(1);
  while (ros::ok()) {
    std_msgs::Bool stop_cmd;
    stop_cmd.data = true;
    pub.publish(stop_cmd);

    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
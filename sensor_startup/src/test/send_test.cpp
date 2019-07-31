#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#define PI 3.141592653

void Send(double v_linear, double p_angular, ros::Publisher& pub) {
  sensor_msgs::JointState js;
  js.header.frame_id = "motor";
  js.header.stamp = ros::Time::now();

  js.name.clear();
  js.name.resize(8);
  js.name = {"front_left_walking",
             "front_right_walking",
             "rear_left_walking",
             "rear_right_walking",
             "front_right_steering",
             "front_left_steering",
             "rear_right_steering",
             "rear_left_steering"};
  // js.name.push_back("front_left_walking");
  // js.name.push_back("front_right_walking");
  // js.name.push_back("rear_left_walking");
  // js.name.push_back("rear_right_walking");
  // js.name.push_back("front_left_steering");
  // js.name.push_back("front_right_steering");
  // js.name.push_back("rear_left_steering");
  // js.name.push_back("rear_right_steering");

  js.position.resize(8);
  js.position = {0, 0, 0, 0, p_angular, p_angular, p_angular, p_angular};
  js.velocity.resize(8);
  js.velocity = {v_linear, v_linear, v_linear, v_linear, 0, 0, 0, 0};

  pub.publish(js);
}

void StatePlan(ros::Publisher& pub) {

  std::cout << "ready to send" << std::endl;
  sleep(3);
/* 
  Send(10, PI/4, pub);
  ros::Duration(5).sleep();

  Send(20, PI/4, pub);
  ros::Duration(5).sleep();

  Send(40, PI/4, pub);
  ros::Duration(5).sleep();
 */

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "send_test");
  ros::NodeHandle nh;

  ros::Publisher pub 
      = nh.advertise<sensor_msgs::JointState>("cmd_base_joint", 100);
  
  ros::Rate r(10);
  while (ros::ok) {
    Send(0, PI/2, pub);
    ros::spinOnce();
    r.sleep();
  }
  // StatePlan(pub);




  return 0;
}
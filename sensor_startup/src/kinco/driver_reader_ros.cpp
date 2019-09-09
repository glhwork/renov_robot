#include "sensor_startup/kinco/driver_reader_ros.h"

namespace mobile_base {

DriverReaderROS::DriverReaderROS(ros::NodeHandle nh,
                                 ros::NodeHandle nh_private) {
  joint_state_pub_ =
      nh.advertise<sensor_msgs::JointState>(joint_state_pub_topic_, 100);

  control_signal_sub_ = nh.subscribe(
      "cmd_vel_base", 10, DriverReaderROS::GetControlSignalCallback, this);
}

DriverReaderROS::~DriverReaderROS() {}

void DriverReaderROS::ParamInit(ros::NodeHandle nh_private) {
  nh_private.param("joint_state_pub_topic", joint_state_pub_topic_,
                   std::string("motor_joint_state"));
}

void DriverReaderROS::GetControlSignalCallback(
    const sensor_msgs::JointState& js_msg) {
  if (js_msg.name.size() < id_num_) {
    ROS_WARN("Incorrect number of joint states");
    return;
  }

  std::vector<int> control_singal;
  for (size_t i = 0; i < walk_id_num_; i++) {
    control_singal.push_back(js_msg.velocity[i]);
  }
  for (size_t i = 0; i < steer_id_num_; i++) {
    control_singal.push_back(js_msg.position[i + walk_id_num_]);
  }

  ControlMotor(control_singal);
}
}  // namespace mobile_base

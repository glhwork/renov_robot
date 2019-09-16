#include "sensor_startup/kinco/driver_controller_ros.h"

namespace mobile_base {

DriverControllerROS::DriverControllerROS(ros::NodeHandle nh,
                                         ros::NodeHandle nh_private) {
  ParamInit(nh_private);
  DebugData(if_debug_);
  GetBaseAddress(base_file_address_);

  CanActivate(can_config_address_);
  if_initial_ = DriverInit();

  joint_state_pub_ =
      nh.advertise<sensor_msgs::JointState>(joint_state_pub_topic_, 100);

  control_signal_sub_ = nh.subscribe(
      "cmd_vel_base", 10, &DriverControllerROS::GetControlSignalCallback, this);
}

DriverControllerROS::~DriverControllerROS() {}

void DriverControllerROS::ParamInit(ros::NodeHandle nh_private) {
  nh_private.param("joint_state_pub_topic", joint_state_pub_topic_,
                   std::string("motor_joint_state"));
  nh_private.param("if_debug", if_debug_, false);
  nh_private.param("can_config_address", can_config_address_,
                   std::string("/config/kinco_can_config.yaml"));
  nh_private.param(
      "base_file_address", base_file_address_,
      std::string("/home/renov_robot/renov_ws/src/renov_robot/sensor_startup"));
}

void DriverControllerROS::GetControlSignalCallback(
    const sensor_msgs::JointState& js_msg) {
  if (!if_initial_) {
    ROS_WARN("The driver controller is waiting for initialization");
    return;
  }
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

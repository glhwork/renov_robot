#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#include "driver_reader.h"

namespace mobile_base {

class DriverReaderROS : public DriverReader {
 public:
  DriverReaderROS(ros::NodeHandle nh, ros::NodeHandle nh_private);
  virtual ~DriverReaderROS();
  void ParamInit(ros::NodeHandle nh_private);
  void GetControlSignalCallback(const sensor_msgs::JointState& js_msg);

 private:
  std::string joint_state_pub_topic_;

  ros::Subscriber control_signal_sub_;
  ros::Publisher joint_state_pub_;

};  // class DriverReaderROS

}  // namespace mobile_base
#include <cmath>
#include "sensor_startup/MobileMotor.h"



namespace mobile {

class Tele : public MobileMotor {
  public:
    Tele() : MobileMotor() { 
      SetTeleopMode();
      ReadTeleopFile(); 
    }
    virtual ~Tele() {}
    void SetTeleopMode();
    void ReadTeleopFile();
    void TeleControl(const geometry_msgs::Twist::ConstPtr& twist);

  private:
    double wheel_dis_len;
    double wheel_dis_wid;
    


};

void Tele::SetTeleopMode() {
  int len;

  len = sizeof(cmd.SET_MODE_VELOCITY) / sizeof(cmd.SET_MODE_VELOCITY[0]);
  ModeCommand(cob_id[0], cob_id[1], len, VELOCITY_MODE);

  len = sizeof(cmd.SET_MODE_POSITION) / sizeof(cmd.SET_MODE_POSITION[0]);
  ModeCommand(cob_id[2], cob_id[3], len, POSITION_MODE);
}

void Tele::ReadTeleopFile() {
  YAML::Node config 
    = YAML::LoadFile("~/renov_ws/src/renov_robot/sensor_startup/data/teleop_config.yaml");
  wheel_dis_len = config["wheel_dis_len"].as<double>();
  wheel_dis_wid = config["wheel_dis_wid"].as<double>(); 

} 


void Tele::TeleControl(const geometry_msgs::Twist::ConstPtr& twist) {

  double vx = twist->linear.x;
  double az = twist->angular.z;
  
  if (vx != 0) {
    std::vector<float> velocity;
    for (size_t i = 0; i < 4; i++) {
      velocity.push_back(vx);
    }
    for (size_t i = 0; i < 4; i++) {
      velocity.push_back(0);
    }
    ControlMotor(velocity);
  } else {
    double hypotenuse = sqrt(pow(wheel_dis_len, 2) + pow(wheel_dis_wid, 2));
    double r = 0.5 * hypotenuse; 

    double alpha = atan(wheel_dis_len / wheel_dis_wid);
    std::vector<float> velocity;



  }
}





}  // namespace mobile


int main(int argc, char** argv) {
  ros::init(argc, argv, "teleop_node");
  ros::NodeHandle n;

  mobile::Tele tele;
  ros::Subscriber tele_sub = n.subscribe("cmd_vel", 100, &mobile::Tele::TeleControl, &tele);

  ros::spin();
  return 0;
}
#include <cmath>
#include "sensor_startup/MobileMotor.h"



namespace mobile {

class Tele : public MobileMotor {
  public:
    Tele() : MobileMotor() { 
      // SetTeleopMode();
      ReadTeleopFile(); 
    }
    virtual ~Tele() {}
    // void SetTeleopMode();
    void ReadTeleopFile();
    void TeleControl(const geometry_msgs::Twist::ConstPtr& twist);
    void TeleStop(const std_msgs::Bool& data);

  private:
    double wheel_dis_len;
    double wheel_dis_wid;
    


};

// void Tele::SetTeleopMode() {
//   int len;

//   len = sizeof(cmd.SET_MODE_VELOCITY) / sizeof(cmd.SET_MODE_VELOCITY[0]);
//   ModeCommand(cob_id[0], cob_id[1], len, VELOCITY_MODE);

//   len = sizeof(cmd.SET_MODE_POSITION) / sizeof(cmd.SET_MODE_POSITION[0]);
//   ModeCommand(cob_id[2], cob_id[3], len, POSITION_MODE);
// }

void Tele::ReadTeleopFile() {
  YAML::Node config 
    = YAML::LoadFile("~/renov_ws/src/renov_robot/sensor_startup/data/teleop_config.yaml");
  wheel_dis_len = config["wheel_dis_len"].as<double>();
  wheel_dis_wid = config["wheel_dis_wid"].as<double>(); 

} 


void Tele::TeleControl(const geometry_msgs::Twist::ConstPtr& twist) {

  double vx = twist->linear.x;
  double az = twist->angular.z;
  
  if (vx != 0 && az == 0) {
    std::vector<float> velocity;
    for (size_t i = 0; i < 4; i++) {
      velocity.push_back((float)vx);
    }
    for (size_t i = 0; i < 4; i++) {
      velocity.push_back(0);
    }
    ControlMotor(velocity);
  } else if (vx == 0 && az != 0) {
    double hypotenuse = sqrt(pow(wheel_dis_len, 2) + pow(wheel_dis_wid, 2));
    double r = 0.5 * hypotenuse; 

    float alpha = fabs(atan(wheel_dis_len / wheel_dis_wid));
    std::vector<float> velocity;
    float v_linear = az * r / (float)0.15;
    velocity.resize(8);
    velocity = {-v_linear, v_linear, -v_linear, v_linear,
                   -alpha,    alpha,     alpha,   -alpha};
    ControlMotor(velocity);
  } else if (0 == vx && 0 == az) {
    std::vector<float> velocity;
    velocity.resize(8);
    velocity = {0, 0, 0, 0, 0, 0, 0, 0};
    ControlMotor(velocity);
  }
}

void Tele::TeleStop(const std_msgs::Bool& stop) {
  int len;

  // send command to disenable the drivers
  PVCI_CAN_OBJ obj = GetVciObject(id_num);
  len = sizeof(cmd.DISENABLE_COMMAND) / sizeof(cmd.DISENABLE_COMMAND[0]);
  for (size_t i = 0; i < id_num; i++) {
    obj[i].ID = obj[i].ID + i + 1;
    obj[i].ExternFlag = 0;
    obj[i].RemoteFlag = 0;
    obj[i].SendType = 0;
    obj[i].DataLen = len;
    DataInitial(obj[i].Data, cmd.DISENABLE_COMMAND, len);
  }
  PrintTest(obj[0].Data, len, "disenable process : ");
  SendCommand(obj, id_num);
  delete [] obj;

  // send command to save the current state of drivers and motors
  obj = GetVciObject(id_num);
  len = sizeof(cmd.SAVE_PARAMETERS) / sizeof(cmd.SAVE_PARAMETERS[0]);
  for (size_t i = 0; i < id_num; i++) {
    obj[i].ID = obj[i].ID + i + 1;
    obj[i].ExternFlag = 0;
    obj[i].RemoteFlag = 0;
    obj[i].SendType = 0;
    obj[i].DataLen = len;
    DataInitial(obj[i].Data, cmd.SAVE_PARAMETERS, len);
  }
  PrintTest(obj[0].Data, len, "save parameter : ");
  SendCommand(obj, id_num);
  delete [] obj;
  if (!VCI_CloseDevice(device_type, device_index)) {
    ROS_WARN("close device failure");
  } else {
    ROS_INFO("close device success");
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
#include "sensor_startup/MobileMotor.h"

using mobile::MobileMotor;

MobileMotor::MobileMotor() {
  n_private = ros::NodeHandle("motor_core");
  if_initial = true;
  ParamInit();
  ReadFile(file_address);
  Setup();
  CanBusInit();
}

void MobileMotor::ParamInit() {

  if (!n_private.getParam("port", port)) {
    port = "/dev/ttyUSB0";
  }
  if (!n_private.getParam("state_topic", state_topic)) {
    state_topic = "motor_state";
  }
  if (!n_private.getParam("file_address", file_address)) {
    file_address = 
      "/home/glh/renov_ws/src/renov_robot/sensor_startup/data/motor_config.yaml";
  }


}

void MobileMotor::ReadFile(const std::string& address) {
  YAML::Node param = YAML::LoadFile(address);

  device_type = param["device_type"].as<int>();
  device_index = param["device_index"].as<int>();
  can_index = param["can_index"].as<int>();
  
  steering_mode = param["steering_mode"].as<int>();
  walking_mode = param["walking_mode"].as<int>();

  encoder_lines = (unsigned int)param["encoder_lines"].as<int>();
  
  cob_id[0] = param["walking_channel"]["chn1"].as<int>();
  cob_id[1] = param["walking_channel"]["chn2"].as<int>();
  cob_id[2] = param["steering_channel"]["chn1"].as<int>();
  cob_id[3] = param["steering_channel"]["chn2"].as<int>();

  IdCheck();
}

void MobileMotor::Setup() {
  state_pub = nh.advertise<sensor_msgs::JointState>(state_topic, 100);
}

void MobileMotor::CanBusInit() {
  
  if (!VCI_OpenDevice(device_type, device_index, 0)) {
    ROS_ERROR("open CAN on ttyUSB-%d failure", device_index);
  } else {
    ROS_INFO("open CAN successfully");
  }

  VCI_INIT_CONFIG config;
  config.AccCode = 0x00;
  config.AccMask = 0x00;
  config.Filter = 8;
  config.Mode = 0;
  config.Timing0 = 0x00;
  config.Timing1 = 0x1c;
  
  if (!VCI_InitCAN(device_type, device_index, can_index, &config)) {
    ROS_ERROR("initialize failure");
  } else {
    ROS_INFO("initialize successfully");
  }

  if (!VCI_StartCAN(device_type, device_index, can_index)) {
    std::cout << "start failure" << std::endl;
  } else {
    std::cout << "start successfully" << std::endl;
  }



}

void MobileMotor::SetMode() {
  if (1 == steering_mode) {
    int obj_num = 2;
    PVCI_CAN_OBJ obj = GetVciObject(obj_num, cob_id[2]);
    // obj[0]->Data = &BASE_VELOCITY_COMMAND;
    // obj[0].DataLen = 
    VCI_Transmit(device_type, device_index, can_index, obj, obj_num);

    delete [] obj;
  }
}

VCI_CAN_OBJ* MobileMotor::GetVciObject(const int& obj_num, const int& chn) {

  PVCI_CAN_OBJ obj;
  obj = new _VCI_CAN_OBJ[obj_num];
  for (size_t i = 0; i < obj_num; i++) {
    obj[i].ID = 0x00000600 + chn;
    obj[i].RemoteFlag = 0;
    obj[i].SendType = 0;
    obj[i].RemoteFlag = 0;
    obj[i].ExternFlag = 0;
  }

  return obj;
 
}

void MobileMotor::IdCheck() {

  int n = sizeof(cob_id) / sizeof(cob_id[0]);
  for (size_t i = 0; i < n; i++) {
    for (size_t j = i+1; j < n; j++) {
      if (cob_id[i] == cob_id[i+1]) {
        ROS_WARN("id-%d and id-%d are equal", i, j);
        if_initial = false;
        break;
      }
    }
  }

  if (if_initial) {
    ROS_INFO("channel set successfully");
  }

}

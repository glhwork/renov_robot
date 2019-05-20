#include "sensor_startup/MobileMotor.h"

using mobile::MobileMotor;

MobileMotor::MobileMotor() {
  n_private = ros::NodeHandle("motor_core");
  if_initial = true;
  ParamInit();
  ReadFile(file_address);
  Setup();
  if (CanBusInit()) {
    if_initial = true;
  } else {
    if_initial = false;
  }
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
  id_num = param["id_num"].as<int>();
  
  steering_mode = param["steering_mode"].as<int>();
  walking_mode = param["walking_mode"].as<int>();

  encoder_lines = (uint)param["encoder_lines"].as<int>();
  
  cob_id[0] = param["walking_channel"]["chn1"].as<int>();
  cob_id[1] = param["walking_channel"]["chn2"].as<int>();
  cob_id[2] = param["steering_channel"]["chn1"].as<int>();
  cob_id[3] = param["steering_channel"]["chn2"].as<int>();

  IdCheck();
}

void MobileMotor::Setup() {
  state_pub = nh.advertise<sensor_msgs::JointState>(state_topic, 100);
}

bool MobileMotor::CanBusInit() {
  
  if (!VCI_OpenDevice(device_type, device_index, 0)) {
    ROS_ERROR("open CAN on ttyUSB-%d failure", device_index);
    return false;
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
    return false;
  } else {
    ROS_INFO("initialize successfully");
  }

  if (!VCI_StartCAN(device_type, device_index, can_index)) {
    std::cout << "start failure" << std::endl;
    return false;
  } else {
    std::cout << "start successfully" << std::endl;
  }

  if (!SetMode()) {
    ROS_WARN("set mode failure!!");
    return false;
  }
  if (!EnableMotor()) {
    ROS_WARN("enable failure");
    return false;
  }

  return true;

}

// bad encapisulation and reuse of codes of this function!!!
bool MobileMotor::SetMode() {
  
  // set mode of steering motors
  switch (steering_mode) {
    case POSITION_MODE: {
      int obj_num = 2;
      PVCI_CAN_OBJ obj = GetVciObject(obj_num);

      obj[0].ID = obj[0].ID + cob_id[2];
      obj[0].DataLen =
          sizeof(cmd.SET_MODE_POSITION) / sizeof(cmd.SET_MODE_POSITION[0]);
      DataTransform(obj[0].Data, cmd.SET_MODE_POSITION, obj[0].DataLen);
      obj[1].ID = obj[1].ID + cob_id[3];
      obj[1].DataLen =
          sizeof(cmd.SET_MODE_POSITION) / sizeof(cmd.SET_MODE_POSITION[0]);
      DataTransform(obj[1].Data, cmd.SET_MODE_POSITION, obj[1].DataLen);
      SendCommand(obj, obj_num);

      std::cout << "steering mode == position mode" << std::endl;
      delete[] obj;
      break;
    }
    case VELOCITY_MODE: {
      int obj_num = 2;
      PVCI_CAN_OBJ obj = GetVciObject(obj_num);

      obj[0].ID = obj[0].ID + cob_id[2];
      obj[0].DataLen =
          sizeof(cmd.SET_MODE_VELOCITY) / sizeof(cmd.SET_MODE_VELOCITY[0]);
      DataTransform(obj[0].Data, cmd.SET_MODE_VELOCITY, obj[0].DataLen);
      obj[1].ID = obj[1].ID + cob_id[3];
      obj[1].DataLen =
          sizeof(cmd.SET_MODE_VELOCITY) / sizeof(cmd.SET_MODE_VELOCITY[0]);
      DataTransform(obj[1].Data, cmd.SET_MODE_VELOCITY, obj[1].DataLen);
      SendCommand(obj, obj_num);

      std::cout << "steering mode == velocity mode" << std::endl;
      delete[] obj;
      break;
    }  
    case CURRENT_MODE: {
      int obj_num = 2;
      PVCI_CAN_OBJ obj = GetVciObject(obj_num);

      obj[0].ID = obj[0].ID + cob_id[2];
      obj[0].DataLen =
          sizeof(cmd.SET_MODE_CURRENT) / sizeof(cmd.SET_MODE_CURRENT[0]);
      DataTransform(obj[0].Data, cmd.SET_MODE_CURRENT, obj[0].DataLen);
      obj[1].ID = obj[1].ID + cob_id[3];
      obj[1].DataLen =
          sizeof(cmd.SET_MODE_CURRENT) / sizeof(cmd.SET_MODE_CURRENT[0]);
      DataTransform(obj[1].Data, cmd.SET_MODE_CURRENT, obj[1].DataLen);
      SendCommand(obj, obj_num);

      std::cout << "steering mode == current mode" << std::endl;
      delete[] obj;
      break;
    }
    default: {
      ROS_WARN("Incorrect mode number of steering motors!!");
      return false;
      break;
    }
  }  // switch-case end of steering mode setting

  // set mode of walking motors
  switch (walking_mode) {
    case POSITION_MODE: {
      int obj_num = 2;
      PVCI_CAN_OBJ obj = GetVciObject(obj_num);

      obj[0].ID = obj[0].ID + cob_id[0];
      obj[0].DataLen =
          sizeof(cmd.SET_MODE_POSITION) / sizeof(cmd.SET_MODE_POSITION[0]);
      DataTransform(obj[0].Data, cmd.SET_MODE_POSITION, obj[0].DataLen);
      obj[1].ID = obj[1].ID + cob_id[1];
      obj[1].DataLen =
          sizeof(cmd.SET_MODE_POSITION) / sizeof(cmd.SET_MODE_POSITION[0]);
      DataTransform(obj[1].Data, cmd.SET_MODE_POSITION, obj[1].DataLen);
      SendCommand(obj, obj_num);

      std::cout << "walking mode == position mode" << std::endl;
      delete[] obj;
      break;
    }
    case VELOCITY_MODE: {
      int obj_num = 2;
      PVCI_CAN_OBJ obj = GetVciObject(obj_num);

      obj[0].ID = obj[0].ID + cob_id[0];
      obj[0].DataLen =
          sizeof(cmd.SET_MODE_VELOCITY) / sizeof(cmd.SET_MODE_VELOCITY[0]);
      DataTransform(obj[0].Data, cmd.SET_MODE_VELOCITY, obj[0].DataLen);
      obj[1].ID = obj[1].ID + cob_id[1];
      obj[1].DataLen =
          sizeof(cmd.SET_MODE_VELOCITY) / sizeof(cmd.SET_MODE_VELOCITY[0]);
      DataTransform(obj[1].Data, cmd.SET_MODE_VELOCITY, obj[1].DataLen);
      SendCommand(obj, obj_num);

      std::cout << "walking mode == velocity mode" << std::endl;
      delete[] obj;
      break;
    }
    case CURRENT_MODE: {
      int obj_num = 2;
      PVCI_CAN_OBJ obj = GetVciObject(obj_num);

      obj[0].ID = obj[0].ID + cob_id[0];
      obj[0].DataLen =
          sizeof(cmd.SET_MODE_CURRENT) / sizeof(cmd.SET_MODE_CURRENT[0]);
      DataTransform(obj[0].Data, cmd.SET_MODE_CURRENT, obj[0].DataLen);
      obj[1].ID = obj[1].ID + cob_id[1];
      obj[1].DataLen =
          sizeof(cmd.SET_MODE_CURRENT) / sizeof(cmd.SET_MODE_CURRENT[0]);
      DataTransform(obj[1].Data, cmd.SET_MODE_CURRENT, obj[1].DataLen);
      SendCommand(obj, obj_num);

      std::cout << "walking mode == current mode" << std::endl;
      delete[] obj;
      break;
    }
    default: {
      ROS_WARN("Incorrect mode number of walking motors!!");
      return false;
      break;
    }
  }  // switch-case end of walking motor setting

  return true;
}

bool MobileMotor::EnableMotor() {
  int ena_cmd_num = 3;
  PVCI_CAN_OBJ obj = GetVciObject(id_num*ena_cmd_num);
  int index = 1;

  for (size_t i = 0; i < id_num; i++) {
    obj[i*3].ID = obj[i].ID + i + 1;
    obj[i * 3].DataLen =
        sizeof(cmd.ENABLE_COMMAND_1) / sizeof(cmd.ENABLE_COMMAND_1[0]);
    DataTransform(obj[i * 3].Data, 
                  cmd.ENABLE_COMMAND_1, 
                  obj[i * 3].DataLen);
    obj[i*3 + 1].ID = obj[i*3 + 1].ID + i + 1;
    obj[i * 3 + 1].DataLen =
        sizeof(cmd.ENABLE_COMMAND_2) / sizeof(cmd.ENABLE_COMMAND_2[0]);
    DataTransform(obj[i * 3 + 1].Data, 
                  cmd.ENABLE_COMMAND_2,
                  obj[i * 3 + 1].DataLen);
    obj[i*3 + 2].ID = obj[i*3 + 2].ID + i + 1;
    obj[i * 3 + 2].DataLen =
        sizeof(cmd.ENABLE_COMMAND_3) / sizeof(cmd.ENABLE_COMMAND_3[0]);
    DataTransform(obj[i * 3 + 2].Data, 
                  cmd.ENABLE_COMMAND_3,
                  obj[i * 3 + 2].DataLen);
  }
  if (!SendCommand(obj, id_num*ena_cmd_num)) {
    return false;
  } else {
    return true;
  }
  delete [] obj;
}

void MobileMotor::DataTransform(BYTE* data, uint8_t* cmd, const uint& len) {
  int n = sizeof(cmd) / sizeof(cmd[0]);
  std::cout << "get data : ";
  for (size_t i = 0; i < n; i++) {
    data[i] = cmd[i];
    std::cout << cmd[i] << "  ";
  }
  std::cout << std::endl;
}

VCI_CAN_OBJ* MobileMotor::GetVciObject(const int& obj_num) {

  PVCI_CAN_OBJ obj;
  obj = new _VCI_CAN_OBJ[obj_num];
  for (size_t i = 0; i < obj_num; i++) {
    obj[i].ID = 0x00000600;
    obj[i].RemoteFlag = 0;
    obj[i].SendType = 0;
    obj[i].RemoteFlag = 0;
    obj[i].ExternFlag = 0;
  }

  return obj;
 
}

void MobileMotor::IdCheck() {
  
  bool flag = true;
  int n = sizeof(cob_id) / sizeof(cob_id[0]);
  for (size_t i = 0; i < n; i++) {
    for (size_t j = i+1; j < n; j++) {
      if (cob_id[i] == cob_id[i+1]) {
        ROS_WARN("id-%d and id-%d are equal", i, j);
        flag = false;
        break;
      }
    }
  }

  if (flag) {
    ROS_INFO("channel set successfully");
  } else {
    ROS_WARN("channel set failure");
  }

}

uint MobileMotor::SendCommand(PVCI_CAN_OBJ obj, uint len) {
  return VCI_Transmit(device_type, device_index, can_index, obj, len);
}

void MobileMotor::ControlCallback(const sensor_msgs::JointState& joint_state) {
  if (!if_initial) {
    ROS_WARN("control failure caused by initialization failure");
    return ;
  }
}

void MobileMotor::TeleopCallback(const geometry_msgs::Twist& twist) {
  if (!if_initial) {
    ROS_WARN("teleop failure caused by initialization failure");
    return ;
  }
}

void MobileMotor::FeedbackCallback(const ros::Timer&) {
  if (!if_initial) {
    ROS_WARN("feedback failure caused by initialization failure");
    return ;
  }
}
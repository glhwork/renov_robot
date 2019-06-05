#include "sensor_startup/MobileMotor.h"

using mobile::MobileMotor;

MobileMotor::MobileMotor() {
  n_private = ros::NodeHandle("motor_core");
  ParamInit();
  ReadFile(file_address);
  Setup();
  if_initial = CanBusInit();
}

MobileMotor::~MobileMotor() {
  
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
  std::cout << " address is : " << address << std::endl;
  YAML::Node param = YAML::LoadFile(address);

  device_type = param["device_type"].as<int>();
  device_index = param["device_index"].as<int>();
  can_index = param["can_index"].as<int>();
  id_num = param["id_num"].as<int>();
  
  steering_mode = param["steering_mode"].as<int>();
  walking_mode = param["walking_mode"].as<int>();

  encoder_lines = (uint)param["encoder_lines"].as<int>();
  max_velocity = (uint)param["max_velocity"].as<int>();
  
  cob_id[0] = param["walking_channel"]["chn1"].as<int>();
  cob_id[1] = param["walking_channel"]["chn2"].as<int>();
  cob_id[2] = param["steering_channel"]["chn1"].as<int>();
  cob_id[3] = param["steering_channel"]["chn2"].as<int>();

  motor_sign[0] = param["motor_sign"]["fwl"].as<int>();
  motor_sign[1] = param["motor_sign"]["fwr"].as<int>();
  motor_sign[2] = param["motor_sign"]["rwl"].as<int>();
  motor_sign[3] = param["motor_sign"]["rwr"].as<int>();
  motor_sign[4] = param["motor_sign"]["fsl"].as<int>();
  motor_sign[5] = param["motor_sign"]["fsr"].as<int>();
  motor_sign[6] = param["motor_sign"]["rsl"].as<int>();
  motor_sign[7] = param["motor_sign"]["rsr"].as<int>();

  IdCheck();
}

void MobileMotor::Setup() {
  state_pub = nh.advertise<sensor_msgs::JointState>(state_topic, 100);
}

bool MobileMotor::CanBusInit() {
  
  if (!VCI_OpenDevice(device_type, device_index, 0)) {
    ROS_ERROR("open CAN on ttyUSB-%d failure", (int)device_index);
    return false;
  } else {
    ROS_INFO("open CAN successfully");
  }

  VCI_INIT_CONFIG config;
  config.AccCode = 0x00000000;
  config.AccMask = 0xFFFFFFFF;
  config.Filter = 0;
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

bool MobileMotor::SetMode() {
  
  // set mode of walking motors
  switch (walking_mode) {
    case POSITION_MODE: {
      int len =
          sizeof(cmd.SET_MODE_POSITION) / sizeof(cmd.SET_MODE_POSITION[0]);
      ModeCommand(cob_id[0], cob_id[1], len, POSITION_MODE);
      std::cout << "walking mode == position mode" << std::endl;

      // pre-set the velocity under position mode
      // i.e. the limit velocity of this driver
      VCI_CAN_OBJ* pre_v;
      pre_v = GetVciObject(2);
      pre_v[0].ID = pre_v[0].ID + cob_id[0];
      pre_v[1].ID = pre_v[1].ID + cob_id[1];
      len = sizeof(cmd.SET_PROFILE_VELOCITY) /
            sizeof(cmd.SET_PROFILE_VELOCITY[0]);
      DataTransform(pre_v[0].Data, cmd.SET_PROFILE_VELOCITY, len, LEFT_MOTOR,
                    (float)max_velocity);
      DataTransform(pre_v[1].Data, cmd.SET_PROFILE_VELOCITY, len, RIGHT_MOTOR,
                    (float)max_velocity);

      SendCommand(pre_v, 2);

      delete[] pre_v;
      break;
    }
    case VELOCITY_MODE: {
      int len =
          sizeof(cmd.SET_MODE_VELOCITY) / sizeof(cmd.SET_MODE_VELOCITY[0]);
      ModeCommand(cob_id[0], cob_id[1], len, VELOCITY_MODE);
      std::cout << "walking mode == velocity mode" << std::endl;
      break;
    }
    case CURRENT_MODE: {
      int len = sizeof(cmd.SET_MODE_CURRENT) / sizeof(cmd.SET_MODE_CURRENT[0]);
      ModeCommand(cob_id[0], cob_id[1], len, CURRENT_MODE);
      std::cout << "walking mode == current mode" << std::endl;
      break;
    }
    default: {
      ROS_WARN("Incorrect mode number of walking motors!!");
      return false;
      break;
    }
  }  // switch-case end of walking motor setting

  // set mode of steering motors
  switch (steering_mode) {
    case POSITION_MODE: {
      int len =
          sizeof(cmd.SET_MODE_POSITION) / sizeof(cmd.SET_MODE_POSITION[0]);
      ModeCommand(cob_id[2], cob_id[3], len, POSITION_MODE);
      std::cout << "steering mode == position mode" << std::endl;

      VCI_CAN_OBJ* pre_v;
      pre_v = GetVciObject(2);
      pre_v[0].ID = pre_v[0].ID + cob_id[2];
      pre_v[1].ID = pre_v[1].ID + cob_id[3];
      len = sizeof(cmd.SET_PROFILE_VELOCITY) /
            sizeof(cmd.SET_PROFILE_VELOCITY[0]);
      DataTransform(pre_v[0].Data, cmd.SET_PROFILE_VELOCITY, len, LEFT_MOTOR,
                    (float)max_velocity);
      DataTransform(pre_v[1].Data, cmd.SET_PROFILE_VELOCITY, len, RIGHT_MOTOR,
                    (float)max_velocity);

      SendCommand(pre_v, 2);

      delete[] pre_v;
      break;
    }
    case VELOCITY_MODE: {
      int len =
          sizeof(cmd.SET_MODE_VELOCITY) / sizeof(cmd.SET_MODE_VELOCITY[0]);
      ModeCommand(cob_id[2], cob_id[3], len, VELOCITY_MODE);
      std::cout << "steering mode == velocity mode" << std::endl;
      break;
    }  
    case CURRENT_MODE: {
      int len =
          sizeof(cmd.SET_MODE_CURRENT) / sizeof(cmd.SET_MODE_CURRENT[0]);
      ModeCommand(cob_id[2], cob_id[3], len, CURRENT_MODE);
      std::cout << "steering mode == current mode" << std::endl;
      break;
    }
    default: {
      ROS_WARN("Incorrect mode number of steering motors!!");
      return false;
      break;
    }
  }  // switch-case end of steering motor setting

  return true;
}

void MobileMotor::ModeCommand(const int& id_0, const int& id_1, 
                              const int& len, const uint8_t& mode) {
  int obj_num = 2;
  PVCI_CAN_OBJ obj = GetVciObject(obj_num);

  obj[0].ID = obj[0].ID + id_0;
  obj[0].DataLen = len;
  // DataInitial(obj[0].Data, cmd.SET_MODE_CURRENT, obj[0].DataLen);
  obj[1].ID = obj[1].ID + id_1;
  obj[1].DataLen = len;
  // DataInitial(obj[1].Data, cmd.SET_MODE_CURRENT, obj[1].DataLen);
  switch (mode) {
    case POSITION_MODE: {
      DataInitial(obj[0].Data, cmd.SET_MODE_POSITION, len);
      DataInitial(obj[1].Data, cmd.SET_MODE_POSITION, len);
      break;
    }
    case VELOCITY_MODE: {
      DataInitial(obj[0].Data, cmd.SET_MODE_VELOCITY, len);
      DataInitial(obj[1].Data, cmd.SET_MODE_VELOCITY, len);
      break;
    }
    case CURRENT_MODE: {
      DataInitial(obj[0].Data, cmd.SET_MODE_CURRENT, len);
      DataInitial(obj[1].Data, cmd.SET_MODE_CURRENT, len);
      break;
    }
    default: {
      break;
    }
  }

  SendCommand(obj, obj_num);

  delete[] obj;
}


bool MobileMotor::EnableMotor() {
  int ena_cmd_num = 3;
  PVCI_CAN_OBJ obj = GetVciObject(id_num*ena_cmd_num);
  int index = 1;

  for (size_t i = 0; i < id_num; i++) {
    obj[i * 3].ID = obj[i].ID + i + 1;
    obj[i * 3].DataLen =
        sizeof(cmd.ENABLE_COMMAND_1) / sizeof(cmd.ENABLE_COMMAND_1[0]);
    DataInitial(obj[i * 3].Data, cmd.ENABLE_COMMAND_1, obj[i * 3].DataLen);
    obj[i * 3 + 1].ID = obj[i * 3 + 1].ID + i + 1;
    obj[i * 3 + 1].DataLen =
        sizeof(cmd.ENABLE_COMMAND_2) / sizeof(cmd.ENABLE_COMMAND_2[0]);
    DataInitial(obj[i * 3 + 1].Data, cmd.ENABLE_COMMAND_2,
                obj[i * 3 + 1].DataLen);
    obj[i * 3 + 2].ID = obj[i * 3 + 2].ID + i + 1;
    obj[i * 3 + 2].DataLen =
        sizeof(cmd.ENABLE_COMMAND_3) / sizeof(cmd.ENABLE_COMMAND_3[0]);
    DataInitial(obj[i * 3 + 2].Data, cmd.ENABLE_COMMAND_3,
                obj[i * 3 + 2].DataLen);
  }
  if (!SendCommand(obj, id_num*ena_cmd_num)) {
    delete [] obj;
    return false;
  } else {
    delete [] obj;
    return true;
  }
}

void MobileMotor::DataInitial(BYTE* data, uint8_t* cmd, const uint& len) {
  for (size_t i = 0; i < len; i++) {
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
        ROS_WARN("id-%d and id-%d are equal", (int)i, (int)j);
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

uint MobileMotor::SendCommand(PVCI_CAN_OBJ obj, const uint& len) {

  return VCI_Transmit(device_type, device_index, can_index, obj, len);

}

void MobileMotor::ControlCallback(const sensor_msgs::JointState& joint_state) {
  if (!if_initial) {
    ROS_WARN("control failure caused by initialization failure");
    return ;
  }
  
  if (joint_state.velocity.size() < 8 || joint_state.position.size() < 8) {
    ROS_WARN("Incorrect quantity of commands");
    return ;
  }
  
  // get data from first four elements of velocity for walking and
  // last four elements of position for steering
  std::vector<float> state_cmds;
  for (size_t i = 0; i < 4; i++) {
    state_cmds.push_back(joint_state.velocity[i]);
  }
  for (size_t i = 0; i < 4; i++) {
    state_cmds.push_back(joint_state.position[i+4]);
  }
  ControlMotor(state_cmds);
  
}

void MobileMotor::DataTransform(BYTE* data, uint8_t* cmd, const uint& len, 
                                const uint8_t& index, const float& velo) {
  DataInitial(data, cmd, len);
  data[2] = index;

  int n;
  switch (cmd[0]) {
    case DATA_UINT8: {
      n = 1;
      break;
    }
    case DATA_UINT16: {
      n = 2;
      break;
    }
    case DATA_UINT32: {
      n = 4;
      break;
    }
    default: {
      ROS_WARN("wrong data type");
      break;  // break or return ???
    }
  }
  for (size_t i = 0; i < n; i++) {
    data[i + 4] = (((int)velo >> (i * 8)) & 0xff);
  }

}

void MobileMotor::TeleopCallback(const geometry_msgs::Twist& twist) {
  if (!if_initial) {
    ROS_WARN("teleop failure caused by initialization failure");
    return ;
  }
 
  

  
}

void MobileMotor::FeedbackCallback(const ros::TimerEvent&) {
  if (!if_initial) {
    ROS_WARN("feedback failure caused by initialization failure");
    return ;
  }
}

void MobileMotor::ControlMotor(const std::vector<float>& state) {

  // 0->left, 1->right for walking driver 1 front
  // 2->left, 3->right for walking driver 2 rear
  // 4->left, 5->right for steering driver 1 front
  // 6->left. 7->right for steering driver 2 rear
  int obj_num = 8;
  PVCI_CAN_OBJ obj = new VCI_CAN_OBJ[obj_num];

  switch (walking_mode) {
    case POSITION_MODE: {
      std::cout << "position mode for walking is under developing" << std::endl;
      break;
    }
    case VELOCITY_MODE: {
      obj[0].ID += cob_id[0];
      obj[1].ID += cob_id[0];

      obj[2].ID += cob_id[1];
      obj[3].ID += cob_id[1];

      int len =
          sizeof(cmd.BASE_VELOCITY_COMMAND) / sizeof(cmd.BASE_VELOCITY_COMMAND);
      obj[0].DataLen = 
      obj[1].DataLen = 
      obj[2].DataLen = 
      obj[3].DataLen = len;

      DataTransform(obj[0].Data, cmd.BASE_VELOCITY_COMMAND, 
                    len, LEFT_MOTOR, state[0]);
      DataTransform(obj[1].Data, cmd.BASE_VELOCITY_COMMAND, 
                    len, RIGHT_MOTOR, state[1]);
      DataTransform(obj[2].Data, cmd.BASE_VELOCITY_COMMAND, 
                    len, LEFT_MOTOR, state[2]);
      DataTransform(obj[3].Data, cmd.BASE_VELOCITY_COMMAND, 
                    len, RIGHT_MOTOR, state[3]);
      break;
    }
    case CURRENT_MODE: {
      std::cout << "current mode for walking is under developing" << std::endl;
      break;
    }
    default: {
      break;  // break or return ???
    }
  }

  switch (steering_mode) {
    case POSITION_MODE: {
      break;
    }
    case VELOCITY_MODE: {
      std::cout << "velocity mode for steering is under developing"
                << std::endl;
      break;
    }
    case CURRENT_MODE: {
      std::cout << "current mode for steering is under developing" << std::endl;
      break;
    }
    default: {
      break;  // break or return ???
    }
  }

  SendCommand(obj, obj_num);

  delete[] obj;
}

void MobileMotor::StopCallback(const std_msgs::Bool& stop) {

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
  SendCommand(obj, id_num);
  delete [] obj;

  if (!VCI_CloseDevice(device_type, device_index)) {
    ROS_WARN("close device failure");
  } else {
    ROS_INFO("close device success");
  }
}
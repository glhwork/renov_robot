#include "sensor_startup/MobileMotor.h"

using mobile::MobileMotor;

MobileMotor::MobileMotor() {
  n_private = ros::NodeHandle("motor_core");
  ParamInit();
  ReadFile(file_address);
  Setup();
  if_initial = DriverInit();
  state_pub_thread = NULL;
}

MobileMotor::~MobileMotor() { delete state_pub_thread; }

void MobileMotor::ParamInit() {
  if (!n_private.getParam("port", port)) {
    port = "/dev/ttyUSB0";
  }
  if (!n_private.getParam("state_topic", state_topic)) {
    state_topic = "motor_state";
  }
  if (!n_private.getParam("file_address", file_address)) {
    file_address =
        "/home/renov_robot/catkin_ws/src/renov_robot/sensor_startup/data/"
        "motor_config.yaml";
  }
  if (!n_private.getParam("delay_time", delay_time)) {
    delay_time = 1000;
  }
  if (!n_private.getParam("wait_time", wait_time)) {
    wait_time = 10;
  }
  if (!n_private.getParam("state_pub_period", state_pub_period)) {
    state_pub_period = 0.1;
  }
}

void MobileMotor::ReadFile(const std::string& address) {
  std::cout << " address is : " << address << std::endl;
  param = YAML::LoadFile(address);

  device_type = param["device_type"].as<int>();
  device_index = param["device_index"].as<int>();
  can_index = param["can_index"].as<int>();
  id_num = param["id_num"].as<int>();

  // steering_mode = param["steering_mode"].as<int>();
  // walking_mode = param["walking_mode"].as<int>();

  encoder_s = (uint)param["encoder_s"].as<int>();
  encoder_w = (uint)param["encoder_w"].as<int>();
  abs_encoder = (uint)param["abs_encoder"].as<int>();
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

  abs_home[0] = param["homing"]["fl"].as<int>();
  abs_home[1] = param["homing"]["fr"].as<int>();
  abs_home[2] = param["homing"]["rl"].as<int>();
  abs_home[3] = param["homing"]["rr"].as<int>();

  error_limit = param["homing_error_limit"].as<int>();
  home_kp = param["home_control"]["p"].as<double>();
  home_ki = param["home_control"]["i"].as<double>();
  home_kd = param["home_control"]["d"].as<double>();

  reduc_ratio_s = param["reduc_ratio_s"].as<double>();
  reduc_ratio_w = param["reduc_ratio_w"].as<double>();

  IdCheck();
}

void MobileMotor::Setup() {
  state_pub = nh.advertise<sensor_msgs::JointState>(state_topic, 100);
}

bool MobileMotor::DriverInit() {
  int flag;
  flag = VCI_OpenDevice(device_type, device_index, 0);
  if (-1 == flag || 0 == flag) {
    ROS_ERROR("open CAN on ttyUSB-%d failure", (int)device_index);
    return false;
  } else if (1 == flag) {
    ROS_INFO("open CAN successfully");
  }

  VCI_INIT_CONFIG config;
  config.AccCode = 0x00000000;
  config.AccMask = 0xFFFFFFFF;
  config.Filter = 0;
  config.Mode = 0;
  config.Timing0 = 0x00;
  config.Timing1 = 0x1c;

  flag = VCI_InitCAN(device_type, device_index, can_index, &config);
  if (-1 == flag || 0 == flag) {
    ROS_ERROR("initialize failure");
    return false;
  } else if (1 == flag) {
    ROS_INFO("initialize successfully");
  }

  flag = VCI_StartCAN(device_type, device_index, can_index);
  if (-1 == flag || 0 == flag) {
    std::cout << "start failure" << std::endl;
    return false;
  } else if (1 == flag) {
    std::cout << "start successfully" << std::endl;
  }

  if (!EnableMotor()) {
    ROS_WARN("enable failure");
    return false;
  } else {
    ROS_INFO("enable motors success!");
  }

  Homing();

  steering_mode = param["steering_mode"].as<int>();
  walking_mode = param["walking_mode"].as<int>();
  if (!SetMode()) {
    ROS_WARN("set mode failure!!");
    return false;
  } else {
    ROS_INFO("mode set success!");
  }

  return true;
}

bool MobileMotor::SetMode() {
  // set mode of walking motors
  switch (walking_mode) { case POSITION_MODE: {
      int len;

      // pre-set the velocity under position mode
      // i.e. the limit velocity of this driver
      PVCI_CAN_OBJ pre_v;
      pre_v = GetVciObject(2);
      pre_v[0].ID = pre_v[0].ID + cob_id[0];
      pre_v[1].ID = pre_v[1].ID + cob_id[1];
      len = sizeof(cmd.SET_PROFILE_VELOCITY) /
            sizeof(cmd.SET_PROFILE_VELOCITY[0]);
      DataTransform(pre_v[0].Data, cmd.SET_PROFILE_VELOCITY, len, 0x60,
                    (float)max_velocity);
      DataTransform(pre_v[1].Data, cmd.SET_PROFILE_VELOCITY, len, 0x60,
                    (float)max_velocity);

      /*****  attention this problem !!!!!!!!!  *****/
      pre_v[0].DataLen = pre_v[1].DataLen = len;
      SendCommand(pre_v, 2);

      delete[] pre_v;

      len = sizeof(cmd.SET_MODE_POSITION) / sizeof(cmd.SET_MODE_POSITION[0]);
      ModeCommand(cob_id[0], cob_id[1], len, POSITION_MODE);
      std::cout << "walking mode == position mode" << std::endl;
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
      int len;

      VCI_CAN_OBJ* pre_v;
      pre_v = GetVciObject(2);
      ;
      pre_v[0].ID = pre_v[0].ID + cob_id[2];
      pre_v[1].ID = pre_v[1].ID + cob_id[3];
      len = sizeof(cmd.SET_PROFILE_VELOCITY) /
            sizeof(cmd.SET_PROFILE_VELOCITY[0]);
      DataTransform(pre_v[0].Data, cmd.SET_PROFILE_VELOCITY, len, 0x60,
                    (float)max_velocity);
      DataTransform(pre_v[1].Data, cmd.SET_PROFILE_VELOCITY, len, 0x60,
                    (float)max_velocity);

      pre_v[0].DataLen = pre_v[1].DataLen = len;

      SendCommand(pre_v, 2);

      PrintTest(pre_v[0].Data, len, "set profile velocity 1 : ");
      PrintTest(pre_v[1].Data, len, "set profile velocity 2 : ");

      delete[] pre_v;

      len = sizeof(cmd.SET_MODE_POSITION) / sizeof(cmd.SET_MODE_POSITION[0]);
      ModeCommand(cob_id[2], cob_id[3], len, POSITION_MODE);
      std::cout << "steering mode == position mode" << std::endl;
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
      int len = sizeof(cmd.SET_MODE_CURRENT) / sizeof(cmd.SET_MODE_CURRENT[0]);
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

void MobileMotor::ModeCommand(const int& id_0, const int& id_1, const int& len,
                              const uint8_t& mode) {
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
      PrintTest(obj[0].Data, len, "set position mode : ");
      break;
    }
    case VELOCITY_MODE: {
      DataInitial(obj[0].Data, cmd.SET_MODE_VELOCITY, len);
      DataInitial(obj[1].Data, cmd.SET_MODE_VELOCITY, len);
      PrintTest(obj[0].Data, len, "set velocity mode : ");
      break;
    }
    case CURRENT_MODE: {
      DataInitial(obj[0].Data, cmd.SET_MODE_CURRENT, len);
      DataInitial(obj[1].Data, cmd.SET_MODE_CURRENT, len);
      PrintTest(obj[0].Data, len, "set current mode : ");
      break;
    }
    default: { break; }
  }

  SendCommand(obj, obj_num);

  delete[] obj;
}

bool MobileMotor::EnableMotor() {
  // number of enable commands
  int ena_cmd_num = 3;
  PVCI_CAN_OBJ obj = GetVciObject(id_num * ena_cmd_num);
  int index = 1;

  for (size_t i = 0; i < id_num; i++) {
    obj[i * 3].ID = obj[i * 3].ID + i + 1;
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

  PrintTest(obj[0].Data, obj[0].DataLen, "enable the motor : ");
  if (!SendCommand(obj, id_num * ena_cmd_num)) {
    delete[] obj;
    return false;
  } else {
    delete[] obj;
    return true;
  }
}

void MobileMotor::DataInitial(BYTE* data, uint8_t* cmd, const uint& len) {
  // std::cout << "initial data : ";
  for (size_t i = 0; i < len; i++) {
    data[i] = cmd[i];
    // std::cout << std::hex << (int)cmd[i] << "  ";
  }
  // std::cout << std::endl;
}

VCI_CAN_OBJ* MobileMotor::GetVciObject(const int& obj_num) {
  PVCI_CAN_OBJ obj_ptr;
  obj_ptr = new VCI_CAN_OBJ[obj_num];
  for (size_t i = 0; i < obj_num; i++) {
    obj_ptr[i].ID = 0x00000600;
    obj_ptr[i].RemoteFlag = 0;
    obj_ptr[i].SendType = 0;
    obj_ptr[i].RemoteFlag = 0;
    obj_ptr[i].ExternFlag = 0;
  }

  return obj_ptr;
}

void MobileMotor::IdCheck() {
  bool flag = true;
  int n = sizeof(cob_id) / sizeof(cob_id[0]);
  for (size_t i = 0; i < n; i++) {
    for (size_t j = i + 1; j < n; j++) {
      if (cob_id[i] == cob_id[i + 1]) {
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

bool MobileMotor::SendCommand(PVCI_CAN_OBJ obj, const uint& len) {
  // VCI_Transmit(device_type, device_index, can_index, obj, len);

  if (0 != delay_time) {
    for (size_t i = 0; i < len; i++) {
      int info_num;
      info_num = VCI_Transmit(device_type, device_index, can_index, &obj[i], 1);
      if (0 == info_num) {
        return false;
      }
      usleep(delay_time);
    }
  } else {
    int info_num;
    info_num = VCI_Transmit(device_type, device_index, can_index, obj, len);
    if (info_num != len) {
      return false;
    }
  }

  return true;
}

void MobileMotor::ControlCallback(const sensor_msgs::JointState& joint_state) {
  if (!if_initial) {
    ROS_WARN("control failure caused by initialization failure");
    return;
  }

  if (joint_state.velocity.size() < 8 || joint_state.position.size() < 8) {
    ROS_WARN("Incorrect quantity of commands");
    return;
  }

  // get data from first four elements of velocity for walking and
  // last four elements of position for steering
  std::vector<float> state_cmds;
  for (size_t i = 0; i < 4; i++) {
    state_cmds.push_back(joint_state.velocity[i]);
  }
  for (size_t i = 0; i < 4; i++) {
    state_cmds.push_back(joint_state.position[i + 4]);
  }
  ControlMotor(state_cmds);
}

void MobileMotor::DataTransform(BYTE* data, uint8_t* cmd, const uint& len,
                                const uint8_t& index, const int& velo) {
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
    return;
  }
}

void MobileMotor::FeedbackCallback() {

  ros::Rate r(1.0 / state_pub_period);
  while (ros::ok()) {
    if (!if_initial) {
      ROS_WARN("feedback failure caused by initialization failure");
      continue;
    }
    FeedbackReq();

    uint data_num;
    data_num = VCI_GetReceiveNum(device_type, device_index, can_index);
    if (0 == data_num) {
      ROS_WARN("no data in buffer of motor feedback!!");
      continue;
    } else if (-1 == data_num) {
      ROS_WARN("get data num of motor feedback failure!");
      continue;
    }

    PVCI_CAN_OBJ rec_obj = new VCI_CAN_OBJ[data_num];
    uint rec_num = VCI_Receive(device_type, device_index, can_index, rec_obj,
                               data_num, wait_time);
    if (-1 == rec_num) {
      delete[] rec_obj;
      ROS_WARN("CAN reading of motor feedback failure!");
      continue;
    }

    sensor_msgs::JointState state;
    state.header.frame_id = "motor";
    state.header.stamp = ros::Time::now();
    state.name.resize(8);
    state.name = {"front_left_walking",  "front_right_walking",
                  "rear_left_walking",   "rear_right_walking",
                  "front_left_steering", "front_right_steering",
                  "rear_left_steering",  "rear_right_steering"};
    state.position.resize(8);
    state.velocity.resize(8);
    state.velocity = {10000.0, 10000.0, 10000.0, 10000.0,
                      10000.0, 10000.0, 10000.0, 10000.0};
    for (size_t i = 0; i < data_num; i++) {
      // if ((REC_BASE_ID + cob_id[0]) == rec_obj[i].ID) {

      // }
      // if ((REC_BASE_ID + cob_id[1]) == rec_obj[i].ID) {

      // }
      // if ((REC_BASE_ID + cob_id[2]) == rec_obj[i].ID) {

      // }
      // if ((REC_BASE_ID + cob_id[3]) == rec_obj[i].ID) {

      // }
      if (LEFT_MOTOR == rec_obj[i].Data[2]) {
        if (POSITION_FD == rec_obj[i].Data[1]) {
          int index = 2 * (rec_obj[i].ID - REC_BASE_ID - 1);
          state.position[index] = FourByteHex2Int(&rec_obj[i].Data[4]);
          state.name[index] = state.name[index] + "_1";
        }
        if (VELOCITY_FD == rec_obj[i].Data[1]) {
          int index = 2 * (rec_obj[i].ID - REC_BASE_ID - 1);
          state.velocity[index] = FourByteHex2Int(&rec_obj[i].Data[4]);
        }
      }
      if (RIGHT_MOTOR == rec_obj[i].Data[2]) {
        if (POSITION_FD == rec_obj[i].Data[1]) {
          int index = 2 * (rec_obj[i].ID - REC_BASE_ID) - 1;
          state.position[index] = FourByteHex2Int(&rec_obj[i].Data[4]);
          state.name[index] = state.name[index] + "_1";
        }
        if (VELOCITY_FD == rec_obj[i].Data[1]) {
          int index = 2 * (rec_obj[i].ID - REC_BASE_ID) - 1;
          state.velocity[index] = FourByteHex2Int(&rec_obj[i].Data[4]);
        }
      }
    }

    bool if_pub = true;
    for (size_t i = 0; i < state.velocity.size(); i++) {
      if (10000.0 == state.velocity[i]) {
        delete[] rec_obj;
        if_pub = false;
        ROS_WARN("no enough velocity feedback!!");
        break;
      }
      if (state.name[i].find("_1") > state.name[i].size()) {
        delete[] rec_obj;
        if_pub = false;
        ROS_WARN("no enough position feedback!!");
        break;
      }
    }

    if (if_pub) {
      for (size_t i = 0; i < 8; i++) {
        state.position[i] = (state.position[i] - home[i]) / encoder_s * (2 * PI);
      }
      state_pub.publish(state);
      delete[] rec_obj;
    }
    r.sleep();
  }
}

void MobileMotor::FeedbackReq() {
  PVCI_CAN_OBJ obj = GetVciObject(id_num * 4);
  for (size_t i = 0; i < id_num; i++) {
    int len;

    len = sizeof(cmd.BASE_POSITION_FEEDBACK) /
          sizeof(cmd.BASE_POSITION_FEEDBACK[0]);
    obj[i * 4 + 0].ID += cob_id[i];
    obj[i * 4 + 0].DataLen = len;
    DataInitial(obj[i * 4 + 0].Data, cmd.BASE_POSITION_FEEDBACK, len);
    obj[i * 4 + 0].Data[2] = LEFT_MOTOR;

    obj[i * 4 + 1].ID += cob_id[i];
    obj[i * 4 + 1].DataLen = len;
    DataInitial(obj[i * 4 + 1].Data, cmd.BASE_POSITION_FEEDBACK, len);
    obj[i * 4 + 1].Data[2] = RIGHT_MOTOR;

    len = sizeof(cmd.BASE_VELOCITY_FEEDBACK) /
          sizeof(cmd.BASE_VELOCITY_FEEDBACK[0]);
    obj[i * 4 + 2].ID += cob_id[i];
    obj[i * 4 + 2].DataLen = len;
    DataInitial(obj[i * 4 + 2].Data, cmd.BASE_VELOCITY_FEEDBACK, len);
    obj[i * 4 + 2].Data[2] = LEFT_MOTOR;

    obj[i * 4 + 3].ID += cob_id[i];
    obj[i * 4 + 3].DataLen = len;
    DataInitial(obj[i * 4 + 3].Data, cmd.BASE_VELOCITY_FEEDBACK, len);
    obj[i * 4 + 3].Data[2] = RIGHT_MOTOR;
  }

  SendCommand(obj, id_num * 4);

  delete[] obj;
}

void MobileMotor::ControlMotor(const std::vector<float>& raw_state) {
  std::vector<int> state = CommandTransform(raw_state);

  // 0->left, 1->right for walking driver 1 front
  // 2->left, 3->right for walking driver 2 rear
  // 4->left, 5->right for steering driver 1 front
  // 6->left. 7->right for steering driver 2 rear
  int obj_num = 8;
  PVCI_CAN_OBJ obj = GetVciObject(obj_num);

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

      int len = sizeof(cmd.BASE_VELOCITY_COMMAND) /
                sizeof(cmd.BASE_VELOCITY_COMMAND[0]);
      obj[0].DataLen = obj[1].DataLen = obj[2].DataLen = obj[3].DataLen = len;

      DataTransform(obj[0].Data, cmd.BASE_VELOCITY_COMMAND, len, LEFT_MOTOR,
                    state[0]);
      DataTransform(obj[1].Data, cmd.BASE_VELOCITY_COMMAND, len, RIGHT_MOTOR,
                    state[1]);
      DataTransform(obj[2].Data, cmd.BASE_VELOCITY_COMMAND, len, LEFT_MOTOR,
                    state[2]);
      DataTransform(obj[3].Data, cmd.BASE_VELOCITY_COMMAND, len, RIGHT_MOTOR,
                    state[3]);
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
      obj[4].ID += cob_id[2];
      obj[5].ID += cob_id[2];
      obj[6].ID += cob_id[3];
      obj[7].ID += cob_id[3];

      int len = sizeof(cmd.BASE_POSITION_COMMAND) /
                sizeof(cmd.BASE_POSITION_COMMAND[0]);
      obj[4].DataLen =
      obj[5].DataLen = 
      obj[6].DataLen = 
      obj[7].DataLen = len;

      DataTransform(obj[4].Data, cmd.BASE_POSITION_COMMAND, len, LEFT_MOTOR,
                    state[4]);
      DataTransform(obj[5].Data, cmd.BASE_POSITION_COMMAND, len, RIGHT_MOTOR,
                    state[5]);
      DataTransform(obj[6].Data, cmd.BASE_POSITION_COMMAND, len, LEFT_MOTOR,
                    state[6]);
      DataTransform(obj[7].Data, cmd.BASE_POSITION_COMMAND, len, RIGHT_MOTOR,
                    state[7]);
      break;
    }
    case VELOCITY_MODE: {
      obj[4].ID += cob_id[2];
      obj[5].ID += cob_id[2];
      obj[6].ID += cob_id[3];
      obj[7].ID += cob_id[3];

      int len = sizeof(cmd.BASE_VELOCITY_COMMAND) /
                sizeof(cmd.BASE_VELOCITY_COMMAND[0]);
      obj[4].DataLen = obj[5].DataLen = obj[6].DataLen = obj[7].DataLen = len;

      DataTransform(obj[4].Data, cmd.BASE_VELOCITY_COMMAND, len, LEFT_MOTOR,
                    state[4]);
      DataTransform(obj[5].Data, cmd.BASE_VELOCITY_COMMAND, len, RIGHT_MOTOR,
                    state[5]);
      DataTransform(obj[6].Data, cmd.BASE_VELOCITY_COMMAND, len, LEFT_MOTOR,
                    state[6]);
      DataTransform(obj[7].Data, cmd.BASE_VELOCITY_COMMAND, len, RIGHT_MOTOR,
                    state[7]);

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

std::vector<int> MobileMotor::CommandTransform(
    const std::vector<float>& raw_state) {
  std::vector<int> state;

  // determine the walking command
  for (size_t i = 0; i < 4; i++) {
    int tmp = raw_state[i] * reduc_ratio_w;
    tmp = tmp * pow(-1, motor_sign[i] + 1);
    state.push_back(tmp);
  }

  // determine the steering command
  for (size_t i = 4; i < 8; i++) {
    double data = raw_state[i] * reduc_ratio_s * encoder_s / (2 * PI);
    int tmp = home[i - 4] + (int)data * pow(-1, motor_sign[i] + 1);
    state.push_back(tmp);
  }
  return state;
}

void MobileMotor::StopMotor() {
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
  delete[] obj;

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
  delete[] obj;

  if (!VCI_CloseDevice(device_type, device_index)) {
    ROS_WARN("close device failure");
  } else {
    ROS_INFO("close device success");
  }
}

void MobileMotor::StopCallback(const std_msgs::Bool& stop) { StopMotor(); }

void MobileMotor::PrintTest(BYTE* data, const int& len,
                            const std::string& str) {
  std::cout << str << "  ";
  for (size_t i = 0; i < len; i++) {
    std::cout << std::hex << "0x" << (int)data[i] << " ";
  }
  std::cout << std::endl;
}

/* apply encoders to help process homing */
int MobileMotor::FourByteHex2Int(uint8_t* data) {
  int result;
  result = (data[3] << 24) + (data[2] << 16) + (data[1] << 8) + (data[0] << 0);
  if (0x80000000 && result) {
    result = -1 * (~(result - 1));
  }
  return result;
}

bool MobileMotor::ReadEncoder(int* encod_data) {
  PVCI_CAN_OBJ cmd_obj = GetVciObject(4);
  int len = sizeof(cmd.REQUEST_ENCODER_1) / 
            sizeof(cmd.REQUEST_ENCODER_1[0]);
  for (size_t i = 0; i < 4; i++) {
    cmd_obj[i].ID = i + 1;
    cmd_obj[i].DataLen = len;
  }
  DataInitial(cmd_obj[0].Data, cmd.REQUEST_ENCODER_1, len);
  DataInitial(cmd_obj[1].Data, cmd.REQUEST_ENCODER_2, len);
  DataInitial(cmd_obj[2].Data, cmd.REQUEST_ENCODER_3, len);
  DataInitial(cmd_obj[3].Data, cmd.REQUEST_ENCODER_4, len);

  SendCommand(cmd_obj, 4);
  delete[] cmd_obj;

  int data_num = VCI_GetReceiveNum(device_type, device_index, can_index);
  if (-1 == data_num) {
    ROS_WARN("Get quantity of CAN data of absolute encoder failure");
    return false;
  }

  PVCI_CAN_OBJ rec_obj = new VCI_CAN_OBJ[data_num];
  // std::vector<int> encod_data;
  int encod_count;  // this is for checking whether enough data is obtained

  int rec_num =
      VCI_Receive(device_type, device_index, can_index, rec_obj, data_num, 0);
  if (-1 == rec_num) {
    delete[] rec_obj;
    ROS_WARN("Get data of absolute encoder from CAN failure");
    return false;
  } else if (0 != rec_num) {
    bool flag_vec[4] = {false, false, false, false};
    for (size_t i = 0; i < rec_num; i++) {
      switch (rec_obj[i].Data[1]) {
        case 0x00000001: {
          encod_data[0] = FourByteHex2Int(&rec_obj[i].Data[3]);
          flag_vec[0] = true;
          break;
        }
        case 0x00000002: {
          encod_data[1] = FourByteHex2Int(&rec_obj[i].Data[3]);
          flag_vec[1] = true;
          break;
        }
        case 0x00000003: {
          encod_data[2] = FourByteHex2Int(&rec_obj[i].Data[3]);
          flag_vec[2] = true;
          break;
        }
        case 0x00000004: {
          encod_data[3] = FourByteHex2Int(&rec_obj[i].Data[3]);
          flag_vec[3] = true;
          break;
        }
        default: { break; }
      }
    }

    bool flag = (flag_vec[0] && flag_vec[1] && flag_vec[2] && flag_vec[3]);
    if (flag) {
      ROS_INFO("ENCODER DATA is : 1: %d, 2: %d, 3: %d, 4: %d", encod_data[0],
               encod_data[1], encod_data[2], encod_data[3]);
    }

    delete[] rec_obj;
    return flag;
  }
}

void MobileMotor::Homing() {
  int mode_len = sizeof(cmd.SET_MODE_VELOCITY) / 
                 sizeof(cmd.SET_MODE_VELOCITY[0]);
  ModeCommand(cob_id[2], cob_id[3], mode_len, VELOCITY_MODE);

  int* encod_data = new int;
  int error_k1[4];
  int error_k2[4];
  while (true) {
    // loop used to continuously read
    // until data of four encoders are got while (true) {
    while (true) {
      if (ReadEncoder(encod_data)) {
        break;
      }
    }
    double error[4];
    float steer_v[4];

    for (size_t i = 0; i < 4; i++) {
      error[i] = encod_data[i] - abs_home[i];
      if (abs(error[i]) <= error_limit) {
        steer_v[i] = 0.0;
      } else {
        double K[3];
        K[0] = home_kp + home_ki + home_kd;
        K[1] = home_kp + 2 * home_kd;
        K[3] = home_kd;
        steer_v[i] = K[0] * error[i] - K[1] * error_k1[i] + K[2] * error_k2[i];
      }
    }
    for (size_t i = 0; i < 4; i++) {
      double k = 0.02;
      steer_v[i] = steer_v[i] / (double)abs_encoder / k / encoder_s;
      error_k2[i] = error_k1[i];
      error_k1[i] = error[i];
    }

    walking_mode = VELOCITY_MODE;
    steering_mode = VELOCITY_MODE;

    std::vector<float> raw_state;
    raw_state.resize(8);
    raw_state = {0, 0, 0, 0, steer_v[0], steer_v[1], steer_v[2], steer_v[3]};
    ControlMotor(raw_state);

    if (steer_v[0] == 0.0 && steer_v[1] == 0.0 && steer_v[2] == 0.0 &&
        steer_v[3] == 0.0) {
      ROS_INFO("homing finish !!");
      int len;
      break;
    }
  }
  delete encod_data;

  // read position feedback of steering motors
  while (true) {
    FeedbackReq();
    uint data_num;
    data_num = VCI_GetReceiveNum(device_type, device_index, can_index);
    if (-1 == data_num) {
      ROS_WARN("Get data num of homing position failure !!");
      continue;
    }

    if (0 == data_num) {
      ROS_WARN("no data in buffer!!");
      continue;
    } else {
      PVCI_CAN_OBJ rec_obj = new VCI_CAN_OBJ[data_num];
      uint rec_num = VCI_Receive(device_type, device_index, can_index, rec_obj,
                                 data_num, wait_time);
      if (-1 == rec_num) {
        ROS_WARN("CAN reading of homing position failure");
        continue;
      }

      bool flag[4] = {false, false, false, false};
      for (size_t i = 0; i < rec_num; i++) {
        if (0x00000700 + cob_id[0] == rec_obj[i].ID ||
            0x00000700 + cob_id[1] == rec_obj[i].ID ||
            0x00000700 + cob_id[2] == rec_obj[i].ID ||
            0x00000700 + cob_id[3] == rec_obj[i].ID) {
          continue;
        }
        if (REC_BASE_ID + cob_id[2] == rec_obj[i].ID) {
          // front left motor position
          if (0x64 == rec_obj[i].Data[1] && LEFT_MOTOR == rec_obj[i].Data[2]) {
            home[0] = FourByteHex2Int(&rec_obj[i].Data[4]);
            flag[0] = true;
          }
          // front right motor position
          if (0x64 == rec_obj[i].Data[1] && RIGHT_MOTOR == rec_obj[i].Data[2]) {
            home[1] = FourByteHex2Int(&rec_obj[i].Data[4]);
            flag[1] = true;
          }
        }  // end of [if] obtaining data of front steering motors
        if (REC_BASE_ID + cob_id[3] == rec_obj[i].ID) {
          // rear left motor position
          if (0x64 == rec_obj[i].Data[1] && LEFT_MOTOR == rec_obj[i].Data[2]) {
            home[2] = FourByteHex2Int(&rec_obj[i].Data[4]);
            flag[2] = true;
          }
          // rear right motor position
          if (0x64 == rec_obj[i].Data[1] && RIGHT_MOTOR == rec_obj[i].Data[2]) {
            home[3] = FourByteHex2Int(&rec_obj[i].Data[4]);
            flag[3] = true;
          }
        }  // end of [if] obtaining data of rear steering motors

      }  // end of [for] loop obtaining data of steering motors
      if (flag[0] && flag[1] && flag[2] && flag[3]) {
        PVCI_CAN_OBJ posi_obj = GetVciObject(4);
        posi_obj[0].ID += cob_id[2];
        posi_obj[1].ID += cob_id[2];
        posi_obj[2].ID += cob_id[3];
        posi_obj[3].ID += cob_id[3];

        // restore the control mode of motors
        steering_mode = POSITION_MODE;
        walking_mode = VELOCITY_MODE;
        int len_tmp = sizeof(cmd.SET_MODE_POSITION) / 
                      sizeof(cmd.SET_MODE_POSITION[0]);
        ModeCommand(cob_id[2], cob_id[3], len_tmp, POSITION_MODE);

        int posi_cmd_len = sizeof(cmd.BASE_POSITION_COMMAND) /
                           sizeof(cmd.BASE_POSITION_COMMAND[0]);
        posi_obj[0].DataLen =
        posi_obj[1].DataLen =
        posi_obj[2].DataLen =
        posi_obj[3].DataLen = posi_cmd_len;

        DataTransform(posi_obj[0].Data, cmd.BASE_POSITION_COMMAND, posi_cmd_len,
                      LEFT_MOTOR, home[0]);
        DataTransform(posi_obj[1].Data, cmd.BASE_POSITION_COMMAND, posi_cmd_len,
                      RIGHT_MOTOR, home[1]);
        DataTransform(posi_obj[2].Data, cmd.BASE_POSITION_COMMAND, posi_cmd_len,
                      LEFT_MOTOR, home[2]);
        DataTransform(posi_obj[3].Data, cmd.BASE_POSITION_COMMAND, posi_cmd_len,
                      RIGHT_MOTOR, home[3]);
        SendCommand(posi_obj, 4);

        delete[] posi_obj;
        delete[] rec_obj;
        ROS_INFO("Get home position successfully");
        sleep(2);
        break;
      }
      delete[] rec_obj;
    }
  }  // end of [while] loop reading position of motors
}

void MobileMotor::Loop() {
  control_sub =
      nh.subscribe("cmd_base_joint", 10, &MobileMotor::ControlCallback, this);
  teleop_sub =
      nh.subscribe("cmd_vel", 10, &MobileMotor::TeleopCallback, this);
  stop_sub =
      nh.subscribe("stop", 10, &MobileMotor::StopCallback, this);
  // ros::Timer feedback_timer =
  //     nh.createTimer(ros::Duration(0.1), &MobileMotor::FeedbackCallback,
  //     this);
  state_pub_thread =
      new boost::thread(boost::bind(&MobileMotor::FeedbackCallback, this));
}

void MobileMotor::SubLoop() {

  // while (ros::ok()) 
  // ros::Subscriber control_sub =
  //     nh.subscribe("cmd_base_joint", 10, &MobileMotor::ControlCallback, this);
  // ros::Subscriber teleop_sub =
  //     nh.subscribe("cmd_vel", 10, &MobileMotor::TeleopCallback, this);
  // ros::Subscriber stop_sub =
  //     nh.subscribe("stop", 10, &MobileMotor::StopCallback, this);
}

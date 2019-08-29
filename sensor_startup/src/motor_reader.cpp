#include "sensor_startup/motor_reader.h"
#include <fstream>

using mobile_base::MotorReader;

MotorReader::MotorReader() {
  n_private = ros::NodeHandle("~");
  ParamInit();
  ReadFile(file_address);
  Setup();
  // if_initial = DriverInit();
  if_home_finish = false;
  if_get_ekf_odom = false;
  state_pub_thread = NULL;

  double frt = param["front_rear_track"].as<double>();
  double lrt = param["left_right_track"].as<double>();

  preset_steer_angle = fabs(atan(frt / lrt));
  base_rotate_radius = 0.5 * sqrt(pow(frt, 2) + pow(lrt, 2));
  /*
    control_sub =
        nh.subscribe("cmd_base_joint", 10, &MotorReader::ControlCallback, this);
    home_sub = nh.subscribe("mobile_platform_driver_position_feedback", 10,
                            &MotorReader::GetHomeCallback, this);
    teleop_sub = nh.subscribe("cmd_vel", 10, &MotorReader::TeleopCallback,
    this); stop_sub = nh.subscribe("stop", 10, &MotorReader::StopCallback,
    this); odom_sub = nh.subscribe("odom", 10, &MotorReader::OdomCallback,
    this);

    state_pub_thread =
        new boost::thread(boost::bind(&MotorReader::FeedbackCallback, this));
   */
}

MotorReader::~MotorReader() {
  if (state_pub_thread) {
    delete state_pub_thread;
  }
}

void MotorReader::ParamInit() {
  if (!n_private.getParam("port", port)) {
    port = "/dev/ttyUSB0";
  }
  if (!n_private.getParam("state_topic", state_topic)) {
    state_topic = "motor_state";
  }
  if (!n_private.getParam("raw_odom_topic", raw_odom_topic)) {
    raw_odom_topic = "raw_odom";
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

void MotorReader::ReadFile(const std::string& address) {
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
  freq_multiplier = param["frequency_multiplier"].as<int>();
  variance_limit = param["variance_limit"].as<double>();

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

  homing_error_limit = param["homing_error_limit"].as<int>();
  home_kp = param["home_control"]["p"].as<double>();
  home_ki = param["home_control"]["i"].as<double>();
  home_kd = param["home_control"]["d"].as<double>();

  reduc_ratio_s = param["reduc_ratio_s"].as<double>();
  reduc_ratio_w = param["reduc_ratio_w"].as<double>();

  wheel_radius = param["wheel_radius"].as<double>();

  IdCheck();
}

void MotorReader::Setup() {
  state_pub = nh.advertise<sensor_msgs::JointState>(state_topic, 100);
  raw_odom_pub = nh.advertise<nav_msgs::Odometry>(raw_odom_topic, 100);
}

bool MotorReader::DriverInit() {
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
    // std::cout << "start failure" << std::endl;
    ROS_WARN("start failure");
    return false;
  } else if (1 == flag) {
    // std::cout << "start successfully" << std::endl;
    ROS_INFO("start successfully");
  }

  if (!EnableMotor()) {
    ROS_WARN("enable failure");
    return false;
  } else {
    ROS_INFO("enable motors success!");
  }

  // Homing();

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

bool MotorReader::SetMode() {
  // set mode of walking motors
  switch (walking_mode) {
    case POSITION_MODE: {
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
      // std::cout << "walking mode == position mode" << std::endl;
      ROS_INFO("walking mode == position mode");
      break;
    }
    case VELOCITY_MODE: {
      int len =
          sizeof(cmd.SET_MODE_VELOCITY) / sizeof(cmd.SET_MODE_VELOCITY[0]);
      ModeCommand(cob_id[0], cob_id[1], len, VELOCITY_MODE);
      // std::cout << "walking mode == velocity mode" << std::endl;
      ROS_INFO("walking mode == velocity mode");
      break;
    }
    case CURRENT_MODE: {
      int len = sizeof(cmd.SET_MODE_CURRENT) / sizeof(cmd.SET_MODE_CURRENT[0]);
      ModeCommand(cob_id[0], cob_id[1], len, CURRENT_MODE);
      std::cout << "walking mode == current mode" << std::endl;
      ROS_INFO("walking mode == current mode");
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

      // PrintTest(pre_v[0].Data, len, "set profile velocity 1 : ");
      // PrintTest(pre_v[1].Data, len, "set profile velocity 2 : ");

      delete[] pre_v;

      len = sizeof(cmd.SET_MODE_POSITION) / sizeof(cmd.SET_MODE_POSITION[0]);
      ModeCommand(cob_id[2], cob_id[3], len, POSITION_MODE);
      // std::cout << "steering mode == position mode" << std::endl;
      ROS_INFO("steering mode == position mode");
      break;
    }
    case VELOCITY_MODE: {
      int len =
          sizeof(cmd.SET_MODE_VELOCITY) / sizeof(cmd.SET_MODE_VELOCITY[0]);
      ModeCommand(cob_id[2], cob_id[3], len, VELOCITY_MODE);
      // std::cout << "steering mode == velocity mode" << std::endl;
      ROS_INFO("steering mode == velocity mode");
      break;
    }
    case CURRENT_MODE: {
      int len = sizeof(cmd.SET_MODE_CURRENT) / sizeof(cmd.SET_MODE_CURRENT[0]);
      ModeCommand(cob_id[2], cob_id[3], len, CURRENT_MODE);
      // std::cout << "steering mode == current mode" << std::endl;
      ROS_INFO("steering mode == current mode");
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

void MotorReader::ModeCommand(const int& id_0, const int& id_1, const int& len,
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
      // PrintTest(obj[0].Data, len, "set position mode : ");
      break;
    }
    case VELOCITY_MODE: {
      DataInitial(obj[0].Data, cmd.SET_MODE_VELOCITY, len);
      DataInitial(obj[1].Data, cmd.SET_MODE_VELOCITY, len);
      // PrintTest(obj[0].Data, len, "set velocity mode : ");
      break;
    }
    case CURRENT_MODE: {
      DataInitial(obj[0].Data, cmd.SET_MODE_CURRENT, len);
      DataInitial(obj[1].Data, cmd.SET_MODE_CURRENT, len);
      // PrintTest(obj[0].Data, len, "set current mode : ");
      break;
    }
    default: { break; }
  }

  SendCommand(obj, obj_num);

  delete[] obj;
}

bool MotorReader::EnableMotor() {
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

  // PrintTest(obj[0].Data, obj[0].DataLen, "enable the motor : ");
  if (!SendCommand(obj, id_num * ena_cmd_num)) {
    delete[] obj;
    return false;
  } else {
    delete[] obj;
    return true;
  }
}

void MotorReader::DataInitial(BYTE* data, uint8_t* cmd, const uint& len) {
  // std::cout << "initial data : ";
  for (size_t i = 0; i < len; i++) {
    data[i] = cmd[i];
    // std::cout << std::hex << (int)cmd[i] << "  ";
  }
  // std::cout << std::endl;
}

VCI_CAN_OBJ* MotorReader::GetVciObject(const int& obj_num) {
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

void MotorReader::IdCheck() {
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

bool MotorReader::SendCommand(PVCI_CAN_OBJ obj, const uint& len) {
  // VCI_Transmit(device_type, device_index, can_index, obj, len);

  if (0 != delay_time) {
    for (size_t i = 0; i < len; i++) {
      int info_num;
      info_num = VCI_Transmit(device_type, device_index, can_index, &obj[i], 1);
      if (0 == info_num) {
        return false;
        ROS_WARN("Lose one frame of data");
      }
      usleep(delay_time);
    }
  } else {
    for (size_t i = 0; i < len; i++) {
      int info_num;
      info_num = VCI_Transmit(device_type, device_index, can_index, &obj[i], 1);
      if (0 == info_num) {
        ROS_WARN("Lose one frame of data");
        return false;
      }
    }
  }

  return true;
}

void MotorReader::ControlCallback(const sensor_msgs::JointState& joint_state) {
  if (!if_initial) {
    // ROS_WARN("control failure caused by initialization failure");
    return;
  }
  /*
  if (!if_get_ekf_odom) {
    return;
  }
  */

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
  // std::cout << "the control input : " << state_cmds[0] << "  " <<
  // state_cmds[1] << "  " << state_cmds[2] << "  " << state_cmds[3] << "  "
  //	                              << state_cmds[4] << "  " << state_cmds[5]
  //<< "  " << state_cmds[6] << "  " << state_cmds[7] << std::endl;
  ControlMotor(state_cmds);
}

void MotorReader::DataTransform(BYTE* data, uint8_t* cmd, const uint& len,
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

void MotorReader::TeleopCallback(const geometry_msgs::Twist& twist) {
  if (!if_initial) {
    // ROS_WARN("teleop failure caused by initialization failure");
    return;
  }
}

void MotorReader::FeedbackCallback() {
  ros::Rate r(1.0 / state_pub_period);
  while (ros::ok()) {
    if (!if_initial) {
      // ROS_WARN("feedback failure caused by initialization failure");
      continue;
    }

    if (!if_get_ekf_odom) {
      continue;
    } else {
      std::cout << "get ekf is true" << std::endl;
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

    VelPosiFeedbackReq(true, false);
    uint data_num;
    uint rec_num;

    data_num = VCI_GetReceiveNum(device_type, device_index, can_index);
    if (0 == data_num) {
      ROS_WARN("no data in buffer of motor feedback!!");
      continue;
    } else if (-1 == data_num) {
      ROS_WARN("get data num of motor feedback failure!");
      continue;
    }

    PVCI_CAN_OBJ rec_obj = new VCI_CAN_OBJ[2500];
    rec_num = VCI_Receive(device_type, device_index, can_index, rec_obj, 2500,
                          wait_time);
    cur_time = ros::Time::now();
    if (-1 == rec_num) {
      delete[] rec_obj;
      ROS_WARN("CAN reading of motor feedback failure!");
      continue;
    }
    GetFeedback(&state, rec_obj);
    delete[] rec_obj;

    usleep(10000);

    VelPosiFeedbackReq(false, true);
    rec_obj = new VCI_CAN_OBJ[2500];
    data_num = VCI_GetReceiveNum(device_type, device_index, can_index);
    if (0 == data_num) {
      ROS_WARN("no data in buffer of motor feedback!!");
      continue;
    } else if (-1 == data_num) {
      ROS_WARN("get data num of motor feedback failure!");
      continue;
    }
    rec_num = VCI_Receive(device_type, device_index, can_index, rec_obj, 2500,
                          wait_time);
    if (-1 == rec_num) {
      delete[] rec_obj;
      ROS_WARN("CAN reading of motor feedback failure!");
      continue;
    }
    GetFeedback(&state, rec_obj);

    std::cout << "the velocity is : ";
    for (size_t j = 0; j < state.velocity.size(); j++) {
      std::cout << std::dec << std::fixed << state.velocity[j] << "  ";
    }
    std::cout << std::endl;

    std::cout << "the position is : ";
    for (size_t j = 0; j < state.position.size(); j++) {
      std::cout << std::dec << std::fixed << state.position[j] << "  ";
    }
    std::cout << std::endl;

    bool if_pub = true;
    for (size_t i = 0; i < state.velocity.size(); i++) {
      if (10000.0 == state.velocity[i]) {
        if_pub = false;
        ROS_WARN("no enough velocity feedback!!");
        break;
      }
      if (state.name[i].find("_1") > state.name[i].size()) {
        if_pub = false;
        ROS_WARN("no enough position feedback!!");
        break;
      }
    }

    if (if_pub) {
      std::cout << "if_pub is true" << std::endl;
      ROS_ERROR("if pub is true");
    } else {
      std::cout << "if_pub is false" << std::endl;
    }

    if (if_pub) {
      for (size_t i = 0; i < 8; i++) {
        state.position[i] = (state.position[i] - home[i]) /
                            (freq_multiplier * encoder_s) * (2 * M_PI);
        state.velocity[i] = state.velocity[i] / reduc_ratio_w;
      }

      PublishOdometry(state);
      state_pub.publish(state);
      std::cout << "success and pub odom" << std::endl;
      delete[] rec_obj;
      std::cout << "success and delete obj" << std::endl;
    } else {
      delete[] rec_obj;
    }
    r.sleep();
  }
}

void MotorReader::GetFeedback(sensor_msgs::JointState* state,
                              const PVCI_CAN_OBJ rec_obj) {
  for (size_t i = 0; i < 2500; i++) {
    if (0x00000700 + cob_id[0] == rec_obj[i].ID ||
        0x00000700 + cob_id[1] == rec_obj[i].ID ||
        0x00000700 + cob_id[2] == rec_obj[i].ID ||
        0x00000700 + cob_id[3] == rec_obj[i].ID) {
      continue;
    }
    if (LEFT_MOTOR == rec_obj[i].Data[2]) {
      if (POSITION_FD == rec_obj[i].Data[1]) {
        int index = 2 * (rec_obj[i].ID - REC_BASE_ID - 1);
        state->position[index] = FourByteHex2Int(&rec_obj[i].Data[4]);
        state->name[index] = state->name[index] + "_1";
      }
      if (VELOCITY_FD == rec_obj[i].Data[1]) {
        int index = 2 * (rec_obj[i].ID - REC_BASE_ID - 1);
        state->velocity[index] = FourByteHex2Int(&rec_obj[i].Data[4]);
      }
    }
    if (RIGHT_MOTOR == rec_obj[i].Data[2]) {
      if (POSITION_FD == rec_obj[i].Data[1]) {
        int index = 2 * (rec_obj[i].ID - REC_BASE_ID) - 1;
        state->position[index] = FourByteHex2Int(&rec_obj[i].Data[4]);
        state->name[index] = state->name[index] + "_1";
      }
      if (VELOCITY_FD == rec_obj[i].Data[1]) {
        int index = 2 * (rec_obj[i].ID - REC_BASE_ID) - 1;
        state->velocity[index] = FourByteHex2Int(&rec_obj[i].Data[4]);
      }
    }
  }
}

void MotorReader::FeedbackReq() {
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

void MotorReader::VelPosiFeedbackReq(const bool& use_velocity_req,
                                     const bool& use_position_req) {
  if (use_velocity_req) {
    PVCI_CAN_OBJ vel_obj = GetVciObject(8);
    int vel_req_len = sizeof(cmd.BASE_VELOCITY_FEEDBACK) /
                      sizeof(cmd.BASE_VELOCITY_FEEDBACK[0]);
    for (size_t i = 0; i < 4; i++) {
      vel_obj[i * 2].ID += cob_id[i];
      vel_obj[i * 2].DataLen = vel_req_len;
      DataInitial(vel_obj[i * 2].Data, cmd.BASE_VELOCITY_FEEDBACK, vel_req_len);
      vel_obj[i * 2].Data[2] = LEFT_MOTOR;

      vel_obj[i * 2 + 1].ID += cob_id[i];
      vel_obj[i * 2 + 1].DataLen = vel_req_len;
      DataInitial(vel_obj[i * 2 + 1].Data, cmd.BASE_VELOCITY_FEEDBACK,
                  vel_req_len);
      vel_obj[i * 2 + 1].Data[2] = RIGHT_MOTOR;
    }

    SendCommand(vel_obj, 8);
    delete[] vel_obj;
  }

  if (use_position_req) {
    PVCI_CAN_OBJ position_obj = GetVciObject(8);
    int position_req_len = sizeof(cmd.BASE_POSITION_FEEDBACK) /
                           sizeof(cmd.BASE_POSITION_FEEDBACK[0]);
    for (size_t i = 0; i < 4; i++) {
      position_obj[i * 2].ID += cob_id[i];
      position_obj[i * 2].DataLen = position_req_len;
      DataInitial(position_obj[i * 2].Data, cmd.BASE_POSITION_FEEDBACK,
                  position_req_len);
      position_obj[i * 2].Data[2] = LEFT_MOTOR;

      position_obj[i * 2 + 1].ID += cob_id[i];
      position_obj[i * 2 + 1].DataLen = position_req_len;
      DataInitial(position_obj[i * 2 + 1].Data, cmd.BASE_POSITION_FEEDBACK,
                  position_req_len);
      position_obj[i * 2 + 1].Data[2] = RIGHT_MOTOR;
    }

    SendCommand(position_obj, 8);
    delete[] position_obj;
  }
}

void MotorReader::FrontRearFeedbackReq(const bool& if_read_front,
                                       const bool& if_read_rear) {
  if (if_read_front) {
    PVCI_CAN_OBJ front_obj = GetVciObject(8);
    for (size_t i = 0; i < 2; i++) {
      int front_position_len;

      front_position_len = sizeof(cmd.BASE_POSITION_FEEDBACK) /
                           sizeof(cmd.BASE_POSITION_FEEDBACK[0]);
      front_obj[i * 4 + 0].ID += cob_id[i * 2];
      front_obj[i * 4 + 0].DataLen = front_position_len;
      DataInitial(front_obj[i * 4 + 0].Data, cmd.BASE_POSITION_FEEDBACK,
                  front_position_len);
      front_obj[i * 4 + 0].Data[2] = LEFT_MOTOR;

      front_obj[i * 4 + 1].ID += cob_id[i * 2];
      front_obj[i * 4 + 1].DataLen = front_position_len;
      DataInitial(front_obj[i * 4 + 1].Data, cmd.BASE_POSITION_FEEDBACK,
                  front_position_len);
      front_obj[i * 4 + 1].Data[2] = RIGHT_MOTOR;

      int front_velocity_len = sizeof(cmd.BASE_VELOCITY_FEEDBACK) /
                               sizeof(cmd.BASE_VELOCITY_FEEDBACK[0]);
      front_obj[i * 4 + 2].ID += cob_id[i * 2];
      front_obj[i * 4 + 2].DataLen = front_velocity_len;
      DataInitial(front_obj[i * 4 + 2].Data, cmd.BASE_VELOCITY_FEEDBACK,
                  front_velocity_len);
      front_obj[i * 4 + 2].Data[2] = LEFT_MOTOR;

      front_obj[i * 4 + 3].ID += cob_id[i * 2];
      front_obj[i * 4 + 3].DataLen = front_velocity_len;
      DataInitial(front_obj[i * 4 + 3].Data, cmd.BASE_VELOCITY_FEEDBACK,
                  front_velocity_len);
      front_obj[i * 4 + 3].Data[2] = RIGHT_MOTOR;
    }

    SendCommand(front_obj, 8);
    delete[] front_obj;
  }

  if (if_read_rear) {
    PVCI_CAN_OBJ rear_obj = GetVciObject(8);
    for (size_t i = 0; i < 2; i++) {
      int rear_position_len;

      rear_position_len = sizeof(cmd.BASE_POSITION_FEEDBACK) /
                          sizeof(cmd.BASE_POSITION_FEEDBACK[0]);
      rear_obj[i * 4 + 0].ID += cob_id[i * 2 + 1];
      rear_obj[i * 4 + 0].DataLen = rear_position_len;
      DataInitial(rear_obj[i * 4 + 0].Data, cmd.BASE_POSITION_FEEDBACK,
                  rear_position_len);
      rear_obj[i * 4 + 0].Data[2] = LEFT_MOTOR;

      rear_obj[i * 4 + 1].ID += cob_id[i * 2 + 1];
      rear_obj[i * 4 + 1].DataLen = rear_position_len;
      DataInitial(rear_obj[i * 4 + 1].Data, cmd.BASE_POSITION_FEEDBACK,
                  rear_position_len);
      rear_obj[i * 4 + 1].Data[2] = RIGHT_MOTOR;

      int rear_velocity_len = sizeof(cmd.BASE_VELOCITY_FEEDBACK) /
                              sizeof(cmd.BASE_VELOCITY_FEEDBACK[0]);
      rear_obj[i * 4 + 2].ID += cob_id[i * 2 + 1];
      rear_obj[i * 4 + 2].DataLen = rear_velocity_len;
      DataInitial(rear_obj[i * 4 + 2].Data, cmd.BASE_VELOCITY_FEEDBACK,
                  rear_velocity_len);
      rear_obj[i * 4 + 2].Data[2] = LEFT_MOTOR;

      rear_obj[i * 4 + 3].ID += cob_id[i * 2 + 1];
      rear_obj[i * 4 + 3].DataLen = rear_velocity_len;
      DataInitial(rear_obj[i * 4 + 3].Data, cmd.BASE_VELOCITY_FEEDBACK,
                  rear_velocity_len);
      rear_obj[i * 4 + 3].Data[2] = RIGHT_MOTOR;
    }

    SendCommand(rear_obj, 8);
    delete[] rear_obj;
  }
}

void MotorReader::ControlMotor(const std::vector<float>& raw_state) {
  std::vector<int> state = CommandTransform(raw_state);

  // 0->left, 1->right for walking driver 1 front
  // 2->left, 3->right for walking driver 2 rear
  // 4->left, 5->right for steering driver 1 front
  // 6->left. 7->right for steering driver 2 rear
  int obj_num = 8;
  PVCI_CAN_OBJ obj = GetVciObject(obj_num);
  // std::cout << "the motor cmd     : " << state[0] << "  " << state[1] << "  "
  // << state[2] << "  " << state[3] << "  "
  //	                              << state[4] << "  " << state[5] << "  " <<
  // state[6] << "  " << state[7] << std::endl;
  switch (walking_mode) {
    case POSITION_MODE: {
      // std::cout << "position mode for walking is under developing" <<
      // std::endl;
      ROS_WARN("position mode for walking is under developing");
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
      // std::cout << "current mode for walking is under developing" <<
      // std::endl;
      ROS_WARN("current mode for walking is under developing");
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
      obj[4].DataLen = obj[5].DataLen = obj[6].DataLen = obj[7].DataLen = len;

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
      // std::cout << "current mode for steering is under developing" <<
      // std::endl;
      ROS_WARN("current mode for steering is under developing");
      break;
    }
    default: {
      break;  // break or return ???
    }
  }

  SendCommand(obj, obj_num);

  delete[] obj;
}

std::vector<int> MotorReader::CommandTransform(
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
    double data =
        raw_state[i] * reduc_ratio_s * encoder_s * freq_multiplier / (2 * M_PI);
    int tmp = home[i - 4] + (int)data * pow(-1, motor_sign[i] + 1);
    state.push_back(tmp);
  }
  return state;
}

void MotorReader::StopMotor() {
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
  // PrintTest(obj[0].Data, len, "disenable process : ");
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
  // PrintTest(obj[0].Data, len, "save parameter : ");
  SendCommand(obj, id_num);
  delete[] obj;

  if (!VCI_CloseDevice(device_type, device_index)) {
    ROS_WARN("close device failure");
  } else {
    ROS_INFO("close device success");
  }
}

void MotorReader::StopCallback(const std_msgs::Bool& stop) { StopMotor(); }

void MotorReader::PrintTest(BYTE* data, const int& len,
                            const std::string& str) {
  for (size_t i = 0; i < len; i++) {
    std::cout << std::hex << "0x" << (int)data[i] << " ";
  }
  std::cout << std::endl;
}

/* apply encoders to help process homing */
int MotorReader::FourByteHex2Int(uint8_t* data) {
  int result;
  result = (data[3] << 24) + (data[2] << 16) + (data[1] << 8) + (data[0] << 0);
  if (0x80000000 && result) {
    result = -1 * (~(result - 1));
  }
  return result;
}

bool MotorReader::ReadEncoder(int* encod_data) {
  PVCI_CAN_OBJ cmd_obj = GetVciObject(4);
  int len = sizeof(cmd.REQUEST_ENCODER_1) / sizeof(cmd.REQUEST_ENCODER_1[0]);
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

void MotorReader::Homing() {
  int len = sizeof(cmd.SET_MODE_VELOCITY) / sizeof(cmd.SET_MODE_VELOCITY[0]);
  ModeCommand(cob_id[2], cob_id[3], len, VELOCITY_MODE);

  int* encod_data = new int;
  double error_k1[4];
  double error_k2[4];

  int home_count = 0;
  while (true) {
    // loop used to continuously read
    // until data of four encoders are got while (true) {
    if (!ReadEncoder(encod_data)) {
      continue;
    }
    double error[4];
    float steer_v[4];

    for (size_t i = 0; i < 4; i++) {
      error[i] = (encod_data[i] - abs_home[i]) * 1.0;
      if (abs(error[i]) <= homing_error_limit) {
        steer_v[i] = 0.0;
      } else {
        double K[3];
        K[0] = home_kp + home_ki + home_kd;
        K[1] = home_kp + 2 * home_kd;
        K[3] = home_kd;
        steer_v[i] = home_kp * (error[i] - error_k1[i]) + home_ki * error[i] +
                     home_kd * (error[i] - 2 * error_k1[i] + error_k2[i]);
      }
    }

    for (size_t i = 0; i < 4; i++) {
      double k = 0.02;
      steer_v[i] = steer_v[i] / (double)abs_encoder / k / (double)encoder_s;
      error_k2[i] = error_k1[i];
      error_k1[i] = error[i];
    }
    ROS_INFO("abs home is : %d, %d, %d, %d", abs_home[0], abs_home[1],
             abs_home[2], abs_home[3]);
    ROS_INFO("steer_v is : %f, %f, %f, %f", steer_v[0], steer_v[1], steer_v[2],
             steer_v[3]);
    ROS_INFO("error is : %f, %f, %f, %f", error[0], error[1], error[2],
             error[3]);

    walking_mode = VELOCITY_MODE;
    steering_mode = VELOCITY_MODE;

    std::vector<float> raw_state;
    raw_state.resize(8);
    raw_state = {0, 0, 0, 0, steer_v[0], steer_v[1], steer_v[2], steer_v[3]};

    ControlMotor(raw_state);

    // std::cout << "======================" << std::endl;
    if (steer_v[0] == 0.0 && steer_v[1] == 0.0 && steer_v[2] == 0.0 &&
        steer_v[3] == 0.0) {
      ROS_INFO("homing finish !!");
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
        int len_tmp =
            sizeof(cmd.SET_MODE_POSITION) / sizeof(cmd.SET_MODE_POSITION[0]);
        ModeCommand(cob_id[2], cob_id[3], len_tmp, POSITION_MODE);

        int posi_cmd_len = sizeof(cmd.BASE_POSITION_COMMAND) /
                           sizeof(cmd.BASE_POSITION_COMMAND[0]);
        posi_obj[0].DataLen = posi_obj[1].DataLen = posi_obj[2].DataLen =
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

double MotorReader::GetVariance(const std::vector<double>& data_vec) {
  double mean = ComputeMean(data_vec);
  double diff_sqr_sum = 0.0;
  for (size_t i = 0; i < data_vec.size(); i++) {
    diff_sqr_sum += pow((data_vec[i] - mean), 2);
  }
  double variance = diff_sqr_sum / data_vec.size();
  return variance;
}

double MotorReader::ComputeMean(const std::vector<double>& data_vec) {
  double sum = 0.0;
  for (size_t i = 0; i < data_vec.size(); i++) {
    sum += data_vec[i];
  }
  double mean = sum / data_vec.size();
  return mean;
}

void MotorReader::PublishOdometry(const sensor_msgs::JointState& joint_state) {
  // get the latest odometry information
  nav_msgs::Odometry raw_odom;
  geometry_msgs::PoseWithCovarianceStamped raw_pose;

  raw_odom.header = raw_pose.header;
  raw_odom.pose = filtered_pose.pose;

  ros::Time pre_time = raw_odom.header.stamp;
  raw_odom.header.stamp = cur_time;
  tf::Quaternion quat_tmp(
      raw_odom.pose.pose.orientation.x, raw_odom.pose.pose.orientation.y,
      raw_odom.pose.pose.orientation.z, raw_odom.pose.pose.orientation.w);
  tf::Matrix3x3 rpy_matrix(quat_tmp);
  double roll, pitch, yaw;
  rpy_matrix.getRPY(roll, pitch, yaw);

  // determine the time between two updates
  double dt = cur_time.toSec() - pre_time.toSec();

  std::vector<double> motor_position_vec = {
      joint_state.position[4], joint_state.position[5], joint_state.position[6],
      joint_state.position[7]};
  std::vector<double> motor_velocity_vec = {
      joint_state.velocity[0], joint_state.velocity[1], joint_state.velocity[2],
      joint_state.velocity[3]};
  // std::vector<double> abs_motor_position_vec = {
  //     fabs(joint_state.position[4]), fabs(joint_state.position[5]),
  //     fabs(joint_state.position[6]), fabs(joint_state.position[7])};
  std::vector<double> abs_motor_velocity_vec = {
      fabs(joint_state.velocity[0]), fabs(joint_state.velocity[1]),
      fabs(joint_state.velocity[2]), fabs(joint_state.velocity[3])};

  bool if_rotate = true;
  for (size_t i = 0; i < motor_position_vec.size(); i++) {
    if_rotate = if_rotate &&
                (fabs(fabs(preset_steer_angle) - fabs(motor_position_vec[i])) <
                 variance_limit);
  }

  double motor_velo_mean;
  double steer_variance = fabs(GetVariance(motor_position_vec));
  if (sqrt(steer_variance) < variance_limit) {
    double steer_angle_mean = ComputeMean(motor_position_vec);
    motor_velo_mean = ComputeMean(motor_velocity_vec);
    // set position value
    raw_odom.pose.pose.position.x =
        raw_odom.pose.pose.position.x +
        motor_velo_mean * cos(steer_angle_mean + yaw) * dt;
    raw_odom.pose.pose.position.y =
        raw_odom.pose.pose.position.y +
        motor_velo_mean * sin(steer_angle_mean + yaw) * dt;
    raw_odom.pose.pose.position.z = 0.0;

    // set velocity value
    raw_odom.twist.twist.linear.x =
        motor_velo_mean * cos(steer_angle_mean + yaw);
    raw_odom.twist.twist.linear.y =
        motor_velo_mean * sin(steer_angle_mean + yaw);
    raw_odom.twist.twist.linear.z = 0;
    raw_odom.twist.twist.angular.x = raw_odom.twist.twist.angular.y =
        raw_odom.twist.twist.angular.z = 0.0;

    raw_odom_pub.publish(raw_odom);
  } else if (if_rotate) {
    motor_velo_mean = ComputeMean(abs_motor_velocity_vec);
    double angular_v = motor_velo_mean * wheel_radius / base_rotate_radius;
    if (motor_velocity_vec[0] > 0) {
      angular_v = -fabs(angular_v);
    } else {
      angular_v = fabs(angular_v);
    }

    double yaw = yaw + angular_v * dt;
    if (fabs(yaw) > M_PI) {
      if (yaw > 0) {
        yaw = -(2 * M_PI - fabs(yaw));
      } else if (yaw < 0) {
        yaw = 2 * M_PI - fabs(yaw);
      }
    }
    tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
    raw_odom.pose.pose.position.z = 0.0;

    // difference between q.x() and q.getX()
    raw_odom.pose.pose.orientation.x = q.x();
    raw_odom.pose.pose.orientation.y = q.y();
    raw_odom.pose.pose.orientation.z = q.z();
    raw_odom.pose.pose.orientation.w = q.w();

    raw_odom.twist.twist.linear.x = raw_odom.twist.twist.linear.y =
        raw_odom.twist.twist.linear.z = 0.0;
    raw_odom.twist.twist.angular.x = raw_odom.twist.twist.angular.y = 0.0;
    raw_odom.twist.twist.angular.z = angular_v;

    raw_odom_pub.publish(raw_odom);
  } else {
    raw_odom_pub.publish(raw_odom);
  }

  if_get_ekf_odom = false;
}

void MotorReader::OdomCallback(
    const geometry_msgs::PoseWithCovarianceStamped& odom_msg) {
  if (if_home_finish && !if_get_ekf_odom) {
    filtered_pose = odom_msg;
    if_get_ekf_odom = true;
  }
}

void MotorReader::GetHomeCallback(const std_msgs::Int64MultiArray& home_state) {
  // std::cout << "if home finish is : " << if_home_finish << std::endl;
  if (!if_home_finish) {
    // std::cout << "the home position i got is :";
    for (size_t i = 0; i < home_state.data.size(); i++) {
      home[i] = home_state.data[i];
      // std::cout << home[i] << "  ";
    }
    // std::cout << std::endl;

    cur_time = ros::Time::now();
    nav_msgs::Odometry init_odom;
    init_odom.header.frame_id = "odom";
    init_odom.header.stamp = cur_time;
    init_odom.child_frame_id = "base_footprint";

    init_odom.pose.pose.position.x = init_odom.pose.pose.position.y =
        init_odom.pose.pose.position.z = 0.0;

    init_odom.pose.pose.orientation.x = init_odom.pose.pose.orientation.y =
        init_odom.pose.pose.orientation.z = 0.0;
    init_odom.pose.pose.orientation.w = 1.0;

    for (size_t i = 0; i < init_odom.pose.covariance.size(); i++) {
      if (i % 7 == 0) {
        init_odom.pose.covariance[i] = 1e-4;
      }
    }
    for (size_t i = 0; i < 6; i++) {
      ROS_WARN("the cov is : %.7f", init_odom.pose.covariance[i * 6 + i]);
    }

    init_odom.twist.twist.linear.x = init_odom.twist.twist.linear.y =
        init_odom.twist.twist.linear.z = 0.0;

    init_odom.twist.twist.angular.x = init_odom.twist.twist.angular.y =
        init_odom.twist.twist.angular.z = 0.0;

    for (size_t i = 0; i < init_odom.twist.covariance.size(); i++) {
      if (i % 7 == 0) {
        init_odom.twist.covariance[i] = 1e-4;
      }
    }

    ROS_INFO(
        "wait for 2 seconds and publish initial odometry info to "
        "robot_poes_ekf node");
    // std::cout << "after ros info" << std::endl;
    sleep(2);

    raw_odom_pub.publish(init_odom);

    // tell the homing node to stop publish
    ros::Publisher receive_signal_pub =
        nh.advertise<std_msgs::Bool>("/close_homing_topic", 10);
    std_msgs::Bool receive_signal;
    receive_signal.data = false;

    int count_tmp = 0;
    while (true) {
      receive_signal_pub.publish(receive_signal);
      ros::Duration(0.1).sleep();
      count_tmp++;
      if (count_tmp > 20) {
        break;
      }
    }

    if_home_finish = true;

    if_initial = DriverInit();

    if (!if_initial) {
      ROS_WARN("driver init failure");
    }

    // send the current position setted to motor drivers
    PVCI_CAN_OBJ posi_obj = GetVciObject(4);
    posi_obj[0].ID += cob_id[2];
    posi_obj[1].ID += cob_id[2];
    posi_obj[2].ID += cob_id[3];
    posi_obj[3].ID += cob_id[3];

    // restore the control mode of motors
    // steering_mode = POSITION_MODE;
    // walking_mode = VELOCITY_MODE;
    int len_tmp =
        sizeof(cmd.SET_MODE_POSITION) / sizeof(cmd.SET_MODE_POSITION[0]);
    ModeCommand(cob_id[2], cob_id[3], len_tmp, POSITION_MODE);

    int posi_cmd_len = sizeof(cmd.BASE_POSITION_COMMAND) /
                       sizeof(cmd.BASE_POSITION_COMMAND[0]);
    posi_obj[0].DataLen = posi_obj[1].DataLen = posi_obj[2].DataLen =
        posi_obj[3].DataLen = posi_cmd_len;

    ROS_INFO("home position i got is : %d, %d, %d, %d", home[0], home[1],
             home[2], home[3]);
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
    ROS_INFO("Get home position successfully");
    sleep(2);
  }
}

void MotorReader::Loop() {
  // control_sub =
  //     nh.subscribe("cmd_base_joint", 10, &MotorReader::ControlCallback,
  //     this);
  // home_sub = nh.subscribe("mobile_platform_driver_position_feedback", 10,
  //                         &MotorReader::GetHomeCallback, this);
  // teleop_sub = nh.subscribe("cmd_vel", 10, &MotorReader::TeleopCallback,
  // this); stop_sub = nh.subscribe("stop", 10, &MotorReader::StopCallback,
  // this); odom_sub = nh.subscribe("odom", 10, &MotorReader::OdomCallback,
  // this);

  // ros::Timer feedback_timer =
  //     nh.createTimer(ros::Duration(0.1), &MotorReader::FeedbackCallback,
  //     this);
  state_pub_thread =
      new boost::thread(boost::bind(&MotorReader::FeedbackCallback, this));
}

#include "sensor_startup/kinco/driver_controller.h"
#include <unistd.h>

namespace mobile_base {

DriverController::DriverController() : CanApplication() {
  if_steer_home_ = false;
  if_debug_ = false;
}

DriverController::~DriverController() { delete[] cob_id_; }

void DriverController::ReadDriverFile(
    const std::string& relative_file_address) {
  std::string final_file_address = base_file_address_ + relative_file_address;
  if (if_debug_) {
    std::cout << "relative is : " << relative_file_address << std::endl;
    std::cout << "Address of yaml file is : " << final_file_address
              << std::endl;
  }
  YAML::Node driver_config = YAML::LoadFile(final_file_address);
  std::cout << "read yaml driver file" << std::endl;

  walking_mode_ = driver_config["walking_mode"].as<int>();
  std::cout << "get mode " << std::endl;
  steering_mode_ = driver_config["steering_mode"].as<int>();

  encoder_s_ = driver_config["encoder_s"].as<int>();
  encoder_w_ = driver_config["encoder_w"].as<int>();
  frequency_multiplier_ = driver_config["frequency_multiplier"].as<int>();
  reduc_ratio_s_ = driver_config["reduc_ratio_s"].as<double>();
  reduc_ratio_w_ = driver_config["reduc_ratio_w"].as<double>();
  max_velocity_ = driver_config["max_velocity"].as<int>();

  walk_id_num_ = driver_config["walk_id_num"].as<int>();
  steer_id_num_ = driver_config["steer_id_num"].as<int>();
  id_num_ = walk_id_num_ + steer_id_num_;

  home_position_ = new int[steer_id_num_];
  std::cout << "before id " << std::endl;

  cob_id_ = new uint[id_num_];
  for (size_t i = 0; i < id_num_; i++) {
    cob_id_[i] = (uint)driver_config["cob_id"][i].as<int>();
  }

  motor_sign_ = new int[id_num_];
  for (size_t i = 0; i < id_num_; i++) {
    motor_sign_[i] = (uint)driver_config["motor_sign"][i].as<int>();
  }
}

bool DriverController::DriverInit() {
  if (if_debug_) {
    while (true) {
      init_data_file_.open(base_file_address_ +
                           "/debug_data/driver_init_data.txt");
      if (init_data_file_.is_open()) {
        break;
      } else {
        std::cout << "open driver_init_data file failure" << std::endl;
      }
    }
  }

  StartPDO();

  if (!DriverStart()) {
    std::cout << "Error!! The drivers starts with failure" << std::endl;
  }

  GetHomePosition();

  if (if_debug_) {
    init_data_file_.close();
  }

  return true;
}

void DriverController::StartPDO() {
  /*
    PVCI_CAN_OBJ pdo_start_obj = GetVciObject(id_num_, NMT_MODULE_CONTROL_ID);

    uint cmd_len =
        sizeof(can_cmd_.PDO_START_WORK) / sizeof(can_cmd_.PDO_START_WORK[0]);
    for (size_t i = 0; i < id_num_; i++) {
      DataInitial(pdo_start_obj[i].Data, can_cmd_.PDO_START_WORK, cmd_len);
      pdo_start_obj[i].Data[1] = cob_id_[i];
      // one of the byte in data should be equal to station number
      pdo_start_obj[i].DataLen = cmd_len;
    }
    if (if_debug_) {
      init_data_file_ << "*****************" << std::endl;
      for (size_t i = 0; i < id_num_; i++) {
        init_data_file_ << "start pdo commands -> ";
        init_data_file_ << "data len : " << (int)pdo_start_obj[i].DataLen << "
    "; init_data_file_ << std::hex << "id is 0x" << pdo_start_obj[i].ID << " :
    "; init_data_file_ << std::hex << "0x" << (int)pdo_start_obj[i].Data[0] << "
    0x"
                        << (int)pdo_start_obj[i].Data[1] << std::endl;
      }
      init_data_file_ << "*****************" << std::endl;
    }

    SendCommand(pdo_start_obj, id_num_);
    delete[] pdo_start_obj;
    */

  PVCI_CAN_OBJ pdo_start_obj = GetVciObject(1, NMT_MODULE_CONTROL_ID);
  pdo_start_obj->DataLen = 2;
  pdo_start_obj->Data[0] = 0x01;
  pdo_start_obj->Data[1] = 0x00;
  if (if_debug_) {
    init_data_file_ << "*****************" << std::endl;
    for (size_t i = 0; i < 1; i++) {
      init_data_file_ << "start pdo commands -> ";
      init_data_file_ << "data len : " << (int)pdo_start_obj[i].DataLen << " ";
      init_data_file_ << std::hex << "id is 0x" << pdo_start_obj[i].ID << " : ";
      init_data_file_ << std::hex << "0x" << (int)pdo_start_obj[i].Data[0]
                      << "  0x" << (int)pdo_start_obj[i].Data[1] << std::endl;
    }
    init_data_file_ << "*****************" << std::endl;
  }
  SendCommand(pdo_start_obj, 1);
  delete pdo_start_obj;
}

bool DriverController::DriverEnable() {
  PVCI_CAN_OBJ enable_obj = GetVciObject(id_num_, RPDO1_ID);

  uint enable_cmd_len =
      sizeof(can_cmd_.ENABLE_MOTOR) / sizeof(can_cmd_.ENABLE_MOTOR[0]);
  for (size_t i = 0; i < id_num_; i++) {
    enable_obj[i].ID += cob_id_[i];
    DataInitial(enable_obj[i].Data, can_cmd_.ENABLE_MOTOR, enable_cmd_len);
    enable_obj[i].DataLen = enable_cmd_len;
  }

  SendCommand(enable_obj, id_num_);
  delete[] enable_obj;
}

bool DriverController::DriverStart() {
  PVCI_CAN_OBJ walk_cmd_obj = GetVciObject(walk_id_num_, RPDO1_ID);
  switch (walking_mode_) {
    case VELOCITY_MODE: {
      uint walk_mode_len = sizeof(can_cmd_.SET_VELOCITY_MODE) /
                           sizeof(can_cmd_.SET_VELOCITY_MODE[0]);
      for (size_t i = 0; i < walk_id_num_; i++) {
        walk_cmd_obj[i].ID += cob_id_[i];
        DataInitial(walk_cmd_obj[i].Data, can_cmd_.SET_VELOCITY_MODE,
                    walk_mode_len);
        walk_cmd_obj[i].Data[1] = ENABLE_CMD;
        walk_cmd_obj[i].Data[2] = 0x00;
        walk_cmd_obj[i].DataLen = 2 + walk_mode_len;
      }
      break;
    }
    case POSITION_MODE: {
      uint walk_mode_len = sizeof(can_cmd_.SET_POSITION_MODE) /
                           sizeof(can_cmd_.SET_POSITION_MODE[0]);
      for (size_t i = 0; i < walk_id_num_; i++) {
        walk_cmd_obj[i].ID += cob_id_[i];
        DataInitial(walk_cmd_obj[i].Data, can_cmd_.SET_POSITION_MODE,
                    walk_mode_len);
        walk_cmd_obj[i].Data[1] = 0x3f;
        walk_cmd_obj[i].Data[2] = 0x10;
        walk_cmd_obj[i].DataLen = 2 + walk_mode_len;
      }
      break;
    }
    case CURRENT_MODE: {
      uint walk_mode_len = sizeof(can_cmd_.SET_CURRENT_MODE) /
                           sizeof(can_cmd_.SET_CURRENT_MODE[0]);
      for (size_t i = 0; i < walk_id_num_; i++) {
        walk_cmd_obj[i].ID += cob_id_[i];
        DataInitial(walk_cmd_obj[i].Data, can_cmd_.SET_CURRENT_MODE,
                    walk_mode_len);
        walk_cmd_obj[i].Data[1] = ENABLE_CMD;
        walk_cmd_obj[i].Data[2] = 0x00;
        walk_cmd_obj[i].DataLen = 2 + walk_mode_len;
      }
      break;
    }
    default: {
      std::cout << "Incorrect mode number of walking" << std::endl;
      return false;
    }
  }
  SendCommand(walk_cmd_obj, walk_id_num_);

  // set mode of steering motors
  PVCI_CAN_OBJ steer_cmd_obj = GetVciObject(steer_id_num_, RPDO1_ID);
  switch (steering_mode_) {
    case VELOCITY_MODE: {
      uint steer_mode_len = sizeof(can_cmd_.SET_VELOCITY_MODE) /
                            sizeof(can_cmd_.SET_VELOCITY_MODE[0]);
      for (size_t i = 0; i < steer_id_num_; i++) {
        steer_cmd_obj[i].ID += cob_id_[i + walk_id_num_];
        DataInitial(steer_cmd_obj[i].Data, can_cmd_.SET_VELOCITY_MODE,
                    steer_mode_len);
        steer_cmd_obj[i].Data[1] = ENABLE_CMD;
        steer_cmd_obj[i].Data[2] = 0x00;
        steer_cmd_obj[i].DataLen = 2 + steer_mode_len;
      }
      break;
    }
    case POSITION_MODE: {
      uint steer_mode_len = sizeof(can_cmd_.SET_POSITION_MODE) /
                            sizeof(can_cmd_.SET_POSITION_MODE[0]);
      for (size_t i = 0; i < steer_id_num_; i++) {
        steer_cmd_obj[i].ID += cob_id_[i + walk_id_num_];
        DataInitial(steer_cmd_obj[i].Data, can_cmd_.SET_POSITION_MODE,
                    steer_mode_len);
        steer_cmd_obj[i].Data[1] = 0x3f;
        steer_cmd_obj[i].Data[2] = 0x10;
        steer_cmd_obj[i].DataLen = 2 + steer_mode_len;
      }
      break;
    }
    case CURRENT_MODE: {
      uint steer_mode_len = sizeof(can_cmd_.SET_CURRENT_MODE) /
                            sizeof(can_cmd_.SET_CURRENT_MODE[0]);
      for (size_t i = 0; i < steer_id_num_; i++) {
        steer_cmd_obj[i].ID += cob_id_[i];
        DataInitial(steer_cmd_obj[i].Data, can_cmd_.SET_CURRENT_MODE,
                    steer_mode_len);
        steer_cmd_obj[i].Data[1] = ENABLE_CMD;
        steer_cmd_obj[i].Data[2] = 0x00;
        steer_cmd_obj[i].DataLen = 2 + steer_mode_len;
      }
      break;
    }
    default: {
      std::cout << "Incorrect mode number of steering" << std::endl;
      return false;
    }
  }
  SendCommand(steer_cmd_obj, steer_id_num_);

  if (if_debug_) {
    for (size_t i = 0; i < walk_id_num_; i++) {
      init_data_file_ << "enable and set mode commands -> ";
      init_data_file_ << "data len : " << (int)walk_cmd_obj[i].DataLen << " ";
      init_data_file_ << std::hex << "id is 0x" << walk_cmd_obj[i].ID << " : ";
      init_data_file_ << std::hex << "0x" << (int)walk_cmd_obj[i].Data[0]
                      << "  0x" << (int)walk_cmd_obj[i].Data[1] << "  0x"
                      << (int)walk_cmd_obj[i].Data[2] << std::endl;
    }
    std::cout << std::endl;
    for (size_t i = 0; i < steer_id_num_; i++) {
      init_data_file_ << "enable and set mode commands -> ";
      init_data_file_ << "data len : " << (int)steer_cmd_obj[i].DataLen << " ";
      init_data_file_ << std::hex << "id is 0x" << steer_cmd_obj[i].ID << " : ";
      init_data_file_ << std::hex << "0x" << (int)steer_cmd_obj[i].Data[0]
                      << "  0x" << (int)steer_cmd_obj[i].Data[1] << "  0x"
                      << (int)steer_cmd_obj[i].Data[2] << std::endl;
    }
  }

  delete[] walk_cmd_obj;
  delete[] steer_cmd_obj;

  return true;
}

void DriverController::ControlMotor(
    const std::vector<int>& raw_control_signal) {
  if (raw_control_signal.size() != id_num_) {
    std::cout << "Quantity of control signal is incorrect" << std::endl;
    return;
  }

  std::vector<int> control_signal = ControlSignalTransform(raw_control_signal);

  if (if_debug_) {
    signal_data_file_.open(base_file_address_ + "/debug_data/signal_data.txt");

    signal_data_file_ << "raw_control_signal : [";
    for (size_t i = 0; i < raw_control_signal.size(); i++) {
      signal_data_file_ << std::dec << raw_control_signal[i] << "  ";
    }
    signal_data_file_ << "]" << std::endl;

    signal_data_file_ << "control_signal : [";
    for (size_t i = 0; i < control_signal.size(); i++) {
      signal_data_file_ << std::dec << control_signal[i] << "  ";
    }
    signal_data_file_ << "]" << std::endl;

    signal_data_file_ << "****************" << std::endl;
    signal_data_file_.close();
  }

  // send control signal to walking motors

  if (if_debug_) {
    cmd_data_file_.open(base_file_address_ + "/debug_data/cmd_data.txt");
  }
  switch (walking_mode_) {
    case VELOCITY_MODE: {
      int* target_velocity = new int[walk_id_num_];
      for (size_t i = 0; i < walk_id_num_; i++) {
        target_velocity[i] = raw_control_signal[i];
      }

      SendVelocity(cob_id_, target_velocity, walk_id_num_);
      delete[] target_velocity;
      break;
    }
    case POSITION_MODE: {
      int* target_position = new int[walk_id_num_];
      for (size_t i = 0; i < walk_id_num_; i++) {
        target_position[i] = raw_control_signal[i];
      }

      SendPosition(cob_id_, target_position, walk_id_num_);
      delete[] target_position;
      break;
    }
    case CURRENT_MODE: {
      int* target_current = new int[walk_id_num_];
      for (size_t i = 0; i < walk_id_num_; i++) {
        target_current[i] = raw_control_signal[i];
      }

      SendCurrent(cob_id_, target_current, walk_id_num_);
      delete[] target_current;
      break;
    }
  }  // end of sending signals to walking motors

  // send control signal to steering motors
  switch (steering_mode_) {
    case VELOCITY_MODE: {
      int* target_velocity = new int[steer_id_num_];
      for (size_t i = walk_id_num_; i < id_num_; i++) {
        target_velocity[i - walk_id_num_] = raw_control_signal[i];
      }

      SendVelocity(cob_id_, target_velocity, steer_id_num_);
      delete[] target_velocity;
      break;
    }
    case POSITION_MODE: {
      int* target_position = new int[steer_id_num_];
      for (size_t i = walk_id_num_; i < id_num_; i++) {
        target_position[i - walk_id_num_] = raw_control_signal[i];
      }

      SendPosition(cob_id_, target_position, steer_id_num_);
      delete[] target_position;
      break;
    }
    case CURRENT_MODE: {
      int* target_current = new int[steer_id_num_];
      for (size_t i = walk_id_num_; i < id_num_; i++) {
        target_current[i - walk_id_num_] = raw_control_signal[i];
      }

      SendCurrent(cob_id_, target_current, steer_id_num_);
      delete[] target_current;
      break;
    }
  }  // end of sending signals to steering motors

  if (if_debug_) {
    cmd_data_file_.close();
  }
}

std::vector<int> DriverController::ControlSignalTransform(
    const std::vector<int>& raw_signal) {
  std::vector<int> signal;
  // determine the walking command
  for (size_t i = 0; i < 4; i++) {
    int tmp = raw_signal[i] * reduc_ratio_w_;
    tmp = tmp * pow(-1, motor_sign_[i] + 1);
    signal.push_back(tmp);
  }

  // determine the steering command
  for (size_t i = 4; i < 8; i++) {
    double data = raw_signal[i] * reduc_ratio_s_ * encoder_s_ *
                  frequency_multiplier_ / (2 * M_PI);
    int tmp = home_position_[i - 4] + (int)data * pow(-1, motor_sign_[i] + 1);
    signal.push_back(tmp);
  }
}

void DriverController::SendVelocity(uint* id, int* target_velocity,
                                    const int& len) {
  PVCI_CAN_OBJ velocity_cmd_obj = GetVciObject(len, RPDO3_ID);

  uint velocity_cmd_len = 8;
  for (size_t i = 0; i < len; i++) {
    velocity_cmd_obj[i].ID += id[i];
    Dec2HexVector(&velocity_cmd_obj[i].Data[0], target_velocity[i], 4);
    Dec2HexVector(&velocity_cmd_obj[i].Data[4], 0, 4);
    velocity_cmd_obj[i].DataLen = velocity_cmd_len;
  }

  if (if_debug_) {
    for (size_t i = 0; i < len; i++) {
      cmd_data_file_ << "velocity -> ";
      cmd_data_file_ << std::hex << "id = 0x" << velocity_cmd_obj[i].ID << ", ";
      cmd_data_file_ << std::dec
                     << "datalen = " << (int)velocity_cmd_obj[i].DataLen
                     << " : ";
      for (size_t j = 0; j < 8; j++) {
        cmd_data_file_ << std::hex << "0x" << (int)velocity_cmd_obj[i].Data[j]
                       << "  ";
      }
      cmd_data_file_ << std::endl;
    }
    cmd_data_file_ << "**************" << std::endl;
  }

  SendCommand(velocity_cmd_obj, len);
  delete[] velocity_cmd_obj;
}

void DriverController::SendPosition(uint* id, int* target_position,
                                    const int& len) {
  PVCI_CAN_OBJ position_cmd_obj = GetVciObject(len, RPDO3_ID);

  uint position_cmd_len = 8;
  for (size_t i = 0; i < len; i++) {
    position_cmd_obj[i].ID += id[i];
    DataInitial(position_cmd_obj[i].Data, can_cmd_.POSITION_COMMAND,
                position_cmd_len);
    Dec2HexVector(&position_cmd_obj[i].Data[0], 0, 4);
    Dec2HexVector(&position_cmd_obj[i].Data[4], target_position[i], 4);
    position_cmd_obj[i].DataLen = position_cmd_len;
  }

  if (if_debug_) {
    for (size_t i = 0; i < len; i++) {
      cmd_data_file_ << "position -> ";
      cmd_data_file_ << std::hex << "id = 0x" << position_cmd_obj[i].ID << ", ";
      cmd_data_file_ << std::dec
                     << "datalen = " << (int)position_cmd_obj[i].DataLen
                     << " : ";
      for (size_t j = 0; j < 8; j++) {
        cmd_data_file_ << std::hex << "0x" << (int)position_cmd_obj[i].Data[j]
                       << "  ";
      }
      cmd_data_file_ << std::endl;
    }
    cmd_data_file_ << "**************" << std::endl;
  }

  SendCommand(position_cmd_obj, len);
  delete[] position_cmd_obj;
}

void DriverController::SendCurrent(uint* id, int* target_current,
                                   const int& len) {
  PVCI_CAN_OBJ current_cmd_obj = GetVciObject(len, RPDO1_ID);

  uint current_cmd_len =
      sizeof(can_cmd_.CURRENT_COMMAND) / sizeof(can_cmd_.CURRENT_COMMAND[0]);
  for (size_t i = 0; i < len; i++) {
    current_cmd_obj[i].ID += cob_id_[i];
    DataInitial(current_cmd_obj[i].Data, can_cmd_.CURRENT_COMMAND,
                current_cmd_len);
    Dec2HexVector(&current_cmd_obj[i].Data[4], target_current[i], 2);
    current_cmd_obj[i].DataLen = current_cmd_len;
  }

  SendCommand(current_cmd_obj, len);
  delete[] current_cmd_obj;
}

void DriverController::FeedbackRequest() {
  PVCI_CAN_OBJ req_obj = GetVciObject(1, SYNC_ID);
  req_obj->DataLen = 0;

  if (if_debug_) {
    req_data_file_.open(base_file_address_ + "/debug_data/req_data.txt",
                        std::ios::app);

    req_data_file_ << "request cmd -> ";
    req_data_file_ << std::hex << "id is : 0x" << req_obj->ID << "  ";
    req_data_file_ << std::dec << "len is : " << (int)req_obj->DataLen;
    req_data_file_ << std::endl;

    req_data_file_.close();
  }

  SendCommand(req_obj, 1);
  delete req_obj;
}

void DriverController::GetHomePosition(int* home_signal, const int& len) {
  for (size_t i = 0; i < len; i++) {
    home_position_[i] = home_signal[i];
  }
  if_steer_home_ = true;
}

void DriverController::GetHomePosition() {
  while (true) {
    FeedbackRequest();
    usleep(10000);

    int receive_obj_len = 2000;
    PVCI_CAN_OBJ receive_obj = new VCI_CAN_OBJ[receive_obj_len];
    GetData(receive_obj, receive_obj_len);

    std::vector<int> walk_fb_int;
    std::vector<int> steer_fb_int;
    walk_fb_int.resize(id_num_);
    steer_fb_int.resize(id_num_);

    int state_word;
    bool if_home;
    bool if_get_fb = true;
    bool get_fb_flag[id_num_];
    for (size_t i = 0; i < id_num_; i++) {
      get_fb_flag[i] = false;
    }
    for (size_t i = 0; i < receive_obj_len; i++) {
      bool if_get_state_word = false;
      for (size_t j = walk_id_num_; j < id_num_; j++) {
        if (cob_id_[j] + TPDO1_ID == receive_obj[i].ID) {
          if_get_state_word = true;
          break;
        }
      }
      if (if_get_state_word) {
        state_word = ByteHex2Int(&receive_obj[i].Data[1], 2);
        if (((state_word >> 15) & 0x01) == 1) {
          if_home = true;
        } else {
          if_home = false;
        }
        continue;
      }

      if (abs(receive_obj[i].ID - TPDO2_ID) < (id_num_ + 1)) {
        walk_fb_int[receive_obj[i].ID - TPDO2_ID - 1] =
            ByteHex2Int(&receive_obj[i].Data[0], 4);
        steer_fb_int[receive_obj[i].ID - TPDO2_ID - 1] =
            ByteHex2Int(&receive_obj[i].Data[4], 4);
        get_fb_flag[receive_obj[i].ID - TPDO2_ID - 1] = true;
      }
    }
    delete[] receive_obj;

    for (size_t i = 0; i < id_num_; i++) {
      if_get_fb = if_get_fb && get_fb_flag[i];
    }
    if (if_home && if_get_fb) {
      for (size_t i = 0; i < steer_id_num_; i++) {
        home_position_[i] = steer_fb_int[i + walk_id_num_];
      }
      if_steer_home_ = true;
      break;
    }
  }
}

void DriverController::GetFeedback(double* walk_fb, double* steer_fb) {
  FeedbackRequest();
  usleep(10000);

  int receive_obj_len = 2000;
  PVCI_CAN_OBJ receive_obj = new VCI_CAN_OBJ[receive_obj_len];
  GetData(receive_obj, receive_obj_len);

  int* walk_fb_int = new int[id_num_];
  int* steer_fb_int = new int[id_num_];
  for (size_t i = 0; i < receive_obj_len; i++) {
    if (abs(receive_obj[i].ID - TPDO2_ID) < (id_num_ + 1)) {
      walk_fb_int[receive_obj[i].ID - TPDO2_ID - 1] =
          ByteHex2Int(&receive_obj[i].Data[0], 4);
      steer_fb_int[receive_obj[i].ID - TPDO2_ID - 1] =
          ByteHex2Int(&receive_obj[i].Data[4], 4);
    }
  }
  for (size_t i = 0; i < id_num_; i++) {
    walk_fb[i] = (double)walk_fb_int[i] / reduc_ratio_w_;
    // steer_fb[i] =
  }

  delete[] walk_fb_int;
  delete[] steer_fb_int;
  delete[] receive_obj;
}

void DriverController::Dec2HexVector(u_char* data_vec, const int& dec_value,
                                     const int& len) {
  for (size_t i = 0; i < len; i++) {
    data_vec[i] = (((int)dec_value >> (i * 8)) & 0xff);
  }
}
int DriverController::ByteHex2Int(uint8_t* data_vec, const int& data_vec_len) {
  int result = 0;
  int sign_judger = 0x80;
  sign_judger = sign_judger << ((data_vec_len - 1) * 8);

  for (size_t i = 0; i < data_vec_len; i++) {
    result += (data_vec[i] << (i * 8));
  }
  if (sign_judger && result) {
    result = -1 * (~(result - 1));
  }
}

void DriverController::DataInitial(u_char* data, uint8_t* cmd,
                                   const uint& cmd_len) {
  for (size_t i = 0; i < cmd_len; i++) {
    data[i] = cmd[i];
  }
}

void DriverController::DebugData(const bool& if_debug_flag) {
  if_debug_ = if_debug_flag;
}

void DriverController::GetBaseAddress(const std::string& base_file_address) {
  base_file_address_ = base_file_address;
}

}  // namespace mobile_base
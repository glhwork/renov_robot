#include "sensor_startup/kinco/driver_reader.h"

namespace mobile_base {

DriverReader::DriverReader() : CanApplication() { if_steer_home_ = false; }

DriverReader::~DriverReader() { delete[] cob_id_; }

void DriverReader::ReadFile(const std::string& file_address) {
  YAML::Node driver_config = YAML::LoadFile(file_address);

  walking_mode_ = driver_config["walking_mode"].as<int>();
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

  cob_id_ = new uint[id_num_];
  for (size_t i = 0; i < id_num_; i++) {
    cob_id_[i] = (uint)driver_config["cob_id"][i].as<int>();
  }

  motor_sign_ = new int[id_num_];
  for (size_t i = 0; i < id_num_; i++) {
    motor_sign_[i] = (uint)driver_config["motor_sign"][i].as<int>();
  }
}

bool DriverReader::DriverInit() {
  StartPDO();

  if (!DriverSetMode()) {
    std::cout << "Setting mode of motors meets mistake" << std::endl;
    return false;
  }
  if (!DriverEnable()) {
    std::cout << "Enabling motors meets mistake" << std::endl;
    return false;
  }

  return true;
}

void DriverReader::StartPDO() {
  PVCI_CAN_OBJ pdo_start_obj = GetVciObject(id_num_, TPDO1_ID);

  uint cmd_len =
      sizeof(can_cmd_.PDO_START_WORK) / sizeof(can_cmd_.PDO_START_WORK[0]);
  for (size_t i = 0; i < id_num_; i++) {
    pdo_start_obj[i].ID += cob_id_[i];
    DataInitial(pdo_start_obj[i].Data, can_cmd_.PDO_START_WORK, cmd_len);
    // one of the byte in data should be equal to station number
    pdo_start_obj[i].DataLen = cmd_len;
  }

  SendCommand(pdo_start_obj, id_num_);
  delete[] pdo_start_obj;
}

bool DriverReader::DriverEnable() {
  PVCI_CAN_OBJ enable_obj = GetVciObject(id_num_, TPDO1_ID);

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

bool DriverReader::DriverSetMode() {
  PVCI_CAN_OBJ walk_mode_obj = GetVciObject(walk_id_num_, TPDO1_ID);
  switch (walking_mode_) {
    case VELOCITY_MODE: {
      uint walk_cmd_len = sizeof(can_cmd_.SET_VELOCITY_MODE) /
                          sizeof(can_cmd_.SET_VELOCITY_MODE[0]);
      for (size_t i = 0; i < walk_id_num_; i++) {
        walk_mode_obj[i].ID += cob_id_[i];
        DataInitial(walk_mode_obj[i].Data, can_cmd_.SET_VELOCITY_MODE,
                    walk_cmd_len);
        walk_mode_obj[i].DataLen = walk_cmd_len;
      }
      break;
    }
    case POSITION_MODE: {
      uint walk_cmd_len = sizeof(can_cmd_.SET_POSITION_MODE) /
                          sizeof(can_cmd_.SET_POSITION_MODE[0]);
      for (size_t i = 0; i < walk_id_num_; i++) {
        walk_mode_obj[i].ID += cob_id_[i];
        DataInitial(walk_mode_obj[i].Data, can_cmd_.SET_POSITION_MODE,
                    walk_cmd_len);
        walk_mode_obj[i].DataLen = walk_cmd_len;
      }
      break;
    }
    case CURRENT_MODE: {
      uint walk_cmd_len = sizeof(can_cmd_.SET_CURRENT_MODE) /
                          sizeof(can_cmd_.SET_CURRENT_MODE[0]);
      for (size_t i = 0; i < walk_id_num_; i++) {
        walk_mode_obj[i].ID += cob_id_[i];
        DataInitial(walk_mode_obj[i].Data, can_cmd_.SET_CURRENT_MODE,
                    walk_cmd_len);
        walk_mode_obj[i].DataLen = walk_cmd_len;
      }
      break;
    }
  }
  SendCommand(walk_mode_obj, walk_id_num_);
  delete[] walk_mode_obj;

  // set mode of steering motors
  PVCI_CAN_OBJ steer_mode_obj = GetVciObject(steer_id_num_, TPDO1_ID);
  switch (steering_mode_) {
    case VELOCITY_MODE: {
      uint steer_cmd_len = sizeof(can_cmd_.SET_VELOCITY_MODE) /
                           sizeof(can_cmd_.SET_VELOCITY_MODE[0]);
      for (size_t i = 0; i < steer_id_num_; i++) {
        steer_mode_obj[i].ID += cob_id_[i];
        DataInitial(steer_mode_obj[i].Data, can_cmd_.SET_VELOCITY_MODE,
                    steer_cmd_len);
        steer_mode_obj[i].DataLen = steer_cmd_len;
      }
      break;
    }
    case POSITION_MODE: {
      uint steer_cmd_len = sizeof(can_cmd_.SET_POSITION_MODE) /
                           sizeof(can_cmd_.SET_POSITION_MODE[0]);
      for (size_t i = 0; i < steer_id_num_; i++) {
        steer_mode_obj[i].ID += cob_id_[i];
        DataInitial(steer_mode_obj[i].Data, can_cmd_.SET_POSITION_MODE,
                    steer_cmd_len);
        steer_mode_obj[i].DataLen = steer_cmd_len;
      }
      break;
    }
    case CURRENT_MODE: {
      uint steer_cmd_len = sizeof(can_cmd_.SET_CURRENT_MODE) /
                           sizeof(can_cmd_.SET_CURRENT_MODE[0]);
      for (size_t i = 0; i < steer_id_num_; i++) {
        steer_mode_obj[i].ID += cob_id_[i];
        DataInitial(steer_mode_obj[i].Data, can_cmd_.SET_CURRENT_MODE,
                    steer_cmd_len);
        steer_mode_obj[i].DataLen = steer_cmd_len;
      }
      break;
    }
  }
  SendCommand(steer_mode_obj, steer_id_num_);
  delete[] steer_mode_obj;
}

void DriverReader::ControlMotor(const std::vector<int>& raw_control_signal) {
  if (raw_control_signal.size() != id_num_) {
    std::cout << "Quantity of control signal is incorrect" << std::endl;
    return;
  }

  std::vector<int> control_signal = ControlSignalTransform(raw_control_signal);

  // send control signal to walking motors
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
  }

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
  }
}

std::vector<int> DriverReader::ControlSignalTransform(
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

void DriverReader::SendVelocity(uint* id, int* target_velocity,
                                const int& len) {
  PVCI_CAN_OBJ velocity_cmd_obj = GetVciObject(len, TPDO1_ID);

  uint velocity_cmd_len =
      sizeof(can_cmd_.VELOCITY_COMMAND) / sizeof(can_cmd_.VELOCITY_COMMAND[0]);
  for (size_t i = 0; i < len; i++) {
    velocity_cmd_obj[i].ID += cob_id_[i];
    DataInitial(velocity_cmd_obj[i].Data, can_cmd_.VELOCITY_COMMAND,
                velocity_cmd_len);
    Dec2HexVector(&velocity_cmd_obj[i].Data[4], target_velocity[i]);
    velocity_cmd_obj[i].DataLen = velocity_cmd_len;
  }

  SendCommand(velocity_cmd_obj, len);
  delete[] velocity_cmd_obj;
}

void DriverReader::SendPosition(uint* id, int* target_position,
                                const int& len) {
  PVCI_CAN_OBJ position_cmd_obj = GetVciObject(len, TPDO1_ID);

  uint position_cmd_len =
      sizeof(can_cmd_.POSITION_COMMAND) / sizeof(can_cmd_.POSITION_COMMAND[0]);
  for (size_t i = 0; i < len; i++) {
    position_cmd_obj[i].ID += cob_id_[i];
    DataInitial(position_cmd_obj[i].Data, can_cmd_.POSITION_COMMAND,
                position_cmd_len);
    Dec2HexVector(&position_cmd_obj[i].Data[4], target_position[i]);
    position_cmd_obj[i].DataLen = position_cmd_len;
  }

  SendCommand(position_cmd_obj, len);
  delete[] position_cmd_obj;
}

void DriverReader::SendCurrent(uint* id, int* target_current, const int& len) {
  PVCI_CAN_OBJ current_cmd_obj = GetVciObject(len, TPDO1_ID);

  uint current_cmd_len =
      sizeof(can_cmd_.CURRENT_COMMAND) / sizeof(can_cmd_.CURRENT_COMMAND[0]);
  for (size_t i = 0; i < len; i++) {
    current_cmd_obj[i].ID += cob_id_[i];
    DataInitial(current_cmd_obj[i].Data, can_cmd_.CURRENT_COMMAND,
                current_cmd_len);
    Dec2HexVector(&current_cmd_obj[i].Data[4], target_current[i]);
    current_cmd_obj[i].DataLen = current_cmd_len;
  }

  SendCommand(current_cmd_obj, len);
  delete[] current_cmd_obj;
}

void DriverReader::GetHomePosition(int* home_signal, const int& len) {
  for (size_t i = 0; i < len; i++) {
    home_position_[i] = home_signal[i];
  }
  if_steer_home_ = true;
}

void DriverReader::Dec2HexVector(u_char* data_vec, const int& dec_value) {
  for (size_t i = 0; i < 4; i++) {
    data_vec[i] = (((int)dec_value >> (i * 8)) & 0xff);
  }
}
int DriverReader::FourByteHex2Int(uint8_t* data_vec, const int& data_vec_len) {}

void DriverReader::DataInitial(u_char* data, uint8_t* cmd,
                               const uint& cmd_len) {
  for (size_t i = 0; i < cmd_len; i++) {
    data[i] = cmd[i];
  }
}

}  // namespace mobile_base

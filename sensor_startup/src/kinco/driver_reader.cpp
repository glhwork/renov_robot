#include "sensor_startup/kinco/driver_reader.h"

namespace mobile_base {

DriverReader::DriverReader() : CanApplication() {}

DriverReader::~DriverReader() { delete[] cob_id; }

void DriverReader::ReadFile(const std::string& file_address) {
  YAML::Node driver_config = YAML::LoadFile(file_address);

  walking_mode = driver_config["walking_mode"].as<int>();
  steering_mode = driver_config["steering_mode"].as<int>();

  encoder_s_ = driver_config["encoder_s"].as<int>();
  encoder_w_ = driver_config["encoder_w"].as<int>();
  frequency_multiplier_ = driver_config["frequency_multiplier"].as<int>();
  reduc_ratio_s_ = driver_config["reduc_ratio_s"].as<double>();
  reduc_ratio_w_ = driver_config["reduc_ratio_w"].as<double>();
  max_velocity_ = driver_config["max_velocity"].as<int>();

  walk_id_num_ = driver_config["walk_id_num"].as<int>();
  steer_id_num_ = driver_config["steer_id_num"].as<int>();
  id_num_ = walk_id_num_ + steer_id_num_;

  cob_id = new uint[id_num_];
  for (size_t i = 0; i < id_num_; i++) {
    cob_id[i] = (uint)driver_config["cob_id"][i].as<int>();
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
      sizeof(can_cmd.PDO_START_WORK) / sizeof(can_cmd.PDO_START_WORK[0]);
  for (size_t i = 0; i < id_num_; i++) {
    pdo_start_obj[i].ID += cob_id[i];
    DataInitial(pdo_start_obj[i].Data, can_cmd.PDO_START_WORK, cmd_len);
    // one of the byte in data should be equal to station number
    pdo_start_obj[i].DataLen = cmd_len;
  }

  SendCommand(pdo_start_obj, id_num_);
  delete[] pdo_start_obj;
}

bool DriverReader::DriverEnable() {
  PVCI_CAN_OBJ enable_obj = GetVciObject(id_num_, TPDO1_ID);

  uint enable_cmd_len =
      sizeof(can_cmd.ENABLE_MOTOR) / sizeof(can_cmd.ENABLE_MOTOR[0]);
  for (size_t i = 0; i < id_num_; i++) {
    enable_obj[i].ID += cob_id[i];
    DataInitial(enable_obj[i].Data, can_cmd.ENABLE_MOTOR, enable_cmd_len);
    enable_obj[i].DataLen = enable_cmd_len;
  }

  SendCommand(enable_obj, id_num_);
  delete[] enable_obj;
}

bool DriverReader::DriverSetMode() {
  PVCI_CAN_OBJ walk_mode_obj = GetVciObject(walk_id_num_, TPDO1_ID);
  switch (walking_mode) {
    case VELOCITY_MODE: {
      uint walk_cmd_len = sizeof(can_cmd.SET_VELOCITY_MODE) /
                          sizeof(can_cmd.SET_VELOCITY_MODE[0]);
      for (size_t i = 0; i < walk_id_num_; i++) {
        walk_mode_obj[i].ID += cob_id[i];
        DataInitial(walk_mode_obj[i].Data, can_cmd.SET_VELOCITY_MODE,
                    walk_cmd_len);
        walk_mode_obj[i].DataLen = walk_cmd_len;
      }
      break;
    }
    case POSITION_MODE: {
      uint walk_cmd_len = sizeof(can_cmd.SET_POSITION_MODE) /
                          sizeof(can_cmd.SET_POSITION_MODE[0]);
      for (size_t i = 0; i < walk_id_num_; i++) {
        walk_mode_obj[i].ID += cob_id[i];
        DataInitial(walk_mode_obj[i].Data, can_cmd.SET_POSITION_MODE,
                    walk_cmd_len);
        walk_mode_obj[i].DataLen = walk_cmd_len;
      }
      break;
    }
    case CURRENT_MODE: {
      uint walk_cmd_len = sizeof(can_cmd.SET_CURRENT_MODE) /
                          sizeof(can_cmd.SET_CURRENT_MODE[0]);
      for (size_t i = 0; i < walk_id_num_; i++) {
        walk_mode_obj[i].ID += cob_id[i];
        DataInitial(walk_mode_obj[i].Data, can_cmd.SET_CURRENT_MODE,
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
  switch (steering_mode) {
    case VELOCITY_MODE: {
      uint steer_cmd_len = sizeof(can_cmd.SET_VELOCITY_MODE) /
                           sizeof(can_cmd.SET_VELOCITY_MODE[0]);
      for (size_t i = 0; i < steer_id_num_; i++) {
        steer_mode_obj[i].ID += cob_id[i];
        DataInitial(steer_mode_obj[i].Data, can_cmd.SET_VELOCITY_MODE,
                    steer_cmd_len);
        steer_mode_obj[i].DataLen = steer_cmd_len;
      }
      break;
    }
    case POSITION_MODE: {
      uint steer_cmd_len = sizeof(can_cmd.SET_POSITION_MODE) /
                           sizeof(can_cmd.SET_POSITION_MODE[0]);
      for (size_t i = 0; i < steer_id_num_; i++) {
        steer_mode_obj[i].ID += cob_id[i];
        DataInitial(steer_mode_obj[i].Data, can_cmd.SET_POSITION_MODE,
                    steer_cmd_len);
        steer_mode_obj[i].DataLen = steer_cmd_len;
      }
      break;
    }
    case CURRENT_MODE: {
      uint steer_cmd_len = sizeof(can_cmd.SET_CURRENT_MODE) /
                           sizeof(can_cmd.SET_CURRENT_MODE[0]);
      for (size_t i = 0; i < steer_id_num_; i++) {
        steer_mode_obj[i].ID += cob_id[i];
        DataInitial(steer_mode_obj[i].Data, can_cmd.SET_CURRENT_MODE,
                    steer_cmd_len);
        steer_mode_obj[i].DataLen = steer_cmd_len;
      }
      break;
    }
  }
  SendCommand(steer_mode_obj, steer_id_num_);
  delete[] steer_mode_obj;
}

int DriverReader::FourByteHex2Int(uint8_t* data_vec, const int& data_vec_len) {}

void DriverReader::DataInitial(u_char* data, uint8_t* cmd,
                               const uint& cmd_len) {
  for (size_t i = 0; i < cmd_len; i++) {
    data[i] = cmd[i];
  }
}

}  // namespace mobile_base

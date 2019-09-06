#include "sensor_startup/kinco/driver_reader.h"

namespace mobile_base {

DriverReader::DriverReader() : CanApplication() {}

DriverReader::~DriverReader() {}

void DriverReader::ReadFile(const std::string& file_address) {
  YAML::Node driver_config = YAML::LoadFile(file_address);

  encoder_s_ = driver_config["encoder_s"].as<int>();
  encoder_w_ = driver_config["encoder_w"].as<int>();
  frequency_multiplier_ = driver_config["frequency_multiplier"].as<int>();
  reduc_ratio_s_ = driver_config["reduc_ratio_s"].as<double>();
  reduc_ratio_w_ = driver_config["reduc_ratio_w"].as<double>();
  max_velocity_ = driver_config["max_velocity"].as<int>();

  walk_id_num_ = driver_config["walk_id_num"].as<int>();
  steer_id_num_ = driver_config["steer_id_num"].as<int>(); 
  id_num_ = walk_id_num_ + steer_id_num_;
  
}

bool DriverReader::DriverInit() {
  if (!DriverSetMode()) {
    return false;
  }
  if (!DriverEnable()) {
    return false;
  }

  return true;
}

bool DriverReader::DriverEnable() {}

bool DriverReader::DriverSetMode() {}

int DriverReader::FourByteHex2Int(uint8_t* data_vec, const int& data_vec_len) {}

}  // namespace mobile_base

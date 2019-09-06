#ifndef DRIVER_READER_H
#define DRIVER_READER_H

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "sensor_startup/kinco/can_application.h"

namespace mobile_base {

class DriverReader : public CanApplication {
 public:
  DriverReader();
  virtual ~DriverReader();
  void ReadFile(const std::string& file_address);

  bool DriverInit();
  bool DriverEnable();
  bool DriverSetMode();

  void SendVelocity();
  void SendPosition();
  void DriverDiagnostic();

  int FourByteHex2Int(uint8_t* data_vec, const int& data_vec_len);

 private:
  /* PARAMETERS READ FROM YAML FILE */
  int encoder_s_;
  int encoder_w_;
  int frequency_multiplier_;
  double reduc_ratio_s_;
  double reduc_ratio_w_;
  int max_velocity_;
  int walk_id_num_;
  int steer_id_num_;
  int id_num_;
  
};  // class DriverReader

}  // namespace mobile_base

#endif

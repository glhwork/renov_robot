#ifndef DRIVER_READER_H
#define DRIVER_READER_H

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "can_application.h"
#include "can_command.h"

typedef unsigned char u_char;

namespace mobile_base {

class DriverReader : public CanApplication {
 public:
  DriverReader();
  virtual ~DriverReader();
  void ReadFile(const std::string& file_address);

  bool DriverInit();
  void StartPDO();
  bool DriverEnable();
  bool DriverSetMode();

  void SendVelocity();
  void SendPosition();
  void DriverDiagnostic();

  void DataInitial(u_char* data, uint8_t* cmd, const uint& cmd_len);
  int FourByteHex2Int(uint8_t* data_vec, const int& data_vec_len);

 private:
  /* PARAMETERS READ FROM YAML FILE */
  int walking_mode;
  int steering_mode;
  int encoder_s_;
  int encoder_w_;
  int frequency_multiplier_;
  double reduc_ratio_s_;
  double reduc_ratio_w_;
  int max_velocity_;
  int walk_id_num_;
  int steer_id_num_;
  int id_num_;
  uint* cob_id;

  /* */
  CanCommand can_cmd;
  
};  // class DriverReader

}  // namespace mobile_base

#endif

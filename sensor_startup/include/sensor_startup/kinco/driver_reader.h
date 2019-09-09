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

  void ControlMotor(const std::vector<int>& control_signal);
  std::vector<int> ControlSignalTransform(const std::vector<int>& raw_signal);
  void SendVelocity(uint* id, int* target_velocity, const int& len);
  void SendPosition(uint* id, int* target_position, const int& len);
  void SendCurrent(uint* id, int* target_current, const int& len);
  void GetHomePosition(int* home_signal, const int& len);
  void DriverDiagnostic();

  void DataInitial(u_char* data, uint8_t* cmd, const uint& cmd_len);
  void Dec2HexVector(u_char* data_vec, const int& dec_value);
  int FourByteHex2Int(uint8_t* data_vec, const int& data_vec_len);

 protected:
  /* PARAMETERS READ FROM YAML FILE */
  int id_num_;
  int walk_id_num_;
  int steer_id_num_;

 private:
  /* PARAMETERS READ FROM YAML FILE */
  int walking_mode_;
  int steering_mode_;
  int encoder_s_;
  int encoder_w_;
  int frequency_multiplier_;
  double reduc_ratio_s_;
  double reduc_ratio_w_;
  int max_velocity_;
  uint* cob_id_;
  int* motor_sign_;

  /* */
  CanCommand can_cmd_;
  int* home_position_;
  bool if_steer_home_;

};  // class DriverReader

}  // namespace mobile_base

#endif

#ifndef CAN_COMMAND_H
#define CAN_COMMAND_H

#define TPDO1_ID 0x180
#define TPDO2_ID 0x280
#define TPDO3_ID 0x380
#define TPDO4_ID 0x480

#define RPDO1_ID 0x100
#define RPDO2_ID 0x200
#define RPDO3_ID 0x300
#define RPDO4_ID 0x400

#define VELOCITY_MODE 0
#define POSITION_MODE 1
#define CURRENT_MODE 2

typedef uint8_t _u8;

namespace mobile_base {

struct CanCommand {
  _u8 PDO_START_WORK[1] = {1};
  _u8 SET_VELOCITY_MODE[1] = {1};
  _u8 SET_POSITION_MODE[1] = {1}; 
  _u8 SET_CURRENT_MODE[1] = {1}; 
  _u8 ENABLE_MOTOR[1] = {1}; 

};  // struct CanCommand

}  // namespace mobile_base

#endif
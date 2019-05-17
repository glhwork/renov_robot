#ifndef CANASSIST_H
#define CANASSIST_H
/*
  macro definition of normal can commands
*/
#include <iostream>

typedef unsigned char _u8;

#define ENABLE_COMMAND_1 \
  {(_u8)0x2b, (_u8)0x40, (_u8)0x60, (_u8)0x00, (_u8)0x06, (_u8)0x00}
#define ENABLE_COMMAND_2 \
  {(_u8)0x2b, (_u8)0x40, (_u8)0x60, (_u8)0x00, (_u8)0x07, (_u8)0x00}
#define ENABLE_COMMAND_3 \
  {(_u8)0x2b, (_u8)0x40, (_u8)0x60, (_u8)0x00, (_u8)0x0f, (_u8)0x00}

#define SET_MODE_VELOCITY \
  {(_u8)0x2f, (_u8)0x60, (_u8)0x60, (_u8)0x00, (_u8)0x03}
#define SET_MODE_POSITION \
  {(_u8)0x2f, (_u8)0x60, (_u8)0x60, (_u8)0x00, (_u8)0x01}
#define SET_MODE_CURRENT \
  {(_u8)0x2f, (_u8)0x60, (_u8)0x60, (_u8)0x00, (_u8)0x04}

#define BASE_VELOCITY_COMMAND \
  {(_u8)0x23, (_u8)0xff, (_u8)0x00, (_u8)0x00, (_u8)0x00, (_u8)0x00, (_u8)0x00, (_u8)0x00}
#define BASE_POSITION_COMMAND \
  {(_u8)0x23, (_u8)0x7a, (_u8)0x00, (_u8)0x00, (_u8)0x00, (_u8)0x00, (_u8)0x00, (_u8)0x00}
#define BASE_CURRENT_COMMAND \
  {(_u8)0x2b, (_u8)0x71, (_u8)0x00, (_u8)0x00, (_u8)0x00, (_u8)0x00}

#define BASE_VELOCITY_FEEDBACK \
  {(_u8)0x40, (_u8)0x69, (_u8)0x00, (_u8)0x00}
#define BASE_POSITION_FEEDBACK \
  {(_u8)0x40, (_u8)0x64, (_u8)0x00, (_u8)0x00}
#define BASE_CURRENT_FEEDBACK \
  {(_u8)0x40, (_u8)0x78, (_u8)0x00, (_u8)0x00}
#define BASE_FORCE_FEEDBACK \
  {(_u8)0x40, (_u8)0x77, (_u8)0x00, (_u8)0x00}

#define BASE_VELOCITY_TARGET \
  {(_u8)0x40, (_u8)0x6b, (_u8)0x00, (_u8)0x00}
// #define BASE_POSITION_TARGET



#define DATA_UINT8 (_u8)0x4f
#define DATA_UINT16 (_u8)0x4b
#define DATA_UINT32 (_u8)0x43

#define ERROR_CODE (_u8)0x80
#endif
#ifndef CAN_APPLICATION_H
#define CAN_APPLICATION_H

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "sensor_startup/controlcan.h"

namespace mobile_base {

class CanApplication {
 public:
  CanApplication();
  virtual ~CanApplication();

 private:
};  // class CanApplication

}  // namespace mobile_base

#endif
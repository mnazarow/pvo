#pragma once
#include "Arduino.h"
class Servo {
  int pin_ = -1;
 public:
  uint8_t attach(int pin) { pin_ = pin; return 1; }
  void detach() {}
  void write(int a) { if (pin_ >= 0) g_servoAngle[pin_] = a; }
  int read() { return pin_ >= 0 ? g_servoAngle[pin_] : 0; }
  bool attached() { return true; }
};

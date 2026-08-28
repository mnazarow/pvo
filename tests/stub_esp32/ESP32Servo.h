#pragma once
#include "Arduino.h"
extern std::map<int,int> g_servoAngle;
class Servo {
  int pin_ = -1;
 public:
  void setPeriodHertz(int) {}
  int  attach(int pin, int = 544, int = 2400) { pin_ = pin; return 1; }
  void write(int a) { if (pin_ >= 0) g_servoAngle[pin_] = a; }
  void detach() {}
};

#pragma once
#include "Arduino.h"
#include <deque>
struct SoftwareSerial {
  std::deque<uint8_t> tx;
  SoftwareSerial(int, int) {}
  void begin(long) {}
  size_t write(const uint8_t *b, size_t n) { for (size_t i = 0; i < n; i++) tx.push_back(b[i]); return n; }
};

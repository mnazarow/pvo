#pragma once
#include <cstring>
#include <cstdint>
struct EEPROMStub {
  uint8_t mem[1024] = {0};
  template <typename T> void get(int addr, T &v) { std::memcpy(&v, mem + addr, sizeof(T)); }
  template <typename T> void put(int addr, const T &v) { std::memcpy(mem + addr, &v, sizeof(T)); }
};
extern EEPROMStub EEPROM;

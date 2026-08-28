#pragma once
#include <map>
#include <string>
#include <cstdint>
class Preferences {
  std::map<std::string, uint16_t> m;
  std::map<std::string, uint32_t> m32;
 public:
  bool begin(const char *, bool = false) { return true; }
  uint16_t getUShort(const char *k, uint16_t d = 0) {
    auto i = m.find(k); return i == m.end() ? d : i->second;
  }
  size_t putUShort(const char *k, uint16_t v) { m[k] = v; return 2; }
  int16_t getShort(const char *k, int16_t d = 0) {
    auto i = m.find(k); return i == m.end() ? d : (int16_t)i->second;
  }
  size_t putShort(const char *k, int16_t v) { m[k] = (uint16_t)v; return 2; }
  uint32_t getUInt(const char *k, uint32_t d = 0) {
    auto i = m32.find(k); return i == m32.end() ? d : i->second;
  }
  size_t putUInt(const char *k, uint32_t v) { m32[k] = v; return 4; }
  void end() {}
};

// Стаб Arduino-ESP32 API для проверки скетча на ПК
#pragma once
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <deque>
#include <map>

#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define SERIAL_8N1 0x800001c

class __FlashStringHelper;
#define F(x) (reinterpret_cast<const __FlashStringHelper *>(x))

extern unsigned long g_millis;
extern std::map<int,int> g_pinState;

inline void pinMode(int, int) {}
inline void digitalWrite(int pin, int v) { g_pinState[pin] = v; }
inline unsigned long millis() { return g_millis; }
inline void delay(unsigned long ms) { g_millis += ms; }
inline void delayMicroseconds(unsigned int) {}
inline void tone(int, unsigned int, unsigned long = 0) {}
inline void noTone(int) {}

template <typename T> T constrain(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }

struct HardwareSerial {
  std::deque<uint8_t> rx;
  std::deque<uint8_t> tx;
  bool isDebug = false;
  void begin(long) {}
  void begin(long, int, int = -1, int = -1) {}
  int  available() { return (int)rx.size(); }
  int  read() { if (rx.empty()) return -1; uint8_t c = rx.front(); rx.pop_front(); return c; }
  size_t write(const uint8_t *b, size_t n) { for (size_t i = 0; i < n; i++) tx.push_back(b[i]); return n; }
  void feed(const char *s) { while (*s) rx.push_back((uint8_t)*s++); }
  void print(const char *s)   { if (isDebug) fputs(s, stdout); }
  void print(const __FlashStringHelper *s) { print(reinterpret_cast<const char *>(s)); }
  void print(int v)           { if (isDebug) printf("%d", v); }
  void print(unsigned v)      { if (isDebug) printf("%u", v); }
  void print(unsigned long v) { if (isDebug) printf("%lu", v); }
  void print(float v)         { if (isDebug) printf("%.2f", v); }
  void println(const char *s) { if (isDebug) { fputs(s, stdout); fputs("\n", stdout); } }
  void println(const __FlashStringHelper *s) { println(reinterpret_cast<const char *>(s)); }
  void println(int v)         { if (isDebug) printf("%d\n", v); }
  void println(unsigned v)    { if (isDebug) printf("%u\n", v); }
  void println(unsigned long v){ if (isDebug) printf("%lu\n", v); }
  void print(char c)          { if (isDebug) printf("%c", c); }
  void print(long v)          { if (isDebug) printf("%ld", v); }
  void println(long v)        { if (isDebug) printf("%ld\n", v); }
  void println()              { if (isDebug) fputs("\n", stdout); }
  void print(const class String &s);
  void println(const class String &s);
  void print(const struct IPAddress &ip);
  void println(const struct IPAddress &ip);
};

// ---- String (минимальный аналог ардуиновского) ----
#include <string>
class String {
 public:
  std::string v;
  String() {}
  String(const char *s) : v(s ? s : "") {}
  String(const std::string &s) : v(s) {}
  String(char c) : v(1, c) {}
  String(int n) { v = std::to_string(n); }
  String(long n) { v = std::to_string(n); }
  String(unsigned n) { v = std::to_string(n); }
  String &operator+=(const char *s) { v += s; return *this; }
  String &operator+=(const String &s) { v += s.v; return *this; }
  String &operator+=(char c) { v += c; return *this; }
  String &operator+=(int n) { v += std::to_string(n); return *this; }
  String &operator+=(long n) { v += std::to_string(n); return *this; }
  String &operator+=(unsigned n) { v += std::to_string(n); return *this; }
  String operator+(const char *s) const { String r(*this); r += s; return r; }
  const char *c_str() const { return v.c_str(); }
  unsigned length() const { return (unsigned)v.size(); }
  long toInt() const { return v.empty() ? 0 : atol(v.c_str()); }
};

// ---- IPAddress ----
struct IPAddress {
  const char *s = "192.168.4.1";
  const char *toString() const { return s; }
};

#define PROGMEM

extern HardwareSerial Serial, Serial2, Serial1;

inline void HardwareSerial::print(const String &s)   { print(s.c_str()); }
inline void HardwareSerial::println(const String &s) { println(s.c_str()); }
inline void HardwareSerial::print(const IPAddress &ip)   { print(ip.s); }
inline void HardwareSerial::println(const IPAddress &ip) { println(ip.s); }

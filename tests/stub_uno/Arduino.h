// Стаб Arduino API (Uno/Nano) для проверки скетча на ПК
#pragma once
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <deque>
#include <map>

#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define F(x) (x)

extern unsigned long g_millis;
extern std::map<int,int> g_pinState;
extern std::map<int,int> g_servoAngle;
extern unsigned long (*g_pulseFn)();   // сценарий эха HC-SR04

inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t pin, uint8_t v) { g_pinState[pin] = v; }
inline int  digitalRead(uint8_t) { return 0; }
inline void delay(unsigned long ms) { g_millis += ms; }
inline void delayMicroseconds(unsigned int) {}
inline unsigned long millis() { return g_millis; }
inline unsigned long pulseIn(uint8_t, uint8_t, unsigned long) { return g_pulseFn ? g_pulseFn() : 0; }
inline void tone(uint8_t, unsigned int, unsigned long = 0) {}
inline void noTone(uint8_t) {}

template <typename T> T constrain(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }
using std::abs;

struct SerialStub {
  std::deque<char> rx;
  bool echoOut = true;
  void begin(long) {}
  int  available() { return (int)rx.size(); }
  int  read() { if (rx.empty()) return -1; char c = rx.front(); rx.pop_front(); return c; }
  void feed(const char *s) { while (*s) rx.push_back(*s++); }
  void print(const char *s)   { if (echoOut) fputs(s, stdout); }
  void print(int v)           { if (echoOut) printf("%d", v); }
  void print(unsigned v)      { if (echoOut) printf("%u", v); }
  void print(char c)          { if (echoOut) printf("%c", c); }
  void println(const char *s) { if (echoOut) { fputs(s, stdout); fputs("\n", stdout); } }
  void println(int v)         { if (echoOut) printf("%d\n", v); }
  void println(unsigned v)    { if (echoOut) printf("%u\n", v); }
  void print(long v)          { if (echoOut) printf("%ld", v); }
  void println(long v)        { if (echoOut) printf("%ld\n", v); }
  void print(unsigned long v) { if (echoOut) printf("%lu", v); }
  void println(unsigned long v){ if (echoOut) printf("%lu\n", v); }
  void println()              { if (echoOut) fputs("\n", stdout); }
};
extern SerialStub Serial;

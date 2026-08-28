#pragma once
#include "Arduino.h"
#define WL_CONNECTED 3
struct WiFiClass {
  bool apStarted = false;
  bool softAP(const char *, const char *) { apStarted = true; return true; }
  void begin(const char *, const char *) {}
  int status() { return WL_CONNECTED; }
  IPAddress softAPIP() { return IPAddress(); }
  IPAddress localIP() { return IPAddress(); }
};
extern WiFiClass WiFi;

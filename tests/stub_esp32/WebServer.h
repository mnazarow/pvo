#pragma once
#include "Arduino.h"
#include <functional>
#include <map>
#include <string>
class WebServer {
 public:
  std::map<std::string, std::function<void()>> routes;
  std::map<std::string, std::string> testArgs;   // подставляются тестом
  int lastCode = 0;
  std::string lastBody, lastType;
  WebServer(int = 80) {}
  void on(const char *p, std::function<void()> f) { routes[p] = f; }
  void begin() {}
  void handleClient() {}
  String arg(const char *k) {
    auto i = testArgs.find(k);
    return String(i == testArgs.end() ? "" : i->second.c_str());
  }
  void send(int c, const char *t, const String &b) { lastCode = c; lastType = t; lastBody = b.v; }
  void send(int c, const char *t, const char *b)   { lastCode = c; lastType = t; lastBody = b; }
  void send_P(int c, const char *t, const char *b) { send(c, t, b); }
  void invoke(const char *p) { routes.at(p)(); }   // для тестов
};

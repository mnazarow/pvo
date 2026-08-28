#pragma once
#include "Arduino.h"
#include "Wire.h"
#include <string>
#define SSD1306_SWITCHCAPVCC 0
#define SSD1306_WHITE 1
class Adafruit_SSD1306 {
 public:
  std::string text;          // всё, что напечатано с последней очистки
  std::string shown;         // что реально на «экране» после display()
  Adafruit_SSD1306(int, int, TwoWire *, int) {}
  bool begin(int, int) { return true; }
  void clearDisplay() { text.clear(); }
  void setTextColor(int) {}
  void setTextSize(int) {}
  void setCursor(int, int) {}
  void print(const char *s) { text += s; }
  void print(int v) { text += std::to_string(v); }
  void print(unsigned v) { text += std::to_string(v); }
  void display() { shown = text; }
};

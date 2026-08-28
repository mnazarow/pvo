// ============================================================
//  Servo для СТЕНДА ЭМУЛЯЦИИ (simavr / avr8js), не для платы.
//
//  В Debian-пакете arduino-core-avr библиотеки Servo нет, а сборка
//  под ATmega328P без неё невозможна. Это совместимая по интерфейсу
//  и по поведению реализация: Timer1, кадр 20 мс (50 Гц), импульс
//  544–2400 мкс, до 8 каналов — ровно то, что меряет эмулятор.
//
//  На настоящей плате используйте штатную Servo из Arduino IDE:
//  она сложнее (общий таймер на 12 каналов, prescaler-логика), но
//  внешне ведёт себя так же.
// ============================================================
#pragma once
#include <Arduino.h>

#define MIN_PULSE_WIDTH   544
#define MAX_PULSE_WIDTH  2400
#define DEFAULT_PULSE_WIDTH 1500
#define REFRESH_INTERVAL 20000
#define MAX_SERVOS 8
#define INVALID_SERVO 255

class Servo {
 public:
  Servo();
  uint8_t attach(int pin);
  uint8_t attach(int pin, int min, int max);
  void detach();
  void write(int angle);              // 0..180 (или мкс, как в оригинале)
  void writeMicroseconds(int us);
  int read();                         // обратно в градусы
  int readMicroseconds();
  bool attached();

 private:
  uint8_t idx;
  int minUs, maxUs;
};

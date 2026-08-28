// Тест pvo_esp32.ino: декодер, захват, поражение, потеря радара и
// восстановление, сервисная консоль, битые кадры.
#include "Arduino.h"
#include <cassert>
#include <vector>

unsigned long g_millis = 0;
std::map<int,int> g_pinState;
std::map<int,int> g_servoAngle;
HardwareSerial Serial, Serial2, Serial1;
#include "WiFi.h"
WiFiClass WiFi;
#include "Wire.h"
TwoWire Wire;

// ESP32Servo стаб
#include "ESP32Servo.h"

#include "sketch.cpp.inc"

static void putRadarInt16(std::vector<uint8_t> &v, int val) {
  uint16_t raw = (val >= 0) ? (uint16_t)(val | 0x8000) : (uint16_t)(-val);
  v.push_back(raw & 0xFF); v.push_back(raw >> 8);
}

static std::vector<uint8_t> makeFrame(bool hasT, int x, int y, int spd, int res, bool badTail = false) {
  std::vector<uint8_t> f = {0xAA, 0xFF, 0x03, 0x00};
  if (hasT) { putRadarInt16(f, x); putRadarInt16(f, y); putRadarInt16(f, spd);
              f.push_back(res & 0xFF); f.push_back(res >> 8); }
  else for (int i = 0; i < 8; i++) f.push_back(0);
  for (int i = 0; i < 16; i++) f.push_back(0);
  if (badTail) { f.push_back(0x11); f.push_back(0x22); }
  else         { f.push_back(0x55); f.push_back(0xCC); }
  return f;
}

static void feedFrame(bool hasT, int x=0, int y=0, int s=0, int r=0, bool bad=false) {
  for (uint8_t b : makeFrame(hasT, x, y, s, r, bad)) Serial2.rx.push_back(b);
}

int main() {
  Serial.isDebug = true;
  setup();

  assert(radarInt16(0x0E, 0x03) == -782);
  assert(radarInt16(0xB1, 0x86) == 1713);
  assert(radarInt16(0x10, 0x00) == -16);
  assert(Serial2.tx.size() == 12 && Serial2.tx[6] == 0x90);
  printf("[TEST] декодер и команда многоцелевого режима: OK\n");

  // Захват и поражение
  for (int i = 0; i < 12; i++) { feedFrame(true, -782, 1713, -16, 320);
    for (int t = 0; t < 4; t++) { g_millis += 10; loop(); } }
  assert(mode == TRACK && g_pinState[23] == HIGH);
  int pan = g_servoAngle[26];
  printf("[TEST] захват, pan=%d (~114): OK\n", pan);
  assert(pan >= 112 && pan <= 117);
  for (int i = 0; i < 92; i++) { feedFrame(true, -782, 1713, -16, 320); g_millis += 35; loop(); }
  assert(kills == 1);
  printf("[TEST] поражение и NVS-журнал: OK (kills=%u)\n", kills);

  // Радар замолчал: тревога, лазер погас, переинициализация
  size_t txBefore = Serial2.tx.size();
  for (int i = 0; i < 200; i++) { g_millis += 35; loop(); }   // 7 c тишины
  assert(radarAlarm && mode == IDLE && g_pinState[23] == LOW);
  assert(Serial2.tx.size() >= txBefore + 12);                 // была повторная инициализация
  printf("[TEST] тревога радара + переинициализация: OK\n");

  // Радар ожил
  for (int i = 0; i < 5; i++) { feedFrame(false); g_millis += 35; loop(); }
  assert(!radarAlarm);
  printf("[TEST] восстановление радара: OK\n");

  // Консоль: статус, центр, тест лазера
  Serial.feed("?\n"); g_millis += 35; loop();
  Serial.feed("C\n"); g_millis += 35; loop();
  assert(holdUntil > g_millis);
  Serial.feed("L1\n"); g_millis += 35; loop();
  assert(g_pinState[23] == HIGH);
  Serial.feed("L0\n"); g_millis += 35; loop();
  assert(g_pinState[23] == LOW);
  Serial.feed("X\n"); g_millis += 35; loop();   // неизвестная команда -> помощь
  printf("[TEST] сервисная консоль: OK\n");

  // Битые кадры -> подсказка (счётчики обнуляются)
  for (int i = 0; i < 60; i++) { feedFrame(false, 0,0,0,0, true); g_millis += 35; loop(); }
  assert(framesBad < 20 && framesOk == 0); // сброс счётчиков произошёл в середине серии
  printf("[TEST] детект битых кадров: OK\n");


  // Протокол настройки: G/S/W/D/M и телеметрия
  Serial.feed("G\n"); g_millis += 35; loop();
  Serial.feed("S MAXR 3000\n"); g_millis += 35; loop();
  assert(MAX_RANGE_MM == 3000);
  Serial.feed("S ALPHA 50\n"); g_millis += 35; loop();
  assert(SMOOTH_ALPHA > 0.49f && SMOOTH_ALPHA < 0.51f);
  Serial.feed("S MAXR 99999\n"); g_millis += 35; loop();
  assert(MAX_RANGE_MM == 3000);
  Serial.feed("W\n"); g_millis += 35; loop();
  Serial.feed("D\n"); g_millis += 35; loop();
  assert(MAX_RANGE_MM == 4000);
  Serial.feed("M1\n"); g_millis += 35; loop();
  for (int i = 0; i < 10; i++) { feedFrame(true, 500, 1500, 10, 320); g_millis += 35; loop(); }
  assert(telemetryOn);
  Serial.feed("M0\n"); g_millis += 35; loop();
  printf("[TEST] протокол G/S/W/D/M и телеметрия: OK\n");


  // Сбрасываем сопровождение: пустые кадры радара до возврата на дежурство
  for (int i = 0; i < 30; i++) { feedFrame(false); g_millis += 35; loop(); }
  assert(mode == IDLE);

  // Веб-панель: маршруты и обработчики
  assert(WiFi.apStarted);
  web.invoke("/");
  assert(web.lastBody.find("ПВО-2К") != std::string::npos);
  web.invoke("/api/status");
  assert(web.lastBody.find("\"mode\":\"I\"") != std::string::npos);
  assert(web.lastBody.find("\"kills\":1") != std::string::npos);
  web.invoke("/api/params");
  assert(web.lastBody.find("\"MAXR\":4000") != std::string::npos);
  web.testArgs = {{"name", "MAXR"}, {"value", "2500"}};
  web.invoke("/api/set");
  assert(web.lastCode == 200 && MAX_RANGE_MM == 2500);
  web.testArgs = {{"name", "BOGUS"}, {"value", "1"}};
  web.invoke("/api/set");
  assert(web.lastCode == 400);
  web.testArgs = {{"c", "L1"}};
  web.invoke("/api/cmd");
  assert(g_pinState[23] == HIGH);
  web.testArgs = {{"c", "L0"}};
  web.invoke("/api/cmd");
  assert(g_pinState[23] == LOW);
  Serial.feed("D\n"); g_millis += 35; loop();
  printf("[TEST] веб-панель (/, status, params, set, cmd): OK\n");

  // OLED: на «экране» режим и счётчик
  g_millis += 300; loop();
  assert(oled.shown.find("DEZHUR") != std::string::npos);
  assert(oled.shown.find("KILLS 1") != std::string::npos);
  printf("[TEST] OLED-экран: OK\n");

  // DFPlayer: при старте ушли команды (громкость + трек 1), кадры валидны
  assert(Serial1.tx.size() >= 20);
  {
    uint8_t f[10];
    for (int i = 0; i < 10; i++) f[i] = Serial1.tx[i];
    assert(f[0] == 0x7E && f[9] == 0xEF && f[3] == 0x06);       // громкость
    int16_t sum = 0; for (int i = 1; i < 7; i++) sum += f[i]; sum = -sum;
    assert(f[7] == (uint8_t)(sum >> 8) && f[8] == (uint8_t)sum); // чексумма
    for (int i = 0; i < 10; i++) f[i] = Serial1.tx[10 + i];
    assert(f[3] == 0x03 && f[6] == 1);                           // play трек 1
  }
  printf("[TEST] DFPlayer: кадры протокола: OK\n");


  // Z: обнуление журнала (NVS)
  Serial.feed("Z\n"); g_millis += 35; loop();
  assert(kills == 0);
  printf("[TEST] Z (обнуление журнала): OK\n");

  printf("\nALL TESTS PASSED (B)\n");
  return 0;
}

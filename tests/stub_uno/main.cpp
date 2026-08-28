// Сценарный тест pvo.ino: патруль -> захват -> поражение -> потеря ->
// внешнее целеуказание (SLAVE) -> таймаут -> патруль.
#include "Arduino.h"
#include <cassert>

unsigned long g_millis = 0;
std::map<int,int> g_pinState;
std::map<int,int> g_servoAngle;
unsigned long (*g_pulseFn)() = nullptr;
SerialStub Serial;
#include "EEPROM.h"
EEPROMStub EEPROM;

#include "sketch.cpp.inc"

static bool targetPresent = false;

// Эхо HC-SR04 по сценарию: цель «стоит» на азимуте 85..95, 40 см
static unsigned long scriptedPulse() {
  int a = g_servoAngle[9];
  if (targetPresent && a >= 85 && a <= 95) return 40UL * 58UL;
  return 180UL * 58UL;                             // эхо стены
}

static void run(int steps) {                       // steps шагов по 30 мс
  for (int i = 0; i < steps; i++) { g_millis += 30; loop(); }
}

int main() {
  g_pulseFn = scriptedPulse;
  setup();

  // 1. Патруль без целей
  targetPresent = false;
  run(400);
  assert(mode == PATROL && g_pinState[7] == LOW);
  printf("[TEST] патруль без целей: OK\n");

  // 2. Появилась цель на азимуте ~90 -> захват
  targetPresent = true;
  run(300);
  assert(mode == TRACK);
  assert(g_pinState[7] == HIGH);
  printf("[TEST] захват (lockAngle=%d): OK\n", lockAngle);

  // 3. Удержание > 3 с -> «поражение»
  run(120);
  assert(kills == 1);
  printf("[TEST] поражение записано: OK\n");

  // 4. Цель исчезла -> сброс и патруль
  targetPresent = false;
  run(60);
  assert(mode == PATROL && g_pinState[7] == LOW);
  printf("[TEST] потеря цели: OK\n");

  // 5. Внешнее целеуказание: P120
  Serial.feed("P120\n");
  run(3);
  assert(mode == SLAVE && g_servoAngle[9] == 120 && g_pinState[7] == HIGH);
  printf("[TEST] SLAVE-наведение на 120: OK\n");

  // 6. Команды продолжают идти -> держим позицию
  for (int i = 0; i < 20; i++) { Serial.feed("P118\n"); run(1); }
  assert(mode == SLAVE && g_servoAngle[9] == 118);

  // 7. Командир замолчал -> таймаут, патруль
  run(30);
  assert(mode == PATROL && g_pinState[7] == LOW);
  printf("[TEST] таймаут SLAVE -> патруль: OK\n");

  // 8. Статус
  Serial.feed("?\n");
  run(1);


  // 9. Протокол настройки: G/S/W/D/M
  Serial.feed("G\n"); run(1);
  Serial.feed("S DETECT 45\n"); run(1);
  assert(DETECT_DIST_CM == 45);
  Serial.feed("S DETECT 9999\n"); run(1);
  assert(DETECT_DIST_CM == 45);                    // вне диапазона -> отказ
  Serial.feed("S BOGUS 5\n"); run(1);
  Serial.feed("S SWMIN 40\n"); run(1);
  assert(SWEEP_MIN == 40);
  Serial.feed("W\n"); run(1);
  Persist pchk; EEPROM.get(0, pchk);
  assert(pchk.detect == 45 && pchk.swMin == 40);
  Serial.feed("D\n"); run(1);
  assert(DETECT_DIST_CM == 60 && SWEEP_MIN == 15);
  Serial.feed("M1\n"); run(10);
  assert(telemetryOn);
  Serial.feed("M0\n"); run(1);
  assert(!telemetryOn);
  printf("[TEST] протокол G/S/W/D/M: OK\n");


  // 10. DFPlayer: стартовые кадры и трек «захват»
#if USE_DFPLAYER
  assert(dfSerial.tx.size() >= 20);
  {
    uint8_t f[10];
    for (int i = 0; i < 10; i++) f[i] = dfSerial.tx[i];
    assert(f[0] == 0x7E && f[9] == 0xEF && f[3] == 0x06);      // громкость
    int16_t sum = 0; for (int i = 1; i < 7; i++) sum += f[i]; sum = -sum;
    assert(f[7] == (uint8_t)(sum >> 8) && f[8] == (uint8_t)sum);
    bool sawLockTrack = false;
    for (size_t o = 0; o + 10 <= dfSerial.tx.size(); o += 10)
      if (dfSerial.tx[o + 3] == 0x03 && dfSerial.tx[o + 6] == 2) sawLockTrack = true;
    assert(sawLockTrack);                                       // трек 2 при захвате
  }
  printf("[TEST] DFPlayer (Uno): OK\n");
#endif

  printf("\nALL TESTS PASSED (журнал: %u)\n", kills);
  return 0;
}

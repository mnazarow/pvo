// Сценарный тест pvo_a_simple.ino: самотест -> патруль -> захват ->
// поражение (EEPROM) -> потеря -> отказ датчика -> восстановление.
#include "Arduino.h"
#include <cassert>

unsigned long g_millis = 0;
std::map<int,int> g_pinState;
std::map<int,int> g_servoAngle;
unsigned long (*g_pulseFn)() = nullptr;
SerialStub Serial;
#include "EEPROM.h"
EEPROMStub EEPROM;

#include "sketch_a.cpp.inc"

static int scenario = 0;  // 0=пустая комната (стены), 1=цель, 2=датчик мёртв

static unsigned long scriptedPulse() {
  if (scenario == 2) return 0;                    // обрыв датчика
  int a = g_servoAngle[9];
  if (scenario == 1 && a >= 85 && a <= 95) return 40UL * 58UL;
  return 180UL * 58UL;                            // эхо от стены, 180 см
}

static void run(int steps) { for (int i = 0; i < steps; i++) { g_millis += 30; loop(); } }

int main() {
  g_pulseFn = scriptedPulse;
  setup();
  assert(!sensorAlarm);
  printf("[TEST] самотест пройден, датчик жив\n");

  run(400);
  assert(mode == PATROL && g_pinState[7] == LOW && !sensorAlarm);
  printf("[TEST] патруль, стены не считаются целью: OK\n");

  scenario = 1;
  run(400);
  assert(mode == TRACK && g_pinState[7] == HIGH);
  run(120);
  assert(kills == 1);
  Persist chk; EEPROM.get(0, chk);
  assert(chk.magic == 0x1D07 && chk.kills == 1 && chk.boots == 1);
  printf("[TEST] захват, поражение, запись в EEPROM: OK\n");

  scenario = 0;
  run(60);
  assert(mode == PATROL && g_pinState[7] == LOW);
  printf("[TEST] потеря цели: OK\n");

  scenario = 2;                                   // датчик умер
  run(1100);                                      // ~33 c
  assert(sensorAlarm);
  printf("[TEST] тревога об отказе датчика: OK\n");

  scenario = 0;                                   // датчик ожил
  run(5);
  assert(!sensorAlarm);
  printf("[TEST] восстановление датчика: OK\n");

  printf("\nALL TESTS PASSED (A-simple)\n");
  return 0;
}

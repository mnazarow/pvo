/*
 * ============================================================
 *  ПВО-1К «Комар» — ВАРИАНТ A (чистый): Uno/Nano + HC-SR04
 * ============================================================
 *  Самодостаточный скетч сканирующей турели БЕЗ режима внешнего
 *  целеуказания (для связки с Raspberry Pi используйте pvo.ino).
 *
 *  Железо: Arduino Uno/Nano, HC-SR04 и лазер <=5 мВт на качалке
 *  серво, буззер. Схема и сборка — «Сборка варианта A.docx».
 *
 *  ЛОГИКА:  ПАТРУЛЬ -> ЗАХВАТ -> СОПРОВОЖДЕНИЕ -> «ПОРАЖЕНИЕ»
 *           -> ПОТЕРЯ -> ПАТРУЛЬ (подробно — в руководстве).
 *
 *  ОБРАБОТКА ОШИБОК:
 *   - самотест при старте: датчик (10 контрольных замеров),
 *     серво (прогон), лазер (2 мигания), буззер;
 *   - слежение за датчиком в работе: если эха нет ВООБЩЕ дольше
 *     SENSOR_FAIL_S секунд — тревога в Serial + двойной «чирп»
 *     (обрыв TRIG/ECHO/питания); восстанавливается само;
 *   - защита от мусора в замерах: медиана из 3 при захвате;
 *   - журнал и счётчик перезагрузок в EEPROM (USE_EEPROM 1):
 *     если счётчик загрузок растёт сам по себе — у вас проседает
 *     питание (см. руководство, раздел «Диагностика»).
 *
 *  ЗАМЕНЫ КОМПОНЕНТОВ (полная таблица — в руководстве):
 *   - HC-SR04P / US-015: тот же код, без изменений;
 *   - JSN-SR04T (влагозащищённый): MIN_VALID_CM = 25;
 *   - серво MG90S/MG996R: без изменений (MG996R — питание 5В/2А);
 *   - активный или пассивный буззер: без изменений;
 *   - Uno / Nano / Pro Mini / Mega: без изменений (пины те же).
 * ============================================================
 */

#include <Servo.h>

#define FW_VERSION "1.1"
#define RADAR_GUI 0        // 1 = поток "угол,дистанция." для Processing-радара
#define USE_EEPROM 1       // 1 = журнал и счётчик загрузок в EEPROM

#if USE_EEPROM
#include <EEPROM.h>
#endif

// ========================= ПИНЫ =============================
const uint8_t PIN_SERVO  = 9;
const uint8_t PIN_TRIG   = 3;
const uint8_t PIN_ECHO   = 2;
const uint8_t PIN_LASER  = 7;
const uint8_t PIN_BUZZER = 8;

// ======================= НАСТРОЙКИ ==========================
const int      SWEEP_MIN      = 15;
const int      SWEEP_MAX      = 165;
const int      DETECT_DIST_CM = 60;
const int      LOST_DIST_CM   = 90;
const int      MIN_VALID_CM   = 3;     // ближе — считаем помехой (JSN-SR04T: 25)
const uint8_t  CONFIRM_PINGS  = 3;
const uint8_t  LOST_PINGS     = 12;
const int      TRACK_WOBBLE   = 5;
const unsigned long STEP_MS      = 30;
const unsigned long KILL_HOLD_MS = 3000;
const unsigned long PULSE_TIMEOUT_US = 20000UL; // ~3,4 м; JSN-SR04T: 30000
const unsigned int  SENSOR_FAIL_S = 30; // «эха нет вообще» столько секунд -> тревога

// ======================= СОСТОЯНИЕ ==========================
enum Mode { PATROL, TRACK };
Mode mode = PATROL;

Servo turret;
int  angle = SWEEP_MIN, sweepDir = 1;
int  lockAngle = 90, wobble = 0, wobbleDir = 1;
uint8_t confirmCnt = 0, lostCnt = 0;
unsigned long lastStep = 0, lockedAt = 0;
bool killLogged = false;
unsigned int kills = 0;

// --- здоровье датчика ---
unsigned long lastEchoMs = 0;      // когда последний раз было ЛЮБОЕ эхо
unsigned long lastAlarmMs = 0;
bool sensorAlarm = false;

#if USE_EEPROM
struct Persist { uint16_t magic; uint16_t boots; uint16_t kills; };
const uint16_t MAGIC = 0x1D07;
Persist pdata;
#endif

// Прототипы
void patrolStep(); void trackStep(); void lockTarget(); void releaseTarget();
int  measureCm(); int measureMedian3(); void report(int a, int d);
void victoryTune(); void selfTest(); void sensorWatch();

// ============================================================
void setup() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_LASER, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_LASER, LOW);
  Serial.begin(9600);
  turret.attach(PIN_SERVO);

#if USE_EEPROM
  EEPROM.get(0, pdata);
  if (pdata.magic != MAGIC) { pdata = {MAGIC, 0, 0}; }
  pdata.boots++;
  EEPROM.put(0, pdata);
  kills = pdata.kills;
#endif

  selfTest();

#if !RADAR_GUI
  Serial.println(F("=== ПВО-1К \"Комар\" (вариант A) на боевом дежурстве ==="));
  Serial.println(F("Версия прошивки: " FW_VERSION));
#if USE_EEPROM
  Serial.print(F("Загрузка N")); Serial.print(pdata.boots);
  Serial.print(F(", журнал за всё время: ")); Serial.println(kills);
  if (pdata.boots > 3)
    Serial.println(F("Подсказка: если счётчик загрузок растёт сам — проверьте питание серво"));
#endif
#endif
  lastEchoMs = millis();
}

// ============================================================
void loop() {
  unsigned long now = millis();
  if (now - lastStep < STEP_MS) return;
  lastStep = now;

  sensorWatch();                       // контроль здоровья датчика

  if (mode == PATROL) patrolStep();
  else                trackStep();
}

// ---------------- САМОТЕСТ ПРИ СТАРТЕ -----------------------
void selfTest() {
  // серво: прогон по сектору
  turret.write(90); delay(400);
  turret.write(SWEEP_MIN); delay(400);
  turret.write(SWEEP_MAX); delay(600);
  turret.write(90); delay(400);
  // лазер: два коротких мигания (визуальная проверка подключения)
  for (int i = 0; i < 2; i++) {
    digitalWrite(PIN_LASER, HIGH); delay(120);
    digitalWrite(PIN_LASER, LOW);  delay(120);
  }
  // датчик: 10 контрольных замеров
  int ok = 0, minCm = 999, maxCm = 0;
  for (int i = 0; i < 10; i++) {
    int d = measureCm();
    if (d < 999) { ok++; if (d < minCm) minCm = d; if (d > maxCm) maxCm = d; }
    delay(40);
  }
#if !RADAR_GUI
  if (ok == 0) {
    Serial.println(F("ОШИБКА САМОТЕСТА: датчик не дал ни одного эха."));
    Serial.println(F("  Проверьте: TRIG->D3, ECHO->D2, VCC->5V, GND; не JSN-SR04T ли у вас (MIN_VALID_CM=25)"));
    tone(PIN_BUZZER, 300, 400);
  } else {
    Serial.print(F("Самотест: датчик OK (эхо в ")); Serial.print(ok);
    Serial.print(F("/10 замеров, ")); Serial.print(minCm);
    Serial.print(F("..")); Serial.print(maxCm); Serial.println(F(" см)"));
  }
#endif
  tone(PIN_BUZZER, 880, 90); delay(120);
  tone(PIN_BUZZER, 1320, 90); delay(150);
}

// ---------------- КОНТРОЛЬ ДАТЧИКА В РАБОТЕ -----------------
void sensorWatch() {
  unsigned long now = millis();
  if (now - lastEchoMs > (unsigned long)SENSOR_FAIL_S * 1000UL) {
    if (!sensorAlarm || now - lastAlarmMs > 60000UL) {
      sensorAlarm = true;
      lastAlarmMs = now;
#if !RADAR_GUI
      Serial.println(F("ТРЕВОГА: датчик молчит (нет ни одного эха). Обрыв TRIG/ECHO/питания?"));
#endif
      tone(PIN_BUZZER, 300, 100); delay(140); tone(PIN_BUZZER, 300, 100);
    }
  }
}

void noteEcho() {
  lastEchoMs = millis();
  if (sensorAlarm) {
    sensorAlarm = false;
#if !RADAR_GUI
    Serial.println(F("Датчик снова отвечает — тревога снята"));
#endif
  }
}

// ---------------- ПАТРУЛЬ -----------------------------------
void patrolStep() {
  angle += sweepDir;
  if (angle >= SWEEP_MAX) { angle = SWEEP_MAX; sweepDir = -1; }
  if (angle <= SWEEP_MIN) { angle = SWEEP_MIN; sweepDir =  1; }
  turret.write(angle);

  int d = measureCm();
  report(angle, d);

  if (d >= MIN_VALID_CM && d <= DETECT_DIST_CM) {
    if (++confirmCnt >= CONFIRM_PINGS) {
      // контрольная медиана из 3 — отсекаем одиночный мусор
      int m = measureMedian3();
      if (m >= MIN_VALID_CM && m <= DETECT_DIST_CM) lockTarget();
      else confirmCnt = 0;
    }
  } else {
    confirmCnt = 0;
  }
}

// ---------------- ЗАХВАТ ------------------------------------
void lockTarget() {
  mode = TRACK;
  lockAngle = angle;
  wobble = 0; wobbleDir = 1;
  lostCnt = 0; killLogged = false;
  lockedAt = millis();
  digitalWrite(PIN_LASER, HIGH);
  tone(PIN_BUZZER, 1200, 80);
#if !RADAR_GUI
  Serial.print(F(">>> ЦЕЛЬ ЗАХВАЧЕНА! Азимут "));
  Serial.print(lockAngle); Serial.println(F(" град — лазер наведён"));
#endif
}

// ---------------- СОПРОВОЖДЕНИЕ -----------------------------
void trackStep() {
  wobble += wobbleDir;
  if (wobble >=  TRACK_WOBBLE) wobbleDir = -1;
  if (wobble <= -TRACK_WOBBLE) wobbleDir =  1;
  int a = constrain(lockAngle + wobble, SWEEP_MIN, SWEEP_MAX);
  turret.write(a);

  int d = measureCm();
  report(a, d);

  if (d >= MIN_VALID_CM && d <= LOST_DIST_CM) {
    lostCnt = 0;
    lockAngle = a;
    if (!killLogged && millis() - lockedAt >= KILL_HOLD_MS) {
      kills++;
      killLogged = true;
#if USE_EEPROM
      pdata.kills = kills; EEPROM.put(0, pdata);
#endif
      victoryTune();
#if !RADAR_GUI
      Serial.print(F("*** Цель N")); Serial.print(kills);
      Serial.println(F(" поражена. Занесена в журнал ***"));
#endif
    }
  } else if (++lostCnt >= LOST_PINGS) {
    releaseTarget();
  }
}

// ---------------- ПОТЕРЯ ------------------------------------
void releaseTarget() {
  digitalWrite(PIN_LASER, LOW);
  angle = lockAngle;
  mode = PATROL;
  confirmCnt = 0;
  tone(PIN_BUZZER, 400, 120);
#if !RADAR_GUI
  Serial.println(F("<<< Цель потеряна — возвращаюсь к патрулированию"));
#endif
}

// ---------------- ЗАМЕРЫ ------------------------------------
int measureCm() {
  digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long us = pulseIn(PIN_ECHO, HIGH, PULSE_TIMEOUT_US);
  if (us == 0) return 999;
  noteEcho();
  int cm = (int)(us / 58);
  return (cm < 1) ? 999 : cm;
}

int measureMedian3() {
  int a = measureCm(); delay(30);
  int b = measureCm(); delay(30);
  int c = measureCm();
  // медиана трёх
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { int t = b; b = c; c = t; }
  if (a > b) { int t = a; a = b; b = t; }
  return b;
}

// ---------------- ТЕЛЕМЕТРИЯ / ЗВУК -------------------------
void report(int a, int d) {
#if RADAR_GUI
  Serial.print(a); Serial.print(','); Serial.print(d); Serial.print('.');
#else
  (void)a; (void)d;
#endif
}

void victoryTune() {
  tone(PIN_BUZZER, 1568, 100); delay(120);
  tone(PIN_BUZZER, 1245, 100); delay(120);
  tone(PIN_BUZZER, 1047, 180);
}

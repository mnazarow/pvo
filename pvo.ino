/*
 * ============================================================
 *  ПВО-1К «Комар» — полная прошивка Uno/Nano (варианты A и C)
 * ============================================================
 *  Uno/Nano + HC-SR04 + серво + лазер <=5 мВт + буззер.
 *  Автономная логика варианта A + режим SLAVE (внешнее
 *  целеуказание от Raspberry Pi) + протокол удалённой настройки
 *  для GUI-приложения pvo_gui.py (Windows/Mac).
 *  Минимальная учебная версия без всего этого — pvo_a_simple.ino.
 *
 *  РЕЖИМЫ:  ПАТРУЛЬ ⇄ СОПРОВОЖДЕНИЕ (автономно), SLAVE (внешн.)
 *
 *  ПРОТОКОЛ (Serial 9600, строки с \n; используется и GUI):
 *   P<угол>       навести на угол, лазер ВКЛ, режим SLAVE
 *   R             сброс SLAVE -> автономный патруль
 *   ?             статус одной строкой
 *   G             выдать все параметры: PARAM ИМЯ=ЗНАЧ … PARAM END
 *   S ИМЯ ЗНАЧ    установить параметр (проверка диапазона)
 *   W             сохранить параметры и журнал в EEPROM
 *   D             вернуть параметры по умолчанию (в ОЗУ)
 *   M1 / M0       вкл/выкл поток телеметрии TL … (5 раз/с)
 *
 *  ОБРАБОТКА ОШИБОК: самотест старта; контроль датчика в работе
 *  (тревога при молчании > SENSOR_FAIL_S, автоснятие); медианная
 *  проверка захвата; валидация команд и параметров; защита
 *  буфера; счётчик загрузок в EEPROM (диагностика питания).
 *
 *  ЗАМЕНЫ КОМПОНЕНТОВ — таблицы в руководстве. Ключевое:
 *  JSN-SR04T -> S MINVALID 25; дальнобойные датчики -> таймаут
 *  PULSE_TIMEOUT_US; серво SG90/MG90S/MG996R — без изменений.
 * ============================================================
 */

#include <Servo.h>

#define FW_VERSION "1.2"
#define RADAR_GUI 0        // 1 = поток "угол,дистанция." для Processing-радара
#define USE_EEPROM 1

#if USE_EEPROM
#include <EEPROM.h>
#endif

// ========================= ПИНЫ =============================
const uint8_t PIN_SERVO  = 9;
const uint8_t PIN_TRIG   = 3;
const uint8_t PIN_ECHO   = 2;
const uint8_t PIN_LASER  = 7;
const uint8_t PIN_BUZZER = 8;

// ---- Озвучка DFPlayer Mini (опция, библиотека не нужна) ----
// Дорожки на microSD: /mp3/0001.mp3 старт, 0002 захват,
// 0003 «поражение», 0004 потеря, 0005 тревога датчика.
#ifndef USE_DFPLAYER
#define USE_DFPLAYER 0     // 1 = включить (D11 -> RX плеера через 1 кОм)
#endif
#if USE_DFPLAYER
#include <SoftwareSerial.h>
SoftwareSerial dfSerial(12, 11);   // RX (не используется), TX = D11
const uint8_t DF_VOLUME = 25;      // 0..30
void dfSend(uint8_t cmd, uint16_t param) {
  uint8_t f[10] = {0x7E, 0xFF, 0x06, cmd, 0x00,
                   (uint8_t)(param >> 8), (uint8_t)param, 0, 0, 0xEF};
  int16_t sum = 0;
  for (int i = 1; i < 7; i++) sum += f[i];
  sum = -sum;
  f[7] = (uint8_t)(sum >> 8);
  f[8] = (uint8_t)sum;
  dfSerial.write(f, 10);
}
void dfPlay(uint16_t track) { dfSend(0x03, track); }
void dfSetup() {
  dfSerial.begin(9600);
  delay(600);
  dfSend(0x06, DF_VOLUME);
  delay(80);
  dfPlay(1);
}
#else
void dfPlay(uint16_t) {}
#endif

// ============ ПАРАМЕТРЫ (настраиваются командой S) ==========
int      SWEEP_MIN      = 15;    // S SWMIN
int      SWEEP_MAX      = 165;   // S SWMAX
int      DETECT_DIST_CM = 60;    // S DETECT
int      LOST_DIST_CM   = 90;    // S LOST
int      MIN_VALID_CM   = 3;     // S MINVALID (JSN-SR04T: 25)
int      CONFIRM_PINGS  = 3;     // S CONFIRM
int      LOST_PINGS     = 12;    // S LOSTP
int      TRACK_WOBBLE   = 5;     // S WOBBLE
unsigned int STEP_MS        = 30;    // S STEP
unsigned int KILL_HOLD_MS   = 3000;  // S KILLMS
unsigned int EXT_TIMEOUT_MS = 700;   // S EXTMS
unsigned long TRACK_MAX_MS   = 60000; // S TRACKMAX: лимит непрерывного
                                      // сопровождения, 0 = без лимита
unsigned long TRACK_COOLDOWN_MS = 10000; // S TRACKCD: пауза после лимита

// ================== НЕИЗМЕНЯЕМЫЕ КОНСТАНТЫ ==================
const unsigned long PULSE_TIMEOUT_US = 20000UL; // JSN-SR04T: 30000
const unsigned int  SENSOR_FAIL_S    = 30;

// ======================= СОСТОЯНИЕ ==========================
enum Mode { PATROL, TRACK, SLAVE };
Mode mode = PATROL;

Servo turret;
int  angle = 15, sweepDir = 1;
int  lockAngle = 90, wobble = 0, wobbleDir = 1;
int  confirmCnt = 0, lostCnt = 0;
unsigned long lastStep = 0, lockedAt = 0;
unsigned long cooldownUntil = 0;   // пауза после лимита сопровождения
bool killLogged = false;
unsigned int kills = 0;

int  extAngle = 90;
unsigned long lastExtMs = 0;
char inBuf[20];
uint8_t inLen = 0;

unsigned long lastEchoMs = 0, lastAlarmMs = 0;
bool sensorAlarm = false;

bool telemetryOn = false;
unsigned long lastTlMs = 0;
int lastDist = 999;

#if USE_EEPROM
struct Persist {
  uint16_t magic, boots, kills;
  int16_t detect, lost, minValid, confirm, lostP, wobble, swMin, swMax;
  uint16_t stepMs, killMs, extMs;
  uint32_t trackMax, trackCd;
};
const uint16_t MAGIC = 0x1D09;   // сменён: в структуру добавлены TRACKMAX/TRACKCD
Persist pdata;
#endif

// Прототипы
void patrolStep(); void trackStep(); void slaveStep();
void lockTarget(); void releaseTarget(bool byLimit = false); void logKill();
void pollSerial(); void handleLine(char *s);
int  measureCm(); int measureMedian3();
void report(int a, int d); void victoryTune();
void selfTest(); void sensorWatch(); void noteEcho();
void printParams(); bool setParam(const char *n, long v);
void setDefaults(); void saveAll(); void emitTelemetry();

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
  if (pdata.magic == MAGIC) {          // параметры из EEPROM
    kills = pdata.kills;
    DETECT_DIST_CM = pdata.detect;  LOST_DIST_CM = pdata.lost;
    MIN_VALID_CM = pdata.minValid;  CONFIRM_PINGS = pdata.confirm;
    LOST_PINGS = pdata.lostP;       TRACK_WOBBLE = pdata.wobble;
    SWEEP_MIN = pdata.swMin;        SWEEP_MAX = pdata.swMax;
    STEP_MS = pdata.stepMs;         KILL_HOLD_MS = pdata.killMs;
    EXT_TIMEOUT_MS = pdata.extMs;
    TRACK_MAX_MS = pdata.trackMax;  TRACK_COOLDOWN_MS = pdata.trackCd;
  } else {
    pdata.magic = MAGIC; pdata.boots = 0; pdata.kills = 0;
  }
  pdata.boots++;
  saveAll();
#endif
  angle = SWEEP_MIN;

  selfTest();
#if USE_DFPLAYER
  dfSetup();
#endif

#if !RADAR_GUI
  Serial.println(F("=== ПВО-1К \"Комар\" (полная прошивка) на боевом дежурстве ==="));
  Serial.println(F("Версия прошивки: " FW_VERSION));
#if USE_EEPROM
  Serial.print(F("Загрузка N")); Serial.print(pdata.boots);
  Serial.print(F(", журнал за всё время: ")); Serial.println(kills);
  if (pdata.boots > 3)
    Serial.println(F("Подсказка: если счётчик загрузок растёт сам — проверьте питание серво"));
#endif
  Serial.println(F("Команды: P<угол> R ? G S W D Z M1/M0 (подробнее — руководство)"));
#endif
  lastEchoMs = millis();
}

// ============================================================
void loop() {
  pollSerial();

  unsigned long now = millis();
  if (now - lastStep < STEP_MS) return;
  lastStep = now;

  if (mode != SLAVE) sensorWatch();

  if (mode != PATROL && TRACK_MAX_MS > 0 && now - lockedAt > TRACK_MAX_MS) {
    cooldownUntil = now + TRACK_COOLDOWN_MS;   // пауза перед новым захватом
    releaseTarget(true);
  }

  if      (mode == PATROL) patrolStep();
  else if (mode == TRACK)  trackStep();
  else                     slaveStep();

  emitTelemetry();
}

// ---------------- САМОТЕСТ ----------------------------------
void selfTest() {
  turret.write(90); delay(400);
  turret.write(SWEEP_MIN); delay(400);
  turret.write(SWEEP_MAX); delay(600);
  turret.write(90); delay(400);
  for (int i = 0; i < 2; i++) {
    digitalWrite(PIN_LASER, HIGH); delay(120);
    digitalWrite(PIN_LASER, LOW);  delay(120);
  }
  int ok = 0;
  for (int i = 0; i < 10; i++) { if (measureCm() < 999) ok++; delay(40); }
#if !RADAR_GUI
  if (ok == 0) {
    Serial.println(F("ОШИБКА САМОТЕСТА: датчик не дал ни одного эха (TRIG->D3, ECHO->D2, 5V, GND?)"));
    Serial.println(F("  Для связки с Raspberry Pi (SLAVE) турель всё равно работоспособна."));
    tone(PIN_BUZZER, 300, 400);
  } else {
    Serial.print(F("Самотест: датчик OK (эхо в ")); Serial.print(ok); Serial.println(F("/10 замеров)"));
  }
#endif
  tone(PIN_BUZZER, 880, 90); delay(120);
  tone(PIN_BUZZER, 1320, 90); delay(150);
}

// ---------------- ЗДОРОВЬЕ ДАТЧИКА --------------------------
void sensorWatch() {
  unsigned long now = millis();
  if (now - lastEchoMs > (unsigned long)SENSOR_FAIL_S * 1000UL) {
    if (!sensorAlarm || now - lastAlarmMs > 60000UL) {
      sensorAlarm = true;
      lastAlarmMs = now;
#if !RADAR_GUI
      Serial.println(F("ТРЕВОГА: датчик молчит (нет ни одного эха). Обрыв TRIG/ECHO/питания?"));
#endif
      dfPlay(5);
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

// ---------------- ПРИЁМ КОМАНД ------------------------------
void pollSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inLen > 0) { inBuf[inLen] = '\0'; handleLine(inBuf); inLen = 0; }
    } else if (inLen < sizeof(inBuf) - 1) {
      inBuf[inLen++] = c;
    } else {
      inLen = 0;
    }
  }
}

void handleLine(char *s) {
  if (s[0] == 'P' && s[1] >= '0' && s[1] <= '9') {
    int a = atoi(s + 1);
    if (a < SWEEP_MIN) a = SWEEP_MIN;
    if (a > SWEEP_MAX) a = SWEEP_MAX;
    extAngle  = a;
    lastExtMs = millis();
    if (mode != SLAVE && millis() >= cooldownUntil) {
      mode = SLAVE;
      killLogged = false;
      lockedAt = millis();
      digitalWrite(PIN_LASER, HIGH);
      tone(PIN_BUZZER, 1200, 80);
#if !RADAR_GUI
      Serial.println(F(">>> Внешнее целеуказание принято (SLAVE)"));
#endif
    }
  } else if (s[0] == 'R' && s[1] == '\0') {
    if (mode == SLAVE) releaseTarget();
  } else if (s[0] == '?' ) {
#if !RADAR_GUI
    Serial.print(F("СТАТУС: режим="));
    Serial.print(mode == PATROL ? F("ПАТРУЛЬ")
                : mode == TRACK ? F("СОПРОВОЖДЕНИЕ") : F("SLAVE"));
    Serial.print(F(", журнал=")); Serial.print(kills);
    Serial.print(F(", датчик=")); Serial.print(sensorAlarm ? F("МОЛЧИТ") : F("OK"));
    if (millis() < cooldownUntil) {
      Serial.print(F(", ПАУЗА ещё "));
      Serial.print((cooldownUntil - millis()) / 1000); Serial.print(F(" c"));
    }
#if USE_EEPROM
    Serial.print(F(", загрузок=")); Serial.print(pdata.boots);
#endif
    Serial.println(F(", прошивка=" FW_VERSION));
#endif
  } else if (s[0] == 'G' && s[1] == '\0') {
    printParams();
  } else if (s[0] == 'S' && s[1] == ' ') {
    char *name = s + 2;
    char *sp = strchr(name, ' ');
    if (sp == NULL) {
      Serial.println(F("ERR формат: S ИМЯ ЗНАЧЕНИЕ"));
      return;
    }
    *sp = '\0';
    long v = atol(sp + 1);
    if (setParam(name, v)) {
      Serial.print(F("OK ")); Serial.print(name);
      Serial.print(F("=")); Serial.println(v);
    } else {
      Serial.print(F("ERR имя или диапазон: ")); Serial.println(name);
    }
  } else if (s[0] == 'W' && s[1] == '\0') {
#if USE_EEPROM
    saveAll();
    Serial.println(F("OK SAVED (параметры и журнал в EEPROM)"));
#else
    Serial.println(F("ERR EEPROM отключён (USE_EEPROM 0)"));
#endif
  } else if (s[0] == 'D' && s[1] == '\0') {
    setDefaults();
    Serial.println(F("OK DEFAULTS (в ОЗУ; W — сохранить)"));
  } else if (s[0] == 'Z' && s[1] == '\0') {
    kills = 0;
#if USE_EEPROM
    saveAll();
#endif
    Serial.println(F("OK Z (журнал поражений обнулён)"));
  } else if (s[0] == 'M' && (s[1] == '0' || s[1] == '1')) {
    telemetryOn = (s[1] == '1');
    Serial.println(telemetryOn ? F("OK M1") : F("OK M0"));
  } else {
#if !RADAR_GUI
    Serial.println(F("ОШИБКА КОМАНДЫ: известны P<угол> R ? G S W D Z M1/M0"));
#endif
  }
}

// ---------------- ПАРАМЕТРЫ: G / S / W / D ------------------
void printParams() {
  Serial.print(F("PARAM DETECT="));   Serial.println(DETECT_DIST_CM);
  Serial.print(F("PARAM LOST="));     Serial.println(LOST_DIST_CM);
  Serial.print(F("PARAM MINVALID=")); Serial.println(MIN_VALID_CM);
  Serial.print(F("PARAM CONFIRM=")); Serial.println(CONFIRM_PINGS);
  Serial.print(F("PARAM LOSTP="));    Serial.println(LOST_PINGS);
  Serial.print(F("PARAM WOBBLE="));   Serial.println(TRACK_WOBBLE);
  Serial.print(F("PARAM STEP="));     Serial.println(STEP_MS);
  Serial.print(F("PARAM KILLMS="));   Serial.println(KILL_HOLD_MS);
  Serial.print(F("PARAM EXTMS="));    Serial.println(EXT_TIMEOUT_MS);
  Serial.print(F("PARAM TRACKMAX=")); Serial.println(TRACK_MAX_MS);
  Serial.print(F("PARAM TRACKCD="));  Serial.println(TRACK_COOLDOWN_MS);
  Serial.print(F("PARAM SWMIN="));    Serial.println(SWEEP_MIN);
  Serial.print(F("PARAM SWMAX="));    Serial.println(SWEEP_MAX);
  Serial.println(F("PARAM END"));
}

bool inRange(long v, long lo, long hi) { return v >= lo && v <= hi; }

bool setParam(const char *n, long v) {
  if      (!strcmp(n, "DETECT")   && inRange(v, 5, 300))  DETECT_DIST_CM = v;
  else if (!strcmp(n, "LOST")     && inRange(v, 5, 400))  LOST_DIST_CM = v;
  else if (!strcmp(n, "MINVALID") && inRange(v, 1, 50))   MIN_VALID_CM = v;
  else if (!strcmp(n, "CONFIRM")  && inRange(v, 1, 10))   CONFIRM_PINGS = v;
  else if (!strcmp(n, "LOSTP")    && inRange(v, 1, 50))   LOST_PINGS = v;
  else if (!strcmp(n, "WOBBLE")   && inRange(v, 1, 20))   TRACK_WOBBLE = v;
  else if (!strcmp(n, "STEP")     && inRange(v, 20, 100)) STEP_MS = v;
  else if (!strcmp(n, "KILLMS")   && inRange(v, 200, 20000)) KILL_HOLD_MS = v;
  else if (!strcmp(n, "EXTMS")    && inRange(v, 200, 5000))  EXT_TIMEOUT_MS = v;
  else if (!strcmp(n, "TRACKMAX") && (v == 0 || inRange(v, 5000, 600000))) TRACK_MAX_MS = v;
  else if (!strcmp(n, "TRACKCD")  && inRange(v, 0, 120000))  TRACK_COOLDOWN_MS = v;
  else if (!strcmp(n, "SWMIN")    && inRange(v, 0, 170) && v < SWEEP_MAX) SWEEP_MIN = v;
  else if (!strcmp(n, "SWMAX")    && inRange(v, 10, 180) && v > SWEEP_MIN) SWEEP_MAX = v;
  else return false;
  return true;
}

void setDefaults() {
  DETECT_DIST_CM = 60; LOST_DIST_CM = 90; MIN_VALID_CM = 3;
  CONFIRM_PINGS = 3; LOST_PINGS = 12; TRACK_WOBBLE = 5;
  STEP_MS = 30; KILL_HOLD_MS = 3000; EXT_TIMEOUT_MS = 700;
  SWEEP_MIN = 15; SWEEP_MAX = 165;
  TRACK_MAX_MS = 60000; TRACK_COOLDOWN_MS = 10000;
}

void saveAll() {
#if USE_EEPROM
  pdata.kills = kills;
  pdata.detect = DETECT_DIST_CM;  pdata.lost = LOST_DIST_CM;
  pdata.minValid = MIN_VALID_CM;  pdata.confirm = CONFIRM_PINGS;
  pdata.lostP = LOST_PINGS;       pdata.wobble = TRACK_WOBBLE;
  pdata.swMin = SWEEP_MIN;        pdata.swMax = SWEEP_MAX;
  pdata.stepMs = STEP_MS;         pdata.killMs = KILL_HOLD_MS;
  pdata.extMs = EXT_TIMEOUT_MS;
  pdata.trackMax = TRACK_MAX_MS;  pdata.trackCd = TRACK_COOLDOWN_MS;
  EEPROM.put(0, pdata);
#endif
}

// ---------------- ТЕЛЕМЕТРИЯ ДЛЯ GUI ------------------------
void emitTelemetry() {
  if (!telemetryOn) return;
  unsigned long now = millis();
  if (now - lastTlMs < 200) return;
  lastTlMs = now;
  int a = (mode == PATROL) ? angle : (mode == TRACK ? lockAngle + wobble : extAngle);
  Serial.print(F("TL m="));
  Serial.print(mode == PATROL ? 'P' : (mode == TRACK ? 'T' : 'S'));
  Serial.print(F(" a=")); Serial.print(a);
  Serial.print(F(" d=")); Serial.print(lastDist);
  Serial.print(F(" k=")); Serial.print(kills);
  Serial.print(F(" s=")); Serial.println(sensorAlarm ? 1 : 0);
}

// ---------------- SLAVE -------------------------------------
void slaveStep() {
  if (millis() - lastExtMs > EXT_TIMEOUT_MS) {
#if !RADAR_GUI
    Serial.println(F("Командир молчит — перехожу в автономию"));
#endif
    releaseTarget();
    return;
  }
  turret.write(extAngle);
  int d = measureCm();
  report(extAngle, d);
  if (!killLogged && millis() - lockedAt >= KILL_HOLD_MS) logKill();
}

// ---------------- ПАТРУЛЬ -----------------------------------
void patrolStep() {
  angle += sweepDir;
  if (angle >= SWEEP_MAX) { angle = SWEEP_MAX; sweepDir = -1; }
  if (angle <= SWEEP_MIN) { angle = SWEEP_MIN; sweepDir =  1; }
  turret.write(angle);

  int d = measureCm();
  report(angle, d);

  if (millis() < cooldownUntil) {
    confirmCnt = 0;                       // пауза после лимита сопровождения
  } else if (d >= MIN_VALID_CM && d <= DETECT_DIST_CM) {
    if (++confirmCnt >= CONFIRM_PINGS) {
      int m = measureMedian3();
      if (m >= MIN_VALID_CM && m <= DETECT_DIST_CM) lockTarget();
      else confirmCnt = 0;
    }
  } else {
    confirmCnt = 0;
  }
}

// ---------------- ЗАХВАТ / СОПРОВОЖДЕНИЕ --------------------
void lockTarget() {
  mode = TRACK;
  lockAngle = angle;
  wobble = 0; wobbleDir = 1;
  lostCnt = 0; killLogged = false;
  lockedAt = millis();
  digitalWrite(PIN_LASER, HIGH);
  dfPlay(2);
  tone(PIN_BUZZER, 1200, 80);
#if !RADAR_GUI
  Serial.print(F(">>> ЦЕЛЬ ЗАХВАЧЕНА! Азимут "));
  Serial.print(lockAngle); Serial.println(F(" град — лазер наведён"));
#endif
}

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
    if (!killLogged && millis() - lockedAt >= KILL_HOLD_MS) logKill();
  } else if (++lostCnt >= LOST_PINGS) {
    releaseTarget();
  }
}

void logKill() {
  kills++;
  killLogged = true;
#if USE_EEPROM
  saveAll();
#endif
  dfPlay(3);
  victoryTune();
#if !RADAR_GUI
  Serial.print(F("*** Цель N")); Serial.print(kills);
  Serial.println(F(" поражена. Занесена в журнал ***"));
#endif
}

void releaseTarget(bool byLimit) {
  digitalWrite(PIN_LASER, LOW);
  angle = (mode == SLAVE) ? extAngle : lockAngle;
  if (angle < SWEEP_MIN) angle = SWEEP_MIN;
  if (angle > SWEEP_MAX) angle = SWEEP_MAX;
  mode = PATROL;
  confirmCnt = 0;
  dfPlay(4);
  tone(PIN_BUZZER, 400, 120);
#if !RADAR_GUI
  if (byLimit)
    Serial.println(F("<<< Лимит непрерывного сопровождения — пауза, лазер погашен"));
  else
    Serial.println(F("<<< Цель потеряна — возвращаюсь к патрулированию"));
#endif
}

// ---------------- ЗАМЕРЫ ------------------------------------
int measureCm() {
  digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long us = pulseIn(PIN_ECHO, HIGH, PULSE_TIMEOUT_US);
  if (us == 0) { lastDist = 999; return 999; }
  noteEcho();
  int cm = (int)(us / 58);
  if (cm < 1) cm = 999;
  lastDist = cm;
  return cm;
}

int measureMedian3() {
  int a = measureCm(); delay(30);
  int b = measureCm(); delay(30);
  int c = measureCm();
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

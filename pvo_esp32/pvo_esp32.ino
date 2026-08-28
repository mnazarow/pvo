/*
 * ============================================================
 *  ПВО-2К «Комар-М» — ВАРИАНТ B: ESP32 + mmWave-радар
 * ============================================================
 *  ESP32 DevKit + LD2450/RD-03D + 2 серво (pan-tilt) + лазер
 *  <=5 мВт через транзистор + буззер. Ядро Arduino-ESP32 3.x,
 *  библиотека ESP32Servo. Схема и сборка — «Сборка варианта B».
 *
 *  ЛОГИКА: ДЕЖУРСТВО (ленивый патруль) ⇄ СОПРОВОЖДЕНИЕ.
 *
 *  ПРОТОКОЛ (Монитор порта / GUI pvo_gui.py, 115200, строки \n):
 *   ?             статус
 *   C             серво в центр на 8 с (юстировка)
 *   L1 / L0       тест лазера (только на дежурстве)
 *   T             повторная инициализация радара
 *   G             все параметры: PARAM ИМЯ=ЗНАЧ … PARAM END
 *   S ИМЯ ЗНАЧ    установить параметр (дроби ×100: S ALPHA 35)
 *   W             сохранить параметры и журнал в NVS
 *   D             параметры по умолчанию (в ОЗУ)
 *   M1 / M0       поток телеметрии TL … (5 раз/с)
 *   V dP dT       точная поправка от зрения, в десятых градуса
 *                 (вариант D: камера на турели; при дежурстве —
 *                 захват зрением, радар не обязателен)
 *   I1 / I0       ИК-подсветка вручную (GPIO18 через ключ)
 *
 *  ОБРАБОТКА ОШИБОК: самотест; контроль связи с радаром
 *  (тревога, автопереинициализация RD-03D, автоснятие);
 *  счётчики битых кадров с подсказкой; проверка attach() серво;
 *  валидация целей и параметров; журнал в NVS.
 *
 *  ЗАМЕНЫ КОМПОНЕНТОВ — таблицы в руководстве. Ключевое:
 *  LD2450 <-> RD-03D: RADAR_TYPE ниже; серво SG90/MG90S/MG996R —
 *  без изменений (MG996R — БП 5В/2А); транзистор 2N2222/BC337/
 *  S8050 — без изменений, BC547 — другая цоколёвка, MOSFET
 *  2N7000 — без резистора базы; LD2410 НЕ подходит (нет координат).
 * ============================================================
 */

#include <ESP32Servo.h>
#include <math.h>

#define FW_VERSION "1.2"
#define RADAR_TYPE 1   // 0 = HLK-LD2450, 1 = Ai-Thinker RD-03D
#define USE_NVS 1

// ---- Wi-Fi веб-панель (радар и настройки в браузере телефона) ----
#ifndef WIFI_ENABLED
#define WIFI_ENABLED 1     // 0 = полностью выключить веб
#endif
#define WIFI_AP_MODE 1     // 1 = турель раздаёт свою сеть, 0 = подключается к домашней
const char *WIFI_SSID = "PVO-Komar";    // AP: имя своей сети / STA: имя домашней сети
const char *WIFI_PASS = "komar12345";   // минимум 8 символов
const char *WEB_USER = "";              // логин веб-панели ("" = без пароля)
const char *WEB_PASS = "";              // пароль веб-панели

// ---- OLED-экран SSD1306 128x64 по I2C (опция, нужна библиотека) ----
#ifndef USE_OLED
#define USE_OLED 0         // 1 = включить (Менеджер библиотек: Adafruit SSD1306 + GFX)
#endif

// ---- Озвучка DFPlayer Mini (опция, библиотека не нужна) ----
#ifndef USE_DFPLAYER
#define USE_DFPLAYER 0     // 1 = включить (TX ESP32 GPIO33 -> RX DFPlayer через 1 кОм)
#endif

#if USE_NVS
#include <Preferences.h>
Preferences prefs;
#endif

#if WIFI_ENABLED
#include <WiFi.h>
#include <WebServer.h>
WebServer web(80);
#endif

#if USE_OLED
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
bool oledOk = false;
uint32_t lastOledMs = 0;
#endif

// ========================= ПИНЫ =============================
const int PIN_RADAR_RX = 16;
const int PIN_RADAR_TX = 17;
const int PIN_SERVO_PAN  = 26;
const int PIN_SERVO_TILT = 27;
const int PIN_LASER  = 23;
const int PIN_BUZZER = 19;
const int PIN_IR     = 18;   // ИК-подсветка через ключ (вариант D)

// ============ ПАРАМЕТРЫ (настраиваются командой S) ==========
int   PAN_CENTER  = 90;    // S PANC
int   PAN_MIN     = 15;    // S PANMIN
int   PAN_MAX     = 165;   // S PANMAX
int   TILT_CENTER = 90;    // S TILTC
int   TILT_MIN    = 60;    // S TILTMIN
int   TILT_MAX    = 120;   // S TILTMAX
bool  PAN_INVERT  = false; // S PANINV 0/1
bool  TILT_INVERT = false; // S TILTINV 0/1
bool  AUTO_TILT   = true;  // S ATILT 0/1
int   TURRET_HEIGHT_MM = 750;   // S TURH
int   TARGET_HEIGHT_MM = 1100;  // S TGTH
int   MIN_RANGE_MM = 300;   // S MINR
int   MAX_RANGE_MM = 4000;  // S MAXR
float MAX_AZ_DEG   = 60.0f; // S MAXAZ (в градусах)
int   LOCK_CONFIRM_FRAMES = 5;    // S CONFIRM
uint32_t LOST_TIMEOUT_MS  = 700;  // S LOSTMS
uint32_t KILL_HOLD_MS     = 3000; // S KILLMS
float SMOOTH_ALPHA = 0.35f; // S ALPHA (×100: 35)
float SLEW_DEG_MAX = 6.0f;  // S SLEW
bool  PATROL_ENABLED = true;   // S PATROL 0/1
float PATROL_SPEED   = 0.35f;  // S PSPEED (×100: 35)
bool  IR_AUTO        = true;   // S IRAUTO 0/1: ИК сам при захвате
uint32_t VISION_TIMEOUT_MS = 700; // S VISTO: тишина зрения -> радар

// ================== НЕИЗМЕНЯЕМОЕ ============================
const uint32_t SERVO_PERIOD_MS = 20;
#define TARGET_SELECT 0   // 0 = ближайшая, 1 = самая быстрая
const uint32_t RADAR_SILENT_MS = 2000;
const uint32_t RADAR_RETRY_MS  = 3000;
const uint32_t RADAR_HINT_MS   = 15000;
const float    BAD_FRAME_RATIO = 0.2f;

// ============================================================
struct RadarTarget { bool valid; int16_t x_mm, y_mm, speed_cms; uint16_t res_mm; };

enum Mode { IDLE, TRACK };
Mode mode = IDLE;

Servo servoPan, servoTilt;
RadarTarget targets[3];
uint8_t frameBuf[30];
int frameFill = 0;

float panNow = 90, panGoal = 90, tiltNow = 90, tiltGoal = 90;
float patrolDir = 1.0f;

int confirmCnt = 0;
uint32_t lastSeenMs = 0, lockedAtMs = 0, lastServoMs = 0;
bool killLogged = false;
unsigned kills = 0;
uint16_t boots = 0;

uint32_t lastFrameMs = 0, lastRetryMs = 0, lastHintMs = 0;
bool radarAlarm = false;
uint32_t framesOk = 0, framesBad = 0, framesOkAtStat = 0, statMs = 0;

char conBuf[24];
uint8_t conLen = 0;
uint32_t holdUntil = 0;
bool laserTest = false;

bool telemetryOn = false;
uint32_t lastTlMs = 0;
int tlX = 0, tlY = 0, tlD = 0;   // последняя цель для телеметрии

// --- слияние с зрением (вариант D) ---
bool visionActive = false;       // свежие поправки V от камеры
uint32_t lastVisionMs = 0;
bool irOn = false;

const uint8_t MULTI_CMD[12] =
  {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x90, 0x00, 0x04, 0x03, 0x02, 0x01};

void dfPlay(uint16_t track);   // озвучка (пустышка, если USE_DFPLAYER 0)

// ------------------------------------------------------------
int16_t radarInt16(uint8_t lo, uint8_t hi) {
  int16_t mag = (int16_t)((((uint16_t)hi << 8) | lo) & 0x7FFF);
  return (hi & 0x80) ? mag : (int16_t)-mag;
}

void parseFrame(const uint8_t *b) {
  for (int i = 0; i < 3; i++) {
    const uint8_t *t = b + 4 + i * 8;
    bool empty = true;
    for (int k = 0; k < 8; k++) if (t[k] != 0) { empty = false; break; }
    targets[i].valid = !empty;
    if (empty) continue;
    targets[i].x_mm      = radarInt16(t[0], t[1]);
    targets[i].y_mm      = radarInt16(t[2], t[3]);
    targets[i].speed_cms = radarInt16(t[4], t[5]);
    targets[i].res_mm    = (uint16_t)t[6] | ((uint16_t)t[7] << 8);
  }
}

bool readRadar() {
  static const uint8_t HDR[4] = {0xAA, 0xFF, 0x03, 0x00};
  bool gotFrame = false;
  while (Serial2.available()) {
    uint8_t c = (uint8_t)Serial2.read();
    if (frameFill < 4) {
      if (c == HDR[frameFill]) frameBuf[frameFill++] = c;
      else frameFill = (c == HDR[0]) ? 1 : 0;
      continue;
    }
    frameBuf[frameFill++] = c;
    if (frameFill == 30) {
      if (frameBuf[28] == 0x55 && frameBuf[29] == 0xCC) {
        parseFrame(frameBuf);
        gotFrame = true;
        framesOk++;
      } else framesBad++;
      frameFill = 0;
    }
  }
  return gotFrame;
}

int pickTarget() {
  int best = -1;
  float bestKey = 1e9f;
  for (int i = 0; i < 3; i++) {
    if (!targets[i].valid) continue;
    float dist = sqrtf((float)targets[i].x_mm * targets[i].x_mm +
                       (float)targets[i].y_mm * targets[i].y_mm);
    if (!isfinite(dist)) continue;
    float az = fabsf(atan2f((float)targets[i].x_mm, (float)targets[i].y_mm)) * 57.2958f;
    if (dist < MIN_RANGE_MM || dist > MAX_RANGE_MM) continue;
    if (az > MAX_AZ_DEG) continue;
#if TARGET_SELECT == 0
    float key = dist;
#else
    float key = -fabsf((float)targets[i].speed_cms);
#endif
    if (key < bestKey) { bestKey = key; best = i; }
  }
  return best;
}

float panFromTarget(const RadarTarget &t) {
  float az = atan2f((float)t.x_mm, (float)t.y_mm) * 57.2958f;
  float a = PAN_INVERT ? (PAN_CENTER + az) : (PAN_CENTER - az);
  return constrain(a, (float)PAN_MIN, (float)PAN_MAX);
}

float tiltFromTarget(const RadarTarget &t) {
  if (!AUTO_TILT) return (float)TILT_CENTER;
  float dist = sqrtf((float)t.x_mm * t.x_mm + (float)t.y_mm * t.y_mm);
  if (dist < 1) dist = 1;
  float el = atan2f((float)(TARGET_HEIGHT_MM - TURRET_HEIGHT_MM), dist) * 57.2958f;
  float a = TILT_INVERT ? (TILT_CENTER - el) : (TILT_CENTER + el);
  return constrain(a, (float)TILT_MIN, (float)TILT_MAX);
}

float slew(float now, float goal) {
  float next = now + SMOOTH_ALPHA * (goal - now);
  float d = next - now;
  if (d >  SLEW_DEG_MAX) next = now + SLEW_DEG_MAX;
  if (d < -SLEW_DEG_MAX) next = now - SLEW_DEG_MAX;
  return next;
}

void setLaser(bool on) { digitalWrite(PIN_LASER, on ? HIGH : LOW); }
void setIr(bool on) { irOn = on; digitalWrite(PIN_IR, on ? HIGH : LOW); }

void victoryTune() {
  tone(PIN_BUZZER, 1568, 100); delay(120);
  tone(PIN_BUZZER, 1245, 100); delay(120);
  tone(PIN_BUZZER, 1047, 180);
}

void radarInit() {
#if RADAR_TYPE == 1
  Serial2.write(MULTI_CMD, sizeof(MULTI_CMD));
#endif
}

// ---------------- ПАРАМЕТРЫ: G / S / W / D ------------------
void printParam(const char *n, long v) {
  Serial.print(F("PARAM ")); Serial.print(n);
  Serial.print(F("=")); Serial.println(v);
}

void printParams() {
  printParam("PANC", PAN_CENTER);   printParam("PANMIN", PAN_MIN);
  printParam("PANMAX", PAN_MAX);    printParam("TILTC", TILT_CENTER);
  printParam("TILTMIN", TILT_MIN);  printParam("TILTMAX", TILT_MAX);
  printParam("PANINV", PAN_INVERT); printParam("TILTINV", TILT_INVERT);
  printParam("ATILT", AUTO_TILT);   printParam("TURH", TURRET_HEIGHT_MM);
  printParam("TGTH", TARGET_HEIGHT_MM);
  printParam("MINR", MIN_RANGE_MM); printParam("MAXR", MAX_RANGE_MM);
  printParam("MAXAZ", (long)MAX_AZ_DEG);
  printParam("CONFIRM", LOCK_CONFIRM_FRAMES);
  printParam("LOSTMS", LOST_TIMEOUT_MS);
  printParam("KILLMS", KILL_HOLD_MS);
  printParam("ALPHA", (long)(SMOOTH_ALPHA * 100 + 0.5f));
  printParam("SLEW", (long)SLEW_DEG_MAX);
  printParam("PATROL", PATROL_ENABLED);
  printParam("PSPEED", (long)(PATROL_SPEED * 100 + 0.5f));
  printParam("IRAUTO", IR_AUTO);
  printParam("VISTO", VISION_TIMEOUT_MS);
  Serial.println(F("PARAM END"));
}

bool inRangeL(long v, long lo, long hi) { return v >= lo && v <= hi; }

bool setParam(const char *n, long v) {
  if      (!strcmp(n, "PANC")    && inRangeL(v, 30, 150)) PAN_CENTER = v;
  else if (!strcmp(n, "PANMIN")  && inRangeL(v, 0, 170) && v < PAN_MAX)  PAN_MIN = v;
  else if (!strcmp(n, "PANMAX")  && inRangeL(v, 10, 180) && v > PAN_MIN) PAN_MAX = v;
  else if (!strcmp(n, "TILTC")   && inRangeL(v, 30, 150)) TILT_CENTER = v;
  else if (!strcmp(n, "TILTMIN") && inRangeL(v, 0, 170) && v < TILT_MAX)  TILT_MIN = v;
  else if (!strcmp(n, "TILTMAX") && inRangeL(v, 10, 180) && v > TILT_MIN) TILT_MAX = v;
  else if (!strcmp(n, "PANINV")  && inRangeL(v, 0, 1)) PAN_INVERT = v;
  else if (!strcmp(n, "TILTINV") && inRangeL(v, 0, 1)) TILT_INVERT = v;
  else if (!strcmp(n, "ATILT")   && inRangeL(v, 0, 1)) AUTO_TILT = v;
  else if (!strcmp(n, "TURH")    && inRangeL(v, 0, 3000)) TURRET_HEIGHT_MM = v;
  else if (!strcmp(n, "TGTH")    && inRangeL(v, 0, 3000)) TARGET_HEIGHT_MM = v;
  else if (!strcmp(n, "MINR")    && inRangeL(v, 100, 2000)) MIN_RANGE_MM = v;
  else if (!strcmp(n, "MAXR")    && inRangeL(v, 500, 8000)) MAX_RANGE_MM = v;
  else if (!strcmp(n, "MAXAZ")   && inRangeL(v, 10, 60)) MAX_AZ_DEG = v;
  else if (!strcmp(n, "CONFIRM") && inRangeL(v, 1, 30)) LOCK_CONFIRM_FRAMES = v;
  else if (!strcmp(n, "LOSTMS")  && inRangeL(v, 200, 5000)) LOST_TIMEOUT_MS = v;
  else if (!strcmp(n, "KILLMS")  && inRangeL(v, 200, 20000)) KILL_HOLD_MS = v;
  else if (!strcmp(n, "ALPHA")   && inRangeL(v, 5, 100)) SMOOTH_ALPHA = v / 100.0f;
  else if (!strcmp(n, "SLEW")    && inRangeL(v, 1, 20)) SLEW_DEG_MAX = v;
  else if (!strcmp(n, "PATROL")  && inRangeL(v, 0, 1)) PATROL_ENABLED = v;
  else if (!strcmp(n, "PSPEED")  && inRangeL(v, 5, 150)) PATROL_SPEED = v / 100.0f;
  else if (!strcmp(n, "IRAUTO")  && inRangeL(v, 0, 1)) IR_AUTO = v;
  else if (!strcmp(n, "VISTO")   && inRangeL(v, 200, 5000)) VISION_TIMEOUT_MS = v;
  else return false;
  return true;
}

void setDefaults() {
  PAN_CENTER = 90; PAN_MIN = 15; PAN_MAX = 165;
  TILT_CENTER = 90; TILT_MIN = 60; TILT_MAX = 120;
  PAN_INVERT = false; TILT_INVERT = false; AUTO_TILT = true;
  TURRET_HEIGHT_MM = 750; TARGET_HEIGHT_MM = 1100;
  MIN_RANGE_MM = 300; MAX_RANGE_MM = 4000; MAX_AZ_DEG = 60.0f;
  LOCK_CONFIRM_FRAMES = 5; LOST_TIMEOUT_MS = 700; KILL_HOLD_MS = 3000;
  SMOOTH_ALPHA = 0.35f; SLEW_DEG_MAX = 6.0f;
  PATROL_ENABLED = true; PATROL_SPEED = 0.35f;
  IR_AUTO = true; VISION_TIMEOUT_MS = 700;
}

#if USE_NVS
void saveAll() {
  prefs.putUShort("ver", 2);
  prefs.putUShort("boots", boots);
  prefs.putUShort("kills", (uint16_t)kills);
  prefs.putShort("panc", PAN_CENTER);   prefs.putShort("panmin", PAN_MIN);
  prefs.putShort("panmax", PAN_MAX);    prefs.putShort("tiltc", TILT_CENTER);
  prefs.putShort("tiltmin", TILT_MIN);  prefs.putShort("tiltmax", TILT_MAX);
  prefs.putShort("paninv", PAN_INVERT); prefs.putShort("tiltinv", TILT_INVERT);
  prefs.putShort("atilt", AUTO_TILT);   prefs.putShort("turh", TURRET_HEIGHT_MM);
  prefs.putShort("tgth", TARGET_HEIGHT_MM);
  prefs.putShort("minr", MIN_RANGE_MM); prefs.putShort("maxr", MAX_RANGE_MM);
  prefs.putShort("maxaz", (int16_t)MAX_AZ_DEG);
  prefs.putShort("confirm", LOCK_CONFIRM_FRAMES);
  prefs.putShort("lostms", (int16_t)LOST_TIMEOUT_MS);
  prefs.putShort("killms", (int16_t)KILL_HOLD_MS);
  prefs.putShort("alpha", (int16_t)(SMOOTH_ALPHA * 100 + 0.5f));
  prefs.putShort("slew", (int16_t)SLEW_DEG_MAX);
  prefs.putShort("patrol", PATROL_ENABLED);
  prefs.putShort("pspeed", (int16_t)(PATROL_SPEED * 100 + 0.5f));
  prefs.putShort("irauto", IR_AUTO);
  prefs.putShort("visto", (int16_t)VISION_TIMEOUT_MS);
}

void loadAll() {
  if (prefs.getUShort("ver", 0) != 2) return;
  kills = prefs.getUShort("kills", 0);
  PAN_CENTER = prefs.getShort("panc", 90);   PAN_MIN = prefs.getShort("panmin", 15);
  PAN_MAX = prefs.getShort("panmax", 165);   TILT_CENTER = prefs.getShort("tiltc", 90);
  TILT_MIN = prefs.getShort("tiltmin", 60);  TILT_MAX = prefs.getShort("tiltmax", 120);
  PAN_INVERT = prefs.getShort("paninv", 0);  TILT_INVERT = prefs.getShort("tiltinv", 0);
  AUTO_TILT = prefs.getShort("atilt", 1);    TURRET_HEIGHT_MM = prefs.getShort("turh", 750);
  TARGET_HEIGHT_MM = prefs.getShort("tgth", 1100);
  MIN_RANGE_MM = prefs.getShort("minr", 300); MAX_RANGE_MM = prefs.getShort("maxr", 4000);
  MAX_AZ_DEG = prefs.getShort("maxaz", 60);
  LOCK_CONFIRM_FRAMES = prefs.getShort("confirm", 5);
  LOST_TIMEOUT_MS = prefs.getShort("lostms", 700);
  KILL_HOLD_MS = prefs.getShort("killms", 3000);
  SMOOTH_ALPHA = prefs.getShort("alpha", 35) / 100.0f;
  SLEW_DEG_MAX = prefs.getShort("slew", 6);
  PATROL_ENABLED = prefs.getShort("patrol", 1);
  PATROL_SPEED = prefs.getShort("pspeed", 35) / 100.0f;
  IR_AUTO = prefs.getShort("irauto", 1);
  VISION_TIMEOUT_MS = prefs.getShort("visto", 700);
}
#endif

void logKill() {
  kills++;
  killLogged = true;
#if USE_NVS
  prefs.putUShort("kills", (uint16_t)kills);
#endif
  dfPlay(3);
  victoryTune();
  Serial.print(F("*** Цель N")); Serial.print(kills);
  Serial.println(F(" поражена. Занесена в журнал ***"));
}

void releaseTarget(const __FlashStringHelper *why) {
  if (mode == TRACK) {
    mode = IDLE;
    confirmCnt = 0;
    setLaser(false);
    visionActive = false;
    if (IR_AUTO) setIr(false);
    dfPlay(4);
    tone(PIN_BUZZER, 400, 120);
    Serial.print(F("<<< ")); Serial.print(why);
    Serial.println(F(" — возвращаюсь на дежурство"));
  }
}

// ---------------- ТЕЛЕМЕТРИЯ ДЛЯ GUI ------------------------
void emitTelemetry() {
  if (!telemetryOn) return;
  uint32_t now = millis();
  if (now - lastTlMs < 200) return;
  lastTlMs = now;
  Serial.print(F("TL m=")); Serial.print(mode == IDLE ? 'I' : 'T');
  Serial.print(F(" p=")); Serial.print((int)(panNow + 0.5f));
  Serial.print(F(" t=")); Serial.print((int)(tiltNow + 0.5f));
  Serial.print(F(" x=")); Serial.print(tlX);
  Serial.print(F(" y=")); Serial.print(tlY);
  Serial.print(F(" d=")); Serial.print(tlD);
  Serial.print(F(" k=")); Serial.print(kills);
  Serial.print(F(" r=")); Serial.print(radarAlarm ? 1 : 0);
  Serial.print(F(" v=")); Serial.print(visionActive ? 1 : 0);
  Serial.print(F(" i=")); Serial.println(irOn ? 1 : 0);
}

// ---------------- СЛИЯНИЕ С ЗРЕНИЕМ (вариант D) -------------
// Камера смонтирована на площадке турели соосно лазеру; зрение
// шлёт СМЕЩЕНИЕ цели от оси камеры (десятые градуса). Прошивка
// превращает его в довод серво. Зрение может и само захватывать
// цель (мелочь, которую радар не видит).
void applyVision(long dp10, long dt10) {
  float dpan  = (PAN_INVERT  ?  1.0f : -1.0f) * dp10 / 10.0f;
  float dtilt = (TILT_INVERT ? -1.0f :  1.0f) * dt10 / 10.0f;
  panGoal  = constrain(panNow + dpan,  (float)PAN_MIN,  (float)PAN_MAX);
  tiltGoal = constrain(tiltNow + dtilt, (float)TILT_MIN, (float)TILT_MAX);
  visionActive = true;
  lastVisionMs = millis();
  lastSeenMs   = millis();          // зрение подтверждает цель
  if (mode == IDLE) {               // захват зрением
    mode = TRACK;
    killLogged = false;
    lockedAtMs = millis();
    laserTest = false;
    setLaser(true);
    if (IR_AUTO) setIr(true);
    dfPlay(2);
    tone(PIN_BUZZER, 1200, 80);
    Serial.println(F(">>> ЦЕЛЬ ЗАХВАЧЕНА ЗРЕНИЕМ — точное сопровождение"));
  }
}

// ---------------- СЕРВИСНАЯ КОНСОЛЬ -------------------------
void handleConsole(char *s) {
  if (s[0] == '?') {
    uint32_t up = millis() / 1000;
    float fps = (millis() - statMs) > 500
      ? (framesOk - framesOkAtStat) * 1000.0f / (millis() - statMs) : 0;
    framesOkAtStat = framesOk; statMs = millis();
    Serial.print(F("СТАТУС: режим="));
    Serial.print(mode == IDLE ? F("ДЕЖУРСТВО") : F("СОПРОВОЖДЕНИЕ"));
    Serial.print(F(", радар=")); Serial.print(radarAlarm ? F("МОЛЧИТ") : F("OK"));
    Serial.print(F(", кадров/с~")); Serial.print((int)fps);
    Serial.print(F(", целых=")); Serial.print(framesOk);
    Serial.print(F(", битых=")); Serial.print(framesBad);
    Serial.print(F(", журнал=")); Serial.print(kills);
    Serial.print(F(", загрузок=")); Serial.print(boots);
    Serial.print(F(", зрение=")); Serial.print(visionActive ? F("ДА") : F("нет"));
    Serial.print(F(", ИК=")); Serial.print(irOn ? F("ВКЛ") : F("выкл"));
    Serial.print(F(", аптайм=")); Serial.print(up);
    Serial.println(F(" c, прошивка=" FW_VERSION));
  } else if (s[0] == 'C' && s[1] == '\0') {
    holdUntil = millis() + 8000;
    panGoal = PAN_CENTER; tiltGoal = TILT_CENTER;
    Serial.println(F("Серво в центр на 8 с — юстируйте крепления"));
  } else if (s[0] == 'L' && (s[1] == '0' || s[1] == '1')) {
    if (mode == TRACK) {
      Serial.println(F("Занят сопровождением — тест лазера только на дежурстве"));
    } else if (s[1] == '1') {
      laserTest = true; setLaser(true);
      Serial.println(F("Лазер ВКЛ (тест). L0 — выключить"));
    } else {
      laserTest = false; setLaser(false);
      Serial.println(F("Лазер ВЫКЛ"));
    }
  } else if (s[0] == 'T' && s[1] == '\0') {
    radarInit();
    Serial.println(F("Команда инициализации радара отправлена повторно"));
  } else if (s[0] == 'G' && s[1] == '\0') {
    printParams();
  } else if (s[0] == 'S' && s[1] == ' ') {
    char *name = s + 2;
    char *sp = strchr(name, ' ');
    if (sp == NULL) { Serial.println(F("ERR формат: S ИМЯ ЗНАЧЕНИЕ")); return; }
    *sp = '\0';
    long v = atol(sp + 1);
    if (setParam(name, v)) {
      Serial.print(F("OK ")); Serial.print(name); Serial.print(F("=")); Serial.println(v);
    } else {
      Serial.print(F("ERR имя или диапазон: ")); Serial.println(name);
    }
  } else if (s[0] == 'W' && s[1] == '\0') {
#if USE_NVS
    saveAll();
    Serial.println(F("OK SAVED (параметры и журнал в NVS)"));
#else
    Serial.println(F("ERR NVS отключён (USE_NVS 0)"));
#endif
  } else if (s[0] == 'D' && s[1] == '\0') {
    setDefaults();
    Serial.println(F("OK DEFAULTS (в ОЗУ; W — сохранить)"));
  } else if (s[0] == 'V' && s[1] == ' ') {
    char *p1 = s + 2;
    char *sp = strchr(p1, ' ');
    if (sp == NULL) { Serial.println(F("ERR формат: V dPAN10 dTILT10")); return; }
    *sp = '\0';
    long dp = atol(p1), dt = atol(sp + 1);
    if (dp < -300 || dp > 300 || dt < -300 || dt > 300) {
      Serial.println(F("ERR V: поправка вне ±300 (±30°)"));
      return;
    }
    applyVision(dp, dt);
  } else if (s[0] == 'I' && (s[1] == '0' || s[1] == '1')) {
    setIr(s[1] == '1');
    Serial.println(s[1] == '1' ? F("ИК-подсветка ВКЛ") : F("ИК-подсветка ВЫКЛ"));
  } else if (s[0] == 'Z' && s[1] == '\0') {
    kills = 0;
#if USE_NVS
    prefs.putUShort("kills", 0);
#endif
    Serial.println(F("OK Z (журнал поражений обнулён)"));
  } else if (s[0] == 'M' && (s[1] == '0' || s[1] == '1')) {
    telemetryOn = (s[1] == '1');
    Serial.println(telemetryOn ? F("OK M1") : F("OK M0"));
  } else {
    Serial.println(F("Команды: ? C L1/L0 T G S W D Z V I1/I0 M1/M0 (подробнее — руководство)"));
  }
}

void pollConsole() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (conLen > 0) { conBuf[conLen] = '\0'; handleConsole(conBuf); conLen = 0; }
    } else if (conLen < sizeof(conBuf) - 1) {
      conBuf[conLen++] = c;
    } else conLen = 0;
  }
}

// ---------------- НАДЗОР ЗА РАДАРОМ -------------------------
void radarWatch() {
  uint32_t now = millis();
  if (now - lastFrameMs > RADAR_SILENT_MS) {
    if (!radarAlarm) {
      radarAlarm = true;
      lastRetryMs = now; lastHintMs = now;
      Serial.println(F("ТРЕВОГА: радар молчит. Проверяю связь…"));
      Serial.println(F("  (питание 5В? радар TX -> GPIO16? скорость 256000?)"));
      dfPlay(5);
      tone(PIN_BUZZER, 300, 100); delay(140); tone(PIN_BUZZER, 300, 100);
      releaseTarget(F("Радар молчит"));
      panGoal = PAN_CENTER; tiltGoal = TILT_CENTER;
    }
    if (now - lastRetryMs >= RADAR_RETRY_MS) { lastRetryMs = now; radarInit(); }
    if (now - lastHintMs >= RADAR_HINT_MS) {
      lastHintMs = now;
      Serial.println(F("Радар всё ещё молчит — жду и переинициализирую…"));
    }
  }
  if (framesBad > 50 && framesOk + framesBad > 0 &&
      (float)framesBad / (float)(framesOk + framesBad) > BAD_FRAME_RATIO) {
    Serial.println(F("ПОДСКАЗКА: много битых кадров — проверьте скорость 256000 и качество проводов TX/RX"));
    framesBad = 0; framesOk = 0;
  }
}

// ---------------- ОЗВУЧКА DFPLAYER --------------------------
// Дорожки на microSD: /mp3/0001.mp3 старт, 0002 захват,
// 0003 «поражение», 0004 потеря, 0005 тревога радара.
#if USE_DFPLAYER
const int PIN_DF_TX = 33;      // ESP32 TX -> RX DFPlayer (через 1 кОм!)
const uint8_t DF_VOLUME = 25;  // 0..30

void dfSend(uint8_t cmd, uint16_t param) {
  uint8_t f[10] = {0x7E, 0xFF, 0x06, cmd, 0x00,
                   (uint8_t)(param >> 8), (uint8_t)param, 0, 0, 0xEF};
  int16_t sum = 0;
  for (int i = 1; i < 7; i++) sum += f[i];
  sum = -sum;
  f[7] = (uint8_t)(sum >> 8);
  f[8] = (uint8_t)sum;
  Serial1.write(f, 10);
}
void dfPlay(uint16_t track) { dfSend(0x03, track); }
void dfSetup() {
  Serial1.begin(9600, SERIAL_8N1, -1, PIN_DF_TX);
  delay(600);                  // DFPlayer долго думает после питания
  dfSend(0x06, DF_VOLUME);
  delay(80);
  dfPlay(1);                   // «на боевом дежурстве»
}
#else
void dfPlay(uint16_t) {}
#endif

// ---------------- OLED-ЭКРАН --------------------------------
#if USE_OLED
void oledSetup() {
  Wire.begin();                // SDA=21, SCL=22
  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (!oledOk) Serial.println(F("ОШИБКА: OLED не найден по адресу 0x3C (SDA=21, SCL=22)"));
}
void oledUpdate() {
  if (!oledOk || millis() - lastOledMs < 250) return;
  lastOledMs = millis();
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);  oled.print("PVO-2K KOMAR");
  oled.setTextSize(2);
  oled.setCursor(0, 12);
  if (radarAlarm)            oled.print("! RADAR");
  else if (mode == TRACK)    oled.print("ZAHVAT");
  else                       oled.print("DEZHUR");
  oled.setTextSize(1);
  oled.setCursor(0, 34);
  oled.print("PAN ");  oled.print((int)(panNow + 0.5f));
  oled.print("  TILT "); oled.print((int)(tiltNow + 0.5f));
  oled.setCursor(0, 45);
  oled.print("DIST "); oled.print(tlD); oled.print(" mm");
  oled.setCursor(0, 56);
  oled.print("KILLS "); oled.print(kills);
  oled.print("  BOOT "); oled.print(boots);
  oled.display();
}
#else
void oledUpdate() {}
#endif

// ---------------- ВЕБ-ПАНЕЛЬ (Wi-Fi) ------------------------
#if WIFI_ENABLED
static const char PAGE_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html><html lang=ru><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1"><title>ПВО-2К</title><style>
body{margin:0;background:#101418;color:#d7e0e7;font-family:system-ui,Arial}
header{background:#1b232b;padding:10px 14px;display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap}
h1{font-size:17px;margin:0;color:#27c0a0}
.chip{display:inline-block;background:#0a1014;border:1px solid #24404a;border-radius:12px;padding:2px 10px;margin-left:6px;font-size:13px}
.alarm{background:#3a1214;border-color:#f0544f;color:#f0544f;margin:8px 0;padding:6px 10px}
main{padding:12px;max-width:560px;margin:0 auto}
canvas{width:100%;background:#0a1014;border:1px solid #24404a;border-radius:8px}
table{width:100%;border-collapse:collapse;font-size:14px;margin-top:8px}
td{padding:4px 6px;border-bottom:1px solid #1b232b}
input{width:70px;background:#1b232b;color:#d7e0e7;border:1px solid #24404a;border-radius:4px;padding:3px;text-align:center}
button{background:#1b232b;color:#d7e0e7;border:1px solid #24404a;border-radius:6px;padding:8px 10px;margin:3px 3px 3px 0;font-size:14px}
button:active{background:#27c0a0;color:#08211c}
h3{margin:14px 0 4px}
</style></head><body>
<header><h1>ПВО-2К «Комар-М»</h1><div><span class=chip id=mode>—</span><span class=chip>журнал: <b id=kills style="color:#27c0a0">—</b></span></div></header>
<main><div id=alarm></div>
<canvas id=cv width=520 height=330></canvas>
<div style="margin-top:8px">
<button onclick="cmd('C')">Серво в центр</button><button onclick="cmd('L1')">Лазер тест</button><button onclick="cmd('L0')">Лазер выкл</button><button onclick="cmd('T')">Реинит радара</button>
<button onclick="cmd('W')">&#128190; Сохранить</button><button onclick="if(confirm('Вернуть заводские параметры?'))cmd('D').then(loadParams)">Заводские</button><button onclick="if(confirm('Обнулить журнал поражений?'))cmd('Z')">Журнал = 0</button>
</div>
<h3>Параметры <button style="font-size:12px;padding:3px 8px" onclick=loadParams()>обновить</button> <span id=pmsg></span></h3>
<table id=pt></table></main><script>
let P={};
async function j(u){return (await fetch(u)).json()}
async function cmd(c){return fetch('/api/cmd?c='+c)}
async function setP(n){let v=document.getElementById('in_'+n).value;
 let r=await fetch('/api/set?name='+n+'&value='+v);
 pmsg.textContent=(r.ok?'✓ ':'✗ ')+n; pmsg.style.color=r.ok?'#57d75b':'#f0544f';
 setTimeout(()=>pmsg.textContent='',1500);}
async function loadParams(){P=await j('/api/params');let t=document.getElementById('pt');t.innerHTML='';
 for(let k in P){t.insertAdjacentHTML('beforeend',
  `<tr><td>${k}</td><td><input id=in_${k} value=${P[k]}></td><td><button style="padding:3px 10px" onclick="setP('${k}')">✓</button></td></tr>`)}}
function draw(s){const c=document.getElementById('cv'),x=c.getContext('2d'),W=c.width,H=c.height;
 x.fillStyle='#0a1014';x.fillRect(0,0,W,H);
 const cx=W/2,cy=H-22,R=H-55,maxr=P.MAXR||4000,azm=(P.MAXAZ||60)*Math.PI/180,sc=R/maxr;
 x.fillStyle='#0f1b20';x.beginPath();x.moveTo(cx,cy);x.arc(cx,cy,R,-Math.PI/2-azm,-Math.PI/2+azm);x.closePath();x.fill();
 x.strokeStyle='#1c333c';for(let r=1000;r<=maxr;r+=1000){x.beginPath();x.arc(cx,cy,r*sc,-Math.PI/2-azm,-Math.PI/2+azm);x.stroke();}
 const az=((P.PANC||90)-s.pan)*Math.PI/180, a=-Math.PI/2+az;
 x.strokeStyle='#27c0a0';x.lineWidth=3;x.beginPath();x.moveTo(cx,cy);x.lineTo(cx+R*Math.cos(a),cy+R*Math.sin(a));x.stroke();x.lineWidth=1;
 if(s.x||s.y){const tx=cx+s.x*sc,ty=cy-s.y*sc;x.fillStyle='#f0544f';x.beginPath();x.arc(tx,ty,6,0,7);x.fill();
  x.fillText(s.d+' мм',tx+9,ty-9);}
 x.fillStyle='#41525c';x.font='12px system-ui';
 x.fillText('1 м/кольцо · pan '+s.pan+'° · tilt '+s.tilt+'° · аптайм '+s.up+' с',10,H-6);}
async function tick(){try{const s=await j('/api/status');
 mode.textContent=(s.mode=='T'?'СОПРОВОЖДЕНИЕ':'ДЕЖУРСТВО')+(s.vision?' +ЗРЕНИЕ':'')+(s.ir?' · ИК':'');
 mode.style.color=s.mode=='T'?'#f0544f':'#27c0a0';
 kills.textContent=s.kills;
 alarm.innerHTML=s.alarm?'<div class="chip alarm">⚠ РАДАР МОЛЧИТ — проверьте питание и провода</div>':'';
 draw(s);}catch(e){mode.textContent='нет связи';}}
loadParams();tick();setInterval(tick,300);
</script></body></html>)rawliteral";

String jsonStatus() {
  String s = "{\"mode\":\"";
  s += (mode == IDLE ? "I" : "T");
  s += "\",\"pan\":";  s += (int)(panNow + 0.5f);
  s += ",\"tilt\":";   s += (int)(tiltNow + 0.5f);
  s += ",\"x\":";      s += tlX;
  s += ",\"y\":";      s += tlY;
  s += ",\"d\":";      s += tlD;
  s += ",\"kills\":";  s += (int)kills;
  s += ",\"boots\":";  s += (int)boots;
  s += ",\"alarm\":";  s += (radarAlarm ? 1 : 0);
  s += ",\"vision\":"; s += (visionActive ? 1 : 0);
  s += ",\"ir\":";     s += (irOn ? 1 : 0);
  s += ",\"fw\":\"" FW_VERSION "\"";
  s += ",\"up\":";     s += (long)(millis() / 1000);
  s += "}";
  return s;
}

void jp(String &s, const char *n, long v) {
  if (s.length() > 1) s += ",";
  s += "\""; s += n; s += "\":"; s += v;
}

String jsonParams() {
  String s = "{";
  jp(s, "PANC", PAN_CENTER);   jp(s, "PANMIN", PAN_MIN);   jp(s, "PANMAX", PAN_MAX);
  jp(s, "TILTC", TILT_CENTER); jp(s, "TILTMIN", TILT_MIN); jp(s, "TILTMAX", TILT_MAX);
  jp(s, "PANINV", PAN_INVERT); jp(s, "TILTINV", TILT_INVERT);
  jp(s, "ATILT", AUTO_TILT);   jp(s, "TURH", TURRET_HEIGHT_MM); jp(s, "TGTH", TARGET_HEIGHT_MM);
  jp(s, "MINR", MIN_RANGE_MM); jp(s, "MAXR", MAX_RANGE_MM);
  jp(s, "MAXAZ", (long)MAX_AZ_DEG);
  jp(s, "CONFIRM", LOCK_CONFIRM_FRAMES);
  jp(s, "LOSTMS", LOST_TIMEOUT_MS); jp(s, "KILLMS", KILL_HOLD_MS);
  jp(s, "ALPHA", (long)(SMOOTH_ALPHA * 100 + 0.5f));
  jp(s, "SLEW", (long)SLEW_DEG_MAX);
  jp(s, "PATROL", PATROL_ENABLED);
  jp(s, "PSPEED", (long)(PATROL_SPEED * 100 + 0.5f));
  s += "}";
  return s;
}

bool webAuth() {
  if (WEB_USER[0] == '\0') return true;             // пароль не задан
  if (web.authenticate(WEB_USER, WEB_PASS)) return true;
  web.requestAuthentication();
  return false;
}

void webSetup() {
#if WIFI_AP_MODE
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  Serial.print(F("Wi-Fi: точка доступа \"")); Serial.print(WIFI_SSID);
  Serial.print(F("\" (пароль в скетче), веб-панель: http://"));
  Serial.println(WiFi.softAPIP());
#else
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print(F("Wi-Fi: подключаюсь к \"")); Serial.print(WIFI_SSID); Serial.print(F("\""));
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) { delay(250); Serial.print('.'); }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F(" OK, веб-панель: http://")); Serial.println(WiFi.localIP());
  } else {
    Serial.println(F(" НЕ ВЫШЛО (SSID/пароль?) — работаю без веб-панели"));
  }
#endif
  web.on("/", []() { if (!webAuth()) return; web.send_P(200, "text/html", PAGE_HTML); });
  web.on("/api/status", []() { if (!webAuth()) return; web.send(200, "application/json", jsonStatus()); });
  web.on("/api/params", []() { if (!webAuth()) return; web.send(200, "application/json", jsonParams()); });
  web.on("/api/set", []() {
    if (!webAuth()) return;
    String n = web.arg("name"), v = web.arg("value");
    if (n.length() && setParam(n.c_str(), v.toInt()))
      web.send(200, "text/plain", "OK");
    else
      web.send(400, "text/plain", "ERR");
  });
  web.on("/api/cmd", []() {
    if (!webAuth()) return;
    String c = web.arg("c");
    char buf[8];
    snprintf(buf, sizeof(buf), "%s", c.c_str());
    handleConsole(buf);              // те же действия, что из консоли
    web.send(200, "text/plain", "OK");
  });
  web.begin();
}
#endif

// ============================================================
void setup() {
  Serial.begin(115200);
  Serial2.begin(256000, SERIAL_8N1, PIN_RADAR_RX, PIN_RADAR_TX);

  pinMode(PIN_LASER, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_IR, OUTPUT);
  setLaser(false);
  setIr(false);

#if USE_NVS
  prefs.begin("pvo", false);
  loadAll();                                  // параметры и журнал из NVS
  boots = prefs.getUShort("boots", 0) + 1;
  prefs.putUShort("boots", boots);
#endif
  panNow = panGoal = PAN_CENTER;
  tiltNow = tiltGoal = TILT_CENTER;

  servoPan.setPeriodHertz(50);
  servoTilt.setPeriodHertz(50);
  bool okPan  = servoPan.attach(PIN_SERVO_PAN, 500, 2400) != 0;
  bool okTilt = servoTilt.attach(PIN_SERVO_TILT, 500, 2400) != 0;
  if (!okPan || !okTilt)
    Serial.println(F("ОШИБКА: не удалось занять канал ШИМ для серво (attach)"));
  servoPan.write(PAN_CENTER);
  servoTilt.write(TILT_CENTER);

  for (int i = 0; i < 2; i++) { setLaser(true); delay(120); setLaser(false); delay(120); }
  delay(200);
  radarInit();
  tone(PIN_BUZZER, 880, 90); delay(120);
  tone(PIN_BUZZER, 1320, 90);

  Serial.println();
  Serial.println(F("=== ПВО-2К \"Комар-М\" (вариант B) на боевом дежурстве ==="));
  Serial.println(F("Версия прошивки: " FW_VERSION));
  Serial.print(F("Загрузка N")); Serial.print(boots);
  Serial.print(F(", журнал за всё время: ")); Serial.println(kills);
  Serial.println(F("Команды: ? C L1/L0 T G S W D Z V I1/I0 M1/M0"));

#if USE_OLED
  oledSetup();
#endif
#if WIFI_ENABLED
  webSetup();
#endif
#if USE_DFPLAYER
  dfSetup();
#endif

  lastFrameMs = millis();
  statMs = millis();
}

// ============================================================
void loop() {
  pollConsole();
#if WIFI_ENABLED
  web.handleClient();
#endif
  bool fresh = readRadar();

  if (fresh) {
    lastFrameMs = millis();
    if (radarAlarm) {
      radarAlarm = false;
      Serial.println(F("Радар снова в строю — тревога снята"));
      tone(PIN_BUZZER, 1320, 90);
    }
    int idx = pickTarget();
    if (idx >= 0) {
      lastSeenMs = millis();
      if (!visionActive) {              // зрение точнее — радар не перетирает
        panGoal  = panFromTarget(targets[idx]);
        tiltGoal = tiltFromTarget(targets[idx]);
      }
      tlX = targets[idx].x_mm; tlY = targets[idx].y_mm;
      tlD = (int)sqrtf((float)tlX * tlX + (float)tlY * tlY);

      if (mode == IDLE) {
        if (++confirmCnt >= LOCK_CONFIRM_FRAMES) {
          mode = TRACK;
          killLogged = false;
          lockedAtMs = millis();
          laserTest = false;
          setLaser(true);
          if (IR_AUTO) setIr(true);     // будим зрение светом
          dfPlay(2);
          tone(PIN_BUZZER, 1200, 80);
          Serial.print(F(">>> ЦЕЛЬ ЗАХВАЧЕНА: x=")); Serial.print(tlX);
          Serial.print(F(" мм, y=")); Serial.print(tlY);
          Serial.print(F(" мм, дальность ")); Serial.print(tlD);
          Serial.print(F(" мм, скорость ")); Serial.print(targets[idx].speed_cms);
          Serial.println(F(" см/с — лазер наведён"));
        }
      } else if (!killLogged && millis() - lockedAtMs >= KILL_HOLD_MS) {
        logKill();
      }
    } else {
      if (mode == IDLE) confirmCnt = 0;
      tlX = tlY = tlD = 0;
    }
  }

  radarWatch();

  if (visionActive && millis() - lastVisionMs > VISION_TIMEOUT_MS) {
    visionActive = false;               // зрение замолчало — ведёт радар
    Serial.println(F("Зрение замолчало — веду по радару"));
  }

  if (mode == TRACK && millis() - lastSeenMs > LOST_TIMEOUT_MS)
    releaseTarget(F("Цель потеряна"));

  uint32_t now = millis();
  if (now - lastServoMs >= SERVO_PERIOD_MS) {
    lastServoMs = now;
    bool holding = now < holdUntil;
    if (mode == IDLE && PATROL_ENABLED && !holding && !radarAlarm) {
      panGoal += patrolDir * PATROL_SPEED;
      if (panGoal >= PAN_MAX) { panGoal = PAN_MAX; patrolDir = -1; }
      if (panGoal <= PAN_MIN) { panGoal = PAN_MIN; patrolDir = 1; }
      tiltGoal = TILT_CENTER;
    }
    panNow  = slew(panNow, panGoal);
    tiltNow = slew(tiltNow, tiltGoal);
    servoPan.write((int)(panNow + 0.5f));
    servoTilt.write((int)(tiltNow + 0.5f));
  }

  emitTelemetry();
  oledUpdate();
}

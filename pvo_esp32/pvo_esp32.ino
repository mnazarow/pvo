/*
 * ============================================================
 *  ПВО-2К «Комар-М» — турель с mmWave-радаром на ESP32
 * ============================================================
 *  Вариант B из руководства: радар сам выдаёт координаты целей,
 *  сканировать сектор не нужно — наведение мгновенное.
 *
 *  Плата:   ESP32 DevKit (WROOM-32), ядро Arduino-ESP32 3.x
 *  Радар:   HLK-LD2450 или Ai-Thinker RD-03D (24 ГГц, UART 256000)
 *  Приводы: 2× серво MG90S на pan-tilt кронштейне
 *  «Оружие»: лазерный модуль KY-008 (<=5 мВт!) через транзистор
 *  Опция:   буззер
 *
 *  Библиотека: ESP32Servo (Менеджер библиотек -> "ESP32Servo")
 *
 *  ПОДКЛЮЧЕНИЕ (подробно — в руководстве):
 *   Радар TX  -> GPIO16 (RX2)      Радар RX <- GPIO17 (TX2)
 *   Радар VCC -> 5V (VIN)          Радар GND -> GND
 *   Серво PAN  -> GPIO26,  серво TILT -> GPIO27 (питание серво — от
 *   отдельных 5 В, земли объединить!)
 *   Лазер -> GPIO23 через NPN (2N2222: база через 1 кОм)
 *   Буззер -> GPIO19
 *
 *  ЛОГИКА:
 *   ДЕЖУРСТВО  целей нет: медленный патрульный обход сектора,
 *              лазер выключен.
 *   ЗАХВАТ     радар видит цель LOCK_CONFIRM_FRAMES кадров подряд
 *              в рабочей зоне -> сопровождение, лазер ВКЛ.
 *   СОПРОВОЖДЕНИЕ  pan = азимут цели (сглаженный), tilt — по
 *              дальности (см. AUTO_TILT). Удержание KILL_HOLD_MS ->
 *              «цель поражена», +1 в журнал.
 *   ПОТЕРЯ     целей нет дольше LOST_TIMEOUT_MS -> ДЕЖУРСТВО.
 *
 *  БЕЗОПАСНОСТЬ: только лазер <=5 мВт, не на уровне глаз.
 * ============================================================
 */

#include <ESP32Servo.h>
#include <math.h>

// ==================== ТИП РАДАРА ============================
// 0 = HLK-LD2450 (многоцелевой из коробки)
// 1 = Ai-Thinker RD-03D (по умолчанию одноцелевой; при старте
//     отправляем команду включения многоцелевого режима)
#define RADAR_TYPE 1

// ========================= ПИНЫ =============================
const int PIN_RADAR_RX = 16;   // RX2 ESP32 <- TX радара
const int PIN_RADAR_TX = 17;   // TX2 ESP32 -> RX радара
const int PIN_SERVO_PAN  = 26;
const int PIN_SERVO_TILT = 27;
const int PIN_LASER  = 23;
const int PIN_BUZZER = 19;

// ================== ГЕОМЕТРИЯ ТУРЕЛИ ========================
const int   PAN_CENTER  = 90;    // угол серво «прямо по курсу», °
const int   PAN_MIN     = 15;    // механические пределы pan, °
const int   PAN_MAX     = 165;
const int   TILT_CENTER = 90;    // горизонт, °
const int   TILT_MIN    = 60;
const int   TILT_MAX    = 120;
const bool  PAN_INVERT  = false; // true, если турель крутится «не туда»
const bool  TILT_INVERT = false;

// Автонаклон: радар не меряет высоту, поэтому tilt считаем из
// дальности в предположении высоты цели над полом.
const bool  AUTO_TILT       = true;
const int   TURRET_HEIGHT_MM = 750;   // высота оси турели над полом
const int   TARGET_HEIGHT_MM = 1100;  // предполагаемая высота цели

// ================== РАБОЧАЯ ЗОНА ============================
const int   MIN_RANGE_MM = 300;    // ближе — игнорируем (свои)
const int   MAX_RANGE_MM = 4000;   // дальше — игнорируем
const float MAX_AZ_DEG   = 60.0f;  // сектор радара: ±60°

// ================== ЗАХВАТ / СБРОС ==========================
const int      LOCK_CONFIRM_FRAMES = 5;     // кадров подряд для захвата
const uint32_t LOST_TIMEOUT_MS     = 700;   // нет цели столько -> сброс
const uint32_t KILL_HOLD_MS        = 3000;  // удержание до «поражения»

// ================== ПЛАВНОСТЬ НАВЕДЕНИЯ =====================
const float SMOOTH_ALPHA   = 0.35f; // 0..1: больше — резче реакция
const float SLEW_DEG_MAX   = 6.0f;  // макс. °/обновление (анти-рывок)
const uint32_t SERVO_PERIOD_MS = 20; // темп обновления серво (50 Гц)

// ================== ПАТРУЛЬ (нет целей) =====================
const bool  PATROL_ENABLED = true;
const float PATROL_SPEED   = 0.35f; // °/обновление при обходе

// ================== ВЫБОР ЦЕЛИ ==============================
// 0 = ближайшая цель, 1 = самая быстрая (по модулю скорости)
#define TARGET_SELECT 0

// ============================================================
//                      ВНУТРЕННОСТИ
// ============================================================
struct RadarTarget {
  bool    valid;
  int16_t x_mm;      // + вправо от радара
  int16_t y_mm;      // вперёд от радара
  int16_t speed_cms; // + к радару, − от радара
  uint16_t res_mm;   // разрешение по дальности
};

enum Mode { IDLE, TRACK };
Mode mode = IDLE;

Servo servoPan, servoTilt;

RadarTarget targets[3];
uint8_t  frameBuf[30];
int      frameFill = 0;

float panNow = PAN_CENTER,  panGoal = PAN_CENTER;
float tiltNow = TILT_CENTER, tiltGoal = TILT_CENTER;
float patrolDir = 1.0f;

int      confirmCnt = 0;
uint32_t lastSeenMs = 0, lockedAtMs = 0, lastServoMs = 0;
bool     killLogged = false;
unsigned kills = 0;

// ------------------------------------------------------------
// Декодирование чисел радара. ВАЖНО: это НЕ обычный int16!
// Старший бит = знак: 1 -> число положительное (младшие 15 бит),
// 0 -> отрицательное. Так в официальном протоколе LD2450/RD-03D.
// ------------------------------------------------------------
int16_t radarInt16(uint8_t lo, uint8_t hi) {
  int16_t mag = (int16_t)((((uint16_t)hi << 8) | lo) & 0x7FFF);
  return (hi & 0x80) ? mag : (int16_t)-mag;
}

void parseFrame(const uint8_t *b) {   // b — 30 байт с заголовком
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

// Приём байтов радара: ищем заголовок AA FF 03 00, копим 30 байт,
// проверяем хвост 55 CC.
bool readRadar() {
  static const uint8_t HDR[4] = {0xAA, 0xFF, 0x03, 0x00};
  bool gotFrame = false;
  while (Serial2.available()) {
    uint8_t c = (uint8_t)Serial2.read();
    if (frameFill < 4) {                      // ловим заголовок
      if (c == HDR[frameFill]) frameBuf[frameFill++] = c;
      else frameFill = (c == HDR[0]) ? 1 : 0;
      continue;
    }
    frameBuf[frameFill++] = c;
    if (frameFill == 30) {
      if (frameBuf[28] == 0x55 && frameBuf[29] == 0xCC) {
        parseFrame(frameBuf);
        gotFrame = true;
      }
      frameFill = 0;
    }
  }
  return gotFrame;
}

// Выбор цели в рабочей зоне
int pickTarget() {
  int best = -1;
  float bestKey = 1e9f;
  for (int i = 0; i < 3; i++) {
    if (!targets[i].valid) continue;
    float dist = sqrtf((float)targets[i].x_mm * targets[i].x_mm +
                       (float)targets[i].y_mm * targets[i].y_mm);
    float az = fabsf(atan2f((float)targets[i].x_mm,
                            (float)targets[i].y_mm)) * 57.2958f;
    if (dist < MIN_RANGE_MM || dist > MAX_RANGE_MM) continue;
    if (az > MAX_AZ_DEG) continue;
#if TARGET_SELECT == 0
    float key = dist;                          // ближайшая
#else
    float key = -fabsf((float)targets[i].speed_cms); // самая быстрая
#endif
    if (key < bestKey) { bestKey = key; best = i; }
  }
  return best;
}

// Азимут цели -> угол серво pan
float panFromTarget(const RadarTarget &t) {
  float az = atan2f((float)t.x_mm, (float)t.y_mm) * 57.2958f; // °, + вправо
  float a = PAN_INVERT ? (PAN_CENTER + az) : (PAN_CENTER - az);
  return constrain(a, (float)PAN_MIN, (float)PAN_MAX);
}

// Дальность цели -> угол серво tilt (если AUTO_TILT)
float tiltFromTarget(const RadarTarget &t) {
  if (!AUTO_TILT) return (float)TILT_CENTER;
  float dist = sqrtf((float)t.x_mm * t.x_mm + (float)t.y_mm * t.y_mm);
  if (dist < 1) dist = 1;
  float el = atan2f((float)(TARGET_HEIGHT_MM - TURRET_HEIGHT_MM), dist)
             * 57.2958f;
  float a = TILT_INVERT ? (TILT_CENTER - el) : (TILT_CENTER + el);
  return constrain(a, (float)TILT_MIN, (float)TILT_MAX);
}

// Плавное движение к цели: сглаживание + ограничение скорости
float slew(float now, float goal) {
  float next = now + SMOOTH_ALPHA * (goal - now);
  float d = next - now;
  if (d >  SLEW_DEG_MAX) next = now + SLEW_DEG_MAX;
  if (d < -SLEW_DEG_MAX) next = now - SLEW_DEG_MAX;
  return next;
}

void setLaser(bool on) { digitalWrite(PIN_LASER, on ? HIGH : LOW); }

void victoryTune() {
  tone(PIN_BUZZER, 1568, 100); delay(120);
  tone(PIN_BUZZER, 1245, 100); delay(120);
  tone(PIN_BUZZER, 1047, 180);
}

// ============================================================
void setup() {
  Serial.begin(115200);                       // отладка/журнал
  Serial2.begin(256000, SERIAL_8N1, PIN_RADAR_RX, PIN_RADAR_TX);

  pinMode(PIN_LASER, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  setLaser(false);

  servoPan.setPeriodHertz(50);
  servoTilt.setPeriodHertz(50);
  servoPan.attach(PIN_SERVO_PAN, 500, 2400);
  servoTilt.attach(PIN_SERVO_TILT, 500, 2400);
  servoPan.write(PAN_CENTER);
  servoTilt.write(TILT_CENTER);

#if RADAR_TYPE == 1
  // RD-03D: включаем многоцелевой режим (для одноцелевого — 0x80 0x00)
  const uint8_t MULTI_CMD[12] =
    {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x90, 0x00,
     0x04, 0x03, 0x02, 0x01};
  delay(200);
  Serial2.write(MULTI_CMD, sizeof(MULTI_CMD));
#endif

  tone(PIN_BUZZER, 880, 90); delay(120);
  tone(PIN_BUZZER, 1320, 90);
  Serial.println();
  Serial.println(F("=== ПВО-2К \"Комар-М\" на боевом дежурстве ==="));
  Serial.println(F("Радар 24 ГГц, сектор ±60°, зона 0.3–4 м"));
}

// ============================================================
void loop() {
  bool fresh = readRadar();

  if (fresh) {
    int idx = pickTarget();
    if (idx >= 0) {
      lastSeenMs = millis();
      panGoal  = panFromTarget(targets[idx]);
      tiltGoal = tiltFromTarget(targets[idx]);

      if (mode == IDLE) {
        if (++confirmCnt >= LOCK_CONFIRM_FRAMES) {
          mode = TRACK;
          killLogged = false;
          lockedAtMs = millis();
          setLaser(true);
          tone(PIN_BUZZER, 1200, 80);
          float dist = sqrtf((float)targets[idx].x_mm * targets[idx].x_mm +
                             (float)targets[idx].y_mm * targets[idx].y_mm);
          Serial.print(F(">>> ЦЕЛЬ ЗАХВАЧЕНА: x="));
          Serial.print(targets[idx].x_mm);
          Serial.print(F(" мм, y="));
          Serial.print(targets[idx].y_mm);
          Serial.print(F(" мм, дальность "));
          Serial.print((int)dist);
          Serial.print(F(" мм, скорость "));
          Serial.print(targets[idx].speed_cms);
          Serial.println(F(" см/с — лазер наведён"));
        }
      } else if (!killLogged && millis() - lockedAtMs >= KILL_HOLD_MS) {
        kills++;
        killLogged = true;
        victoryTune();
        Serial.print(F("*** Цель N"));
        Serial.print(kills);
        Serial.println(F(" поражена. Занесена в журнал ***"));
      }
    } else if (mode == IDLE) {
      confirmCnt = 0;
    }
  }

  // Потеря цели
  if (mode == TRACK && millis() - lastSeenMs > LOST_TIMEOUT_MS) {
    mode = IDLE;
    confirmCnt = 0;
    setLaser(false);
    tone(PIN_BUZZER, 400, 120);
    Serial.println(F("<<< Цель потеряна — возвращаюсь на дежурство"));
  }

  // Обновление приводов с фиксированным темпом
  uint32_t now = millis();
  if (now - lastServoMs >= SERVO_PERIOD_MS) {
    lastServoMs = now;

    if (mode == IDLE && PATROL_ENABLED) {      // ленивый патруль
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
}

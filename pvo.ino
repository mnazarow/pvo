/*
 * ============================================================
 *  ПВО-1К «Комар» — радарная турель на Arduino
 * ============================================================
 *  По мотивам вирусного видео «ПВО от комаров».
 *
 *  Плата:   Arduino Uno / Nano (ATmega328P)
 *  Датчик:  HC-SR04 (ультразвуковой дальномер) — крепится на серво
 *  Привод:  серво SG90 / MG90S (азимут, 0–180°)
 *  «Оружие»: лазерный модуль KY-008 (НЕ мощнее 5 мВт — только
 *            целеуказатель!) — крепится рядом с датчиком, смотрит
 *            туда же, куда и он
 *  Опция:   буззер — звуки захвата и «поражения» цели
 *
 *  ЛОГИКА (конечный автомат):
 *   ПАТРУЛЬ        серво сканирует сектор туда-сюда, каждый шаг —
 *                  замер дистанции. Цель ближе DETECT_DIST_CM
 *                  CONFIRM_PINGS раз подряд -> ЗАХВАТ.
 *   СОПРОВОЖДЕНИЕ  лазер ВКЛ, турель делает микро-скан ±TRACK_WOBBLE°
 *                  вокруг цели и «ведёт» её, если та смещается.
 *                  Продержали цель KILL_HOLD_MS -> «поражена»,
 *                  +1 в журнал (как тетрадь смерти из видео).
 *                  Цель пропала LOST_PINGS замеров подряд ->
 *                  лазер ВЫКЛ, снова ПАТРУЛЬ.
 *
 *  ТЕЛЕМЕТРИЯ: Serial 9600.
 *   RADAR_GUI 0 — человекочитаемый лог событий (Монитор порта).
 *   RADAR_GUI 1 — чистый поток "угол,дистанция." — совместим с
 *   классическим Processing-радаром (зелёный экран ПВО на ПК),
 *   см. README.
 *
 *  ВНЕШНЕЕ ЦЕЛЕУКАЗАНИЕ (режим SLAVE, для связки с Raspberry Pi
 *  и камерой — см. pvo_vision.py и руководство):
 *   по Serial принимаются строки-команды:
 *     P<угол>\n  — навести турель на угол 15..165 и включить лазер
 *                  (напр. "P120"); команды нужно слать регулярно
 *     R\n        — сброс: выключить лазер, вернуться к патрулю
 *     ?\n        — напечатать статус (режим, журнал)
 *   если команд P нет дольше EXT_TIMEOUT_MS, турель сама
 *   возвращается к автономному патрулированию.
 *
 *  БЕЗОПАСНОСТЬ: лазер мощнее 5 мВт (класс 3B/4) необратимо
 *  повреждает зрение, в том числе отражённым лучом. Использовать
 *  только модуль <=5 мВт и не ставить турель на уровне глаз.
 * ============================================================
 */

#include <Servo.h>

// ==================== РЕЖИМ ТЕЛЕМЕТРИИ ======================
#define RADAR_GUI 0   // 0 = лог для человека, 1 = поток для Processing-радара

// ========================= ПИНЫ =============================
const uint8_t PIN_SERVO  = 9;   // сигнал серво (оранжевый провод)
const uint8_t PIN_TRIG   = 3;   // HC-SR04 TRIG
const uint8_t PIN_ECHO   = 2;   // HC-SR04 ECHO
const uint8_t PIN_LASER  = 7;   // KY-008: S (сигнал)
const uint8_t PIN_BUZZER = 8;   // буззер «+» (можно не ставить)

// ======================= НАСТРОЙКИ ==========================
const int      SWEEP_MIN      = 15;    // границы сектора патруля, °
const int      SWEEP_MAX      = 165;
const int      DETECT_DIST_CM = 60;    // ближе — считаем целью
const int      LOST_DIST_CM   = 90;    // дальше — цель потеряна (гистерезис)
const uint8_t  CONFIRM_PINGS  = 3;     // замеров подряд для захвата
const uint8_t  LOST_PINGS     = 12;    // пустых замеров подряд для сброса
const int      TRACK_WOBBLE   = 5;     // амплитуда микро-скана, °
const unsigned long STEP_MS      = 30;    // темп шагов (HC-SR04 нужна пауза)
const unsigned long KILL_HOLD_MS = 3000;  // удержание до «поражения», мс
const unsigned long EXT_TIMEOUT_MS = 700; // нет команд P -> выход из SLAVE

// ======================= СОСТОЯНИЕ ==========================
enum Mode { PATROL, TRACK, SLAVE };
Mode mode = PATROL;

Servo turret;

int  angle     = SWEEP_MIN;  // текущий угол патруля
int  sweepDir  = 1;          // направление патруля: +1 / -1
int  lockAngle = 90;         // азимут захваченной цели
int  wobble    = 0;          // смещение микро-скана
int  wobbleDir = 1;

uint8_t confirmCnt = 0;      // подряд «есть цель» (патруль)
uint8_t lostCnt    = 0;      // подряд «цели нет» (сопровождение)

unsigned long lastStep = 0;  // тайминг шагов
unsigned long lockedAt = 0;  // момент захвата
bool killLogged = false;     // «поражение» уже записано в этом захвате

unsigned int kills = 0;      // журнал поражённых целей

// --- внешнее целеуказание (SLAVE) ---
int  extAngle  = 90;         // угол, скомандованный извне
unsigned long lastExtMs = 0; // время последней команды P
char inBuf[12];              // буфер строки команды
uint8_t inLen = 0;

// Прототипы (Arduino IDE делает их сама, но явно — надёжнее)
void patrolStep();
void trackStep();
void slaveStep();
void lockTarget();
void releaseTarget();
void pollSerial();
void handleLine(const char *s);
int  measureCm();
void report(int a, int d);
void victoryTune();

// ============================================================
void setup() {
  pinMode(PIN_TRIG,   OUTPUT);
  pinMode(PIN_ECHO,   INPUT);
  pinMode(PIN_LASER,  OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_LASER, LOW);

  Serial.begin(9600);

  turret.attach(PIN_SERVO);

  // Самотест: прогон по сектору + короткий сигнал
  turret.write(90);          delay(400);
  turret.write(SWEEP_MIN);   delay(400);
  turret.write(SWEEP_MAX);   delay(600);
  turret.write(SWEEP_MIN);   delay(600);
  tone(PIN_BUZZER, 880, 90); delay(120);
  tone(PIN_BUZZER, 1320, 90);

#if !RADAR_GUI
  Serial.println(F("=== ПВО-1К \"Комар\" на боевом дежурстве ==="));
  Serial.println(F("Сектор патрулирования: 15..165 град"));
#endif
}

// ============================================================
void loop() {
  pollSerial();                           // команды внешнего наведения

  unsigned long now = millis();
  if (now - lastStep < STEP_MS) return;   // держим темп без delay()
  lastStep = now;

  if      (mode == PATROL) patrolStep();
  else if (mode == TRACK)  trackStep();
  else                     slaveStep();
}

// ---------------- ПРИЁМ КОМАНД ПО SERIAL --------------------
void pollSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inLen > 0) { inBuf[inLen] = '\0'; handleLine(inBuf); inLen = 0; }
    } else if (inLen < sizeof(inBuf) - 1) {
      inBuf[inLen++] = c;
    } else {
      inLen = 0;                          // мусор — сбрасываем буфер
    }
  }
}

void handleLine(const char *s) {
  if (s[0] == 'P') {                      // P<угол>: внешнее наведение
    int a = atoi(s + 1);
    if (a < SWEEP_MIN) a = SWEEP_MIN;
    if (a > SWEEP_MAX) a = SWEEP_MAX;
    extAngle  = a;
    lastExtMs = millis();
    if (mode != SLAVE) {                  // входим в режим SLAVE
      mode = SLAVE;
      killLogged = false;
      lockedAt = millis();
      digitalWrite(PIN_LASER, HIGH);
      tone(PIN_BUZZER, 1200, 80);
#if !RADAR_GUI
      Serial.println(F(">>> Внешнее целеуказание принято (SLAVE)"));
#endif
    }
  } else if (s[0] == 'R') {               // R: сброс внешнего наведения
    if (mode == SLAVE) releaseTarget();
  } else if (s[0] == '?') {               // ?: статус
#if !RADAR_GUI
    Serial.print(F("СТАТУС: режим="));
    Serial.print(mode == PATROL ? F("ПАТРУЛЬ")
                : mode == TRACK ? F("СОПРОВОЖДЕНИЕ") : F("SLAVE"));
    Serial.print(F(", журнал="));
    Serial.println(kills);
#endif
  }
}

// ---------------- SLAVE: шаг внешнего наведения -------------
void slaveStep() {
  if (millis() - lastExtMs > EXT_TIMEOUT_MS) {  // командир замолчал
    releaseTarget();
    return;
  }
  turret.write(extAngle);

  int d = measureCm();
  report(extAngle, d);

  // «поражение» и в этом режиме заносим в журнал
  if (!killLogged && millis() - lockedAt >= KILL_HOLD_MS) {
    kills++;
    killLogged = true;
    victoryTune();
#if !RADAR_GUI
    Serial.print(F("*** Цель N"));
    Serial.print(kills);
    Serial.println(F(" поражена. Занесена в журнал ***"));
#endif
  }
}

// ---------------- ПАТРУЛЬ: шаг сканирования -----------------
void patrolStep() {
  angle += sweepDir;
  if (angle >= SWEEP_MAX) { angle = SWEEP_MAX; sweepDir = -1; }
  if (angle <= SWEEP_MIN) { angle = SWEEP_MIN; sweepDir =  1; }
  turret.write(angle);

  int d = measureCm();
  report(angle, d);

  if (d <= DETECT_DIST_CM) {
    if (++confirmCnt >= CONFIRM_PINGS) lockTarget();
  } else {
    confirmCnt = 0;
  }
}

// ---------------- ЗАХВАТ ЦЕЛИ -------------------------------
void lockTarget() {
  mode       = TRACK;
  lockAngle  = angle;
  wobble     = 0;
  wobbleDir  = 1;
  lostCnt    = 0;
  killLogged = false;
  lockedAt   = millis();

  digitalWrite(PIN_LASER, HIGH);          // лазер на цель
  tone(PIN_BUZZER, 1200, 80);

#if !RADAR_GUI
  Serial.print(F(">>> ЦЕЛЬ ЗАХВАЧЕНА! Азимут "));
  Serial.print(lockAngle);
  Serial.println(F(" град — лазер наведён"));
#endif
}

// ---------------- СОПРОВОЖДЕНИЕ: шаг ------------------------
void trackStep() {
  // Микро-скан вокруг точки захвата — так ведём движущуюся цель
  wobble += wobbleDir;
  if (wobble >=  TRACK_WOBBLE) wobbleDir = -1;
  if (wobble <= -TRACK_WOBBLE) wobbleDir =  1;

  int a = constrain(lockAngle + wobble, SWEEP_MIN, SWEEP_MAX);
  turret.write(a);

  int d = measureCm();
  report(a, d);

  if (d <= LOST_DIST_CM) {
    // Цель видим на этом азимуте — смещаем точку удержания к ней
    lostCnt   = 0;
    lockAngle = a;

    if (!killLogged && millis() - lockedAt >= KILL_HOLD_MS) {
      kills++;
      killLogged = true;
      victoryTune();
#if !RADAR_GUI
      Serial.print(F("*** Цель N"));
      Serial.print(kills);
      Serial.println(F(" поражена. Занесена в журнал ***"));
#endif
    }
  } else {
    if (++lostCnt >= LOST_PINGS) releaseTarget();
  }
}

// ---------------- ПОТЕРЯ ЦЕЛИ -------------------------------
void releaseTarget() {
  digitalWrite(PIN_LASER, LOW);
  angle      = (mode == SLAVE) ? extAngle : lockAngle; // патруль с этого места
  mode       = PATROL;
  confirmCnt = 0;
  tone(PIN_BUZZER, 400, 120);

#if !RADAR_GUI
  Serial.println(F("<<< Цель потеряна — возвращаюсь к патрулированию"));
#endif
}

// ---------------- ЗАМЕР ДИСТАНЦИИ, см -----------------------
// Возвращает 999, если эха нет (пусто / вне дальности).
int measureCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  // Таймаут 20 мс ~= 3.4 м в одну сторону — нам с запасом хватает
  unsigned long us = pulseIn(PIN_ECHO, HIGH, 20000UL);
  if (us == 0) return 999;
  return (int)(us / 58);   // мкс -> сантиметры
}

// ---------------- ТЕЛЕМЕТРИЯ --------------------------------
void report(int a, int d) {
#if RADAR_GUI
  // Формат классического Processing-радара: "угол,дистанция."
  Serial.print(a);
  Serial.print(',');
  Serial.print(d);
  Serial.print('.');
#else
  (void)a; (void)d;  // в режиме лога печатаем только события
#endif
}

// ---------------- САЛЮТ ПРИ «ПОРАЖЕНИИ» ---------------------
void victoryTune() {
  tone(PIN_BUZZER, 1568, 100); delay(120);
  tone(PIN_BUZZER, 1245, 100); delay(120);
  tone(PIN_BUZZER, 1047, 180);
}

// Реализация: см. комментарий в Servo.h (стенд эмуляции).
#include "Servo.h"
#include <avr/interrupt.h>

struct ServoChan { volatile uint8_t pin; volatile uint16_t ticks; volatile bool active; };
static ServoChan chans[MAX_SERVOS];
static volatile int8_t cur = -1;          // канал, чей импульс идёт сейчас
static volatile uint16_t frameUsed = 0;   // тиков израсходовано в кадре
static bool timerReady = false;

#define US_TO_TICKS(u) ((clockCyclesPerMicrosecond() * (u)) / 8)   // prescaler 8
#define TICKS_TO_US(t) (((unsigned)(t) * 8) / clockCyclesPerMicrosecond())

static void startTimer() {
  if (timerReady) return;
  timerReady = true;
  noInterrupts();
  TCCR1A = 0; TCCR1B = 0;
  TCNT1 = 0;
  TCCR1B = _BV(CS11);                 // prescaler 8 -> 0,5 мкс/тик при 16 МГц
  OCR1A = TCNT1 + US_TO_TICKS(100);
  TIFR1 |= _BV(OCF1A);
  TIMSK1 |= _BV(OCIE1A);
  interrupts();
}

ISR(TIMER1_COMPA_vect) {
  if (cur >= 0 && chans[cur].active)   // импульс текущего канала закончился
    digitalWrite(chans[cur].pin, LOW);

  int8_t next = -1;
  for (int8_t i = cur + 1; i < MAX_SERVOS; i++)
    if (chans[i].active) { next = i; break; }

  if (next >= 0) {                     // начинаем импульс следующего канала
    cur = next;
    digitalWrite(chans[cur].pin, HIGH);
    frameUsed += chans[cur].ticks;
    OCR1A = TCNT1 + chans[cur].ticks;
  } else {                             // кадр закончился — досыпаем до 20 мс
    uint16_t rest = US_TO_TICKS(REFRESH_INTERVAL);
    rest = (frameUsed < rest) ? (rest - frameUsed) : US_TO_TICKS(100);
    cur = -1; frameUsed = 0;
    OCR1A = TCNT1 + rest;
  }
}

Servo::Servo() : idx(INVALID_SERVO), minUs(MIN_PULSE_WIDTH), maxUs(MAX_PULSE_WIDTH) {}

uint8_t Servo::attach(int pin) { return attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH); }

uint8_t Servo::attach(int pin, int mn, int mx) {
  if (idx == INVALID_SERVO) {
    for (uint8_t i = 0; i < MAX_SERVOS; i++)
      if (!chans[i].active) { idx = i; break; }
    if (idx == INVALID_SERVO) return INVALID_SERVO;
  }
  minUs = mn; maxUs = mx;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  chans[idx].pin = pin;
  chans[idx].ticks = US_TO_TICKS(DEFAULT_PULSE_WIDTH);
  chans[idx].active = true;
  startTimer();
  return idx;
}

void Servo::detach() {
  if (idx == INVALID_SERVO) return;
  chans[idx].active = false;
  digitalWrite(chans[idx].pin, LOW);
  idx = INVALID_SERVO;
}

void Servo::write(int value) {
  if (value < MIN_PULSE_WIDTH) {                 // как в оригинале: <544 = градусы
    if (value < 0) value = 0;
    if (value > 180) value = 180;
    value = map(value, 0, 180, minUs, maxUs);
  }
  writeMicroseconds(value);
}

void Servo::writeMicroseconds(int us) {
  if (idx == INVALID_SERVO) return;
  if (us < minUs) us = minUs;
  if (us > maxUs) us = maxUs;
  uint16_t t = US_TO_TICKS(us);
  uint8_t old = SREG; noInterrupts();
  chans[idx].ticks = t;
  SREG = old;
}

int Servo::readMicroseconds() {
  if (idx == INVALID_SERVO) return 0;
  return TICKS_TO_US(chans[idx].ticks);
}

int Servo::read() { return map(readMicroseconds() + 1, minUs, maxUs, 0, 180); }

bool Servo::attached() { return idx != INVALID_SERVO; }

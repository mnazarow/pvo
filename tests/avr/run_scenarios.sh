#!/bin/bash
# ============================================================
#  Сценарии на ЭМУЛЯТОРЕ ATmega328P (avr8js): настоящая прошивка,
#  собранная avr-gcc, исполняется по тактам — без платы.
#
#  Что проверяется сверх тестов на заглушках:
#    · прошивка собирается под AVR и влезает во flash/RAM;
#    · настоящие тайминги (pulseIn, delay, ШИМ серво 50 Гц);
#    · настоящая EEPROM: параметры и журнал переживают перезагрузку.
#
#  Требуется один раз:
#    sudo apt install gcc-avr avr-libc arduino-core-avr
#    (cd tests/avr/js && npm install)
#  Запуск: bash tests/avr/run_scenarios.sh
# ============================================================
set -e
cd "$(dirname "$0")/../.."
RUN="node tests/avr/js/run_avr.mjs /tmp/avrbuild/firmware.hex"
EE=/tmp/pvo_avr_eeprom.bin

command -v avr-gcc >/dev/null || { echo "нет avr-gcc: sudo apt install gcc-avr avr-libc arduino-core-avr"; exit 2; }
[ -d tests/avr/js/node_modules ] || { echo "нет avr8js: (cd tests/avr/js && npm install)"; exit 2; }

echo "=== 1/5 Сборка под ATmega328P: вариант A (минимальный) ==="
bash tests/avr/build_avr.sh pvo_a_simple/pvo_a_simple.ino | tail -6

echo "=== 2/5 Дежурство, захват, поражение, потеря (эмуляция 14 с) ==="
$RUN --seconds 14 --target-at 40,7 --target-gone 11 \
  --expect "Самотест: датчик OK" \
  --expect "на боевом дежурстве" \
  --expect "ЦЕЛЬ ЗАХВАЧЕНА" \
  --expect "поражена" \
  --expect "Цель потеряна" | tail -8

echo "=== 3/5 Сборка под ATmega328P: вариант A/C (полная прошивка) ==="
bash tests/avr/build_avr.sh pvo.ino | tail -6

echo "=== 4/5 Протокол, предохранитель TRACKMAX и запись в EEPROM (22 с) ==="
rm -f "$EE"
$RUN --seconds 22 --eeprom "$EE" \
  --send "S TRACKMAX 5000@5" --send "W@6" --target-at 40,7 \
  --expect "OK TRACKMAX=5000" \
  --expect "OK SAVED" \
  --expect "ЦЕЛЬ ЗАХВАЧЕНА" \
  --expect "Лимит непрерывного сопровождения" | tail -8

echo "=== 5/5 Перезагрузка: параметры и журнал уцелели (8 с) ==="
$RUN --seconds 8 --eeprom "$EE" --send "G@6" \
  --expect "Загрузка N2" \
  --expect "PARAM TRACKMAX=5000" \
  --forbid "журнал за всё время: 0" | tail -8

echo
echo "ЭМУЛЯЦИЯ: ВСЕ СЦЕНАРИИ ПРОЙДЕНЫ ✔"

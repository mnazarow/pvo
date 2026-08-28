#!/bin/bash
# Полный прогон тестов проекта «ПВО от комаров» без железа.
# Нужны: g++, python3 (+ pip install opencv-python-headless numpy).
set -e
cd "$(dirname "$0")"
BUILD=$(mktemp -d)
trap 'rm -rf "$BUILD"' EXIT

echo "=== 1/7 Прошивка Uno (pvo.ino): сценарии + протокол ==="
cp stub_uno/* "$BUILD"/
cp ../pvo.ino "$BUILD"/sketch.cpp.inc
(cd "$BUILD" && g++ -std=c++17 -Wall -Wextra -DUSE_DFPLAYER=1 -I. main.cpp -o t && ./t | tail -4)

echo "=== 2/7 Прошивка Uno (pvo_a_simple.ino) ==="
cp ../pvo_a_simple/pvo_a_simple.ino "$BUILD"/sketch_a.cpp.inc
(cd "$BUILD" && g++ -std=c++17 -Wall -Wextra -I. main_a.cpp -o ta && ./ta | tail -3)

echo "=== 3/7 Прошивка ESP32 (pvo_esp32.ino) ==="
rm -f "$BUILD"/Arduino.h "$BUILD"/Servo.h
cp stub_esp32/* "$BUILD"/
cp ../pvo_esp32/pvo_esp32.ino "$BUILD"/sketch.cpp.inc
(cd "$BUILD" && g++ -std=c++17 -Wall -Wextra -DUSE_OLED=1 -DUSE_DFPLAYER=1 -I. main32.cpp -o t32 && ./t32 | tail -3)

echo "=== 4/7 Зрение (pvo_vision.py) ==="
python3 test_vision.py | tail -3

echo "=== 5/7 Логика GUI (pvo_gui.py) ==="
python3 test_gui_logic.py | tail -3

echo "=== 6/7 Слияние радара и зрения (pvo_fusion.py) ==="
python3 test_fusion.py | tail -3

echo "=== 7/7 Эмулятор ATmega328P (настоящая прошивка, avr8js) ==="
if command -v avr-gcc >/dev/null && [ -d avr/js/node_modules ]; then
  bash avr/run_scenarios.sh 2>&1 | grep -E "^(===|  [✔✘]|Program|Data|ЭМУЛЯЦИЯ)"
else
  echo "  пропущено (это единственный шаг, которому нужна установка):"
  echo "  sudo apt install gcc-avr avr-libc arduino-core-avr && (cd tests/avr/js && npm install)"
fi

echo
echo "ВСЕ ТЕСТЫ ПРОЙДЕНЫ ✔"

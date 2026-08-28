#!/bin/bash
# Сборка НАСТОЯЩЕЙ прошивки под ATmega328P (Uno/Nano) — avr-gcc + ядро Arduino.
# Показывает, сколько занято flash и RAM: заглушечные тесты этого не видят.
#   bash tests/avr/build_avr.sh pvo.ino [доп. -D...]
set -e
SKETCH="${1:-pvo.ino}"
shift || true
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
CORE=/usr/share/arduino/hardware/arduino/avr/cores/arduino
VAR=/usr/share/arduino/hardware/arduino/avr/variants/standard
LIBS=/usr/share/arduino/hardware/arduino/avr/libraries
SERVO="$(cd "$(dirname "$0")" && pwd)/Servo"
OUT="${OUT:-/tmp/avrbuild}"
MCU=atmega328p
F=16000000L

CFLAGS="-mmcu=$MCU -DF_CPU=$F -DARDUINO=10819 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR \
  -Os -w -ffunction-sections -fdata-sections -MMD \
  -I$CORE -I$VAR -I$LIBS/EEPROM/src -I$LIBS/SoftwareSerial/src -I$SERVO $*"

rm -rf "$OUT"; mkdir -p "$OUT/core"

# 1. ядро Arduino -> core.a (кэшируется между запусками)
CACHE=/tmp/avrcore-$MCU.a
if [ ! -f "$CACHE" ]; then
  for f in $CORE/*.c;   do avr-gcc  -std=gnu11   $CFLAGS -c "$f" -o "$OUT/core/$(basename $f).o"; done
  for f in $CORE/*.cpp; do avr-g++  -std=gnu++11 -fno-exceptions -fpermissive $CFLAGS -c "$f" -o "$OUT/core/$(basename $f).o"; done
  for f in $CORE/*.S;   do avr-gcc  -x assembler-with-cpp $CFLAGS -c "$f" -o "$OUT/core/$(basename $f).o"; done
  avr-gcc-ar rcs "$CACHE" $OUT/core/*.o
fi

# 2. библиотеки
avr-g++ -std=gnu++11 -fno-exceptions -fpermissive $CFLAGS -c "$LIBS/SoftwareSerial/src/SoftwareSerial.cpp" -o "$OUT/SoftwareSerial.o"
avr-g++ -std=gnu++11 -fno-exceptions -fpermissive $CFLAGS -c "$SERVO/Servo.cpp" -o "$OUT/Servo.o"

# 3. скетч (.ino -> .cpp: дописываем include, прототипы в скетчах уже есть)
{ echo '#include <Arduino.h>'; cat "$ROOT/$SKETCH"; } > "$OUT/sketch.cpp"
avr-g++ -std=gnu++11 -fno-exceptions -fpermissive $CFLAGS -c "$OUT/sketch.cpp" -o "$OUT/sketch.o"

# 4. линковка
avr-gcc -mmcu=$MCU -Os -Wl,--gc-sections -o "$OUT/firmware.elf" \
  "$OUT/sketch.o" "$OUT/SoftwareSerial.o" "$OUT/Servo.o" "$CACHE" -lm
avr-objcopy -O ihex -R .eeprom "$OUT/firmware.elf" "$OUT/firmware.hex"

echo "--- $SKETCH -> $OUT/firmware.elf ---"
avr-size -C --mcu=$MCU "$OUT/firmware.elf" | sed -n '2,8p'

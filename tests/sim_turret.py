# -*- coding: utf-8 -*-
"""Симулятор прошивки турели на PTY — для скриншотов GUI."""
import sys, time, threading, math

import serial

VARIANT = sys.argv[2] if len(sys.argv) > 2 else "uno"
PORT = sys.argv[1]

if VARIANT == "uno":
    params = {"DETECT":60,"LOST":90,"MINVALID":3,"CONFIRM":3,"LOSTP":12,
              "WOBBLE":5,"STEP":30,"KILLMS":3000,"EXTMS":700,"SWMIN":15,"SWMAX":165}
else:
    params = {"PANC":90,"PANMIN":15,"PANMAX":165,"TILTC":90,"TILTMIN":60,"TILTMAX":120,
              "PANINV":0,"TILTINV":0,"ATILT":1,"TURH":750,"TGTH":1100,
              "MINR":300,"MAXR":4000,"MAXAZ":60,"CONFIRM":5,"LOSTMS":700,
              "KILLMS":3000,"ALPHA":35,"SLEW":6,"PATROL":1,"PSPEED":35}

ser = serial.Serial(PORT, 9600, timeout=0.05)
telemetry = False
kills = 3
mode = "P" if VARIANT == "uno" else "I"
slave_angle = 90
t0 = time.time()

def w(line):
    ser.write((line + "\n").encode("utf-8"))

def banner():
    if VARIANT == "uno":
        w("=== ПВО-1К \"Комар\" (полная прошивка) на боевом дежурстве ===")
        w("Загрузка N7, журнал за всё время: 3")
        w("Команды: P<угол> R ? G S W D M1/M0 (подробнее — руководство)")
        w("Самотест: датчик OK (эхо в 9/10 замеров)")
    else:
        w("=== ПВО-2К \"Комар-М\" (вариант B) на боевом дежурстве ===")
        w("Загрузка N4, журнал за всё время: 3")
        w("Команды: ? C L1/L0 T G S W D M1/M0")

def tl_line():
    global mode, kills
    t = time.time() - t0
    if VARIANT == "uno":
        phase = t % 14.0
        if mode == "S":
            return f"TL m=S a={slave_angle} d=999 k={kills} s=0"
        if phase < 6:                      # патруль
            a = int(15 + (abs((phase * 25) % 300 - 150)))
            return f"TL m=P a={a} d=999 k={kills} s=0"
        else:                              # сопровождение цели
            if 8.9 < phase < 9.1:
                kills += 1
                w(f"*** Цель N{kills} поражена. Занесена в журнал ***")
            a = int(95 + 5 * math.sin(t * 6))
            d = int(41 + 3 * math.sin(t * 2.3))
            return f"TL m=T a={a} d={d} k={kills} s=0"
    else:
        phase = t % 16.0
        if phase < 5:
            p = int(90 + 60 * math.sin(t * 0.5))
            return f"TL m=I p={p} t=90 x=0 y=0 d=0 k={kills} r=0"
        else:
            x = int(900 * math.sin(t * 0.9))
            y = 1500 + int(220 * math.sin(t * 0.53))
            d = int(math.hypot(x, y))
            az = math.degrees(math.atan2(x, y))
            p = int(90 - az)
            tilt = 90 + int(math.degrees(math.atan2(350, d)))
            if 10.9 < phase < 11.1:
                kills += 1
                w(f"*** Цель N{kills} поражена. Занесена в журнал ***")
            return f"TL m=T p={p} t={tilt} x={x} y={y} d={d} k={kills} r=0"

def telemetry_loop():
    while True:
        if telemetry:
            w(tl_line())
        time.sleep(0.2)

threading.Thread(target=telemetry_loop, daemon=True).start()
time.sleep(0.3)
banner()

buf = b""
while True:
    chunk = ser.read(64)
    if not chunk:
        continue
    buf += chunk
    while b"\n" in buf:
        raw, buf = buf.split(b"\n", 1)
        cmd = raw.decode("ascii", errors="replace").strip()
        if not cmd:
            continue
        if cmd == "G":
            for k, v in params.items():
                w(f"PARAM {k}={v}")
            w("PARAM END")
        elif cmd.startswith("S "):
            try:
                _, name, val = cmd.split()
                if name in params:
                    params[name] = int(val)
                    w(f"OK {name}={val}")
                else:
                    w(f"ERR имя или диапазон: {name}")
            except ValueError:
                w("ERR формат: S ИМЯ ЗНАЧЕНИЕ")
        elif cmd == "W":
            w("OK SAVED (параметры и журнал в EEPROM)" if VARIANT == "uno"
              else "OK SAVED (параметры и журнал в NVS)")
        elif cmd == "D":
            w("OK DEFAULTS (в ОЗУ; W — сохранить)")
        elif cmd == "M1":
            telemetry = True; w("OK M1")
        elif cmd == "M0":
            telemetry = False; w("OK M0")
        elif cmd == "?":
            if VARIANT == "uno":
                w(f"СТАТУС: режим=ПАТРУЛЬ, журнал={kills}, датчик=OK, загрузок=7")
            else:
                w(f"СТАТУС: режим=ДЕЖУРСТВО, радар=OK, кадров/с~28, целых=48211, битых=3, журнал={kills}, загрузок=4, аптайм=1841 c")
        elif cmd.startswith("P") and cmd[1:].isdigit():
            mode = "S"; slave_angle = int(cmd[1:])
            w(">>> Внешнее целеуказание принято (SLAVE)")
        elif cmd == "R":
            mode = "P" if VARIANT == "uno" else "I"
            w("<<< Цель потеряна — возвращаюсь к патрулированию")
        elif cmd == "C":
            w("Серво в центр на 8 с — юстируйте крепления")
        elif cmd == "L1":
            w("Лазер ВКЛ (тест). L0 — выключить")
        elif cmd == "L0":
            w("Лазер ВЫКЛ")
        elif cmd == "T":
            w("Команда инициализации радара отправлена повторно")

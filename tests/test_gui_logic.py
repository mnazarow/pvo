# -*- coding: utf-8 -*-
"""Тесты логики pvo_gui.py: разбор протокола, CSV, демо-турель.
Не требуют дисплея и pyserial. Запуск: python3 tests/test_gui_logic.py"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import pvo_gui as g

# --- разбор протокола ---
tl = g.parse_kv_line("TL m=T a=93 d=42 k=5 s=0", "TL")
assert tl == {"m": "T", "a": "93", "d": "42", "k": "5", "s": "0"}
assert g.parse_kv_line("прочее", "TL") is None
assert g.parse_param_line("PARAM DETECT=60") == ("DETECT", 60)
assert g.parse_param_line("PARAM END") == ("END", None)
assert g.parse_param_line("OK SAVED") is None
assert g.detect_variant(["DETECT", "LOST"]) == "uno"
assert g.detect_variant(["MAXR", "PANC"]) == "esp32"
print("[TEST] разбор протокола: OK")

# --- имена параметров GUI существуют в прошивках ---
root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
uno = open(os.path.join(root, "pvo.ino"), encoding="utf8").read()
esp = open(os.path.join(root, "pvo_esp32", "pvo_esp32.ino"), encoding="utf8").read()
for n, *_ in g.PARAMS_UNO:
    assert f'"{n}"' in uno, f"параметра {n} нет в pvo.ino"
for n, *_ in g.PARAMS_ESP32:
    assert f'"{n}"' in esp, f"параметра {n} нет в pvo_esp32.ino"
print("[TEST] сверка имён параметров с прошивками: OK")

# --- CSV журнала ---
rows = [
    {"time": "2026-08-28 10:00:00", "variant": "Uno", "a": 93, "d": 42,
     "x": "", "y": "", "dist_mm": ""},
    {"time": "2026-08-28 10:05:00", "variant": "ESP32", "a": "", "d": "",
     "x": -500, "y": 1500, "dist_mm": 1581},
]
csv = g.format_hits_csv(rows)
assert csv.splitlines()[0].startswith("время;прошивка")
assert "93;42" in csv and "-500;1500;1581" in csv
print("[TEST] CSV журнала: OK")


# --- демо-турель ---
def drain(dev, dur=0.0):
    end = time.monotonic() + dur
    out = b""
    while True:
        out += dev.read(256)
        if time.monotonic() >= end and not dev.out:
            break
        time.sleep(0.01)
    return out.decode("utf-8", "replace")


d = g.DemoSerial("uno")
assert "ДЕМО" in drain(d)
d.write(b"G\n")
txt = drain(d)
assert "PARAM DETECT=60" in txt and "PARAM END" in txt
d.write(b"S DETECT 45\n")
assert "OK DETECT=45" in drain(d) and d.params["DETECT"] == 45
d.write(b"M1\n"); drain(d)
time.sleep(0.45)
assert "TL m=" in drain(d)
d.t0 = time.monotonic() - 9.5           # фаза «сопровождение» с поражением
k0 = d.kills
time.sleep(0.25)
txt = drain(d)
assert d.kills == k0 + 1 and "поражена" in txt
print("[TEST] демо-турель Uno: OK")

e = g.DemoSerial("esp32")
drain(e)
e.write(b"G\n")
names = [l.split()[1].split("=")[0] for l in drain(e).splitlines()
         if l.startswith("PARAM ") and "=" in l]
assert g.detect_variant(names) == "esp32"
print("[TEST] демо-турель ESP32: OK")

print("\nALL GUI LOGIC TESTS PASSED")

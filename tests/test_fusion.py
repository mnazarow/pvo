# -*- coding: utf-8 -*-
"""Тесты pvo_fusion.py: геометрия, детектор ярких точек, мозг слияния.
Запуск: python3 tests/test_fusion.py"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import numpy as np
import cv2
import pvo_fusion as f

cfg = dict(f.CONFIG)
W, H = cfg["FRAME_W"], cfg["FRAME_H"]

# --- геометрия ---
assert abs(f.elevation_from_px(H / 2, H, 46.5)) < 1e-9
assert f.elevation_from_px(0, H, 40.0) == 20.0            # верх кадра -> +пол-FOV
assert f.elevation_from_px(H, H, 40.0) == -20.0
assert abs(f.fov_v(cfg) - cfg["FOV_H_DEG"] * H / W) < 1e-6
tl = f.parse_tl("TL m=T p=95 t=100 x=1 y=2 d=3 k=4 r=0 v=0 i=1")
assert tl["m"] == "T" and tl["i"] == "1"
assert f.parse_tl("не телеметрия") is None
print("[TEST] геометрия и разбор TL: OK")

# --- детектор ярких точек ---
bcfg = dict(cfg); bcfg["DETECTOR"] = "bright"
det = f.BrightDetector(bcfg)
dark = np.zeros((H, W, 3), np.uint8)
assert det.find_target(dark) is None
spot = dark.copy()
cv2.circle(spot, (420, 180), 4, (255, 255, 255), -1)      # «комар в ИК-луче»
hit = det.find_target(spot)
assert hit is not None and abs(hit[0] - 420) < 3 and abs(hit[1] - 180) < 3
one_px = dark.copy(); one_px[10, 10] = (255, 255, 255)    # одиночный пиксель — шум
assert det.find_target(one_px) is None
print("[TEST] детектор ярких точек: OK")

# --- мозг: дрёма и пробуждение по радару ---
brain = f.FusionBrain(bcfg)
processed = 0
for i in range(16):                                        # дрёма: 1 кадр из WAKE_EVERY
    cmds, st = brain.on_frame(dark, now=i * 0.1)
    if st != "дрёма":
        processed += 1
assert processed == 16 // bcfg["WAKE_EVERY"]
cmds = brain.on_telemetry({"m": "T", "p": "90"})
assert brain.awake and cmds == ["I1"]                      # проснулись + ИК
print("[TEST] мозг: дрёма и пробуждение радаром: OK")

# --- поправки: знаки и формат V ---
right_up = np.zeros((H, W, 3), np.uint8)
cv2.circle(right_up, (W // 2 + 160, H // 2 - 120), 5, (255, 255, 255), -1)
cmds, st = brain.on_frame(right_up, now=100.0)
v = [c for c in cmds if c.startswith("V ")]
assert len(v) == 1
dp10, dt10 = map(int, v[0].split()[1:])
assert dp10 > 0 and dt10 > 0                               # правее -> dp>0, выше -> dt>0
# ограничение частоты: следующий кадр сразу — без V
cmds2, _ = brain.on_frame(right_up, now=100.01)
assert not any(c.startswith("V ") for c in cmds2)
print(f"[TEST] поправки V (dp={dp10}, dt={dt10}) и rate-limit: OK")

# --- захват зрением из дрёмы (радар молчит) ---
brain2 = f.FusionBrain(bcfg)
brain2.frame_i = bcfg["WAKE_EVERY"] - 1                    # следующий кадр — рабочий
cmds, st = brain2.on_frame(right_up, now=5.0)
assert brain2.awake and "I1" in cmds and any(c.startswith("V ") for c in cmds)
print("[TEST] захват зрением при молчащем радаре: OK")

# --- потеря: оба молчат -> сон и I0 ---
for i in range(bcfg["LOST_FRAMES"]):
    brain2.on_frame(dark, now=6.0 + i)
cmds = brain2.on_telemetry({"m": "I", "p": "90"})
assert not brain2.awake and cmds == ["I0"]
print("[TEST] отбой: зрение засыпает, ИК гаснет: OK")

# --- движение турели: motion-кадры пропускаются, bright — нет ---
mcfg = dict(cfg); mcfg["DETECTOR"] = "motion"
bm = f.FusionBrain(mcfg)
bm.on_telemetry({"m": "T", "p": "90"})
bm.on_telemetry({"m": "T", "p": "104"})                    # рывок 14°
assert bm.turret_moving
cmds, st = bm.on_frame(dark, now=1.0)
assert st == "турель движется"
bb = f.FusionBrain(bcfg)
bb.on_telemetry({"m": "T", "p": "90"})
bb.on_telemetry({"m": "T", "p": "104"})
cmds, st = bb.on_frame(right_up, now=2.0)
assert any(c.startswith("V ") for c in cmds)               # bright движения не боится
print("[TEST] пропуск кадров при движении (только motion): OK")

# --- ограничение поправки MAX_CORR_DEG ---
edge = np.zeros((H, W, 3), np.uint8)
cv2.circle(edge, (530, H // 2), 5, (255, 255, 255), -1)   # у края ROI: азимут > лимита
be = f.FusionBrain(bcfg)
be.on_telemetry({"m": "T", "p": "90"})
cmds, _ = be.on_frame(edge, now=50.0)
v = [c for c in cmds if c.startswith("V ")][0]
assert abs(int(v.split()[1])) <= bcfg["MAX_CORR_DEG"] * 10
assert int(v.split()[1]) == int(bcfg["MAX_CORR_DEG"] * 10)   # именно кламп
print("[TEST] ограничение поправки: OK")

# --- пристрелка: поправки считаются от точки лазера, а не от центра ---
acfg = dict(bcfg)
acfg["AIM_DX_PX"], acfg["AIM_DY_PX"] = 60.0, -40.0     # лазер правее и выше центра
ax, ay = f.aim_point(acfg)
assert ax == W / 2 + 60 and ay == H / 2 - 40
dp, dt = f.offsets_deg(ax, ay, W, H, acfg)             # цель ровно на лазере
assert abs(dp) < 1e-9 and abs(dt) < 1e-9               # доворачивать нечего
dp0, _ = f.offsets_deg(W / 2, H / 2, W, H, acfg)       # цель в центре кадра
assert dp0 < 0                                          # ...значит левее лазера
print("[TEST] пристрелка: поправки от точки лазера: OK")

# сохранение и загрузка файла пристрелки
import tempfile, os as _os
tmp = _os.path.join(tempfile.mkdtemp(), "aim.json")
f.save_aim(acfg, 12.5, -7.5, tmp)
lcfg = dict(cfg)
assert f.load_aim(lcfg, tmp) and lcfg["AIM_DX_PX"] == 12.5 and lcfg["AIM_DY_PX"] == -7.5
assert not f.load_aim(dict(cfg), tmp + ".нет")         # нет файла -> False, без падения
print("[TEST] файл пристрелки: сохранение и загрузка: OK")

# поиск лазерного пятна
shot = np.zeros((H, W, 3), np.uint8)
cv2.circle(shot, (400, 150), 6, (255, 255, 255), -1)
dot = f.find_laser_dot(shot, cfg)
assert dot is not None and abs(dot[0] - 400) < 6 and abs(dot[1] - 150) < 6
assert f.find_laser_dot(np.full((H, W, 3), 40, np.uint8), cfg) is None   # темно -> None
print("[TEST] поиск точки лазера: OK")

# --- мёртвая зона: цель в перекрестье -> V 0 0 (захват не отпускаем) ---
dz = f.FusionBrain(bcfg)
dz.on_telemetry({"m": "T", "p": "90"})
center = np.zeros((H, W, 3), np.uint8)
cv2.circle(center, (W // 2, H // 2), 5, (255, 255, 255), -1)
cmds, st = dz.on_frame(center, now=200.0)
assert "V 0 0" in cmds and "перекрестье" in st
print("[TEST] мёртвая зона: V 0 0 удерживает захват: OK")

# за пределами мёртвой зоны команда снова содержит поправку
off = np.zeros((H, W, 3), np.uint8)
cv2.circle(off, (W // 2 + 90, H // 2), 5, (255, 255, 255), -1)
cmds, _ = dz.on_frame(off, now=201.0)
v = [c for c in cmds if c.startswith("V ")][0]
assert int(v.split()[1]) > 0
print("[TEST] вне мёртвой зоны: поправка идёт: OK")

# --- проверка версии прошивки ---
assert f.parse_fw("СТАТУС: режим=ДЕЖУРСТВО, аптайм=12 c, прошивка=1.3") == "1.3"
assert f.parse_fw("TL m=I p=90") is None
assert f.fw_ok("1.2") and f.fw_ok("1.3") and f.fw_ok("2.0")
assert not f.fw_ok("1.1") and not f.fw_ok(None) and not f.fw_ok("абв")
assert f.report_firmware("1.3") is True
assert f.report_firmware("1.1") is False and f.report_firmware(None) is False
print("[TEST] проверка версии прошивки: OK")

print("\nALL FUSION TESTS PASSED")

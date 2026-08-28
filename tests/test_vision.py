# -*- coding: utf-8 -*-
"""Тесты pvo_vision.py: детектор, геометрия, конфиг, фотофиксация.
Зависимости: pip install opencv-python-headless numpy
Запуск: python3 tests/test_vision.py (из корня проекта или из tests/)."""
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import numpy as np
import cv2
import pvo_vision as v

cfg = dict(v.CONFIG)
det = v.MotionDetector(cfg)
W, H = cfg["FRAME_W"], cfg["FRAME_H"]


def frame_with_blob(cx):
    f = np.zeros((H, W, 3), np.uint8)
    if cx is not None:
        cv2.circle(f, (int(cx), H // 2), 12, (255, 255, 255), -1)
    return f


# прогрев фона
for _ in range(60):
    det.find_target(frame_with_blob(None))

# движущийся шарик слева направо: азимут монотонно растёт, pan — падает
angles = []
for cx in range(60, 580, 20):
    hit = det.find_target(frame_with_blob(cx))
    if hit:
        az = v.azimuth_from_px(hit[0], W, cfg["FOV_H_DEG"])
        angles.append((az, v.pan_from_azimuth(az, cfg)))
assert len(angles) >= 20, f"мало детекций: {len(angles)}"
azs = [a for a, _ in angles]
pans = [p for _, p in angles]
assert all(azs[i] < azs[i + 1] for i in range(len(azs) - 1))
assert all(pans[i] >= pans[i + 1] for i in range(len(pans) - 1))
assert azs[0] < -20 and azs[-1] > 20
print("[TEST] детектор и монотонность наведения: OK")

# геометрия
assert abs(v.azimuth_from_px(W / 2, W, 62.0)) < 1e-9
assert v.pan_from_azimuth(0.0, cfg) == 90
assert v.pan_from_azimuth(31.0, cfg) == 59
assert v.pan_from_azimuth(-31.0, cfg) == 121
assert v.pan_from_azimuth(200.0, cfg) == cfg["PAN_MIN"]
print("[TEST] геометрия: OK")

# валидация конфига
bad = dict(cfg); bad["FOV_H_DEG"] = 200; bad["MIN_AREA_PX"] = 10**6
assert len(v.validate_config(bad)) == 2
assert v.validate_config(dict(v.CONFIG)) == []
print("[TEST] валидация конфига: OK")

# dry-run линк
link = v.TurretLink(cfg, dry_run=True)
assert link.connected()
link.last_send = 0
assert link.point(115) is True
link.release()
print("[TEST] линк (dry-run): OK")

# фотофиксация
with tempfile.TemporaryDirectory() as tmp:
    for _ in range(40):
        det.find_target(frame_with_blob(None))
    hit = det.find_target(frame_with_blob(420))
    assert hit is not None
    p = v.save_hit_frame(frame_with_blob(420), hit, 9.7, tmp)
    assert p and os.path.exists(p)
    assert cv2.imread(p) is not None
print("[TEST] фотофиксация: OK")


# Telegram: кулдаун, транспорт, устойчивость к ошибкам сети
import time as _t
sent = []
cfgT = dict(v.CONFIG); cfgT["TG_TOKEN"] = "T"; cfgT["TG_CHAT"] = "42"; cfgT["TG_COOLDOWN_S"] = 0.3
tg = v.TelegramNotifier(cfgT, transport=lambda tok, ch, tx, im: sent.append((tx, im)))
assert tg.enabled()
assert tg.notify("захват", "/tmp/x.jpg") is True
assert tg.notify("слишком часто") is False
_t.sleep(0.45)
assert tg.notify("ещё захват") is True
_t.sleep(0.2)
assert len(sent) == 2 and sent[0] == ("захват", "/tmp/x.jpg")
assert not v.TelegramNotifier(dict(v.CONFIG)).enabled()
def _boom(*a): raise RuntimeError("сеть")
tg2 = v.TelegramNotifier(cfgT, transport=_boom); tg2.last = 0
assert tg2.notify("авария") is True
_t.sleep(0.2)
print("[TEST] Telegram-уведомления: OK")

print("\nALL VISION TESTS PASSED")

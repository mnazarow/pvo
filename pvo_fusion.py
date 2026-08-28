#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
============================================================
 ПВО-4К «Комар-Ф» — слияние радара и зрения (вариант D)
============================================================
Работает в паре с прошивкой pvo_esp32.ino (v1.2+): радар даёт
ДАЛЬНЕЕ обнаружение и грубый доворот турели, камера НА ТУРЕЛИ
(соосно лазеру) даёт ТОЧНОЕ сопровождение, ИК-подсветка
открывает охоту на мелочь, которую радар не видит вовсе.

Как это работает:
  1. Скрипт слушает телеметрию прошивки (M1 -> строки TL).
  2. Пока радар никого не видит — зрение «дремлет»: обрабатывает
     лишь каждый WAKE_EVERY-й кадр (экономия CPU Raspberry Pi).
     Если в дрёме цель всё же найдена (мелочь!) — зрение само
     захватывает её командой V: радар для захвата не обязателен.
  3. Радар захватил (TL m=T) — прошивка уже грубо довернула
     турель; зрение просыпается, включает ИК (I1) и ищет цель
     около центра кадра.
  4. Нашло — шлёт точные поправки «V dPAN dTILT» (десятые
     градуса). Прошивка ведёт цель по зрению; радар страхует.
  5. Зрение потеряло цель — просто перестаёт слать V, прошивка
     сама вернётся к радарному сопровождению (таймаут VISTO).
     Потерялись оба — турель на дежурство, ИК гаснет (I0).

Детекторы:
  motion — вычитание фона (как в варианте C); при движущейся
           турели кадры пропускаются, фон переучивается.
  bright — ИК-охота: порог яркости. Мелкая цель в луче ИК на
           тёмном фоне = яркая точка; работает и при движении
           турели. Нужна камера БЕЗ ИК-фильтра (Pi NoIR или
           вебка со снятым фильтром) и тёмный однородный фон.

Запуск:
  python3 pvo_fusion.py                     # порт ESP32 сам найдётся
  python3 pvo_fusion.py --detector bright   # режим ИК-охоты
  python3 pvo_fusion.py --display --save-hits
  python3 pvo_fusion.py --dry-run           # без турели (отладка)

Зависимости — как у варианта C (python3-opencv, python3-serial).
Подробности и монтаж камеры на турель — руководство, глава 13.
============================================================
"""

import argparse
import sys
import time
from typing import Optional

import cv2
import numpy as np

import pvo_vision as pv   # общие части проекта (вариант C)

# ==================== ПАРАМЕТРЫ =============================
CONFIG = {
    # --- камера (на площадке турели, соосно лазеру!) ---
    "CAMERA_INDEX": 0,
    "FRAME_W": 640,
    "FRAME_H": 480,
    "FOV_H_DEG": 62.0,     # горизонтальный угол обзора ВАШЕЙ камеры
    "FOV_V_DEG": 0.0,      # вертикальный; 0 = вычислить из FOV_H и сторон кадра

    # --- детектор ---
    "DETECTOR": "motion",  # motion | bright (ИК-охота)
    "BRIGHT_THRESH": 200,  # порог яркой точки (0..255), режим bright
    "MIN_AREA_PX": 4,      # мелочь! (у варианта C по умолчанию 25)
    "MAX_AREA_PX": 20000,
    "BG_HISTORY": 90,      # параметры MOG2 для режима motion
    "BG_THRESHOLD": 28.0,
    "MORPH_KERNEL": 3,

    # --- логика слияния ---
    "ROI": 0.7,            # доля центра кадра для поиска (радар уже довернул)
    "WAKE_EVERY": 8,       # в дрёме обрабатывать каждый N-й кадр
    "LOST_FRAMES": 10,     # пустых кадров подряд -> зрение отпускает цель
    "SEND_HZ": 15,         # частота команд V
    "MAX_CORR_DEG": 20.0,  # ограничение одной поправки, °
    "PAN_MOVE_EPS": 1.5,   # °/период телеметрии: турель движется -> кадр мимо (motion)
    "IR_CONTROL": True,    # управлять ИК-подсветкой (I1/I0)

    # --- связь с ESP32 ---
    "SERIAL_PORT": "auto",
    "BAUD": 115200,
    "RECONNECT_S": 2.0,
    "CAM_FAIL_LIMIT": 25,

    # --- фотофиксация и Telegram (как в варианте C) ---
    "HITS_DIR": "hits",
    "HITS_COOLDOWN_S": 3.0,
    "TG_TOKEN": "",
    "TG_CHAT": "",
    "TG_COOLDOWN_S": 30.0,
}


# ==================== ГЕОМЕТРИЯ =============================
def elevation_from_px(cy: float, frame_h: int, fov_v_deg: float) -> float:
    """Пиксель центра цели -> угол места от оси камеры, ° (+ вверх)."""
    half = frame_h / 2.0
    return (half - cy) / half * (fov_v_deg / 2.0)


def fov_v(cfg: dict) -> float:
    if cfg["FOV_V_DEG"] > 0:
        return cfg["FOV_V_DEG"]
    return cfg["FOV_H_DEG"] * cfg["FRAME_H"] / cfg["FRAME_W"]


def parse_tl(line: str) -> Optional[dict]:
    """Строка телеметрии 'TL k=v …' -> словарь (значения строками)."""
    if not line.startswith("TL "):
        return None
    out = {}
    for tok in line[3:].split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k] = v
    return out


# ==================== ДЕТЕКТОР ЯРКИХ ТОЧЕК ==================
class BrightDetector:
    """ИК-охота: цель = яркое пятно на тёмном фоне. Не боится
    движения турели, видит объекты в считанные пиксели."""

    def __init__(self, cfg: dict):
        self.cfg = cfg
        k = cfg["MORPH_KERNEL"]
        self.kernel = np.ones((k, k), np.uint8)

    def find_target(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        _, mask = cv2.threshold(gray, self.cfg["BRIGHT_THRESH"], 255, cv2.THRESH_BINARY)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best, best_area = None, 0.0
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.cfg["MIN_AREA_PX"] or area > self.cfg["MAX_AREA_PX"]:
                continue
            if area > best_area:
                best_area, best = area, c
        if best is None:
            return None
        x, y, w, h = cv2.boundingRect(best)
        return (x + w / 2.0, y + h / 2.0, best_area, (x, y, w, h))


def make_detector(cfg: dict):
    if cfg["DETECTOR"] == "bright":
        return BrightDetector(cfg)
    mcfg = dict(pv.CONFIG)
    mcfg.update({k: cfg[k] for k in
                 ("BG_HISTORY", "BG_THRESHOLD", "MORPH_KERNEL",
                  "MIN_AREA_PX", "MAX_AREA_PX")})
    return pv.MotionDetector(mcfg)


# ==================== МОЗГ СЛИЯНИЯ ==========================
class FusionBrain:
    """Ядро логики (тестируется без камеры и порта): принимает
    телеметрию и кадры, возвращает команды для прошивки."""

    def __init__(self, cfg: dict, detector_factory=None):
        self.cfg = cfg
        self.make_det = detector_factory or (lambda: make_detector(cfg))
        self.det = self.make_det()
        self.awake = False
        self.ir_on = False
        self.frame_i = 0
        self.lost = 0
        self.last_pan = None
        self.turret_moving = False
        self.last_send = 0.0
        self.radar_mode = "I"
        self.hit_flash = None          # последний найденный hit (для HUD/фото)

    # ---- телеметрия прошивки ----
    def on_telemetry(self, tl: dict) -> list:
        cmds = []
        try:
            p = int(tl.get("p", "90"))
            if self.last_pan is not None:
                self.turret_moving = abs(p - self.last_pan) > self.cfg["PAN_MOVE_EPS"]
            self.last_pan = p
        except ValueError:
            pass
        self.radar_mode = tl.get("m", self.radar_mode)

        if self.radar_mode == "T" and not self.awake:
            cmds += self._wake("радар захватил — зрение проснулось")
        elif self.radar_mode == "I" and self.awake and self.lost >= self.cfg["LOST_FRAMES"]:
            cmds += self._sleep("оба потеряли цель — зрение дремлет")
        return cmds

    def _wake(self, why: str) -> list:
        self.awake = True
        self.lost = 0
        self.det = self.make_det()          # свежий фон
        print(f"[fusion] {why}")
        if self.cfg["IR_CONTROL"] and not self.ir_on:
            self.ir_on = True
            return ["I1"]
        return []

    def _sleep(self, why: str) -> list:
        self.awake = False
        print(f"[fusion] {why}")
        if self.ir_on:
            self.ir_on = False
            return ["I0"]
        return []

    # ---- кадры камеры ----
    def on_frame(self, frame, now: Optional[float] = None):
        """-> (список команд, строка-статус для HUD)."""
        now = time.monotonic() if now is None else now
        cfg = self.cfg
        self.frame_i += 1
        self.hit_flash = None

        if not self.awake and self.frame_i % cfg["WAKE_EVERY"] != 0:
            return [], "дрёма"
        if (cfg["DETECTOR"] == "motion") and self.turret_moving:
            self.det = self.make_det()      # кадр «поплыл» — фон заново
            return [], "турель движется"

        h, w = frame.shape[:2]
        roi = cfg["ROI"] if self.awake else 1.0   # в дрёме смотрим весь кадр
        x0 = int(w * (1 - roi) / 2)
        y0 = int(h * (1 - roi) / 2)
        crop = frame[y0:h - y0 or h, x0:w - x0 or w]

        hit = self.det.find_target(crop)
        if hit is None:
            self.lost += 1
            return [], "цели нет"
        cx, cy = hit[0] + x0, hit[1] + y0
        self.hit_flash = (cx, cy, hit[2], (hit[3][0] + x0, hit[3][1] + y0,
                                           hit[3][2], hit[3][3]))
        self.lost = 0

        cmds = []
        if not self.awake:                  # зрение само нашло мелочь
            cmds += self._wake("зрение видит цель — захват (радар молчит)")

        dp = pv.azimuth_from_px(cx, w, cfg["FOV_H_DEG"])
        dt = elevation_from_px(cy, h, fov_v(cfg))
        lim = cfg["MAX_CORR_DEG"]
        dp = max(-lim, min(lim, dp))
        dt = max(-lim, min(lim, dt))

        if now - self.last_send >= 1.0 / cfg["SEND_HZ"]:
            self.last_send = now
            cmds.append(f"V {round(dp * 10)} {round(dt * 10)}")
        return cmds, f"цель {dp:+.1f}°/{dt:+.1f}°"


# ==================== СВЯЗЬ С ESP32 =========================
class EspLink:
    """Порт ESP32: отправка команд + чтение телеметрии, с
    автопереподключением (та же философия, что в варианте C)."""

    def __init__(self, cfg: dict, dry_run=False):
        self.cfg = cfg
        self.dry = dry_run
        self.ser = None
        self.buf = b""
        self.last_reconnect = 0.0
        self.last_m1 = 0.0
        if not dry_run:
            self._connect(initial=True)

    def _connect(self, initial=False):
        import serial
        port = pv.find_serial_port(self.cfg["SERIAL_PORT"])
        if port is None:
            if initial:
                print("ОШИБКА: порт ESP32 не найден (USB-кабель? питание?) — ищу дальше…")
            return False
        try:
            self.ser = serial.Serial(port, self.cfg["BAUD"], timeout=0)
            print(f"[link] ESP32 подключена: {port}")
            time.sleep(1.5)
            self.send("M1")
            return True
        except Exception as e:
            if initial:
                print(f"[link] не открыть {port}: {e}")
            self.ser = None
            return False

    def _maybe_reconnect(self):
        now = time.monotonic()
        if now - self.last_reconnect < self.cfg["RECONNECT_S"]:
            return
        self.last_reconnect = now
        if self._connect():
            print("[link] связь с турелью восстановлена")

    def send(self, cmd: str):
        if self.dry:
            print(f"[serial] {cmd}")
            return
        if self.ser is None:
            self._maybe_reconnect()
            return
        try:
            self.ser.write((cmd + "\n").encode("ascii"))
        except Exception as e:
            print(f"[link] ОБРЫВ СВЯЗИ ({e}) — переподключаюсь…")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def read_lines(self) -> list:
        if self.dry or self.ser is None:
            if not self.dry:
                self._maybe_reconnect()
            return []
        # периодически напоминаем M1 (вдруг плата перезагрузилась)
        now = time.monotonic()
        if now - self.last_m1 > 5.0:
            self.last_m1 = now
            self.send("M1")
        out = []
        try:
            chunk = self.ser.read(512)
        except Exception as e:
            print(f"[link] ОБРЫВ СВЯЗИ ({e}) — переподключаюсь…")
            self.ser = None
            return []
        if chunk:
            self.buf += chunk
            while b"\n" in self.buf:
                raw, self.buf = self.buf.split(b"\n", 1)
                line = raw.decode("utf-8", errors="replace").strip()
                if line:
                    out.append(line)
        return out

    def close(self):
        if self.ser is not None:
            try:
                self.send("I0")
                self.send("M0")
                self.ser.close()
            except Exception:
                pass


# ==================== ГЛАВНЫЙ ЦИКЛ ==========================
def main():
    p = argparse.ArgumentParser(description="ПВО-4К: слияние радара и зрения")
    p.add_argument("--camera", type=int, help="индекс USB-камеры")
    p.add_argument("--picamera", action="store_true", help="CSI-модуль камеры")
    p.add_argument("--port", help="порт ESP32 (по умолчанию автопоиск)")
    p.add_argument("--fov", type=float, help="горизонтальный FOV камеры, °")
    p.add_argument("--detector", choices=["motion", "bright"], help="детектор")
    p.add_argument("--display", action="store_true", help="окно отладки")
    p.add_argument("--dry-run", action="store_true", help="без турели")
    p.add_argument("--save-hits", action="store_true", help="кадры захватов в hits/")
    p.add_argument("--tg-token", help="токен Telegram-бота (или env PVO_TG_TOKEN)")
    p.add_argument("--tg-chat", help="id чата Telegram (или env PVO_TG_CHAT)")
    args = p.parse_args()

    cfg = dict(CONFIG)
    if args.camera is not None: cfg["CAMERA_INDEX"] = args.camera
    if args.port: cfg["SERIAL_PORT"] = args.port
    if args.fov: cfg["FOV_H_DEG"] = args.fov
    if args.detector: cfg["DETECTOR"] = args.detector
    if args.tg_token: cfg["TG_TOKEN"] = args.tg_token
    if args.tg_chat: cfg["TG_CHAT"] = args.tg_chat

    cam = pv.open_camera_retry(cfg, args.picamera)
    brain = FusionBrain(cfg)
    link = EspLink(cfg, dry_run=args.dry_run)
    tg = pv.TelegramNotifier(cfg)

    was_awake = False
    last_hit_shot = 0.0
    frames = 0
    t0 = time.monotonic()

    print(f"ПВО-4К: слияние запущено (детектор: {cfg['DETECTOR']}). Ctrl+C — выход.")
    if cfg["DETECTOR"] == "bright":
        print("Режим ИК-охоты: нужна камера без ИК-фильтра и тёмный фон (глава 13).")

    try:
        while True:
            for line in link.read_lines():
                tl = parse_tl(line)
                if tl is not None:
                    for c in brain.on_telemetry(tl):
                        link.send(c)
                elif "ЗАХВАЧЕНА" in line or "потеряна" in line or "ТРЕВОГА" in line:
                    print(f"[турель] {line}")

            frame = pv.read_frame(cam)
            if frame is None:
                time.sleep(0.03)
                continue
            frames += 1

            cmds, status = brain.on_frame(frame)
            for c in cmds:
                link.send(c)

            # фотофиксация и Telegram — по свежему «своему» захвату
            if brain.hit_flash and not was_awake and brain.awake:
                if args.save_hits and time.monotonic() - last_hit_shot >= cfg["HITS_COOLDOWN_S"]:
                    last_hit_shot = time.monotonic()
                    saved = pv.save_hit_frame(frame, brain.hit_flash, 0.0, cfg["HITS_DIR"])
                    if saved:
                        print(f"[hits] кадр сохранён: {saved}")
                        tg.notify("🎯 ПВО-4К: зрение захватило цель", image_path=saved)
                else:
                    tg.notify("🎯 ПВО-4К: зрение захватило цель")
            was_awake = brain.awake

            if args.display:
                if brain.hit_flash:
                    x, y, w, h = brain.hit_flash[3]
                    cv2.rectangle(frame, (int(x), int(y)), (int(x + w), int(y + h)),
                                  (0, 0, 255), 2)
                cv2.putText(frame, f"{'БОДРСТВУЮ' if brain.awake else 'дрёма'} · {status}",
                            (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.imshow("PVO-4K fusion", frame)
                if cv2.waitKey(1) & 0xFF == 27:
                    break

            if frames % 300 == 0:
                fps = frames / (time.monotonic() - t0)
                print(f"[stat] {fps:.1f} кадров/с, зрение: "
                      f"{'активно' if brain.awake else 'дремлет'}")
    except KeyboardInterrupt:
        pass
    finally:
        link.close()
        pv.close_camera(cam)
        cv2.destroyAllWindows()
        print("Остановлено. Турель осталась в автономном (радарном) режиме.")


if __name__ == "__main__":
    main()

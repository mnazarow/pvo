#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
============================================================
 ПВО-3К «Комар-В» — компьютерное зрение для турели (вариант C)
============================================================
Raspberry Pi + камера находят цель, Arduino (pvo.ino, режим
SLAVE) наводит турель. Схема и сборка — «Сборка варианта C».

Конвейер: кадр -> вычитание фона MOG2 -> контуры -> цель ->
азимут (через FOV камеры) -> угол серво -> "P<угол>\n" в порт.
Нет цели LOST_FRAMES кадров -> "R\n" (турель — в автономию).

ОБРАБОТКА ОШИБОК:
 - автопоиск порта Arduino (если SERIAL_PORT не найден,
   сканируются /dev/ttyUSB*, /dev/ttyACM*, COM*);
 - автопереподключение к Arduino: обрыв USB не роняет скрипт,
   связь восстанавливается сама (RECONNECT_S);
 - контроль камеры: серия пустых кадров -> переоткрытие камеры;
 - проверка конфигурации на старте с понятными сообщениями;
 - корректное завершение по Ctrl+C: команда R, освобождение
   камеры и порта.

Запуск:
  python3 pvo_vision.py                      # автопоиск порта
  python3 pvo_vision.py --display            # окно отладки
  python3 pvo_vision.py --dry-run            # без Arduino
  python3 pvo_vision.py --picamera --fov 66  # CSI-модуль камеры

Зависимости (Raspberry Pi OS Bookworm):
  sudo apt install python3-opencv python3-serial python3-numpy
  (CSI-камера: sudo apt install python3-picamera2, флаг --picamera)
============================================================
"""

import argparse
import glob
import sys
import time

import cv2
import numpy as np

# ==================== ПАРАМЕТРЫ =============================
CONFIG = {
    # --- камера ---
    "CAMERA_INDEX": 0,      # номер /dev/video* для USB-камеры
    "FRAME_W": 640,
    "FRAME_H": 480,
    "FOV_H_DEG": 62.0,      # горизонтальный угол обзора ВАШЕЙ камеры, °

    # --- геометрия турели (должна совпадать с pvo.ino) ---
    "PAN_CENTER": 90,
    "PAN_MIN": 15,
    "PAN_MAX": 165,
    "PAN_INVERT": False,

    # --- детектор движения ---
    "BG_HISTORY": 120,
    "BG_THRESHOLD": 32.0,
    "MORPH_KERNEL": 3,
    "MIN_AREA_PX": 25,
    "MAX_AREA_PX": 30000,

    # --- наведение ---
    "SMOOTH_ALPHA": 0.4,
    "SEND_HZ": 15,
    "LOST_FRAMES": 12,

    # --- связь и устойчивость ---
    "SERIAL_PORT": "auto",  # "auto" = найти самому; или /dev/ttyUSB0, COM5…
    "BAUD": 9600,
    "RECONNECT_S": 2.0,     # период попыток переподключения к Arduino
    "CAM_FAIL_LIMIT": 25,   # пустых кадров подряд до переоткрытия камеры
}

# ==================== ГЕОМЕТРИЯ =============================
def azimuth_from_px(cx: float, frame_w: int, fov_h_deg: float) -> float:
    """Пиксель центра цели -> азимут от оси камеры, ° (+ вправо)."""
    half = frame_w / 2.0
    return (cx - half) / half * (fov_h_deg / 2.0)


def pan_from_azimuth(az_deg: float, cfg: dict) -> int:
    """Азимут -> угол серво с учётом инверсии и пределов."""
    a = cfg["PAN_CENTER"] + az_deg if cfg["PAN_INVERT"] else cfg["PAN_CENTER"] - az_deg
    return int(round(max(cfg["PAN_MIN"], min(cfg["PAN_MAX"], a))))


def validate_config(cfg: dict) -> list:
    """Возвращает список предупреждений по конфигурации."""
    warn = []
    if not (30 <= cfg["FOV_H_DEG"] <= 120):
        warn.append(f"FOV_H_DEG={cfg['FOV_H_DEG']} выглядит странно (обычно 40–110)")
    if cfg["PAN_MIN"] >= cfg["PAN_MAX"]:
        warn.append("PAN_MIN должен быть меньше PAN_MAX")
    if cfg["MIN_AREA_PX"] >= cfg["MAX_AREA_PX"]:
        warn.append("MIN_AREA_PX должен быть меньше MAX_AREA_PX")
    if not (0 < cfg["SMOOTH_ALPHA"] <= 1):
        warn.append("SMOOTH_ALPHA должен быть в (0..1]")
    if cfg["SEND_HZ"] < 1 or cfg["SEND_HZ"] > 50:
        warn.append("SEND_HZ разумен в пределах 5–30")
    return warn


# ==================== ПОРТ ARDUINO ==========================
def find_serial_port(preferred: str) -> str | None:
    """Ищет порт Arduino: сначала preferred, затем автоскан."""
    candidates = []
    if preferred and preferred != "auto":
        candidates.append(preferred)
    try:
        from serial.tools import list_ports
        for p in list_ports.comports():
            dev = p.device
            if any(k in dev for k in ("ttyUSB", "ttyACM", "COM", "usbserial", "usbmodem")):
                candidates.append(dev)
    except Exception:
        pass
    candidates += sorted(glob.glob("/dev/ttyUSB*")) + sorted(glob.glob("/dev/ttyACM*"))
    seen, ordered = set(), []
    for c in candidates:
        if c not in seen:
            seen.add(c)
            ordered.append(c)
    return ordered[0] if ordered else None


# ==================== ДЕТЕКТОР ==============================
class MotionDetector:
    """Выделение движущейся цели вычитанием фона (MOG2)."""

    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.bg = cv2.createBackgroundSubtractorMOG2(
            history=cfg["BG_HISTORY"],
            varThreshold=cfg["BG_THRESHOLD"],
            detectShadows=False,
        )
        k = cfg["MORPH_KERNEL"]
        self.kernel = np.ones((k, k), np.uint8)

    def find_target(self, frame):
        """(cx, cy, area, bbox) самой крупной цели или None."""
        mask = self.bg.apply(frame)
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


# ==================== СВЯЗЬ С ТУРЕЛЬЮ =======================
class TurretLink:
    """Команды в Arduino c автопереподключением при обрыве USB."""

    def __init__(self, cfg: dict, dry_run: bool = False):
        self.cfg = cfg
        self.dry = dry_run
        self.ser = None
        self.port = None
        self.last_send = 0.0
        self.last_reconnect = 0.0
        self.was_connected = False
        if not dry_run:
            self._connect(initial=True)

    # --- внутреннее ---
    def _connect(self, initial=False):
        import serial
        port = find_serial_port(self.cfg["SERIAL_PORT"])
        if port is None:
            if initial:
                print("ОШИБКА: порт Arduino не найден. Подключите USB-кабель;")
                print("  Nano(CH340) обычно /dev/ttyUSB0, Uno — /dev/ttyACM0, Windows — COM*.")
                print("  Продолжаю работу и буду искать порт дальше…")
            return False
        try:
            self.ser = serial.Serial(port, self.cfg["BAUD"], timeout=0)
            self.port = port
            time.sleep(2.0)          # Uno перезагружается при открытии порта
            print(f"[link] Arduino подключена: {port}")
            self.was_connected = True
            return True
        except Exception as e:
            if initial:
                print(f"[link] не открыть {port}: {e}")
                print("  Linux: sudo usermod -aG dialout $USER и перелогин")
            self.ser = None
            return False

    def _maybe_reconnect(self):
        now = time.monotonic()
        if now - self.last_reconnect < self.cfg["RECONNECT_S"]:
            return
        self.last_reconnect = now
        if self._connect():
            print("[link] связь с турелью восстановлена")

    def _write(self, line: str):
        if self.dry:
            print(f"[serial] {line.strip()}")
            return
        if self.ser is None:
            self._maybe_reconnect()
            return
        try:
            self.ser.write(line.encode("ascii"))
        except Exception as e:
            print(f"[link] ОБРЫВ СВЯЗИ с турелью ({e}) — переподключаюсь…")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    # --- команды ---
    def connected(self) -> bool:
        return self.dry or self.ser is not None

    def point(self, angle: int) -> bool:
        now = time.monotonic()
        if now - self.last_send < 1.0 / self.cfg["SEND_HZ"]:
            return False
        self.last_send = now
        self._write(f"P{angle}\n")
        return True

    def release(self):
        self._write("R\n")

    def close(self):
        if self.ser is not None:
            try:
                self.release()
                self.ser.close()
            except Exception:
                pass


# ==================== КАМЕРА ================================
def open_camera(cfg: dict, use_picamera: bool):
    if use_picamera:
        try:
            from picamera2 import Picamera2
        except ImportError:
            sys.exit("Нет picamera2: sudo apt install python3-picamera2 (или уберите --picamera)")
        cam = Picamera2()
        cam.configure(cam.create_video_configuration(
            main={"size": (cfg["FRAME_W"], cfg["FRAME_H"]), "format": "RGB888"}))
        cam.start()
        return ("picamera", cam)
    cap = cv2.VideoCapture(cfg["CAMERA_INDEX"])
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg["FRAME_W"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg["FRAME_H"])
    if not cap.isOpened():
        return None
    return ("opencv", cap)


def open_camera_retry(cfg, use_picamera, tries=3):
    for i in range(tries):
        cam = open_camera(cfg, use_picamera)
        if cam is not None:
            return cam
        print(f"Камера {cfg['CAMERA_INDEX']} не открылась (попытка {i+1}/{tries})…")
        time.sleep(1.5)
    sys.exit("ОШИБКА: камера недоступна. Проверьте подключение, индекс (--camera N), "
             "не занята ли она другим приложением; список камер: ls /dev/video*")


def read_frame(cam):
    kind, dev = cam
    if kind == "picamera":
        try:
            return dev.capture_array()
        except Exception:
            return None
    ok, frame = dev.read()
    return frame if ok else None


def close_camera(cam):
    kind, dev = cam
    try:
        if kind == "opencv":
            dev.release()
        else:
            dev.stop()
    except Exception:
        pass


# ==================== ГЛАВНЫЙ ЦИКЛ ==========================
def main():
    p = argparse.ArgumentParser(description="ПВО-3К: зрение для турели")
    p.add_argument("--camera", type=int, help="индекс USB-камеры")
    p.add_argument("--picamera", action="store_true", help="CSI-модуль камеры")
    p.add_argument("--port", help="порт Arduino (по умолчанию автопоиск)")
    p.add_argument("--fov", type=float, help="горизонтальный FOV камеры, °")
    p.add_argument("--display", action="store_true", help="окно отладки")
    p.add_argument("--dry-run", action="store_true", help="без Arduino")
    args = p.parse_args()

    cfg = dict(CONFIG)
    if args.camera is not None: cfg["CAMERA_INDEX"] = args.camera
    if args.port: cfg["SERIAL_PORT"] = args.port
    if args.fov: cfg["FOV_H_DEG"] = args.fov

    for w in validate_config(cfg):
        print(f"ПРЕДУПРЕЖДЕНИЕ КОНФИГА: {w}")

    cam = open_camera_retry(cfg, args.picamera)
    det = MotionDetector(cfg)
    link = TurretLink(cfg, dry_run=args.dry_run)

    pan_smooth = float(cfg["PAN_CENTER"])
    lost = 0
    cam_fails = 0
    tracking = False
    frames = 0
    t0 = time.monotonic()

    print("ПВО-3К: зрение запущено. Ctrl+C — выход.")
    try:
        while True:
            frame = read_frame(cam)
            if frame is None:
                cam_fails += 1
                if cam_fails >= cfg["CAM_FAIL_LIMIT"]:
                    print("КАМЕРА МОЛЧИТ — переоткрываю…")
                    close_camera(cam)
                    cam = open_camera_retry(cfg, args.picamera)
                    det = MotionDetector(cfg)      # фон копится заново
                    cam_fails = 0
                time.sleep(0.05)
                continue
            cam_fails = 0
            frames += 1

            hit = det.find_target(frame)
            if hit is not None:
                cx, cy, area, bbox = hit
                az = azimuth_from_px(cx, frame.shape[1], cfg["FOV_H_DEG"])
                goal = pan_from_azimuth(az, cfg)
                pan_smooth += cfg["SMOOTH_ALPHA"] * (goal - pan_smooth)
                link.point(int(round(pan_smooth)))
                lost = 0
                if not tracking:
                    tracking = True
                    print(f">>> Цель: азимут {az:+.1f}°, площадь {int(area)} px")
            else:
                lost += 1
                if tracking and lost >= cfg["LOST_FRAMES"]:
                    tracking = False
                    link.release()
                    print("<<< Цель потеряна, турель — в автономный патруль")

            if args.display:
                if hit is not None:
                    x, y, w, h = hit[3]
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    cv2.drawMarker(frame, (int(hit[0]), int(hit[1])),
                                   (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
                hud = f"pan={int(round(pan_smooth))} link={'OK' if link.connected() else 'НЕТ'}"
                cv2.putText(frame, hud, (8, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.imshow("PVO-3K vision", frame)
                if cv2.waitKey(1) & 0xFF == 27:
                    break

            if frames % 300 == 0:
                fps = frames / (time.monotonic() - t0)
                state = "OK" if link.connected() else "ПЕРЕПОДКЛЮЧЕНИЕ"
                print(f"[stat] {fps:.1f} кадров/с, связь: {state}")
                if fps < 10:
                    print("[stat] ПОДСКАЗКА: FPS низкий — уменьшите FRAME_W/H или закройте лишнее")
    except KeyboardInterrupt:
        pass
    finally:
        link.close()
        close_camera(cam)
        cv2.destroyAllWindows()
        print("Остановлено. Турель переведена в автономный режим.")


if __name__ == "__main__":
    main()

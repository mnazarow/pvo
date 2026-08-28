#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
============================================================
 ПВО-3К «Комар-В» — компьютерное зрение для турели
============================================================
Вариант C из руководства: Raspberry Pi + камера находят цель,
Arduino (pvo.ino с режимом SLAVE) наводит турель и лазер.

Как это работает:
  1. Камера отдаёт кадры; вычитатель фона MOG2 выделяет движение.
  2. Из контуров движения выбирается цель (по площади).
  3. Горизонтальная позиция цели в кадре пересчитывается в азимут
     (через угол обзора камеры FOV_H_DEG), азимут — в угол серво.
  4. Угол сглаживается и уходит в Arduino строкой "P<угол>\n"
     (не чаще SEND_HZ раз в секунду). Если цели нет дольше
     LOST_FRAMES кадров — отправляется "R\n", турель возвращается
     к автономному патрулированию.

Запуск:
  python3 pvo_vision.py                        # USB-камера 0, /dev/ttyUSB0
  python3 pvo_vision.py --display              # с окном отладки
  python3 pvo_vision.py --dry-run              # без Arduino (лог в консоль)
  python3 pvo_vision.py --port /dev/ttyACM0 --camera 0 --fov 62

Зависимости (Raspberry Pi OS Bookworm):
  sudo apt install python3-opencv python3-serial
  (для камеры-модуля CSI дополнительно: python3-picamera2, флаг --picamera)

Все параметры собраны в CONFIG и подробно описаны в руководстве.
============================================================
"""

import argparse
import sys
import time

import cv2
import numpy as np

# ==================== ПАРАМЕТРЫ =============================
CONFIG = {
    # --- камера ---
    "CAMERA_INDEX": 0,      # номер /dev/video* для USB-камеры
    "FRAME_W": 640,         # рабочее разрешение; больше = точнее, но медленнее
    "FRAME_H": 480,
    "FOV_H_DEG": 62.0,      # горизонтальный угол обзора КОНКРЕТНОЙ камеры, °

    # --- геометрия турели (должна совпадать со скетчем pvo.ino) ---
    "PAN_CENTER": 90,       # угол серво «прямо по оси камеры»
    "PAN_MIN": 15,
    "PAN_MAX": 165,
    "PAN_INVERT": False,    # True, если турель поворачивается «не туда»

    # --- детектор движения ---
    "BG_HISTORY": 120,      # кадров в модели фона MOG2
    "BG_THRESHOLD": 32.0,   # чувствительность MOG2 (varThreshold)
    "MORPH_KERNEL": 3,      # ядро морфологической очистки шума, px
    "MIN_AREA_PX": 25,      # контуры меньше — шум, игнорируем
    "MAX_AREA_PX": 30000,   # контуры больше — «не наша» цель (тень, свет)

    # --- наведение ---
    "SMOOTH_ALPHA": 0.4,    # сглаживание угла 0..1 (больше = резче)
    "SEND_HZ": 15,          # частота команд P, Гц
    "LOST_FRAMES": 12,      # пустых кадров подряд до команды R

    # --- связь с Arduino ---
    "SERIAL_PORT": "/dev/ttyUSB0",  # Nano c CH340: ttyUSB0; Uno: ttyACM0
    "BAUD": 9600,           # как в pvo.ino
}

# ==================== ГЕОМЕТРИЯ =============================
def azimuth_from_px(cx: float, frame_w: int, fov_h_deg: float) -> float:
    """Пиксель центра цели -> азимут от оси камеры, ° (+ вправо)."""
    half = frame_w / 2.0
    return (cx - half) / half * (fov_h_deg / 2.0)


def pan_from_azimuth(az_deg: float, cfg: dict) -> int:
    """Азимут -> угол серво с учётом инверсии и пределов."""
    if cfg["PAN_INVERT"]:
        a = cfg["PAN_CENTER"] + az_deg
    else:
        a = cfg["PAN_CENTER"] - az_deg
    return int(round(max(cfg["PAN_MIN"], min(cfg["PAN_MAX"], a))))


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
        """Возвращает (cx, cy, area, bbox) самой крупной цели или None."""
        mask = self.bg.apply(frame)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_area = 0.0
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.cfg["MIN_AREA_PX"] or area > self.cfg["MAX_AREA_PX"]:
                continue
            if area > best_area:
                best_area = area
                best = c
        if best is None:
            return None

        x, y, w, h = cv2.boundingRect(best)
        return (x + w / 2.0, y + h / 2.0, best_area, (x, y, w, h))


# ==================== СВЯЗЬ С ТУРЕЛЬЮ =======================
class TurretLink:
    """Отправка команд наведения в Arduino (pvo.ino, режим SLAVE)."""

    def __init__(self, cfg: dict, dry_run: bool = False):
        self.cfg = cfg
        self.dry = dry_run
        self.last_send = 0.0
        self.ser = None
        if not dry_run:
            import serial  # pyserial
            self.ser = serial.Serial(cfg["SERIAL_PORT"], cfg["BAUD"], timeout=0)
            time.sleep(2.0)  # Uno перезагружается при открытии порта

    def _write(self, line: str):
        if self.dry:
            print(f"[serial] {line.strip()}")
        else:
            self.ser.write(line.encode("ascii"))

    def point(self, angle: int) -> bool:
        """Команда P с ограничением частоты. True, если отправлена."""
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
            self.release()
            self.ser.close()


# ==================== КАМЕРА ================================
def open_camera(cfg: dict, use_picamera: bool):
    if use_picamera:
        from picamera2 import Picamera2  # только на Raspberry Pi OS
        cam = Picamera2()
        cam.configure(cam.create_video_configuration(
            main={"size": (cfg["FRAME_W"], cfg["FRAME_H"]), "format": "RGB888"}))
        cam.start()
        return ("picamera", cam)
    cap = cv2.VideoCapture(cfg["CAMERA_INDEX"])
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg["FRAME_W"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg["FRAME_H"])
    if not cap.isOpened():
        sys.exit(f"Не удалось открыть камеру {cfg['CAMERA_INDEX']}")
    return ("opencv", cap)


def read_frame(cam):
    kind, dev = cam
    if kind == "picamera":
        return dev.capture_array()
    ok, frame = dev.read()
    return frame if ok else None


# ==================== ГЛАВНЫЙ ЦИКЛ ==========================
def main():
    p = argparse.ArgumentParser(description="ПВО-3К: зрение для турели")
    p.add_argument("--camera", type=int, help="индекс USB-камеры")
    p.add_argument("--picamera", action="store_true", help="CSI-модуль камеры")
    p.add_argument("--port", help="последовательный порт Arduino")
    p.add_argument("--fov", type=float, help="горизонтальный FOV камеры, °")
    p.add_argument("--display", action="store_true", help="окно отладки")
    p.add_argument("--dry-run", action="store_true", help="без Arduino")
    args = p.parse_args()

    cfg = dict(CONFIG)
    if args.camera is not None: cfg["CAMERA_INDEX"] = args.camera
    if args.port: cfg["SERIAL_PORT"] = args.port
    if args.fov: cfg["FOV_H_DEG"] = args.fov

    cam = open_camera(cfg, args.picamera)
    det = MotionDetector(cfg)
    link = TurretLink(cfg, dry_run=args.dry_run)

    pan_smooth = float(cfg["PAN_CENTER"])
    lost = 0
    tracking = False
    frames = 0
    t0 = time.monotonic()

    print("ПВО-3К: зрение запущено. Ctrl+C — выход.")
    try:
        while True:
            frame = read_frame(cam)
            if frame is None:
                time.sleep(0.05)
                continue
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
                cv2.putText(frame, f"pan={int(round(pan_smooth))}", (8, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.imshow("PVO-3K vision", frame)
                if cv2.waitKey(1) & 0xFF == 27:   # Esc
                    break

            if frames % 300 == 0:
                fps = frames / (time.monotonic() - t0)
                print(f"[stat] {fps:.1f} кадров/с")
    except KeyboardInterrupt:
        pass
    finally:
        link.close()
        if cam[0] == "opencv":
            cam[1].release()
        cv2.destroyAllWindows()
        print("Остановлено. Турель переведена в автономный режим.")


if __name__ == "__main__":
    main()

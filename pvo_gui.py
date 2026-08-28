#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
============================================================
 Пульт ПВО «Комар» — графическое приложение (Windows / macOS / Linux)
============================================================
Удалённая настройка, управление и наблюдение за турелью по USB.
Работает с прошивками:
  - pvo.ino        (вариант A/C: Uno/Nano, порт 9600)
  - pvo_esp32.ino  (вариант B: ESP32 + радар, порт 115200)

Возможности:
  - живой «радарный экран»: положение турели, цель, зоны;
  - просмотр и изменение ВСЕХ параметров прошивки (G/S),
    сохранение в память платы (W), сброс к заводским (D);
  - ручное управление: наведение слайдером (P), сброс (R),
    тест лазера и центровка серво (вариант B);
  - журнал поражений, счётчик загрузок, тревоги датчика/радара;
  - консоль: весь обмен с платой + отправка любых команд.

Установка:
  Windows: поставить Python 3.10+ с python.org (галочка Add to PATH),
           затем:  pip install pyserial
  macOS:   python3 уже есть (или с python.org), затем:
           pip3 install pyserial
  Запуск:  python pvo_gui.py     (macOS: python3 pvo_gui.py)

Демо-режим (железо не нужно): в списке портов выберите
«ДЕМО: Uno» или «ДЕМО: ESP32» и нажмите «Подключить» — пульт
подключится к встроенной виртуальной турели. Удобно освоить
интерфейс, пока посылка с деталями едет.

Журнал поражений: каждое «поражение» записывается с временем и
координатами (вкладка «Управление»), кнопка «Сохранить CSV…»
выгружает историю для Excel.

Сборка в отдельное приложение (по желанию):
  pip install pyinstaller
  pyinstaller --onefile --windowed pvo_gui.py
  (готовый файл появится в папке dist; собирать нужно на той ОС,
   под которую нужно приложение)
============================================================
"""

import queue
import sys
import threading
import time

# ==================== ОПИСАНИЯ ПАРАМЕТРОВ ===================
# (имя в протоколе, подпись, мин, макс, подсказка)
PARAMS_UNO = [
    ("DETECT",   "Порог захвата, см",          5, 300,  "ближе — цель"),
    ("LOST",     "Порог удержания, см",        5, 400,  "гистерезис, больше порога захвата"),
    ("MINVALID", "Мин. валидная дистанция, см",1, 50,   "JSN-SR04T: 25"),
    ("CONFIRM",  "Замеров для захвата",        1, 10,   "защита от ложных срабатываний"),
    ("LOSTP",    "Пустых замеров до потери",   1, 50,   "устойчивость сопровождения"),
    ("WOBBLE",   "Микро-скан, °",              1, 20,   "амплитуда слежения"),
    ("STEP",     "Шаг цикла, мс",              20, 100, "меньше 25 нельзя (эхо)"),
    ("KILLMS",   "Удержание до «поражения», мс",200, 20000, ""),
    ("EXTMS",    "Таймаут SLAVE, мс",          200, 5000, "для связки с Raspberry Pi"),
    ("SWMIN",    "Сектор: от, °",              0, 170,  ""),
    ("SWMAX",    "Сектор: до, °",              10, 180, ""),
]
PARAMS_ESP32 = [
    ("PANC",    "Центр PAN, °",        30, 150, "куда «прямо»"),
    ("PANMIN",  "PAN мин, °",          0, 170,  ""),
    ("PANMAX",  "PAN макс, °",         10, 180, ""),
    ("TILTC",   "Центр TILT, °",       30, 150, "горизонт"),
    ("TILTMIN", "TILT мин, °",         0, 170,  ""),
    ("TILTMAX", "TILT макс, °",        10, 180, ""),
    ("PANINV",  "Инверсия PAN (0/1)",  0, 1,    "если крутит «не туда»"),
    ("TILTINV", "Инверсия TILT (0/1)", 0, 1,    ""),
    ("ATILT",   "Автонаклон (0/1)",    0, 1,    "tilt по дальности"),
    ("TURH",    "Высота турели, мм",   0, 3000, ""),
    ("TGTH",    "Высота цели, мм",     0, 3000, ""),
    ("MINR",    "Мин. дальность, мм",  100, 2000, "мёртвая зона"),
    ("MAXR",    "Макс. дальность, мм", 500, 8000, ""),
    ("MAXAZ",   "Сектор ±, °",         10, 60,  ""),
    ("CONFIRM", "Кадров для захвата",  1, 30,   ""),
    ("LOSTMS",  "Таймаут потери, мс",  200, 5000, ""),
    ("KILLMS",  "Удержание «поражения», мс", 200, 20000, ""),
    ("ALPHA",   "Плавность ×100",      5, 100,  "35 = 0.35"),
    ("SLEW",    "Макс. скорость, °/шаг",1, 20,  ""),
    ("PATROL",  "Патруль на дежурстве (0/1)", 0, 1, ""),
    ("PSPEED",  "Скорость патруля ×100", 5, 150, ""),
]

# ==================== РАЗБОР ПРОТОКОЛА ======================
def parse_kv_line(line: str, prefix: str):
    """'TL a=1 b=2' -> {'a':'1','b':'2'} (значения строками)."""
    if not line.startswith(prefix + " "):
        return None
    out = {}
    for tok in line[len(prefix) + 1:].split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k] = v
    return out


def parse_param_line(line: str):
    """'PARAM DETECT=60' -> ('DETECT', 60); 'PARAM END' -> ('END', None)."""
    if not line.startswith("PARAM "):
        return None
    body = line[6:].strip()
    if body == "END":
        return ("END", None)
    if "=" in body:
        k, v = body.split("=", 1)
        try:
            return (k.strip(), int(v.strip()))
        except ValueError:
            return None
    return None


def detect_variant(param_names):
    """По набору параметров понимаем, какая прошивка подключена."""
    s = set(param_names)
    if "MAXR" in s or "PANC" in s:
        return "esp32"
    if "DETECT" in s:
        return "uno"
    return None


# ==================== ЖУРНАЛ ПОРАЖЕНИЙ =====================
def format_hits_csv(rows) -> str:
    """Строки журнала -> CSV (разделитель ';' — дружит с русским Excel)."""
    out = ["время;прошивка;угол,°;дистанция,см;x,мм;y,мм;дальность,мм"]
    for r in rows:
        out.append("{t};{v};{a};{d};{x};{y};{dm}".format(
            t=r.get("time", ""), v=r.get("variant", ""),
            a=r.get("a", ""), d=r.get("d", ""),
            x=r.get("x", ""), y=r.get("y", ""), dm=r.get("dist_mm", "")))
    return "\n".join(out) + "\n"


# ==================== ДЕМО-ТУРЕЛЬ ===========================
DEMO_UNO = "ДЕМО: Uno (без железа)"
DEMO_ESP = "ДЕМО: ESP32 (без железа)"

class DemoSerial:
    """Виртуальная турель: тот же протокол, что у прошивок, но без железа.
    Подставляется вместо serial.Serial при выборе порта «ДЕМО…»."""

    def __init__(self, variant="uno"):
        import math
        self.math = math
        self.variant = variant
        self.t0 = time.monotonic()
        self.out = bytearray()
        self.buf = b""
        self.telemetry = False
        self.kills = 3
        self.mode = "P" if variant == "uno" else "I"
        self.slave_angle = 90
        self._last_tl = 0.0
        self._kill_done_phase = -1
        if variant == "uno":
            self.params = {"DETECT": 60, "LOST": 90, "MINVALID": 3, "CONFIRM": 3,
                           "LOSTP": 12, "WOBBLE": 5, "STEP": 30, "KILLMS": 3000,
                           "EXTMS": 700, "SWMIN": 15, "SWMAX": 165}
            self._push("=== ПВО-1К \"Комар\" (ДЕМО) на боевом дежурстве ===")
        else:
            self.params = {"PANC": 90, "PANMIN": 15, "PANMAX": 165, "TILTC": 90,
                           "TILTMIN": 60, "TILTMAX": 120, "PANINV": 0, "TILTINV": 0,
                           "ATILT": 1, "TURH": 750, "TGTH": 1100, "MINR": 300,
                           "MAXR": 4000, "MAXAZ": 60, "CONFIRM": 5, "LOSTMS": 700,
                           "KILLMS": 3000, "ALPHA": 35, "SLEW": 6, "PATROL": 1,
                           "PSPEED": 35}
            self._push("=== ПВО-2К \"Комар-М\" (ДЕМО) на боевом дежурстве ===")
        self._push("Загрузка N7, журнал за всё время: 3")
        self._push("Это встроенный демонстрационный режим: железо не подключено.")

    # --- внутреннее ---
    def _push(self, line):
        self.out += (line + "\n").encode("utf-8")

    def _tl_line(self):
        m, t = self.math, time.monotonic() - self.t0
        if self.variant == "uno":
            phase = t % 14.0
            if self.mode == "S":
                return f"TL m=S a={self.slave_angle} d=999 k={self.kills} s=0"
            if phase < 6:
                a = int(15 + abs((phase * 50) % 300 - 150))
                return f"TL m=P a={a} d=999 k={self.kills} s=0"
            if phase > 9 and self._kill_done_phase != int(t // 14):
                self._kill_done_phase = int(t // 14)
                self.kills += 1
                self._push(f"*** Цель N{self.kills} поражена. Занесена в журнал ***")
            a = int(95 + 5 * m.sin(t * 6))
            d = int(41 + 3 * m.sin(t * 2.3))
            return f"TL m=T a={a} d={d} k={self.kills} s=0"
        phase = t % 16.0
        if phase < 5:
            p = int(90 + 60 * m.sin(t * 0.5))
            return f"TL m=I p={p} t=90 x=0 y=0 d=0 k={self.kills} r=0"
        if phase > 11 and self._kill_done_phase != int(t // 16):
            self._kill_done_phase = int(t // 16)
            self.kills += 1
            self._push(f"*** Цель N{self.kills} поражена. Занесена в журнал ***")
        x = int(900 * m.sin(t * 0.9))
        y = 1500 + int(220 * m.sin(t * 0.53))
        d = int(m.hypot(x, y))
        p = int(90 - m.degrees(m.atan2(x, y)))
        tilt = 90 + int(m.degrees(m.atan2(350, d)))
        return f"TL m=T p={p} t={tilt} x={x} y={y} d={d} k={self.kills} r=0"

    def _handle(self, cmd):
        if cmd == "G":
            for k, v in self.params.items():
                self._push(f"PARAM {k}={v}")
            self._push("PARAM END")
        elif cmd.startswith("S "):
            try:
                _, name, val = cmd.split()
                if name in self.params:
                    self.params[name] = int(val)
                    self._push(f"OK {name}={val}")
                else:
                    self._push(f"ERR имя или диапазон: {name}")
            except ValueError:
                self._push("ERR формат: S ИМЯ ЗНАЧЕНИЕ")
        elif cmd == "W":
            self._push("OK SAVED (демо: сохранено понарошку)")
        elif cmd == "D":
            self._push("OK DEFAULTS (в ОЗУ; W — сохранить)")
        elif cmd == "Z":
            self.kills = 0
            self._push("OK Z (журнал поражений обнулён)")
        elif cmd == "M1":
            self.telemetry = True; self._push("OK M1")
        elif cmd == "M0":
            self.telemetry = False; self._push("OK M0")
        elif cmd == "?":
            self._push(f"СТАТУС: ДЕМО-режим, журнал={self.kills}")
        elif cmd.startswith("P") and cmd[1:].isdigit() and self.variant == "uno":
            self.mode = "S"; self.slave_angle = int(cmd[1:])
            self._push(">>> Внешнее целеуказание принято (SLAVE)")
        elif cmd == "R":
            self.mode = "P" if self.variant == "uno" else "I"
            self._push("<<< Цель потеряна — возвращаюсь к патрулированию")
        elif cmd == "C":
            self._push("Серво в центр на 8 с — юстируйте крепления")
        elif cmd in ("L1", "L0"):
            self._push("Лазер ВКЛ (тест). L0 — выключить" if cmd == "L1" else "Лазер ВЫКЛ")
        elif cmd == "T":
            self._push("Команда инициализации радара отправлена повторно")
        else:
            self._push("Команды: ? G S W D M1/M0 (демо)")

    # --- интерфейс serial.Serial ---
    def read(self, n=1):
        now = time.monotonic()
        if self.telemetry and now - self._last_tl >= 0.2:
            self._last_tl = now
            self._push(self._tl_line())
        if not self.out:
            return b""
        chunk = bytes(self.out[:n])
        del self.out[:n]
        return chunk

    def write(self, data):
        self.buf += data
        while b"\n" in self.buf:
            raw, self.buf = self.buf.split(b"\n", 1)
            cmd = raw.decode("ascii", errors="replace").strip()
            if cmd:
                self._handle(cmd)
        return len(data)

    def close(self):
        pass


# ==================== ПОТОК ЧТЕНИЯ ПОРТА ====================
class SerialWorker(threading.Thread):
    def __init__(self, ser, rx_queue):
        super().__init__(daemon=True)
        self.ser = ser
        self.q = rx_queue
        self.alive = True
        self.buf = b""

    def run(self):
        while self.alive:
            try:
                chunk = self.ser.read(256)
            except Exception as e:
                self.q.put(("__error__", f"обрыв связи: {e}"))
                return
            if chunk:
                self.buf += chunk
                while b"\n" in self.buf:
                    raw, self.buf = self.buf.split(b"\n", 1)
                    line = raw.decode("utf-8", errors="replace").strip()
                    if line:
                        self.q.put(("line", line))
            else:
                time.sleep(0.02)


# ==================== ПРИЛОЖЕНИЕ ============================
def run_gui():
    import argparse
    import tkinter as tk
    from tkinter import ttk, messagebox
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        print("Нет pyserial: установите командой  pip install pyserial")
        sys.exit(1)

    ap = argparse.ArgumentParser(description="Пульт ПВО «Комар»")
    ap.add_argument("--port", help="порт (например COM5 или /dev/ttyUSB0)")
    ap.add_argument("--baud", help="скорость: 9600 (Uno) или 115200 (ESP32)")
    ap.add_argument("--connect", action="store_true", help="подключиться сразу при запуске")
    ap.add_argument("--tab", type=int, default=0, help="номер вкладки при старте (0..3)")
    opts = ap.parse_args()

    BG, PANEL, FG = "#101418", "#1B232B", "#D7E0E7"
    ACCENT, GOOD, BAD, WARN = "#27C0A0", "#57D75B", "#F0544F", "#F2B44D"

    class App(tk.Tk):
        def __init__(self):
            super().__init__()
            self.title("Пульт ПВО «Комар»")
            self.geometry("980x640")
            self.configure(bg=BG)
            self.minsize(900, 560)

            self.ser = None
            self.worker = None
            self.rxq = queue.Queue()
            self.variant = None          # 'uno' | 'esp32'
            self.tl = {}                 # последняя телеметрия
            self.params = {}             # загруженные параметры
            self.entries = {}            # name -> Entry
            self.loading_params = False
            self.slider_live = tk.BooleanVar(value=False)
            self.hits = []               # журнал поражений за сеанс
            self._last_k = None

            self._build_ui()
            self.after(100, self._poll)

        # ---------- интерфейс ----------
        def _build_ui(self):
            style = ttk.Style(self)
            try:
                style.theme_use("clam")
            except Exception:
                pass
            style.configure(".", background=BG, foreground=FG, fieldbackground=PANEL)
            style.configure("TNotebook", background=BG, borderwidth=0)
            style.configure("TNotebook.Tab", background=PANEL, foreground=FG, padding=(14, 6))
            style.map("TNotebook.Tab", background=[("selected", ACCENT)],
                      foreground=[("selected", "#08211C")])
            style.configure("TFrame", background=BG)
            style.configure("Panel.TFrame", background=PANEL)
            style.configure("TLabel", background=BG, foreground=FG)
            style.configure("Panel.TLabel", background=PANEL, foreground=FG)
            style.configure("TButton", padding=6)
            style.configure("TCheckbutton", background=BG, foreground=FG)
            style.configure("TCombobox", fieldbackground="#EDF2F5", foreground="#1A2228")
            style.map("TCombobox",
                      fieldbackground=[("readonly", "#EDF2F5")],
                      foreground=[("readonly", "#1A2228")])

            # --- верхняя панель подключения ---
            top = ttk.Frame(self, style="Panel.TFrame", padding=8)
            top.pack(fill="x")
            ttk.Label(top, text="Порт:", style="Panel.TLabel").pack(side="left")
            self.port_cb = ttk.Combobox(top, width=22, values=[])
            self.port_cb.pack(side="left", padx=4)
            ttk.Button(top, text="⟳", width=3, command=self.refresh_ports).pack(side="left")
            ttk.Label(top, text="Скорость:", style="Panel.TLabel").pack(side="left", padx=(12, 0))
            self.baud_cb = ttk.Combobox(top, width=8, values=["Авто", "9600", "115200"],
                                        state="readonly")
            self.baud_cb.set("Авто")
            self.baud_cb.pack(side="left", padx=4)
            ttk.Label(top, text="(Авто найдёт сам; Uno — 9600, ESP32 — 115200)",
                      style="Panel.TLabel", foreground="#8A98A5").pack(side="left")
            self.conn_btn = ttk.Button(top, text="Подключить", command=self.toggle_conn)
            self.conn_btn.pack(side="left", padx=12)
            self.status_lbl = tk.Label(top, text="● не подключено", bg=PANEL, fg=BAD)
            self.status_lbl.pack(side="left", padx=8)
            self.kills_lbl = tk.Label(top, text="Журнал: —", bg=PANEL, fg=ACCENT,
                                      font=("TkDefaultFont", 11, "bold"))
            self.kills_lbl.pack(side="right", padx=8)

            # --- вкладки ---
            nb = ttk.Notebook(self)
            nb.pack(fill="both", expand=True, padx=8, pady=8)

            self.nb = nb

            # Обзор
            tab1 = ttk.Frame(nb); nb.add(tab1, text="  Обзор  ")
            self.canvas = tk.Canvas(tab1, bg="#0A1014", highlightthickness=0)
            self.canvas.pack(fill="both", expand=True)

            # Параметры
            tab2 = ttk.Frame(nb); nb.add(tab2, text="  Параметры  ")
            bar = ttk.Frame(tab2); bar.pack(fill="x", pady=4)
            ttk.Button(bar, text="Считать с платы (G)", command=self.read_params).pack(side="left", padx=4)
            ttk.Button(bar, text="Применить изменённые", command=self.apply_params).pack(side="left", padx=4)
            ttk.Button(bar, text="Сохранить в память платы (W)", command=lambda: self.send("W")).pack(side="left", padx=4)
            ttk.Button(bar, text="Заводские (D)", command=self.factory_defaults).pack(side="left", padx=4)
            self.param_hint = ttk.Label(tab2, text="Подключитесь и нажмите «Считать с платы»")
            self.param_hint.pack(anchor="w", padx=6)
            wrap = ttk.Frame(tab2); wrap.pack(fill="both", expand=True)
            self.param_canvas = tk.Canvas(wrap, bg=BG, highlightthickness=0)
            sb = ttk.Scrollbar(wrap, orient="vertical", command=self.param_canvas.yview)
            self.param_frame = ttk.Frame(self.param_canvas)
            self.param_frame.bind("<Configure>", lambda e: self.param_canvas.configure(
                scrollregion=self.param_canvas.bbox("all")))
            self.param_canvas.create_window((0, 0), window=self.param_frame, anchor="nw")
            self.param_canvas.configure(yscrollcommand=sb.set)
            self.param_canvas.pack(side="left", fill="both", expand=True)
            sb.pack(side="right", fill="y")

            # Управление
            tab3 = ttk.Frame(nb); nb.add(tab3, text="  Управление  ")
            box = ttk.Frame(tab3, style="Panel.TFrame", padding=14)
            box.pack(fill="x", padx=10, pady=10)
            ttk.Label(box, text="Ручное наведение (режим SLAVE, вариант A/C):",
                      style="Panel.TLabel").pack(anchor="w")
            row = ttk.Frame(box, style="Panel.TFrame"); row.pack(fill="x", pady=6)
            self.angle_var = tk.IntVar(value=90)
            self.angle_scale = ttk.Scale(row, from_=15, to=165, variable=self.angle_var,
                                         command=self._on_slider)
            self.angle_scale.pack(side="left", fill="x", expand=True, padx=(0, 8))
            self.angle_lbl = tk.Label(row, text="90°", width=5, bg=PANEL, fg=ACCENT,
                                      font=("TkDefaultFont", 12, "bold"))
            self.angle_lbl.pack(side="left")
            row2 = ttk.Frame(box, style="Panel.TFrame"); row2.pack(fill="x", pady=4)
            ttk.Button(row2, text="Навести (P)", command=self.send_point).pack(side="left", padx=4)
            ttk.Checkbutton(row2, text="слать при движении слайдера",
                            variable=self.slider_live).pack(side="left", padx=8)
            ttk.Button(row2, text="Автономия (R)", command=lambda: self.send("R")).pack(side="left", padx=12)

            box2 = ttk.Frame(tab3, style="Panel.TFrame", padding=14)
            box2.pack(fill="x", padx=10)
            ttk.Label(box2, text="Сервис (вариант B — ESP32):", style="Panel.TLabel").pack(anchor="w")
            row3 = ttk.Frame(box2, style="Panel.TFrame"); row3.pack(fill="x", pady=6)
            ttk.Button(row3, text="Серво в центр (C)", command=lambda: self.send("C")).pack(side="left", padx=4)
            ttk.Button(row3, text="Лазер тест ВКЛ (L1)", command=lambda: self.send("L1")).pack(side="left", padx=4)
            ttk.Button(row3, text="Лазер ВЫКЛ (L0)", command=lambda: self.send("L0")).pack(side="left", padx=4)
            ttk.Button(row3, text="Реинит радара (T)", command=lambda: self.send("T")).pack(side="left", padx=4)
            ttk.Button(row3, text="Статус (?)", command=lambda: self.send("?")).pack(side="left", padx=12)

            box3 = ttk.Frame(tab3, style="Panel.TFrame", padding=14)
            box3.pack(fill="both", expand=True, padx=10, pady=10)
            hrow = ttk.Frame(box3, style="Panel.TFrame"); hrow.pack(fill="x")
            ttk.Label(hrow, text="Журнал поражений за сеанс:", style="Panel.TLabel").pack(side="left")
            ttk.Button(hrow, text="Сохранить CSV…", command=self.save_hits_csv).pack(side="right", padx=4)
            ttk.Button(hrow, text="Очистить список", command=self.clear_hits).pack(side="right", padx=4)
            ttk.Button(hrow, text="Обнулить журнал платы (Z)",
                       command=self.zero_board_kills).pack(side="right", padx=4)
            self.hits_list = tk.Listbox(box3, height=7, bg="#0A1014", fg="#C8D2DA",
                                        selectbackground=ACCENT, font=("Courier", 10))
            self.hits_list.pack(fill="both", expand=True, pady=(6, 0))

            # Консоль
            tab4 = ttk.Frame(nb); nb.add(tab4, text="  Консоль  ")
            self.log = tk.Text(tab4, bg="#0A1014", fg="#C8D2DA", insertbackground=FG,
                               font=("Courier", 10), state="disabled", wrap="none")
            self.log.pack(fill="both", expand=True, padx=4, pady=4)
            crow = ttk.Frame(tab4); crow.pack(fill="x", padx=4, pady=(0, 6))
            self.cmd_entry = ttk.Entry(crow)
            self.cmd_entry.pack(side="left", fill="x", expand=True, padx=(0, 6))
            self.cmd_entry.bind("<Return>", lambda e: self.send_custom())
            ttk.Button(crow, text="Отправить", command=self.send_custom).pack(side="left")

            self.refresh_ports()
            self.protocol("WM_DELETE_WINDOW", self.on_close)

            # параметры командной строки (--port/--baud/--connect/--tab)
            if opts.port:
                self.port_cb.set(opts.port)
            if opts.baud:
                self.baud_cb.set(opts.baud)
            if opts.tab:
                try:
                    self.nb.select(opts.tab)
                except Exception:
                    pass
            if opts.connect:
                self.after(200, self.connect)

        # ---------- подключение ----------
        def refresh_ports(self):
            from serial.tools import list_ports
            ports = [p.device for p in list_ports.comports()]
            self.port_cb["values"] = ports + [DEMO_UNO, DEMO_ESP]
            if ports and not self.port_cb.get():
                self.port_cb.set(ports[0])
            elif not ports and not self.port_cb.get():
                self.port_cb.set(DEMO_UNO)

        def toggle_conn(self):
            if self.ser:
                self.disconnect()
            else:
                self.connect()

        def connect(self):
            import serial
            from tkinter import messagebox
            port = self.port_cb.get().strip()
            if not port:
                messagebox.showwarning("Порт", "Выберите порт (кнопка ⟳ обновляет список)")
                return
            if port in (DEMO_UNO, DEMO_ESP):
                self.ser = DemoSerial("uno" if port == DEMO_UNO else "esp32")
                self.worker = SerialWorker(self.ser, self.rxq)
                self.worker.start()
                self.status_lbl.config(text="● ДЕМО", fg=WARN)
                self.conn_btn.config(text="Отключить")
                self.log_line("[пульт] демо-режим: виртуальная турель, железо не нужно")
                self.after(400, self._hello)
                return
            baud_sel = self.baud_cb.get()
            if baud_sel == "Авто":
                self._auto_baud = ["9600", "115200"]
                baud = self._auto_baud[0]
            else:
                self._auto_baud = None
                baud = baud_sel
            if not self._open_port(port, baud):
                return
            if self._auto_baud:
                self.after(6000, self._auto_check)

        def _open_port(self, port, baud):
            """Открыть порт на заданной скорости и представиться плате."""
            import serial
            from tkinter import messagebox
            try:
                self.ser = serial.Serial(port, int(baud), timeout=0.05)
            except Exception as e:
                messagebox.showerror("Порт", f"Не удалось открыть {port}:\n{e}")
                return False
            self.worker = SerialWorker(self.ser, self.rxq)
            self.worker.start()
            self.status_lbl.config(text=f"● {port}", fg=GOOD)
            self.conn_btn.config(text="Отключить")
            self.log_line(f"[пульт] подключено: {port} @ {baud}")
            # Uno перезагружается при открытии порта — подождём и представимся
            self.after(2500, self._hello)
            return True

        def _auto_check(self):
            """Авто-скорость: если на текущей скорости плата молчит — пробуем другую."""
            auto = getattr(self, "_auto_baud", None)
            if not auto:
                return
            if self.params:
                self.log_line(f"[пульт] авто-скорость: подтверждена {auto[0]}")
                self._auto_baud = None
                return
            if not self.ser:
                return
            tried = auto.pop(0)
            if not auto:
                self._auto_baud = None
                self.log_line("[пульт] авто-скорость: плата молчит и на 9600, и на 115200 — "
                              "проверьте порт и прошивку")
                return
            nxt = auto[0]
            self.log_line(f"[пульт] авто-скорость: на {tried} тишина — пробую {nxt}")
            port = self.port_cb.get().strip()
            self.disconnect(silent=True)
            self._auto_baud = auto
            if self._open_port(port, nxt):
                self.after(6000, self._auto_check)

        def _hello(self):
            if not self.ser:
                return
            self.send("?")
            self.read_params()
            self.send("M1")

        def disconnect(self, silent=False):
            if self.worker:
                self.worker.alive = False
            if self.ser:
                try:
                    self.ser.write(b"M0\n")
                    self.ser.close()
                except Exception:
                    pass
            self.ser = None
            self.worker = None
            self.variant = None
            self._last_k = None
            self.status_lbl.config(text="● не подключено", fg=BAD)
            self.conn_btn.config(text="Подключить")
            if not silent:
                self.log_line("[пульт] отключено")

        # ---------- отправка ----------
        def send(self, cmd: str):
            if not self.ser:
                self.log_line("[пульт] нет подключения")
                return
            try:
                self.ser.write((cmd + "\n").encode("ascii", errors="replace"))
                self.log_line(f"→ {cmd}")
            except Exception as e:
                self.log_line(f"[пульт] ошибка отправки: {e}")
                self.disconnect(silent=True)
                self.status_lbl.config(text="● обрыв связи", fg=BAD)

        def send_point(self):
            self.send(f"P{int(self.angle_var.get())}")

        def _on_slider(self, _=None):
            a = int(self.angle_var.get())
            self.angle_lbl.config(text=f"{a}°")
            if self.slider_live.get():
                now = time.monotonic()
                if now - getattr(self, "_last_live", 0) > 0.07:
                    self._last_live = now
                    self.send_point()

        def send_custom(self):
            cmd = self.cmd_entry.get().strip()
            if cmd:
                self.send(cmd)
                self.cmd_entry.delete(0, "end")

        # ---------- параметры ----------
        def read_params(self):
            self.params.clear()
            self.loading_params = True
            self.send("G")

        def factory_defaults(self):
            from tkinter import messagebox
            if messagebox.askyesno("Заводские параметры",
                                   "Вернуть параметры по умолчанию (в ОЗУ платы)?\n"
                                   "Для записи в память платы затем нажмите W."):
                self.send("D")
                self.after(300, self.read_params)

        def _rebuild_param_rows(self):
            for w in self.param_frame.winfo_children():
                w.destroy()
            self.entries.clear()
            defs = PARAMS_ESP32 if self.variant == "esp32" else PARAMS_UNO
            known = {n for n, *_ in defs}
            header = ("Параметр", "Значение", "Диапазон", "Подсказка")
            for col, h in enumerate(header):
                tk_l = tk.Label(self.param_frame, text=h, bg=BG, fg="#8A98A5")
                tk_l.grid(row=0, column=col, sticky="w", padx=8, pady=(4, 6))
            r = 1
            for name, label, lo, hi, hint in defs:
                if name not in self.params:
                    continue
                tk.Label(self.param_frame, text=f"{label}  [{name}]",
                         bg=BG, fg=FG).grid(row=r, column=0, sticky="w", padx=8, pady=2)
                e = tk.Entry(self.param_frame, width=10, bg=PANEL, fg=FG,
                             insertbackground=FG, justify="center")
                e.insert(0, str(self.params[name]))
                e.grid(row=r, column=1, padx=8)
                self.entries[name] = e
                tk.Label(self.param_frame, text=f"{lo}…{hi}", bg=BG,
                         fg="#8A98A5").grid(row=r, column=2, padx=8)
                tk.Label(self.param_frame, text=hint, bg=BG,
                         fg="#8A98A5").grid(row=r, column=3, sticky="w", padx=8)
                r += 1
            extra = [n for n in self.params if n not in known]
            for name in extra:
                tk.Label(self.param_frame, text=name, bg=BG, fg=FG).grid(row=r, column=0, sticky="w", padx=8)
                e = tk.Entry(self.param_frame, width=10, bg=PANEL, fg=FG, justify="center")
                e.insert(0, str(self.params[name]))
                e.grid(row=r, column=1, padx=8)
                self.entries[name] = e
                r += 1
            vtxt = "ESP32 (вариант B)" if self.variant == "esp32" else "Uno/Nano (вариант A/C)"
            self.param_hint.config(text=f"Прошивка: {vtxt}. Меняйте значения и жмите «Применить». "
                                        f"W — сохранить в плату насовсем.")
            # ручное наведение слайдером есть только у прошивки Uno (команда P)
            state = ["disabled"] if self.variant == "esp32" else ["!disabled"]
            try:
                self.angle_scale.state(state)
            except Exception:
                pass

        def apply_params(self):
            changed = 0
            for name, e in self.entries.items():
                try:
                    v = int(e.get().strip())
                except ValueError:
                    self.log_line(f"[пульт] {name}: не число — пропускаю")
                    continue
                if self.params.get(name) != v:
                    self.send(f"S {name} {v}")
                    changed += 1
            if changed == 0:
                self.log_line("[пульт] изменённых параметров нет")
            else:
                self.after(400, self.read_params)   # перечитать фактические

        # ---------- приём ----------
        def _poll(self):
            try:
                while True:
                    kind, line = self.rxq.get_nowait()
                    if kind == "__error__":
                        self.log_line(f"[пульт] {line}")
                        self.disconnect(silent=True)
                        self.status_lbl.config(text="● обрыв связи", fg=BAD)
                        break
                    self._handle_line(line)
            except queue.Empty:
                pass
            self._draw()
            self.after(100, self._poll)

        def _record_hit(self, tl):
            row = {"time": time.strftime("%Y-%m-%d %H:%M:%S"),
                   "variant": "ESP32" if ("p" in tl) else "Uno",
                   "a": tl.get("a", ""), "d": tl.get("d", "") if "a" in tl else "",
                   "x": tl.get("x", ""), "y": tl.get("y", ""),
                   "dist_mm": tl.get("d", "") if "p" in tl else ""}
            self.hits.append(row)
            if len(self.hits) > 500:
                self.hits.pop(0)
            where = (f"угол {row['a']}°, {row['d']} см" if row["variant"] == "Uno"
                     else f"x={row['x']} y={row['y']} ({row['dist_mm']} мм)")
            self.hits_list.insert("end", f"{row['time']}  ☠ №{len(self.hits)}  {where}")
            self.hits_list.see("end")

        def save_hits_csv(self):
            from tkinter import filedialog, messagebox
            if not self.hits:
                messagebox.showinfo("Журнал", "Пока ни одного поражения за сеанс")
                return
            path = filedialog.asksaveasfilename(
                defaultextension=".csv", initialfile="pvo_журнал.csv",
                filetypes=[("CSV", "*.csv")])
            if not path:
                return
            with open(path, "w", encoding="utf-8-sig") as f:
                f.write(format_hits_csv(self.hits))
            self.log_line(f"[пульт] журнал сохранён: {path} ({len(self.hits)} записей)")

        def clear_hits(self):
            self.hits.clear()
            self.hits_list.delete(0, "end")

        def zero_board_kills(self):
            from tkinter import messagebox
            if messagebox.askyesno("Журнал платы",
                                   "Обнулить счётчик поражений в памяти самой платы?\n"
                                   "(история сеанса в списке останется)"):
                self.send("Z")

        def _handle_line(self, line: str):
            tl = parse_kv_line(line, "TL")
            if tl is not None:
                try:
                    k = int(tl.get("k", -1))
                    if self._last_k is not None and k > self._last_k:
                        self._record_hit(tl)
                    self._last_k = k
                except ValueError:
                    pass
                self.tl = tl
                if "k" in tl:
                    self.kills_lbl.config(text=f"Журнал: {tl['k']}")
                return                     # телеметрию в лог не сыплем
            if line.count("\ufffd") >= 3:              # мусор на неверной скорости
                self._noise = getattr(self, "_noise", 0) + 1
                if self._noise == 1:
                    self.log_line("[пульт] в порту шум — похоже, скорость не совпадает "
                                  "(жду авто-подбор или смените вручную)")
                return
            self._noise = 0
            self.log_line(line)
            p = parse_param_line(line)
            if p is not None:
                name, val = p
                if name == "END":
                    if self.loading_params:
                        self.loading_params = False
                        self.variant = detect_variant(self.params.keys()) or self.variant
                        if self.variant == "esp32" and self.baud_cb.get() == "9600":
                            self.log_line("[пульт] похоже, это ESP32 — обычно порт 115200")
                        self._rebuild_param_rows()
                else:
                    self.params[name] = val

        # ---------- отрисовка ----------
        def _draw(self):
            c = self.canvas
            c.delete("all")
            w = max(c.winfo_width(), 300)
            h = max(c.winfo_height(), 240)
            cx, cy = w / 2, h - 40
            R = min(w / 2 - 30, h - 90)
            import math

            if self.variant == "esp32" or (self.tl and "p" in self.tl):
                self._draw_radar(c, cx, cy, R, math)
            else:
                self._draw_sonar(c, cx, cy, R, math)

            # журнал поражений — крупно на экране
            if self.tl.get("k") is not None:
                c.create_text(w - 16, 16, text=f"Журнал: {self.tl['k']}",
                              fill=ACCENT, anchor="ne",
                              font=("TkDefaultFont", 15, "bold"))

            # тревоги
            alarm = None
            if self.tl.get("r") == "1":
                alarm = "РАДАР МОЛЧИТ — проверьте питание и провода"
            if self.tl.get("s") == "1":
                alarm = "ДАТЧИК МОЛЧИТ — проверьте TRIG/ECHO/питание"
            if alarm:
                c.create_rectangle(10, 10, w - 10, 44, fill="#3A1214", outline=BAD)
                c.create_text(w / 2, 27, text="⚠ " + alarm, fill=BAD,
                              font=("TkDefaultFont", 12, "bold"))
            if not self.ser:
                c.create_text(w / 2, h / 2, text="Подключитесь к турели",
                              fill="#5A6873", font=("TkDefaultFont", 15))

        def _sector(self, c, cx, cy, R, a0, a1, **kw):
            c.create_arc(cx - R, cy - R, cx + R, cy + R, start=a0,
                         extent=a1 - a0, style="pieslice", **kw)

        def _draw_sonar(self, c, cx, cy, R, math):
            sw_min = self.params.get("SWMIN", 15)
            sw_max = self.params.get("SWMAX", 165)
            det = self.params.get("DETECT", 60)
            lost = self.params.get("LOST", 90)
            scale = R / 200.0                      # 200 см на весь радиус
            self._sector(c, cx, cy, R, sw_min, sw_max, fill="#0F1B20", outline="#24404A")
            for r_cm, col in ((lost, "#5A2A2E"), (det, "#7A1F24")):
                rr = r_cm * scale
                self._sector(c, cx, cy, rr, sw_min, sw_max, fill="", outline=col)
                c.create_text(cx + 6, cy - rr - 8, text=f"{r_cm} см", fill=col, anchor="w",
                              font=("TkDefaultFont", 8))
            for ang in range(0, 181, 30):
                rad = math.radians(ang)
                c.create_line(cx, cy, cx + R * math.cos(rad), cy - R * math.sin(rad),
                              fill="#16262D")
                c.create_text(cx + (R + 14) * math.cos(rad), cy - (R + 14) * math.sin(rad),
                              text=f"{ang}°", fill="#41525C", font=("TkDefaultFont", 8))
            a = int(self.tl.get("a", 90))
            d = int(self.tl.get("d", 999))
            rad = math.radians(a)
            c.create_line(cx, cy, cx + R * math.cos(rad), cy - R * math.sin(rad),
                          fill=ACCENT, width=3)
            if d < 200:
                tx = cx + d * scale * math.cos(rad)
                ty = cy - d * scale * math.sin(rad)
                c.create_oval(tx - 6, ty - 6, tx + 6, ty + 6, fill=BAD, outline="")
                c.create_text(tx, ty - 14, text=f"{d} см", fill=BAD, font=("TkDefaultFont", 9))
            mode = {"P": "ПАТРУЛЬ", "T": "СОПРОВОЖДЕНИЕ", "S": "SLAVE"}.get(
                self.tl.get("m", ""), "—")
            col = {"ПАТРУЛЬ": ACCENT, "СОПРОВОЖДЕНИЕ": BAD, "SLAVE": WARN}.get(mode, FG)
            c.create_text(cx, cy + 24, text=f"{mode} · {a}°", fill=col,
                          font=("TkDefaultFont", 13, "bold"))

        def _draw_radar(self, c, cx, cy, R, math):
            max_r = self.params.get("MAXR", 4000)
            min_r = self.params.get("MINR", 300)
            max_az = self.params.get("MAXAZ", 60)
            scale = R / max(max_r, 500)
            self._sector(c, cx, cy, R, 90 - max_az, 90 + max_az,
                         fill="#0F1B20", outline="#24404A")
            for rr_mm in range(1000, max_r + 1, 1000):
                rr = rr_mm * scale
                self._sector(c, cx, cy, rr, 90 - max_az, 90 + max_az, fill="", outline="#1C333C")
                c.create_text(cx + 6, cy - rr - 8, text=f"{rr_mm//1000} м", fill="#41525C",
                              anchor="w", font=("TkDefaultFont", 8))
            self._sector(c, cx, cy, min_r * scale, 90 - max_az, 90 + max_az,
                         fill="#141014", outline="#3A2A2E")
            p = int(self.tl.get("p", 90))
            pan_c = self.params.get("PANC", 90)
            az = pan_c - p                       # обратный пересчёт
            rad = math.radians(90 - az)
            c.create_line(cx, cy, cx + R * math.cos(rad), cy - R * math.sin(rad),
                          fill=ACCENT, width=3)
            x = int(self.tl.get("x", 0)); y = int(self.tl.get("y", 0))
            if x or y:
                tx = cx + x * scale
                ty = cy - y * scale
                c.create_oval(tx - 6, ty - 6, tx + 6, ty + 6, fill=BAD, outline="")
                c.create_text(tx, ty - 14, text=f"{self.tl.get('d','')} мм", fill=BAD,
                              font=("TkDefaultFont", 9))
            mode = {"I": "ДЕЖУРСТВО", "T": "СОПРОВОЖДЕНИЕ"}.get(self.tl.get("m", ""), "—")
            col = ACCENT if mode == "ДЕЖУРСТВО" else BAD
            tilt = self.tl.get("t", "—")
            c.create_text(cx, cy + 24, text=f"{mode} · pan {p}° · tilt {tilt}°",
                          fill=col, font=("TkDefaultFont", 13, "bold"))

        # ---------- журнал ----------
        def log_line(self, text: str):
            self.log.configure(state="normal")
            self.log.insert("end", time.strftime("[%H:%M:%S] ") + text + "\n")
            if int(self.log.index("end-1c").split(".")[0]) > 3000:
                self.log.delete("1.0", "500.0")
            self.log.see("end")
            self.log.configure(state="disabled")

        def on_close(self):
            self.disconnect(silent=True)
            self.destroy()

    App().mainloop()


if __name__ == "__main__":
    run_gui()

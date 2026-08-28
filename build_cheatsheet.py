# -*- coding: utf-8 -*-
"""Шпаргалка А4 «ПВО от комаров»: команды, параметры, распиновки, первая помощь.
Печатать на одном листе А4 (можно с двух сторон — страница 2 про вариант D)."""
import os

from reportlab.lib.colors import HexColor
from reportlab.lib.pagesizes import A4
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "ПВО_шпаргалка_А4.pdf")
F = "/usr/share/fonts/truetype/dejavu/"
pdfmetrics.registerFont(TTFont("DJ", F + "DejaVuSans.ttf"))
pdfmetrics.registerFont(TTFont("DJB", F + "DejaVuSans-Bold.ttf"))
pdfmetrics.registerFont(TTFont("DJM", F + "DejaVuSansMono.ttf"))
pdfmetrics.registerFont(TTFont("DJMB", F + "DejaVuSansMono-Bold.ttf"))

TEAL = HexColor("#00575C")
TEAL2 = HexColor("#1B6E75")
GREY = HexColor("#555555")
LIGHT = HexColor("#E8F1F2")
RED = HexColor("#B3261E")
REDBG = HexColor("#FDECEA")
PURP = HexColor("#7B1FA2")

W, H = A4
MARGIN = 12 * 2.2
COLS = 2
GUT = 14
COL_W = (W - 2 * MARGIN - GUT * (COLS - 1)) / COLS
SC = 1.13          # общий масштаб текста: подобран так, чтобы лист заполнялся


class Sheet:
    """Простой потоковый вывод в две колонки с переносом на новую страницу."""

    def __init__(self, c):
        self.c = c
        self.col = 0
        self.y = 0
        self.page = 0
        self.break_y = None
        self.break_col = 0
        self.new_page()

    def new_page(self):
        if self.page:
            self.c.showPage()
        self.page += 1
        self.col = 0
        self.y = H - MARGIN
        if self.page == 1:
            self.title()
        else:
            self.y -= 4

    def x0(self):
        return MARGIN + self.col * (COL_W + GUT)

    def title(self):
        c = self.c
        c.setFillColor(TEAL)
        c.rect(MARGIN, self.y - 26, W - 2 * MARGIN, 26, stroke=0, fill=1)
        c.setFillColor(HexColor("#FFFFFF"))
        c.setFont("DJB", 13)
        c.drawString(MARGIN + 8, self.y - 18, "ПВО ОТ КОМАРОВ — ШПАРГАЛКА")
        c.setFont("DJ", 7.6)
        c.drawRightString(W - MARGIN - 8, self.y - 17.5,
                          "прошивки: Uno 1.2 · ESP32 1.3   ·   печатать на А4")
        self.y -= 34

    def space(self, dy=5):
        self.y -= dy

    def need(self, h):
        if self.y - h < MARGIN + 10:
            if self.col < COLS - 1:
                self.col += 1
                self.y = H - MARGIN - (34 if self.page == 1 else 4)
            else:
                self.new_page()

    def column_break(self):
        """Перейти в следующую колонку (или на новую страницу)."""
        self.break_y = self.y          # где кончилась колонка — под заметки
        self.break_col = self.col
        if self.col < COLS - 1:
            self.col += 1
            self.y = H - MARGIN - (34 if self.page == 1 else 4)
        else:
            self.new_page()

    def notes_area(self, title="ЗАМЕТКИ"):
        """Разлинованное поле в остатке колонки — для записей у верстака."""
        if self.break_y is None:
            return
        top = self.break_y - 6
        bottom = MARGIN + 10
        if top - bottom < 60:
            return
        x = MARGIN + self.break_col * (COL_W + GUT)
        c = self.c
        c.setFillColor(TEAL2)
        c.rect(x, top - 14, COL_W, 14, stroke=0, fill=1)
        c.setFillColor(HexColor("#FFFFFF"))
        c.setFont("DJB", 8.2 * SC)
        c.drawString(x + 5, top - 10.2, title)
        c.setStrokeColor(HexColor("#C9D6D8"))
        c.setLineWidth(0.5)
        yy = top - 14 - 16
        while yy > bottom:
            c.line(x + 4, yy, x + COL_W - 4, yy)
            yy -= 16

    def head(self, text, color=TEAL2):
        self.need(26)
        c = self.c
        c.setFillColor(color)
        c.rect(self.x0(), self.y - 14, COL_W, 14, stroke=0, fill=1)
        c.setFillColor(HexColor("#FFFFFF"))
        c.setFont("DJB", 8.2 * SC)
        c.drawString(self.x0() + 5, self.y - 10.2, text)
        self.y -= 18.5

    def row(self, left, right, mono=True, size=6.8, gap=8.6, lcol=None):
        size, gap = size * SC, gap * SC
        self.need(gap)
        c = self.c
        c.setFont("DJMB" if mono else "DJB", size)
        c.setFillColor(lcol or HexColor("#222222"))
        c.drawString(self.x0() + 4, self.y - size, left)
        lw = c.stringWidth(left, "DJMB" if mono else "DJB", size)
        c.setFont("DJ", size)
        c.setFillColor(GREY)
        c.drawString(self.x0() + 8 + max(lw, 44), self.y - size, right)
        self.y -= gap

    def line(self, text, size=6.8, gap=8.4, bold=False, color=None):
        size, gap = size * SC, gap * SC
        self.need(gap)
        self.c.setFont("DJB" if bold else "DJ", size)
        self.c.setFillColor(color or HexColor("#222222"))
        self.c.drawString(self.x0() + 4, self.y - size, text)
        self.y -= gap

    def box(self, lines, bg=REDBG, fg=RED, size=6.7):
        size = size * SC
        h = len(lines) * (size + 2.2) + 8
        self.need(h + 4)
        c = self.c
        c.setFillColor(bg)
        c.setStrokeColor(fg)
        c.setLineWidth(0.7)
        c.roundRect(self.x0(), self.y - h, COL_W, h, 3, stroke=1, fill=1)
        c.setFillColor(fg)
        yy = self.y - size - 3
        for i, t in enumerate(lines):
            c.setFont("DJB" if i == 0 else "DJ", size)
            c.drawString(self.x0() + 5, yy, t)
            yy -= size + 2.2
        self.y -= h + 5


def build():
    c = canvas.Canvas(OUT, pagesize=A4)
    c.setTitle("ПВО от комаров — шпаргалка")
    s = Sheet(c)

    # ---------- команды ----------
    s.head("КОМАНДЫ · Uno/Nano (pvo.ino, 9600 бод)")
    for a, b in [("P<угол>", "внешнее целеуказание (режим SLAVE)"),
                 ("R", "вернуть автономию"),
                 ("?", "статус: режим, журнал, датчик, пауза"),
                 ("G", "показать все параметры"),
                 ("S ИМЯ ЗНАЧ", "задать параметр в ОЗУ"),
                 ("W / D", "сохранить в EEPROM / заводские"),
                 ("Z", "обнулить журнал поражений"),
                 ("M1 / M0", "телеметрия для пульта вкл/выкл")]:
        s.row(a, b)

    s.space(3)
    s.head("КОМАНДЫ · ESP32 (pvo_esp32.ino, 115200 бод)")
    for a, b in [("?", "статус (в т.ч. зрение, ИК, пауза, версия)"),
                 ("C", "серво в центр на 8 с — юстировка"),
                 ("L1 / L0", "тест лазера (только с дежурства)"),
                 ("T", "переинициализировать радар"),
                 ("G / S / W / D", "параметры: читать, задать, сохранить, сброс"),
                 ("Z", "обнулить журнал"),
                 ("M1 / M0", "телеметрия"),
                 ("V dP dT", "поправка от зрения, 0,1° (вариант D)"),
                 ("I1 / I0", "ИК-подсветка вкл/выкл (вариант D)")]:
        s.row(a, b)

    # ---------- параметры Uno ----------
    s.space(3)
    s.head("ПАРАМЕТРЫ · Uno (S ИМЯ ЗНАЧ)")
    for a, b in [("DETECT", "60 · порог захвата, см (3–400)"),
                 ("LOST", "90 · порог потери, см (> DETECT)"),
                 ("MINVALID", "3 · ближе — помеха (JSN: 25)"),
                 ("CONFIRM", "3 · подтверждений для захвата"),
                 ("LOSTP", "12 · пустых замеров до отбоя"),
                 ("WOBBLE", "5 · качание при сопровождении, °"),
                 ("STEP", "30 · шаг цикла, мс"),
                 ("KILLMS", "3000 · удержание до «поражения»"),
                 ("EXTMS", "700 · таймаут SLAVE, мс"),
                 ("SWMIN/SWMAX", "15/165 · сектор патруля, °"),
                 ("TRACKMAX", "60000 · лимит сопровождения, мс"),
                 ("TRACKCD", "10000 · пауза после лимита, мс")]:
        s.row(a, b, size=6.5, gap=8.2)

    # ---------- параметры ESP32 ----------
    s.space(3)
    s.head("ПАРАМЕТРЫ · ESP32 (S ИМЯ ЗНАЧ)")
    for a, b in [("PANC/TILTC", "90/90 · центры осей, °"),
                 ("PANMIN/PANMAX", "15/165 · пределы PAN, °"),
                 ("TILTMIN/TILTMAX", "60/120 · пределы TILT, °"),
                 ("PANINV/TILTINV", "0/0 · инверсия направления"),
                 ("ATILT", "1 · автонаклон по дальности"),
                 ("TURH/TGTH", "750/1100 · высоты турели/цели, мм"),
                 ("MINR/MAXR", "300/4000 · рабочая дальность, мм"),
                 ("MAXAZ", "60 · сектор ±, °"),
                 ("CONFIRM", "5 · кадров радара для захвата"),
                 ("LOSTMS", "700 · таймаут потери, мс"),
                 ("KILLMS", "3000 · удержание до «поражения»"),
                 ("ALPHA", "35 · плавность ×100 (0,35)"),
                 ("SLEW", "6 · макс. скорость, °/шаг"),
                 ("PATROL/PSPEED", "1/35 · патруль и его скорость"),
                 ("IRAUTO", "1 · ИК сам при захвате (вар. D)"),
                 ("VISTO", "700 · таймаут зрения, мс (вар. D)"),
                 ("TRACKMAX", "60000 · лимит сопровождения, мс"),
                 ("TRACKCD", "10000 · пауза после лимита, мс")]:
        s.row(a, b, size=6.5, gap=8.2)

    # ---------- вторая колонка ----------
    s.column_break()
    s.head("РАСПИНОВКА · Uno/Nano")
    for a, b in [("D2 / D3", "HC-SR04 ECHO / TRIG"),
                 ("D7 / D8 / D9", "лазер / буззер / серво"),
                 ("D11", "DFPlayer RX через 1 кОм (опция)")]:
        s.row(a, b)
    s.space(2)
    s.head("РАСПИНОВКА · ESP32")
    for a, b in [("GPIO16 / GPIO17", "радар TX / RX (256000 бод)"),
                 ("GPIO26 / GPIO27", "серво PAN / TILT"),
                 ("GPIO23 / GPIO19", "лазер (2N2222) / буззер"),
                 ("GPIO18", "ИК-подсветка (вариант D)"),
                 ("GPIO21 / GPIO22", "OLED SDA / SCL (опция)"),
                 ("GPIO33", "DFPlayer RX через 1 кОм (опция)")]:
        s.row(a, b)

    # ---------- скрипты ----------
    s.space(3)
    s.head("СКРИПТЫ НА КОМПЬЮТЕРЕ", PURP)
    s.line("Пульт (Windows/macOS/Linux):", bold=True, size=6.9)
    s.row("python pvo_gui.py", "порт «ДЕМО» — без железа", size=6.5, gap=8.2)
    s.line("Вариант C — зрение:", bold=True, size=6.9)
    s.row("pvo_vision.py --display", "окно отладки", size=6.5, gap=8.2)
    s.row("--dry-run  --save-hits", "без турели · кадры в hits/", size=6.5, gap=8.2)
    s.line("Вариант D — слияние:", bold=True, size=6.9)
    s.row("pvo_fusion.py --calibrate", "ПРИСТРЕЛКА (обязательно!)", size=6.5, gap=8.2)
    s.row("--detector bright", "ИК-охота на мелочь", size=6.5, gap=8.2)
    s.row("--display  --no-aim", "окно · без пристрелки", size=6.5, gap=8.2)

    # ---------- первая помощь ----------
    s.space(3)
    s.head("ЕСЛИ ЧТО-ТО НЕ ТАК — ПЕРВЫМ ДЕЛОМ")
    for t in ["Турель молчит → «?»: что в статусе (тревога? пауза?)",
              "Серво дёргаются/греются → отдельный БП 5 В, общая земля",
              "Ложные захваты → сузьте зону, поднимите CONFIRM",
              "Лазер мимо цели (вар. D) → сделайте --calibrate",
              "Зрение не слушают → версия прошивки < 1.2",
              "Пауза после лимита → это TRACKMAX, так задумано",
              "Кракозябры в пульте → не та скорость порта",
              "Всё сломал → D (заводские), затем W"]:
        s.line("• " + t, size=6.6, gap=8.4)

    s.space(2)
    s.box(["БЕЗОПАСНОСТЬ",
           "Лазер — только ≤5 мВт (KY-008). «Burning/engraving»",
           "в турель не ставить: травма зрения необратима.",
           "Турель — не на уровне глаз, сектор — мимо окон и дверей.",
           "ИК — только рассеянные светодиоды, не ИК-лазеры:",
           "инфракрасный луч невидим, рефлекс не защищает."])

    s.space(3)
    s.head("ТЕЛЕМЕТРИЯ TL (режим M1)")
    s.line("Uno:  TL m= a= d= k= s=", size=6.6, gap=8.4)
    s.row("m / a / d", "режим (P/T/S) · азимут · дистанция, см", size=6.4, gap=8.0)
    s.row("k / s", "журнал поражений · тревога датчика", size=6.4, gap=8.0)
    s.line("ESP32:  TL m= p= t= x= y= d= k= r= v= i=", size=6.6, gap=8.4)
    s.row("m / p / t", "режим (I/T) · углы PAN и TILT, °", size=6.4, gap=8.0)
    s.row("x / y / d", "координаты цели и дальность, мм", size=6.4, gap=8.0)
    s.row("k / r", "журнал · тревога радара", size=6.4, gap=8.0)
    s.row("v / i", "ведёт зрение · горит ИК (вариант D)", size=6.4, gap=8.0)

    s.space(3)
    s.head("ПОРЯДОК ПЕРВОГО ЗАПУСКА")
    for i, t in enumerate([
            "Питание серво — от БП 5 В, земля общая с платой.",
            "Прошить, дождаться самотеста (прогон + 2 мигания).",
            "«C» (ESP32): серво в центр — затянуть качалки «прямо».",
            "Отъюстировать лазер по «взгляду» датчика/камеры.",
            "Проверить направление: пройти слева направо.",
            "Подобрать пороги (DETECT/MAXR/CONFIRM) под комнату.",
            "Плавность: WOBBLE (A) или ALPHA/SLEW (B).",
            "Вариант D: --calibrate, затем боевой запуск.",
            "Понравилось — «W» (сохранить). Сломалось — «D», потом «W».",
    ], 1):
        s.line(f"{i}. {t}", size=6.5, gap=8.3)

    s.space(3)
    s.head("БЫСТРЫЕ ПРОФИЛИ")
    s.row("демо гостям", "сектор уже, KILLMS 1000, TRACKMAX 120000", size=6.4, gap=8.0)
    s.row("ночь, тихо", "PATROL 0 (B) или STEP 50 (A)", size=6.4, gap=8.0)
    s.row("без присмотра", "TRACKMAX 30000, TRACKCD 30000", size=6.4, gap=8.0)
    s.row("мелочь (D)", "bright + ИК, MIN_AREA_PX 4, тёмный фон", size=6.4, gap=8.0)
    s.row("слабый Pi", "WAKE_EVERY 14, SEND_HZ 8, VISTO 1500", size=6.4, gap=8.0)

    s.space(4)
    s.line("Подробности — «ПВО_от_комаров_руководство.docx»,", size=6.2, color=GREY, gap=7.4)
    s.line("сборка — «Сборка_вариант_A/B/C/D.docx», тесты — tests/run_tests.sh.",
           size=6.2, color=GREY)

    s.notes_area()
    c.save()
    print("OK", OUT)


if __name__ == "__main__":
    build()

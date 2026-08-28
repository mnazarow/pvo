# -*- coding: utf-8 -*-
"""Схемы варианта D: слияние радара и зрения + ИК-подсветка."""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Circle, Rectangle, Wedge
import os

OUT = os.path.dirname(os.path.abspath(__file__))
DPI = 170
C_ESP, C_RPI, C_MOD, C_S1, C_S4 = "#37474F", "#C51A4A", "#546E7A", "#F57C00", "#7B1FA2"
C_IR = "#8E24AA"
plt.rcParams.update({"font.family": "DejaVu Sans"})


def new_ax(w, h):
    fig, ax = plt.subplots(figsize=(w, h))
    ax.set_xlim(0, 100); ax.set_ylim(0, 100)
    ax.axis("off"); fig.patch.set_facecolor("white")
    return fig, ax


def rbox(ax, x, y, w, h, fc, ec=None):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=1.6",
                                fc=fc, ec=ec or fc, lw=1.2))


def arrow(ax, p1, p2, color="#444", lw=1.8, ls="-"):
    ax.add_patch(FancyArrowPatch(p1, p2, arrowstyle="-|>", mutation_scale=14,
                                 color=color, lw=lw, linestyle=ls, zorder=5))


def save(fig, name):
    fig.savefig(os.path.join(OUT, name), dpi=DPI, bbox_inches="tight",
                facecolor="white", pad_inches=0.12)
    plt.close(fig)
    from PIL import Image
    p = os.path.join(OUT, name)
    im = Image.open(p)
    if im.mode != "RGB":
        im.convert("RGB").save(p)
    print("ok", name)


# ========= F24: архитектура слияния =========================
def f24():
    fig, ax = new_ax(11.5, 6.4)
    ax.text(50, 97, "Вариант D «Комар-Ф»: слияние радара и зрения", ha="center",
            fontsize=13.5, fontweight="bold")

    # Турель (площадка со всем хозяйством)
    rbox(ax, 3, 46, 44, 42, "#ECEFF1", ec="#B0BEC5")
    ax.text(25, 84.5, "ТУРЕЛЬ (pan-tilt, ESP32 v1.2)", ha="center", fontsize=9.5,
            color="#37474F", fontweight="bold")
    rbox(ax, 6, 68, 18, 12, C_MOD)
    ax.text(15, 75.5, "Радар", color="white", ha="center", fontsize=9, fontweight="bold")
    ax.text(15, 71, "LD2450/RD-03D", color="#CFD8DC", ha="center", fontsize=6.6)
    rbox(ax, 26, 68, 18, 12, "#263238")
    ax.text(35, 75.5, "Камера", color="white", ha="center", fontsize=9, fontweight="bold")
    ax.text(35, 71, "NoIR, соосно лазеру", color="#B0BEC5", ha="center", fontsize=6.4)
    rbox(ax, 6, 52, 18, 12, C_S4)
    ax.text(15, 59.5, "Лазер", color="white", ha="center", fontsize=9, fontweight="bold")
    ax.text(15, 55, "≤5 мВт", color="#E1BEE7", ha="center", fontsize=6.6)
    rbox(ax, 26, 52, 18, 12, C_IR)
    ax.text(35, 59.5, "ИК-диоды", color="white", ha="center", fontsize=9, fontweight="bold")
    ax.text(35, 55, "GPIO18, вокруг камеры", color="#E1BEE7", ha="center", fontsize=6.2)

    # ESP32 и Pi
    rbox(ax, 51, 58, 18, 24, C_ESP)
    ax.text(60, 74.5, "ESP32", color="white", ha="center", fontsize=10.5, fontweight="bold")
    ax.text(60, 66, "радар, серво,\nлазер, ИК,\nтаймауты", color="#CFD8DC",
            ha="center", fontsize=7)
    rbox(ax, 80, 58, 18, 24, C_RPI)
    ax.text(89, 74.5, "Raspberry Pi", color="white", ha="center", fontsize=9.5, fontweight="bold")
    ax.text(89, 66, "pvo_fusion.py\nдетект в кадре,\nточные поправки", color="#F8BBD0",
            ha="center", fontsize=6.8)

    arrow(ax, (47, 74), (51, 70), color="#455A64")
    ax.text(48.6, 76.8, "UART", fontsize=6.6, color="#455A64", ha="center")
    ax.add_patch(FancyArrowPatch((35, 67), (86, 58), arrowstyle="-|>", mutation_scale=14,
                                 color="#1976D2", lw=1.8, zorder=4,
                                 connectionstyle="arc3,rad=0.32"))
    ax.text(53, 49.5, "USB/CSI-камера → Pi", fontsize=6.6, color="#1976D2", ha="center")
    arrow(ax, (69, 72), (80, 72), color="#00897B")
    arrow(ax, (80, 64), (69, 64), color="#5C6BC0")
    ax.text(74.5, 74.6, "TL (телеметрия)", fontsize=6.6, color="#00897B", ha="center")
    ax.text(74.5, 60.6, "V dP dT · I1/I0", fontsize=6.6, color="#5C6BC0", ha="center",
            fontweight="bold")

    # этапы
    steps = [
        ("1. ДАЛЬНЕЕ ОБНАРУЖЕНИЕ", "#546E7A",
         "радар видит крупную цель за 4–6 м,\nESP32 грубо доворачивает турель\n(зрение в это время дремлет: 1 кадр из 8)"),
        ("2. ПРОБУЖДЕНИЕ", "#8E24AA",
         "по телеметрии «захват» зрение просыпается,\nвключает ИК (I1); цель уже около центра\nкадра — ищем в центральных 70%"),
        ("3. ТОЧНОЕ СОПРОВОЖДЕНИЕ", "#B71C1C",
         "смещение цели от оси камеры → поправки\n«V dPAN dTILT» (0,1°); мелочь зрение\nзахватывает и БЕЗ радара"),
    ]
    for i, (t, col, d) in enumerate(steps):
        x = 3 + i * 33
        rbox(ax, x, 8, 30, 26, "#FAFAFA", ec=col)
        ax.text(x + 15, 29, t, ha="center", fontsize=8.6, color=col, fontweight="bold")
        ax.text(x + 15, 18, d, ha="center", fontsize=6.8, color="#37474F")
        if i < 2:
            arrow(ax, (x + 30.5, 21), (x + 32.5, 21), color="#78909C")
    ax.text(50, 2.5, "Потеря цели: зрение перестаёт слать V → прошивка через VISTO (700 мс) возвращается к радару;\nпотеряли оба → дежурство, ИК гаснет. Обрыв USB безопасен: турель полностью автономна на радаре.",
            ha="center", fontsize=7.6, color="#37474F")
    return fig


# ========= F25: ИК-подсветка ================================
def f25():
    fig, ax = new_ax(11.5, 5.8)
    ax.text(50, 96, "ИК-подсветка для охоты на мелочь (вариант D)", ha="center",
            fontsize=13, fontweight="bold")

    # --- слева: схема ключа ---
    ax.text(24, 87, "Подключение (ключ как у лазера)", ha="center", fontsize=9.5,
            color="#37474F", fontweight="bold")
    ax.plot([6, 42], [78, 78], color="#D32F2F", lw=2)
    ax.text(4.5, 78, "+5 В", color="#D32F2F", fontsize=8.5, ha="right", va="center")
    rbox(ax, 16, 60, 16, 12, C_IR)
    ax.text(24, 67.5, "ИК-модуль", color="white", ha="center", fontsize=8.5, fontweight="bold")
    ax.text(24, 63, "850 нм, 5 В", color="#E1BEE7", ha="center", fontsize=6.8)
    ax.plot([24, 24], [78, 72], color="#D32F2F", lw=2)
    ax.plot([24, 24], [60, 52], color="#212121", lw=2)
    ax.add_patch(Circle((24, 45), 6.5, fc="white", ec="#263238", lw=1.6))
    ax.plot([21.2, 21.2], [41, 49], color="#263238", lw=2)
    ax.plot([21.2, 24], [46.5, 51], color="#263238", lw=2)
    ax.plot([21.2, 24], [43.5, 39], color="#263238", lw=2)
    ax.text(29.5, 50, "К", fontsize=7.5, color="#37474F")
    ax.text(29.5, 39.5, "Э", fontsize=7.5, color="#37474F")
    ax.plot([10, 21.2], [45, 45], color=C_IR, lw=2)
    ax.add_patch(Rectangle((12, 43.7), 4.6, 2.6, fc="white", ec=C_IR, lw=1.3))
    ax.text(14.3, 48.5, "1 кОм", fontsize=6.8, color=C_IR, ha="center")
    ax.text(9.5, 45, "GPIO18", color=C_IR, fontsize=8, ha="right", va="center")
    ax.plot([24, 24], [39, 32], color="#212121", lw=2)
    ax.plot([17, 31], [32, 32], color="#212121", lw=2.2)
    ax.text(33, 32, "GND", fontsize=8, color="#212121", va="center")
    ax.text(24, 22, "2N2222 — для модулей до ~150 мА;\nмощные (48 LED, 300+ мА) — через MOSFET\n(2N7000/AO3400: затвор к GPIO18, без 1 кОм)",
            ha="center", fontsize=7.4, color="#37474F")

    # --- справа: голова турели, вид спереди ---
    import math
    ax.text(75, 87, "Голова турели — вид спереди", ha="center", fontsize=9.5,
            color="#37474F", fontweight="bold")
    rbox(ax, 56, 44, 38, 36, "#ECEFF1", ec="#90A4AE")
    rbox(ax, 59, 63, 11, 10, C_MOD)
    ax.text(64.5, 67.3, "радар", color="white", ha="center", fontsize=7.5)
    ax.add_patch(Circle((80, 61), 3.2, fc="#263238", ec="#111111", zorder=5))
    ax.add_patch(Circle((80, 61), 1.4, fc="#90A4AE", zorder=6))
    for k in range(6):
        a = math.radians(60 * k + 30)
        ax.add_patch(Circle((80 + 6.4 * math.cos(a), 61 + 6.4 * math.sin(a)), 1.25,
                            fc=C_IR, ec="#4A148C", zorder=5))
    ax.text(80, 50.2, "камера (NoIR)", ha="center", fontsize=6.8, color="#263238")
    ax.add_patch(Circle((66, 52), 2.2, fc=C_S4, ec="#E65100", zorder=5))
    ax.text(66, 46.8, "лазер ≤5 мВт", ha="center", fontsize=6.8, color="#E65100")
    ax.annotate("ИК-диоды ×6", xy=(85.5, 64.2), xytext=(93, 74),
                fontsize=6.8, color=C_IR, ha="center",
                arrowprops=dict(arrowstyle="-", color=C_IR, lw=0.9))
    ax.text(75, 37, "ИК-диоды — кольцом вокруг объектива (или готовый модуль вплотную\nк камере). Камера соосна лазеру, ИК светит туда же, куда они смотрят.",
            ha="center", fontsize=7.4, color="#37474F")
    ax.text(75, 26.5, "Камера — БЕЗ ИК-фильтра: Raspberry Pi Camera NoIR,\nлибо обычная вебка со снятым фильтром (аккуратно вынуть\nстёклышко за объективом). С фильтром ИК-охота не работает!",
            ha="center", fontsize=7.6, color="#0D47A1",
            bbox=dict(boxstyle="round,pad=0.4", fc="#E3F2FD", ec="#64B5F6"))
    ax.text(50, 6, "Безопасность: только рассеянные ИК-СВЕТОДИОДЫ, никаких ИК-лазеров. ИК не видим глазом — моргательный рефлекс\nне защищает: не смотрите в излучатель в упор, направляйте подсветку в рабочую зону, а не на людей.",
            ha="center", fontsize=8, color="#721C24",
            bbox=dict(boxstyle="round,pad=0.5", fc="#FDECEA", ec="#F0544F"))
    return fig


save(f24(), "fig24_fusion.png")
save(f25(), "fig25_ir.png")
print("DONE")

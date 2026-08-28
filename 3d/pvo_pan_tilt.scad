// ============================================================
//  ПВО от комаров — печатный pan-tilt кронштейн (вариант B)
//  Под серво SG90 / MG90S. OpenSCAD, все размеры в мм.
//
//  ПЕРЕД ПЕЧАТЬЮ: замерьте штангенциркулем СВОИ серво и поправьте
//  параметры ниже — у клонов размеры гуляют на ±0,5 мм.
//
//  Печать: PLA/PETG, слой 0.2, 3 периметра, заполнение 25%,
//  поддержки не нужны (все детали печатаются плашмя).
//
//  Сборка: base на основание → серво PAN в окно base →
//  bracket на качалку PAN → серво TILT в окно bracket →
//  plate на качалку TILT → радар и лазер стяжками в прорези.
// ============================================================

// ---- что рендерить: "all" | "base" | "bracket" | "plate" ----
part = "all";

// ---- размеры серво (SG90/MG90S типовые) --------------------
servo_l    = 23.2;   // длина корпуса
servo_w    = 12.6;   // ширина корпуса
ear_span   = 32.6;   // размах ушей
ear_hole_d = 2.4;    // отверстия ушей (саморез 2 мм)
ear_hole_s = 27.8;   // расстояние между отверстиями
tol        = 0.4;    // зазор посадки

// ---- прочее ------------------------------------------------
wall       = 3;      // толщина стенок/плит
horn_d     = 7.2;    // отверстие под втулку качалки
horn_r     = 8;      // радиус винтов крестовины качалки
horn_hole  = 2.2;    // отверстия под винты качалки
$fn = 48;

// ============================================================
module servo_window() {                     // сквозное окно под корпус
  square([servo_l + tol, servo_w + tol], center = true);
}
module ear_holes() {                        // отверстия ушей
  for (dx = [-ear_hole_s / 2, ear_hole_s / 2])
    translate([dx, 0]) circle(d = ear_hole_d);
}
module horn_mount() {                       // под круглую крестовину качалки
  circle(d = horn_d);
  for (a = [0, 90, 180, 270])
    rotate(a) translate([horn_r, 0]) circle(d = horn_hole);
}
module ziptie_pair(len = 6, gap = 14) {     // пара прорезей под стяжку
  for (dy = [-gap / 2, gap / 2])
    translate([0, dy]) square([len, 2.6], center = true);
}

// ============================================================
//  1) BASE — плита с окном серво PAN
// ============================================================
module base() {
  difference() {
    linear_extrude(wall) hull()
      for (x = [-32, 32], y = [-27, 27])
        translate([x, y]) circle(r = 4);
    translate([0, 0, -1]) linear_extrude(wall + 2) {
      servo_window();
      ear_holes();
      for (x = [-32, 32], y = [-27, 27])      // крепёж к основанию
        translate([x, y]) circle(d = 3.4);
    }
  }
}

// ============================================================
//  2) BRACKET — Г-скоба: низ на качалку PAN, стенка с серво TILT
// ============================================================
br_w = 34;            // ширина скобы
br_base_l = 34;       // длина подошвы
br_h = 46;            // высота стенки

module bracket() {
  // подошва с крестовиной качалки PAN
  difference() {
    linear_extrude(wall) translate([br_base_l / 2, 0])
      square([br_base_l, br_w], center = true);
    translate([br_base_l / 2, 0, -1]) linear_extrude(wall + 2) horn_mount();
  }
  // стенка с окном серво TILT (осью качалки вперёд)
  difference() {
    translate([0, -br_w / 2, 0]) cube([wall, br_w, br_h]);
    translate([-1, 0, br_h - 6 - servo_w / 2])
      rotate([0, 90, 0]) linear_extrude(wall + 2) {
        servo_window();
        ear_holes();
      }
  }
  // косынка жёсткости
  translate([0, -br_w / 2, 0])
    rotate([90, 0, 90]) linear_extrude(wall)
      polygon([[0, 0], [br_w, 0], [br_w / 2, 12]]);
}

// ============================================================
//  3) PLATE — площадка радара и лазера на качалку TILT
// ============================================================
pl_l = 72;
pl_w = 36;

module plate() {
  difference() {
    linear_extrude(wall) hull()
      for (x = [-pl_l / 2 + 4, pl_l / 2 - 4], y = [-pl_w / 2 + 4, pl_w / 2 - 4])
        translate([x, y]) circle(r = 4);
    translate([0, 0, -1]) linear_extrude(wall + 2) {
      translate([-pl_l / 2 + 12, 0]) horn_mount();      // на качалку TILT
      translate([2, 0])  ziptie_pair(gap = 18);         // радар (2 стяжки)
      translate([14, 0]) ziptie_pair(gap = 18);
      translate([27, 0]) ziptie_pair(gap = 10);         // лазер
      for (x = [-6 : 8 : 30], y = [-12, 12])            // запасные 2 мм
        translate([x, y]) circle(d = 2.2);
    }
  }
}

// ============================================================
if (part == "base")         base();
else if (part == "bracket") bracket();
else if (part == "plate")   plate();
else {                                    // все детали рядом
  base();
  translate([0, 75, 0])  bracket();
  translate([0, 145, 0]) plate();
}

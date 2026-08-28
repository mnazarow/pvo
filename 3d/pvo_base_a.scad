// ============================================================
//  ПВО от комаров — печатное основание варианта A (Uno + HC-SR04)
//  Плита: окно под серво + монтажное поле для платы и макетки.
//  OpenSCAD, размеры в мм. Печать: PLA, слой 0.2, без поддержек.
//
//  Серво ставится в окно ушами вниз и прикручивается саморезами;
//  Arduino и макетка крепятся стяжками через поле отверстий
//  (шаг 10 мм) или на двусторонний скотч.
// ============================================================

plate_l = 130;
plate_w = 90;
wall    = 4;

// серво SG90/MG90S (замерьте свои!)
servo_l    = 23.2;
servo_w    = 12.6;
ear_hole_d = 2.4;
ear_hole_s = 27.8;
tol        = 0.4;

$fn = 48;

module servo_cut() {
  square([servo_l + tol, servo_w + tol], center = true);
  for (dx = [-ear_hole_s / 2, ear_hole_s / 2])
    translate([dx, 0]) circle(d = ear_hole_d);
}

difference() {
  linear_extrude(wall) hull()
    for (x = [-plate_l / 2 + 5, plate_l / 2 - 5],
         y = [-plate_w / 2 + 5, plate_w / 2 - 5])
      translate([x, y]) circle(r = 5);

  translate([0, 0, -1]) linear_extrude(wall + 2) {
    // окно серво — ближе к «носу» плиты
    translate([plate_l / 2 - 26, 0]) rotate(90) servo_cut();

    // поле отверстий 2×3 мм с шагом 10 мм под стяжки/винты
    for (x = [-plate_l / 2 + 14 : 10 : plate_l / 2 - 40],
         y = [-plate_w / 2 + 12 : 10 : plate_w / 2 - 12])
      translate([x, y]) circle(d = 3.2);

    // угловые крепления к столу/фанере
    for (x = [-plate_l / 2 + 6, plate_l / 2 - 6],
         y = [-plate_w / 2 + 6, plate_w / 2 - 6])
      translate([x, y]) circle(d = 3.4);
  }
}

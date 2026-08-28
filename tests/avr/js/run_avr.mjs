// ============================================================
//  Эмулятор ATmega328P (avr8js — тот же движок, что у Wokwi):
//  гоняет НАСТОЯЩУЮ прошивку, собранную avr-gcc, без железа.
//
//  Что это добавляет к тестам на заглушках (tests/stub_uno):
//    · код скомпилирован под AVR (int 16 бит, F() в PROGMEM, EEPROM);
//    · настоящие тайминги: pulseIn, delay, ШИМ серво 50 Гц;
//    · настоящая периферия: UART 9600, EEPROM между перезагрузками.
//
//  Запуск:  bash tests/avr/build_avr.sh pvo_a_simple/pvo_a_simple.ino
//           node tests/avr/js/run_avr.mjs /tmp/avrbuild/firmware.hex
//  Ключи:   --seconds N        сколько модельного времени прогнать
//           --target-at N,M    «цель» на дистанции N см с секунды M
//           --target-gone M    цель ушла на секунде M
//           --send "ТЕКСТ@M"   отправить команду в UART на секунде M
//           --expect "ТЕКСТ"   проверка: строка должна появиться в выводе
//           --forbid "ТЕКСТ"   проверка: строки быть НЕ должно
//           --eeprom ФАЙЛ      EEPROM переживает перезапуск (как на плате)
// ============================================================
import fs from "fs";
import { CPU, avrInstruction, AVRTimer, timer0Config, timer1Config, timer2Config,
         AVRIOPort, portBConfig, portCConfig, portDConfig, AVRUSART,
         usart0Config, AVREEPROM, EEPROMMemoryBackend } from "avr8js";

const MHZ = 16e6;
const args = process.argv.slice(2);
const hexPath = args.find(a => !a.startsWith("--")) || "/tmp/avrbuild/firmware.hex";
const opt = (name, def) => {
  const i = args.indexOf("--" + name);
  return i >= 0 ? args[i + 1] : def;
};
const SECONDS = parseFloat(opt("seconds", "6"));

// --- Intel HEX -> память программ ---
function loadHex(file, target) {
  for (const line of fs.readFileSync(file, "utf-8").split("\n")) {
    if (line[0] !== ":") continue;
    const len = parseInt(line.substr(1, 2), 16);
    const addr = parseInt(line.substr(3, 4), 16);
    const type = parseInt(line.substr(7, 2), 16);
    if (type !== 0) continue;
    for (let i = 0; i < len; i++)
      target[addr + i] = parseInt(line.substr(9 + i * 2, 2), 16);
  }
}

const program = new Uint16Array(0x8000);
loadHex(hexPath, new Uint8Array(program.buffer));

const cpu = new CPU(program);
new AVRTimer(cpu, timer0Config);
const timer1 = new AVRTimer(cpu, timer1Config);
new AVRTimer(cpu, timer2Config);
const portB = new AVRIOPort(cpu, portBConfig);   // D8..D13
const portC = new AVRIOPort(cpu, portCConfig);   // A0..A5
const portD = new AVRIOPort(cpu, portDConfig);   // D0..D7
const usart = new AVRUSART(cpu, usart0Config, MHZ);

// EEPROM обязательна: без неё прошивка вечно ждёт окончания записи
// (EEPE никто не сбрасывает) и виснет прямо в setup().
const eepromFile = opt("eeprom", null);
const eepromBackend = new EEPROMMemoryBackend(1024);
if (eepromFile && fs.existsSync(eepromFile))
  eepromBackend.memory.set(new Uint8Array(fs.readFileSync(eepromFile)));
new AVREEPROM(cpu, eepromBackend);

// ---------- UART: вывод прошивки и ввод команд ----------
// Байты копим и декодируем как UTF-8 целиком: прошивка шлёт кириллицу,
// побайтовый вывод превратил бы её в кракозябры.
const uartBytes = [];
let uartOut = "";
usart.onByteTransmit = (b) => {
  uartBytes.push(b);
  if (b === 10) {                       // строка закончилась — печатаем
    const line = Buffer.from(uartBytes).toString("utf-8");
    uartOut += line;
    process.stdout.write(line);
    uartBytes.length = 0;
  }
};
const txQueue = [];
function sendLine(s) { for (const ch of s + "\n") txQueue.push(ch.charCodeAt(0)); }

// ---------- наблюдение за выходами турели ----------
// A-вариант: серво D9 (портB бит 1), лазер D7 (портD бит 7), буззер D8 (портB бит 0)
const pulses = [];            // измеренные импульсы серво, мкс
let servoHighAt = null;
portB.addListener(() => {
  const high = (portB.pinState(1) === 1);
  if (high && servoHighAt === null) servoHighAt = cpu.cycles;
  else if (!high && servoHighAt !== null) {
    pulses.push((cpu.cycles - servoHighAt) / (MHZ / 1e6));
    servoHighAt = null;
    if (pulses.length > 400) pulses.shift();
  }
});
const laserOn = () => portD.pinState(7) === 1;

// ---------- имитация HC-SR04 ----------
// TRIG D3 (портD бит 3) -> через 400 мкс выдаём ECHO D2 (портD бит 2)
let targetCm = null;                       // null = «пусто», иначе дистанция
const TRIG_BIT = 3, ECHO_BIT = 2;
let echoTask = null;
portD.addListener(() => {
  if (portD.pinState(TRIG_BIT) === 1 && !echoTask) {
    const cm = targetCm === null ? 180 : targetCm;      // нет цели — эхо «стены»
    const startAt = cpu.cycles + 0.0004 * MHZ;          // пауза перед эхом
    const width = cm * 58 * (MHZ / 1e6);                // 58 мкс на см
    echoTask = { startAt, endAt: startAt + width, started: false };
  }
});
function serviceEcho() {
  if (!echoTask) return;
  if (!echoTask.started && cpu.cycles >= echoTask.startAt) {
    portD.setPin(ECHO_BIT, true); echoTask.started = true;
  } else if (echoTask.started && cpu.cycles >= echoTask.endAt) {
    portD.setPin(ECHO_BIT, false); echoTask = null;
  }
}

// ---------- сценарий ----------
const events = [];
for (let i = 0; i < args.length; i++) {
  if (args[i] === "--target-at") {
    const [cm, at] = args[i + 1].split(",").map(Number);
    events.push({ at, run: () => { targetCm = cm; console.error(`\n[стенд] цель на ${cm} см`); } });
  }
  if (args[i] === "--target-gone") {
    events.push({ at: Number(args[i + 1]), run: () => { targetCm = null; console.error("\n[стенд] цель ушла"); } });
  }
  if (args[i] === "--send") {
    const [text, at] = args[i + 1].split("@");
    events.push({ at: Number(at || 1), run: () => { sendLine(text); console.error(`\n[стенд] -> ${text}`); } });
  }
}
events.sort((a, b) => a.at - b.at);

// ---------- главный цикл ----------
const endCycles = SECONDS * MHZ;
let nextUart = 0;
while (cpu.cycles < endCycles) {
  avrInstruction(cpu);
  cpu.tick();
  if ((cpu.cycles & 0x3f) === 0) {
    serviceEcho();
    const t = cpu.cycles / MHZ;
    while (events.length && events[0].at <= t) events.shift().run();
    if (txQueue.length && cpu.cycles > nextUart && !usart.rxBusy) {
      usart.writeByte(txQueue.shift());
      nextUart = cpu.cycles + MHZ / 900;        // ~9600 бод
    }
  }
}

// ---------- проверки ----------
const expects = [], forbids = [];
for (let i = 0; i < args.length; i++) {
  if (args[i] === "--expect") expects.push(args[i + 1]);
  if (args[i] === "--forbid") forbids.push(args[i + 1]);
}
let failed = 0;
for (const e of expects) {
  const ok = uartOut.includes(e);
  if (!ok) failed++;
  console.error(`${ok ? "  ✔" : "  ✘ НЕТ"} ожидалось: «${e}»`);
}
for (const f of forbids) {
  const bad = uartOut.includes(f);
  if (bad) failed++;
  console.error(`${bad ? "  ✘ ЕСТЬ" : "  ✔"} не должно быть: «${f}»`);
}

// ---------- отчёт ----------
const recent = pulses.slice(-8);
const avgUs = recent.length ? recent.reduce((a, b) => a + b, 0) / recent.length : 0;
const angle = avgUs ? Math.round((avgUs - 544) / (2400 - 544) * 180) : null;
console.error("\n================ ИТОГ ЭМУЛЯЦИИ ================");
console.error(`модельное время: ${SECONDS} с, тактов: ${cpu.cycles}`);
console.error(`импульсов серво: ${pulses.length}` +
  (avgUs ? `, последний ~${Math.round(avgUs)} мкс = ${angle}°` : ""));
console.error(`лазер: ${laserOn() ? "ГОРИТ" : "погашен"}`);
console.error(`строк UART: ${uartOut.split("\n").length}`);
if (eepromFile) {                       // сохранить EEPROM для следующего запуска
  fs.writeFileSync(eepromFile, Buffer.from(eepromBackend.memory));
  console.error(`EEPROM сохранена: ${eepromFile}`);
}
process.exit(failed ? 1 : 0);

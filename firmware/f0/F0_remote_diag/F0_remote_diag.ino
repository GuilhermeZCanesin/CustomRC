// F0_remote_diag.ino — Diagnóstico elétrico do CONTROLE (fase F0)
//
// O que faz:
//   - Lê os dois HW-504 (IO35 aceleração, IO34 direção) com média de 16 amostras,
//     em valor bruto (0-4095) e em mV, e guarda mínimo/máximo desde o boot.
//   - Lê os 6 botões (cliques dos sticks + 4 trims) com pull-up interno.
//   - Faz scan I2C e, se o OLED SH1106 estiver em 0x3C, mostra os valores nele.
//
// Comandos pela Serial (115200): z = zera min/max, ? = ajuda

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

constexpr int PIN_THR = 35;  // HW-504 aceleração, VRy
constexpr int PIN_STR = 34;  // HW-504 direção, VRx
constexpr int PIN_SDA = 21;
constexpr int PIN_SCL = 22;

struct Btn { int pin; const char *name; };
const Btn BUTTONS[] = {
  {13, "ARM"}, {14, "MENU"},
  {5, "ACC-"}, {18, "ACC+"}, {19, "STR+"}, {23, "STR-"},
};
constexpr int N_BTN = sizeof(BUTTONS) / sizeof(BUTTONS[0]);

Adafruit_SH1106G display(128, 64, &Wire, -1);
bool hasOled = false;

struct Axis {
  int pin;
  int raw, mv;
  int rawMin, rawMax;
  explicit Axis(int p) : pin(p), raw(0), mv(0), rawMin(4095), rawMax(0) {}
};
Axis thr(PIN_THR), str(PIN_STR);

uint32_t lastPrint = 0;

void readAxis(Axis &a) {
  uint32_t accRaw = 0, accMv = 0;
  for (int i = 0; i < 16; i++) {
    accRaw += analogRead(a.pin);
    accMv  += analogReadMilliVolts(a.pin);
  }
  a.raw = accRaw / 16;
  a.mv  = accMv / 16;
  a.rawMin = min(a.rawMin, a.raw);
  a.rawMax = max(a.rawMax, a.raw);
}

bool i2cScan() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Serial.print(F("I2C scan (SDA 21 / SCL 22): "));
  bool oled = false;
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("0x%02X ", addr);
      found++;
      if (addr == 0x3C || addr == 0x3D) oled = true;
    }
  }
  if (found == 0) Serial.print(F("nada encontrado (OLED desligado ou fiacao)"));
  Serial.println();
  return oled;
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'z') {
      thr.rawMin = str.rawMin = 4095;
      thr.rawMax = str.rawMax = 0;
      Serial.println(F("Min/max zerados"));
    } else if (c == '?') {
      Serial.println(F("Comandos: z=zerar min/max | ?=ajuda"));
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  for (const Btn &b : BUTTONS) pinMode(b.pin, INPUT_PULLUP);

  Serial.println();
  Serial.println(F("=== F0 diagnostico - CONTROLE ==="));
  hasOled = i2cScan();
  if (hasOled) {
    Wire.setClock(400000);
    hasOled = display.begin(0x3C, true);
  }
  Serial.printf("OLED: %s\n", hasOled ? "OK" : "nao encontrado");
  Serial.println(F("Mova cada stick ate o fim nos dois sentidos e aperte cada botao."));
  Serial.println(F("ACC raw (mV) [min-max]      | STR raw (mV) [min-max]      | botoes"));
}

void loop() {
  handleSerial();
  uint32_t now = millis();
  if (now - lastPrint < 100) return;
  lastPrint = now;

  readAxis(thr);
  readAxis(str);

  char btns[64] = "";
  for (const Btn &b : BUTTONS) {
    if (digitalRead(b.pin) == LOW) {
      strcat(btns, b.name);
      strcat(btns, " ");
    }
  }

  Serial.printf("%4d (%4d) [%4d-%4d] | %4d (%4d) [%4d-%4d] | %s\n",
                thr.raw, thr.mv, thr.rawMin, thr.rawMax,
                str.raw, str.mv, str.rawMin, str.rawMax,
                btns[0] ? btns : "-");

  if (hasOled) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.println(F("F0 CONTROLE"));
    display.printf("ACC %4d %4dmV\n", thr.raw, thr.mv);
    display.printf("    %4d-%4d\n", thr.rawMin, thr.rawMax);
    display.printf("STR %4d %4dmV\n", str.raw, str.mv);
    display.printf("    %4d-%4d\n", str.rawMin, str.rawMax);
    display.println(btns[0] ? btns : "-");
    display.display();
  }
}

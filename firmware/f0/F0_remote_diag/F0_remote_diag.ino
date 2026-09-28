// F0_remote_diag.ino — Diagnóstico elétrico do CONTROLE (fase F0)
//
// O que faz:
//   - Lê os dois HW-504 (IO35 aceleração, IO34 direção) com média de 16 amostras,
//     em valor bruto (0-4095) e em mV, e guarda mínimo/máximo desde o boot.
//   - Lê os botões com pull-up interno: os 4 trims (ligados) e ARM/MENU em
//     IO32/IO33 (previstos, ainda sem fio — não contam como falha).
//   - Faz scan I2C e, se o OLED SH1106 estiver em 0x3C, mostra os valores nele.
//
// Comandos pela Serial (115200): z = zera min/max e botões vistos,
//                                b = resumo dos botões (quais já apareceram), ? = ajuda

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// Os DOIS joysticks usam o VRx (o VRy de ambos fica sem uso) — o stick da
// aceleração esta montado girado, entao frente/tras cai no VRx dele.
constexpr int PIN_THR = 35;  // HW-504 aceleração, VRx
constexpr int PIN_STR = 34;  // HW-504 direção, VRx
constexpr int PIN_SDA = 21;
constexpr int PIN_SCL = 22;

// wired=false: previsto mas ainda sem fio. Nao conta como falha no resumo.
// ARM e MENU ficam em IO32/IO33 (e nao nos cliques dos sticks, que nao serao
// usados): 32 e 33 tem pull-up interno, ao contrario de 34/35/36/39.
struct Btn { int pin; const char *name; bool wired; };
const Btn BUTTONS[] = {
  {32, "ARM",  false}, {33, "MENU", false},
  {5,  "ACC-", true }, {18, "ACC+", true }, {19, "STR+", true}, {23, "STR-", true},
};
constexpr int N_BTN = sizeof(BUTTONS) / sizeof(BUTTONS[0]);

// Latch: uma vez visto em LOW, fica marcado. Assim da para apertar sem pressa
// e consultar depois, sem depender de olhar a serial na hora certa.
bool btnSeen[N_BTN] = {false};

Adafruit_SH1106G display(128, 64, &Wire, -1);
bool hasOled = false;

struct Axis {
  int pin;
  int raw, mv;
  int rawMin, rawMax;
  int mvMin, mvMax;   // em mV, para ver em que tensao o ADC satura
  explicit Axis(int p)
    : pin(p), raw(0), mv(0), rawMin(4095), rawMax(0), mvMin(9999), mvMax(0) {}
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
  a.mvMin  = min(a.mvMin, a.mv);
  a.mvMax  = max(a.mvMax, a.mv);
}

void printButtons() {
  Serial.println(F("--- botoes vistos desde o boot ---"));
  int missing = 0, expected = 0;
  for (int i = 0; i < N_BTN; i++) {
    const char *estado;
    if (!BUTTONS[i].wired)      estado = "sem fio (previsto)";
    else if (btnSeen[i])        estado = "JA APARECEU       ";
    else                      { estado = "NUNCA (!)         "; missing++; }
    if (BUTTONS[i].wired) expected++;
    Serial.printf("  %-5s IO%-2d  %s  (agora: %s)\n",
                  BUTTONS[i].name, BUTTONS[i].pin, estado,
                  digitalRead(BUTTONS[i].pin) == LOW ? "apertado" : "solto");
  }
  if (missing == 0) Serial.printf("  >>> os %d botoes ligados OK.\n", expected);
  else Serial.printf("  >>> faltam %d de %d. Fio solto, GND ausente ou pino errado.\n",
                     missing, expected);
}

void printAxes() {
  Serial.println(F("--- eixos ---"));
  Serial.printf("  ACC IO%d: bruto %4d..%4d | mV %4d..%4d | repouso %4d (%d mV)\n",
                thr.pin, thr.rawMin, thr.rawMax, thr.mvMin, thr.mvMax, thr.raw, thr.mv);
  Serial.printf("  STR IO%d: bruto %4d..%4d | mV %4d..%4d | repouso %4d (%d mV)\n",
                str.pin, str.rawMin, str.rawMax, str.mvMin, str.mvMax, str.raw, str.mv);
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
      thr.mvMin  = str.mvMin  = 9999;
      thr.mvMax  = str.mvMax  = 0;
      for (int i = 0; i < N_BTN; i++) btnSeen[i] = false;
      Serial.println(F("Min/max e botoes vistos zerados"));
    } else if (c == 'b') {
      printButtons();
      printAxes();
    } else if (c == '?') {
      Serial.println(F("Comandos: z=zerar min/max e botoes | b=resumo botoes+eixos | ?=ajuda"));
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
  Serial.println(F("Sem pressa: os botoes ficam marcados. Depois mande 'b' para o resumo."));
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
  for (int i = 0; i < N_BTN; i++) {
    if (digitalRead(BUTTONS[i].pin) == LOW) {
      btnSeen[i] = true;            // latch, para consultar depois com 'b'
      strcat(btns, BUTTONS[i].name);
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

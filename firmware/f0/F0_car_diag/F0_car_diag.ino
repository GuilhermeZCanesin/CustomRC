// F0_car_diag.ino — Diagnóstico elétrico do CARRO (fase F0)
//
// O que faz:
//   - Registra o motivo de cada boot em NVS e conta brownouts entre reinícios.
//   - Lê a tensão da bateria no IO36 (divisor 100k/22k) e mostra V total e V/célula.
//   - Mantém o servo (IO25) no centro ou em varredura, para testar pico de corrente.
//   - Liga o rádio e envia broadcast ESP-NOW a 100 Hz em potência máxima (picos de TX).
//   - Faz scan I2C (IO21/IO22) para achar o OLED.
//   - Desenha tensão e contadores no OLED 128x32, se ele estiver presente.
//
// Segurança: os pinos dos ESCs (26, 27, 32, 33) ficam em LOW fixo, sem pulsos.
// ESC sem sinal não arma. Mesmo assim, teste com rodas fora do chão.
//
// Comandos pela Serial (115200): s = liga/desliga varredura do servo,
//                                c = servo no centro, z = zera contadores,
//                                i = refaz scan I2C, I = scan I2C com SDA/SCL trocados,
//                                d = tenta reconectar o OLED, ? = ajuda

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

constexpr int PIN_SERVO = 25;
constexpr int PIN_VBAT  = 36;
constexpr int PIN_SDA   = 21;
constexpr int PIN_SCL   = 22;
constexpr int ESC_PINS[] = {26, 27, 32, 33};

constexpr float DIV_RATIO = (100.0f + 22.0f) / 22.0f;  // 5,545
constexpr int   CELLS     = 4;

constexpr int     OLED_W    = 128;
constexpr int     OLED_H    = 32;
constexpr uint8_t OLED_ADDR = 0x3C;

Servo servo;
Preferences prefs;
Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, -1);

bool     oledOk      = false;
uint32_t oledDrops   = 0;   // quantas vezes o display sumiu do barramento
uint32_t bootsNow = 0, brownNow = 0, crashNow = 0;

bool     sweep     = false;
int      servoUs   = 1500;
uint32_t lastSweep = 0;
uint32_t lastPrint = 0;
uint32_t lastTx    = 0;
uint32_t txOk = 0, txErr = 0;
float    vbatMin = 99.0f, vbatMax = 0.0f;

uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

const char *reasonStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "POWERON (liga/EN/upload)";
    case ESP_RST_SW:        return "SW (reinicio por software)";
    case ESP_RST_PANIC:     return "PANIC (crash)";
    case ESP_RST_INT_WDT:   return "INT_WDT (watchdog)";
    case ESP_RST_TASK_WDT:  return "TASK_WDT (watchdog)";
    case ESP_RST_WDT:       return "WDT (watchdog)";
    case ESP_RST_BROWNOUT:  return "BROWNOUT (queda de tensao!)";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    default:                return "OUTRO";
  }
}

void logBoot() {
  esp_reset_reason_t r = esp_reset_reason();
  prefs.begin("f0diag", false);
  uint32_t boots = prefs.getUInt("boots", 0) + 1;
  uint32_t brown = prefs.getUInt("brown", 0) + (r == ESP_RST_BROWNOUT ? 1 : 0);
  bool crash = (r == ESP_RST_PANIC || r == ESP_RST_INT_WDT || r == ESP_RST_TASK_WDT || r == ESP_RST_WDT);
  uint32_t crashes = prefs.getUInt("crash", 0) + (crash ? 1 : 0);
  prefs.putUInt("boots", boots);
  prefs.putUInt("brown", brown);
  prefs.putUInt("crash", crashes);
  bootsNow = boots; brownNow = brown; crashNow = crashes;

  Serial.println();
  Serial.println(F("=== F0 diagnostico - CARRO ==="));
  Serial.printf("Boot #%lu | motivo: %s\n", (unsigned long)boots, reasonStr(r));
  Serial.printf("Historico: %lu brownout(s), %lu crash/watchdog\n",
                (unsigned long)brown, (unsigned long)crashes);
  if (brown > 0) Serial.println(F(">>> ATENCAO: houve brownout. Revise alimentacao do ESP / C1."));
}

// Scan com os pinos escolhidos. Chamar com (22, 21) testa a hipotese de
// SDA/SCL trocados sem precisar mexer nos fios.
void i2cScan(int sda, int scl) {
  Wire.end();
  Wire.begin(sda, scl, 100000);
  Serial.printf("I2C scan (SDA %d / SCL %d): ", sda, scl);
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("0x%02X ", addr);
      found++;
    }
  }
  if (found == 0) Serial.print(F("nada encontrado"));
  Serial.println();

  // HIGH aqui NAO prova que ha algo conectado: sao os pull-ups internos que o
  // Wire.begin liga. So serve para achar curto para GND (LOW) ou pull-up ausente.
  // Para decidir presenca, use o scan acima ou o comando I (pinos trocados).
  Wire.end();
  pinMode(sda, INPUT);
  pinMode(scl, INPUT);
  delayMicroseconds(50);
  Serial.printf("  linhas em repouso: IO%d=%s  IO%d=%s\n",
                sda, digitalRead(sda) ? "HIGH" : "LOW (!)",
                scl, digitalRead(scl) ? "HIGH" : "LOW (!)");
  Wire.begin(sda, scl, 100000);
}

// O OLED do carro e opcional: se nao inicializar, o diagnostico segue sem ele.
// Retorna true se o display respondeu.
bool oledInit(bool verbose) {
  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oledOk) {
    oled.setTextColor(SSD1306_WHITE);
    oled.clearDisplay();
    oled.display();
  }
  if (verbose) {
    Serial.printf("OLED 0x%02X: %s\n", OLED_ADDR,
                  oledOk ? "OK" : "ausente (segue sem display)");
  }
  return oledOk;
}

// Confere se o display ainda responde. Jumper solto so aparece aqui.
void oledCheckPresence() {
  Wire.beginTransmission(OLED_ADDR);
  bool present = (Wire.endTransmission() == 0);
  if (oledOk && !present) {
    oledOk = false;
    oledDrops++;
    Serial.printf(">>> OLED sumiu do barramento (queda #%lu). Rode 'I': se achar 0x3C,\n"
                  "    SDA/SCL estao trocados. Se nao achar em nenhum, veja 3V3/GND.\n",
                  (unsigned long)oledDrops);
  } else if (!oledOk && present) {
    Serial.println(F(">>> OLED voltou ao barramento, reinicializando."));
    oledInit(true);
  }
}

void oledDraw(float vbat, float pinMv) {
  if (!oledOk) return;
  oled.clearDisplay();

  // Linha grande: tensao total da bateria
  oled.setTextSize(2);
  oled.setCursor(0, 0);
  oled.printf("%.2fV", vbat);

  // Canto direito, pequeno: estado do servo
  oled.setTextSize(1);
  oled.setCursor(80, 0);
  oled.print(sweep ? "SWEEP" : "CENTRO");
  oled.setCursor(80, 9);
  oled.printf("%4dus", servoUs);

  // Rodape: V/celula, pino e contadores que importam na F0
  oled.setCursor(0, 17);
  oled.printf("%.2fV/cel %4.0fmV", vbat / CELLS, pinMv);
  oled.setCursor(0, 25);
  oled.printf("bt%lu bo%lu cr%lu tx%lu",
              (unsigned long)bootsNow, (unsigned long)brownNow,
              (unsigned long)crashNow, (unsigned long)txOk);

  oled.display();
}

void radioLoadInit() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    Serial.println(F("ERRO: esp_now_init falhou"));
    return;
  }
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, bcast, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
  esp_wifi_set_max_tx_power(80);  // 20 dBm: maior pico de corrente possivel
  Serial.println(F("Radio: broadcast ESP-NOW 100 Hz, potencia maxima"));
}

float readVbatPinMv() {
  uint32_t acc = 0;
  for (int i = 0; i < 32; i++) acc += analogReadMilliVolts(PIN_VBAT);
  return acc / 32.0f;
}

void printHelp() {
  Serial.println(F("Comandos: s=varredura servo on/off | c=servo centro | z=zerar contadores"));
  Serial.println(F("          i=scan I2C (21/22) | I=scan I2C trocado (22/21)"));
  Serial.println(F("          d=reconectar OLED | ?=ajuda"));
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 's': sweep = !sweep; Serial.printf("Varredura: %s\n", sweep ? "ON" : "OFF"); break;
      case 'c': sweep = false; servoUs = 1500; servo.writeMicroseconds(servoUs); Serial.println(F("Servo no centro")); break;
      case 'z': prefs.clear(); vbatMin = 99; vbatMax = 0; txOk = txErr = 0; oledDrops = 0; Serial.println(F("Contadores zerados")); break;
      // O scan reinicia o barramento, entao o display precisa voltar depois dele
      case 'i': i2cScan(PIN_SDA, PIN_SCL); oledInit(true); break;
      case 'I': i2cScan(PIN_SCL, PIN_SDA); i2cScan(PIN_SDA, PIN_SCL); oledInit(true); break;
      case 'd': oledInit(true); break;
      case '?': printHelp(); break;
      default: break;
    }
  }
}

void setup() {
  // ESCs primeiro: LOW fixo = sem sinal = nao armam
  for (int p : ESC_PINS) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }

  Serial.begin(115200);
  delay(300);
  logBoot();
  i2cScan(PIN_SDA, PIN_SCL);
  oledInit(true);

  servo.setPeriodHertz(50);
  servo.attach(PIN_SERVO, 1000, 2000);
  servo.writeMicroseconds(servoUs);

  radioLoadInit();
  printHelp();
  Serial.println(F("t(s)  | pino mV | bateria V | V/celula | min-max V   | servo us | tx ok/erro"));
}

void loop() {
  uint32_t now = millis();
  handleSerial();

  if (now - lastTx >= 10) {
    lastTx = now;
    uint8_t payload[16] = {0xF0};
    if (esp_now_send(bcast, payload, sizeof(payload)) == ESP_OK) txOk++; else txErr++;
  }

  if (sweep && now - lastSweep >= 700) {
    lastSweep = now;
    servoUs = (servoUs >= 1500) ? 1000 : 2000;
    servo.writeMicroseconds(servoUs);
  }

  if (now - lastPrint >= 500) {
    lastPrint = now;
    float pinMv = readVbatPinMv();
    float vbat  = pinMv * DIV_RATIO / 1000.0f;
    if (vbat > 1.0f) {
      vbatMin = min(vbatMin, vbat);
      vbatMax = max(vbatMax, vbat);
    }
    Serial.printf("%5lu | %7.0f | %9.2f | %8.2f | %5.2f-%5.2f | %8d | %lu/%lu\n",
                  (unsigned long)(now / 1000), pinMv, vbat, vbat / CELLS,
                  vbatMin > 90 ? 0.0f : vbatMin, vbatMax, servoUs,
                  (unsigned long)txOk, (unsigned long)txErr);
    if (pinMv > 3200) Serial.println(F(">>> ATENCAO: pino IO36 acima de 3,2 V. Confira R1/R2!"));

    oledCheckPresence();
    oledDraw(vbat, pinMv);
  }
}

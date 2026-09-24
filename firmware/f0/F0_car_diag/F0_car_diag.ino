// F0_car_diag.ino — Diagnóstico elétrico do CARRO (fase F0)
//
// O que faz:
//   - Registra o motivo de cada boot em NVS e conta brownouts entre reinícios.
//   - Lê a tensão da bateria no IO36 (divisor 100k/22k) e mostra V total e V/célula.
//   - Mantém o servo (IO25) no centro ou em varredura, para testar pico de corrente.
//   - Liga o rádio e envia broadcast ESP-NOW a 100 Hz em potência máxima (picos de TX).
//   - Faz scan I2C (IO21/IO22) para achar o OLED.
//
// Segurança: os pinos dos ESCs (26, 27, 32, 33) ficam em LOW fixo, sem pulsos.
// ESC sem sinal não arma. Mesmo assim, teste com rodas fora do chão.
//
// Comandos pela Serial (115200): s = liga/desliga varredura do servo,
//                                c = servo no centro, z = zera contadores, ? = ajuda

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include <Wire.h>
#include <ESP32Servo.h>

constexpr int PIN_SERVO = 25;
constexpr int PIN_VBAT  = 36;
constexpr int PIN_SDA   = 21;
constexpr int PIN_SCL   = 22;
constexpr int ESC_PINS[] = {26, 27, 32, 33};

constexpr float DIV_RATIO = (100.0f + 22.0f) / 22.0f;  // 5,545
constexpr int   CELLS     = 4;

Servo servo;
Preferences prefs;

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

  Serial.println();
  Serial.println(F("=== F0 diagnostico - CARRO ==="));
  Serial.printf("Boot #%lu | motivo: %s\n", (unsigned long)boots, reasonStr(r));
  Serial.printf("Historico: %lu brownout(s), %lu crash/watchdog\n",
                (unsigned long)brown, (unsigned long)crashes);
  if (brown > 0) Serial.println(F(">>> ATENCAO: houve brownout. Revise alimentacao do ESP / C1."));
}

void i2cScan() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Serial.print(F("I2C scan (SDA 21 / SCL 22): "));
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("0x%02X ", addr);
      found++;
    }
  }
  if (found == 0) Serial.print(F("nada encontrado (OLED desligado ou fiacao)"));
  Serial.println();
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
  Serial.println(F("Comandos: s=varredura servo on/off | c=servo centro | z=zerar contadores | ?=ajuda"));
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 's': sweep = !sweep; Serial.printf("Varredura: %s\n", sweep ? "ON" : "OFF"); break;
      case 'c': sweep = false; servoUs = 1500; servo.writeMicroseconds(servoUs); Serial.println(F("Servo no centro")); break;
      case 'z': prefs.clear(); vbatMin = 99; vbatMax = 0; txOk = txErr = 0; Serial.println(F("Contadores zerados")); break;
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
  i2cScan();

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
  }
}

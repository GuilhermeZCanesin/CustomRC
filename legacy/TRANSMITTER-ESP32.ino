#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <esp_now.h>
#include <WiFi.h>
#include <EEPROM.h>

#define SIGNAL_TIMEOUT 1000

#define i2c_Address 0x3c
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET -1     //   QT-PY / XIAO

Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

bool showNoBattery = true;
unsigned long lastBlinkTimeBattery = 0;
const unsigned long blinkIntervalBattery = 500;  // milissegundos

unsigned long lastRecvTime = 0;

// define the number of bytes you want to access
#define EEPROM_SIZE 3
#define trimbut_1 5   // Trim button 1 / Pin 19
#define trimbut_2 18  // Trim button 2 / Pin 18
#define trimbut_3 19  // Trim button 1 / Pin 23
#define trimbut_4 23  // Trim button 2 / Pin 22

int throttleComp = EEPROM.read(0);  // Reading trim values from Eprom
int steeringComp = EEPROM.read(2);  // Reading trim values from Eprom

int throttle_offset = 0;  //throttle offset

uint8_t broadcastAddress[] = { 0xf8, 0xb3, 0xb7, 0x22, 0x1d, 0x94 };  //f8:b3:b7:22:1d:94

float incomingBatt;

String success;

typedef struct transmitter_struct {
  int throttle;
  int steering;
} transmitter_struct;

typedef struct receiver_struct {
  int battery;
} receiver_struct;

transmitter_struct transmissionInfo;
receiver_struct incomingInfo;

float filteredValueThrottle = 0;
float alphaThrottle = 0.1;
float filteredValueSteering = 0;
float alphaSteering = 0.1;

esp_now_peer_info_t peerInfo;

bool shouldTransmit = false;

void ResetData() {
  transmissionInfo.throttle = 90;
  transmissionInfo.steering = 0;
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == 0) {
    success = "Delivery Success :)";
  } else {
    success = "Delivery Fail :(";
  }
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingInfo, incomingData, sizeof(incomingInfo));
  lastRecvTime = millis();
  incomingBatt = incomingInfo.battery;
}

void setup() {
  // Initializing Serial Monitor
  Serial.begin(115200);

  display.begin(i2c_Address, true);
  display.display();
  delay(2000);
  display.clearDisplay();

  // initialize EEPROM with predefined size
  EEPROM.begin(EEPROM_SIZE);
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    displayErrorMessage("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  if (shouldTransmit) {
    esp_now_register_send_cb(OnDataSent);
  }

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    displayErrorMessage("Failed to add peer");
    return;
  }
  ResetData();

  pinMode(trimbut_1, INPUT_PULLUP);
  pinMode(trimbut_2, INPUT_PULLUP);
  pinMode(trimbut_3, INPUT_PULLUP);
  pinMode(trimbut_4, INPUT_PULLUP);
  throttleComp = EEPROM.read(0);
  steeringComp = EEPROM.read(2);

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop() {
  getReadings();

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&transmissionInfo, sizeof(transmissionInfo));

  int connectionSignal = 3;
  unsigned long now = millis();

  if (now - lastRecvTime > SIGNAL_TIMEOUT) {
    connectionSignal = 0;
  }

  // simule níveis de 0 a 4
  if (now - lastBlinkTimeBattery > blinkIntervalBattery) {
    showNoBattery = !showNoBattery;
    lastBlinkTimeBattery = now;
  }

  drawWiFiIcon(connectionSignal);
  drawBatteryIcon(incomingInfo.battery, showNoBattery);
  drawThrottleBar(transmissionInfo.throttle, throttleComp);
  drawSteeringBar(transmissionInfo.steering, steeringComp);
  display.display();
  delay(150);
}


void getReadings() {
  if (digitalRead(trimbut_2) == LOW and throttleComp < 20) {
    throttleComp = throttleComp + 1;
    EEPROM.write(0, throttleComp);
    EEPROM.commit();
    delay(130);
  }
  if (digitalRead(trimbut_1) == LOW and throttleComp > -20) {
    throttleComp = throttleComp - 1;
    EEPROM.write(0, throttleComp);
    EEPROM.commit();
    delay(130);
  }

  if (digitalRead(trimbut_3) == LOW and steeringComp < 20) {
    steeringComp = steeringComp + 1;
    EEPROM.write(2, steeringComp);
    EEPROM.commit();
    delay(130);
  }
  if (digitalRead(trimbut_4) == LOW and steeringComp > -20) {
    steeringComp = steeringComp - 1;
    EEPROM.write(2, steeringComp);
    EEPROM.commit();
    delay(130);
  }

  // Control Stick Calibration for channels
  int throttle = analogRead(35);
  int steering = analogRead(34);

  transmissionInfo.throttle = map(analogRead(35), 0, 4095, 0, 180) + throttleComp;
  transmissionInfo.steering = map(analogRead(34), 0, 4095, 0, 180) + steeringComp;
}

void displayErrorMessage(String message) {
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.println(message);
  display.display();
  delay(2000);
  display.clearDisplay();
}

void drawWiFiIcon(int level) {
  int x = 0;
  int y = 0;
  int spacing = 3;

  display.clearDisplay();
  if (level == 0) {
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.println("NO SIGNAL");
  } else {
    for (int i = 0; i < 4; i++) {
      int barHeight = (i + 1) * 2;  // 2, 4, 6, 8 pixels
      if (i < level) {
        display.fillRect(x + i * spacing, y + (10 - barHeight), 2, barHeight, SH110X_WHITE);
      } else {
        display.drawRect(x + i * spacing, y + (10 - barHeight), 2, barHeight, SH110X_WHITE);
      }
    }
  }
}

void drawBatteryIcon(int level, bool blink) {
  int x = SCREEN_WIDTH - 22;  // posição horizontal (canto superior direito)
  int y = 0;                  // posição vertical
  int width = 18;             // largura do corpo da bateria
  int height = 8;             // altura
  int terminalWidth = 2;      // largura do terminal da bateria

  bool isLow = (level <= 15);
  bool showOutline = !(isLow && blink);  // pisca o contorno quando fraco

  // Desenhar contorno da bateria (só se não estiver piscando)
  if (showOutline) {
    display.drawRect(x, y, width, height, SH110X_WHITE);                 // corpo
    display.fillRect(x + width, y + 2, terminalWidth, 4, SH110X_WHITE);  // terminal
  }

  // Preenchimento do nível da bateria
  int innerWidth = width - 2;  // espaço interno para preenchimento
  int fillWidth = map(level, 0, 100, 0, innerWidth);
  if (fillWidth > 0) {
    display.fillRect(x + 1, y + 2, fillWidth, height - 4, SH110X_WHITE);
  }

  // (Opcional) Mostrar porcentagem numérica ao lado do ícone

  display.setTextSize(1);
  display.setCursor(x - 25, y);
  display.printf("%d%%", level);
}

void drawThrottleBar(int16_t value, int16_t comp) {
  int barHeight = 50;
  int barWidth = 6;
  int halfHeight = barHeight / 2;
  int yCenter = 39;
  int xCenter = 3;
  char *label = "ACC";

  // Limita o valor de 0 a 180
  value = constrain(value, 0, 180);

  // Coordenadas principais
  int xBar = xCenter - (barWidth / 2);
  int yTop = yCenter - halfHeight;
  int yZero = yCenter;

  // Desenha contorno
  display.drawRect(xBar, yTop, barWidth, barHeight, SH110X_WHITE);

  // Calcular preenchimento
  if (value < 90) {
    // Negativo → para baixo
    int fillLength = map(90 - value, 0, 90, 0, halfHeight);
    display.fillRect(xBar + 1, yZero + 1, barWidth - 2, fillLength, SH110X_WHITE);
  } else if (value > 90) {
    // Positivo → para cima
    int fillLength = map(value - 90, 0, 90, 0, halfHeight);
    display.fillRect(xBar + 1, yZero - fillLength, barWidth - 2, fillLength, SH110X_WHITE);
  } else {
    // valor == 90 → barra zerada, opcional: traço no meio
    display.drawLine(xBar, yZero, xBar + barWidth - 1, yZero, SH110X_WHITE);
  }

  // Texto (label) à direita da barra
  int xLabel = xBar + barWidth + 4;
  int yLabel = yCenter - 4;
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(xLabel, yLabel);
  display.print(label);

  // Valor numérico abaixo do label
  char buf[8];
  sprintf(buf, "%d", value);
  display.setCursor(xLabel, yLabel + 10);
  display.print(buf);

  // Valor numérico acima do label
  char buf2[8];
  sprintf(buf2, "%d", comp);
  display.setCursor(xLabel, yLabel - 10);
  display.print(buf2);
}

void drawSteeringBar(int16_t value, int16_t comp) {
  int barWidth = 80;
  int barHeight = 6;
  int halfWidth = barWidth / 2;
  int xCenter = 50;
  int yBottom = 64;
  char *label = "STR";

  // Limita valor entre 0 e 180
  value = constrain(value, 0, 180);

  // Coordenadas
  int xStart = xCenter - halfWidth;
  int yBar = yBottom - barHeight;

  // Desenha contorno da barra
  display.drawRect(xStart, yBar, barWidth, barHeight, SH110X_WHITE);

  // Preenchimento
  if (value < 90) {
    int fillLen = map(90 - value, 0, 90, 0, halfWidth);
    display.fillRect(xCenter - fillLen, yBar + 1, fillLen, barHeight - 2, SH110X_WHITE);
  } else if (value > 90) {
    int fillLen = map(value - 90, 0, 90, 0, halfWidth);
    display.fillRect(xCenter + 1, yBar + 1, fillLen, barHeight - 2, SH110X_WHITE);
  } else {
    // Zero visual, opcional linha central
    display.drawLine(xCenter, yBar, xCenter, yBar + barHeight - 1, SH110X_WHITE);
  }

  // Label acima da barra
  int16_t labelX = xCenter - (strlen(label) * 6) / 2;
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(labelX, yBar - 10);
  display.print(label);

  // Valor ao lado da barra
  int xText = xStart + barWidth + 4;
  display.setCursor(xText, yBar - 1);
  display.print(comp);
}

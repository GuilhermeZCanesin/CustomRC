#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

#define SIGNAL_TIMEOUT 1000  // This is signal timeout in milli seconds. We will reset the data if no signal

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C
#define OLED_RESET -1  //   QT-PY / XIAO

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

bool showNoBattery = true;
unsigned long lastBlinkTimeBattery = 0;
const unsigned long blinkIntervalBattery = 500;  // milissegundos

unsigned long lastRecvTime = 0;

// REPLACE WITH THE MAC Address of your receiver
uint8_t broadcastAddress[] = { 0x3c, 0x8a, 0x1f, 0x63, 0x5e, 0x6c };  //3c:8a:1f:63:5e:6c

Servo steering;  //steering
Servo engbl;     //engine back left
Servo engbr;     //engine back right
Servo engfl;     //engine front left
Servo engfr;     //engine front right

// Variable to store if sending data was successful
String success;

//Structure example to send data
//Must match the receiver structure
typedef struct transmitter_struct {
  int throttle;
  int steering;
} transmitter_struct;

typedef struct receiver_struct {
  int battery;
} receiver_struct;


transmitter_struct incomingInfo;
receiver_struct trasmissionInfo;

esp_now_peer_info_t peerInfo;

void setInputDefaultValues() {
  incomingInfo.throttle = 90;
  incomingInfo.steering = 90;
}

void mapAndWriteValues() {
  engbl.write(incomingInfo.throttle);     // back left
  engbr.write(incomingInfo.throttle);     // back right
  engfl.write(incomingInfo.throttle);     // front left
  engfr.write(incomingInfo.throttle);     // front right
  steering.write(incomingInfo.steering);  //servo
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == 0) {
    success = "Delivery Success :)";
  } else {
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len == 0) {
    return;
  }
  memcpy(&incomingInfo, incomingData, sizeof(incomingInfo));
  mapAndWriteValues();
  lastRecvTime = millis();
}

void setUpPinModes() {
  steering.attach(25);  //25 - STEERING
  engbr.attach(26, 1000, 2000);     //26 - BACK RIGHT ENGINE
  engbl.attach(27, 1000, 2000);     //27 - BACK LEFT ENGINE
  engfr.attach(33, 1000, 2000);     //27 - FRONT RIGHT ENGINE
  engfl.attach(32, 1000, 2000);     //27 - BACK LEFT ENGINE
}

void startUpEscs() {
  engfl.write(90);  // front left
  engfr.write(90);  // front right
  engbl.write(0);  // back left
  engbr.write(0);  // back right
  displayConfig("start");
  delay(2250);
  engbl.write(180);  // back left
  engbr.write(180);  // back right
  engfl.write(180);  // front left
  engfr.write(180);  // front right
  displayConfig("mid");
  delay(2250);
  engbl.write(90);  // back left
  engbr.write(90);  // back right
  displayConfig("end");
  delay(2250);
}

void displayConfig(String stage) {
  display.clearDisplay();
  display.setTextSize(1);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  if (stage == "start") {
    display.setCursor(0, 22);
    display.println(F("PREPARING..."));
  }
  if (stage == "mid") {
    display.setCursor(30, 11);
    display.println(F("SETTING..."));
  }
  if (stage == "end") {
    display.setCursor(80, 0);
    display.println(F("READY!"));
  }
  display.display();
}

void setup() {
  Serial.begin(115200);

  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.display();
  delay(2000);
  display.clearDisplay();

  setUpPinModes();
  startUpEscs();

  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    displayErrorMessage("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop() {

  //Check Signal lost.
  unsigned long now = millis();
  if (now - lastRecvTime > SIGNAL_TIMEOUT) {
    setInputDefaultValues();
    mapAndWriteValues();
  }

  getReadings();
  displayinfo();
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&trasmissionInfo, sizeof(trasmissionInfo));
}

void getReadings() {
  // get battery status
  trasmissionInfo.battery = 100;
}

void displayinfo() {
  display.clearDisplay();
  display.setTextSize(1);               // Normal 1:1 pixel scale 
  display.setTextColor(SSD1306_WHITE);  // Draw white text

  display.setCursor(0, 0);
  display.println(incomingInfo.throttle);

  display.setCursor(0, 11);
  display.println(incomingInfo.steering);

  display.display();
}

void displayErrorMessage(String message) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(message);
  display.display();
  delay(2000);
  display.clearDisplay();
}

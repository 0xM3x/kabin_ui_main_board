// Main board code using hardcoded screen MAC and fixed ESP-NOW logic

#include <WiFi.h>
#include <MQTT.h>
#include <ArduinoJson.h>
#include <time.h>
#include <Preferences.h>
#include <DHT.h>
#include <esp_now.h>
#include "esp_wifi.h"

// ------------------------------ CONFIG ------------------------------
const uint32_t VollyBinID = 301001;
const char* chrVollyBinID = "301001";

const char* mqttDefaultBrokerIP = "185.198.72.57";
const char* mqttBrokerClientID  = chrVollyBinID;
const char* mqttBrokerUserName  = "vollydevice";
const char* mqttBrokerPassword  = "volley34mqtt";
char mqttSubscribeTopic[25];

const char ssid[] = "VOLLY-TECH";
const char pass[] = "prof1907";

uint8_t screenMac[] = {0x2C, 0xBC, 0xBB, 0xA2, 0x09, 0xEC};

// ----------------------------- PINS ---------------------------------
#define MQ2PIN 34
#define analogCurentPin 32
#define pinDoorLOCK 18
#define pinDrawerLOCK 5
#define pinServoUP 22
#define pinServoDOWN 23
#define pinDoorFBack 26
#define pinDrawerFBack 27
#define pinRX2 16
#define pinTX2 17
#define pinDoorButton 21
#define pinRX3 25
#define pinTX3 33
#define ledR  15
#define ledG  2
#define ledB  4
#define echoPin 25
#define trigPin 33
#define DHTPIN 19
#define DHTTYPE DHT22

// ---------------------------- OBJECTS -------------------------------
WiFiClient wifiNetwork;
MQTTClient mqttClient;
Preferences preferencesObj;
DHT dht(DHTPIN, DHTTYPE);

// ----------------------------- VARIABLES ----------------------------
unsigned long sec = 0;
bool ledON = false;
float curVal = 0;
long duration;
int distance;
float temperature = 0.0;
float humidity = 0.0;
int doluluk = 0;
String resMQTTBrokerIP = "";

const float sensitivity = 0.185;
const float VREF = 3.3;
const int ADC_RESOLUTION = 4095;

// ---------------------------- ESP-NOW -------------------------------
typedef struct struct_message {
  float gaz;
  float nem;
  int doluluk;
  //bool show_press_screen;
} struct_message;

struct_message outgoingData;

void onSend(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("📤 Data sent: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// ---------------------------- SETUP ----------------------------
void setup() {
  Serial.begin(115200);
  dht.begin();
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(pinDoorButton, INPUT_PULLUP);
  pinMode(MQ2PIN, INPUT);
  pinMode(pinDoorLOCK, OUTPUT);
  pinMode(pinDrawerLOCK, OUTPUT);
  pinMode(pinDoorFBack, INPUT);
  pinMode(pinDrawerFBack, INPUT);
  pinMode(pinServoUP, OUTPUT);
  pinMode(pinServoDOWN, OUTPUT);
  digitalWrite(pinDoorLOCK, HIGH);
  digitalWrite(pinDrawerLOCK, HIGH);
  digitalWrite(pinServoUP, HIGH);
  digitalWrite(pinServoDOWN, HIGH);

  ledcAttach(ledR, 12000, 8);
  ledcAttach(ledG, 12000, 8);
  ledcAttach(ledB, 12000, 8);
  ledSetColors(128, 128, 128);
  ledON = true;

  preferencesObj.begin("masterVariables", false);
  resMQTTBrokerIP = preferencesObj.getString("MQTTBrokerIP", mqttDefaultBrokerIP);
  preferencesObj.putString("MQTTBrokerIP", resMQTTBrokerIP);
  preferencesObj.end();

  WiFi.begin(ssid, pass);
  mqttClient.begin(resMQTTBrokerIP.c_str(), wifiNetwork);
  mqttClient.onMessage(mqttMessageReceived);

  Serial2.begin(115200, SERIAL_8N1, pinRX2, pinTX2);
  Serial.println("⏳ Waiting for Wi-Fi...");

  // ------------------- ESP-NOW INIT -------------------
  WiFi.mode(WIFI_STA);

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
  wifi_second_chan_t second;
  uint8_t primary;
  esp_wifi_get_channel(&primary, &second);
  Serial.printf("📡 Channel: %d\n", primary);
  esp_wifi_set_promiscuous(false);

  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW Init Failed");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, screenMac, 6);
  peerInfo.channel = 0;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(screenMac)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("❌ Failed to add CrowPanel peer!");
    } else {
      Serial.println("✅ CrowPanel peer added!");
    }
  } else {
    Serial.println("🧠 CrowPanel already added as peer");
  }

  esp_now_register_send_cb(onSend);
  Serial.println("✅ Main board setup complete!");
}

// ---------------------------- LOOP ----------------------------
void loop() {
  if (!mqttClient.connected()) {
    connectWiFiandMQTTBroker();
  }

  mqttClient.loop();
  delay(10);

  ReadDistance();

  if (millis() - sec > 1000) {
    curVal = measureCurrent();
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();
    doluluk = distance;

    outgoingData.gaz = analogRead(MQ2PIN);
    outgoingData.nem = humidity;
    outgoingData.doluluk = doluluk;

    esp_err_t result = esp_now_send(screenMac, (uint8_t *)&outgoingData, sizeof(outgoingData));

    Serial.print("ESP-NOW send result: ");
    Serial.println(result);

    sec = millis();
  }
}

// ------------------------- UTILITIES -------------------------
void ReadDistance() {
  digitalWrite(trigPin, LOW); delay(20);
  digitalWrite(trigPin, HIGH); delay(100);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  Serial.print("📏 Distance: "); Serial.print(distance); Serial.println(" cm");
}

float measureCurrent() {
  int adcValue = analogRead(analogCurentPin);
  float voltage = (adcValue / float(ADC_RESOLUTION)) * VREF;
  return (voltage - (VREF / 2)) / sensitivity;
}

void ledSetColors(uint32_t R, uint32_t G, uint32_t B) {
  ledcWrite(ledR, R);
  ledcWrite(ledG, G);
  ledcWrite(ledB, B);
}

void connectWiFiandMQTTBroker() {
  Serial.print("🔌 Connecting to Wi-Fi...");
  while (WiFi.status() != WL_CONNECTED) {
    ledSetColors(128, 0, 0);
    delay(1000);
  }
  Serial.println("✅ Wi-Fi connected");

  Serial.print("🔌 Connecting to MQTT...");
  while (!mqttClient.connect(mqttBrokerClientID, mqttBrokerUserName, mqttBrokerPassword)) {
    ledSetColors(0, 0, 128);
    delay(1000);
  }

  snprintf(mqttSubscribeTopic, sizeof(mqttSubscribeTopic), "/CONTROL/%s/#", chrVollyBinID);
  mqttClient.subscribe(mqttSubscribeTopic);

  Serial.println("✅ MQTT connected");
  ledSetColors(0, 128, 0);
}

void mqttMessageReceived(String &topic, String &payload) {
  Serial.println("📩 MQTT: " + topic + " - " + payload);
  String topicParts[4];
  splitString(topic, '/', topicParts, 4);

  if (topicParts[3] == "BIN-RELEASE") {
    doDoorReleaseCommand();
    doDrawerReleaseCommand();
  } else if (topicParts[3] == "BIN-SERVOUP") {
    digitalWrite(pinServoUP, LOW); 
    delay(1000);
    digitalWrite(pinServoUP, HIGH);
  
  } else if (topicParts[3] == "BIN-SERVODOWN") {
    digitalWrite(pinServoDOWN, LOW);
    delay(1000); 
    digitalWrite(pinServoDOWN, HIGH);
  }
}

void doDoorReleaseCommand() {
  digitalWrite(pinDoorLOCK, LOW); delay(500); digitalWrite(pinDoorLOCK, HIGH);
  delay(500);
  digitalWrite(pinDoorLOCK, LOW); delay(500); digitalWrite(pinDoorLOCK, HIGH);
}

void doDrawerReleaseCommand() {
  digitalWrite(pinDrawerLOCK, LOW); delay(500); digitalWrite(pinDrawerLOCK, HIGH);
  delay(500);
  digitalWrite(pinDrawerLOCK, LOW); delay(500); digitalWrite(pinDrawerLOCK, HIGH);
}

void splitString(String str, char separator, String* resultArray, int maxParts) {
  int currentIndex = 0;
  int startIndex = 0;
  int endIndex = 0;

  while (currentIndex < maxParts && (endIndex = str.indexOf(separator, startIndex)) >= 0) {
    resultArray[currentIndex++] = str.substring(startIndex, endIndex);
    startIndex = endIndex + 1;
  }

  if (currentIndex < maxParts) {
    resultArray[currentIndex] = str.substring(startIndex);
  }
}

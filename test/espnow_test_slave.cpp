// espnow_test_slave.cpp
// Bare-bones ESP-NOW sender, now sending real ultrasonic sensor readings
// instead of a counter. Still no WiFi join, no UDP - just ESP-NOW on a
// fixed channel, for isolated testing.
//
// Sensor wiring matches the rest of the project:
//   Sensor 1: TRIG=32, ECHO=35
//   Sensor 2: TRIG=12, ECHO=14
//
// Flash espnow_test_master.cpp to the OTHER board first, read the MAC
// it prints, and paste it into masterAddress[] below before flashing
// this file.
//
// Copy/rename this file into src/ before building (only one file with
// setup()/loop() can live in src/ at a time).

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>

// TODO: set this to the MASTER board's MAC address, printed by
// espnow_test_master.cpp on boot.
uint8_t masterAddress[] = {0x90, 0x15, 0x06, 0x73, 0x70, 0x40};

const uint8_t FIXED_CHANNEL = 1; // must match the master's value

const int SENSOR_1_TRIG_PIN = 32;
const int SENSOR_1_ECHO_PIN = 35;
const int SENSOR_2_TRIG_PIN = 12;
const int SENSOR_2_ECHO_PIN = 14;
const int LED_PIN = 2;

struct SensorPacket {
  float sensor1Cm;
  float sensor2Cm;
};

float readSensorDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  return duration <= 0 ? -1.0f : duration * 0.0343f / 2.0f;
}

void printDistanceValue(float distance) {
  if (distance < 0) {
    Serial.print("NaN");
  } else {
    Serial.print(distance, 2);
    Serial.print(" cm");
  }
}

void onDataSent(
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  const wifi_tx_info_t *,
#else
  const uint8_t *,
#endif
  esp_now_send_status_t status
) {
  digitalWrite(LED_PIN, status == ESP_NOW_SEND_SUCCESS ? HIGH : LOW);
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "send OK" : "send FAILED");
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(SENSOR_1_TRIG_PIN, OUTPUT);
  pinMode(SENSOR_1_ECHO_PIN, INPUT);
  pinMode(SENSOR_2_TRIG_PIN, OUTPUT);
  pinMode(SENSOR_2_ECHO_PIN, INPUT);
  digitalWrite(SENSOR_1_TRIG_PIN, LOW);
  digitalWrite(SENSOR_2_TRIG_PIN, LOW);

  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);
  delay(100);
  WiFi.disconnect();
  WiFi.setSleep(false); // keep the radio awake to hear the master's ACK
  esp_wifi_set_channel(FIXED_CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.print("Slave MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Requested channel: ");
  Serial.println(FIXED_CHANNEL);
  Serial.print("Actual channel: ");
  Serial.println(WiFi.channel());

  if (esp_now_init() != ESP_OK) {
    Serial.println("esp_now_init FAILED");
    while (true) delay(1000);
  }
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, masterAddress, 6);
  peerInfo.channel = FIXED_CHANNEL;
  peerInfo.encrypt = false;

  esp_err_t addResult = esp_now_add_peer(&peerInfo);
  Serial.print("esp_now_add_peer result: ");
  Serial.println(addResult == ESP_OK ? "OK" : String(addResult));

  Serial.println("Slave ready");
}

void loop() {
  SensorPacket packet;
  packet.sensor1Cm = readSensorDistance(SENSOR_1_TRIG_PIN, SENSOR_1_ECHO_PIN);
  delay(60); // let sensor 1's pulse die down before firing sensor 2
  packet.sensor2Cm = readSensorDistance(SENSOR_2_TRIG_PIN, SENSOR_2_ECHO_PIN);

  Serial.print("Sending -> sensor1: ");
  printDistanceValue(packet.sensor1Cm);
  Serial.print(" | sensor2: ");
  printDistanceValue(packet.sensor2Cm);
  Serial.println();

  esp_now_send(masterAddress, reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

  delay(200);
}
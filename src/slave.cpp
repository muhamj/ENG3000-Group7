// slave.cpp
//
// SLAVE board. Responsibilities:
//   1. Reads its own two ultrasonic sensors.
//   2. Sends them to the master over ESP-NOW.
//
// Does NOT join any Wi-Fi network and does NOT run a web server - it
// only needs its radio, fixed to the same channel the master's AP uses,
// to reach the master directly.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>

// Physical MAC address of the master board (burned into its hardware,
// doesn't change with firmware). Update if you swap which physical
// board is the master.
const uint8_t MASTER_MAC[] = {0x90, 0x15, 0x06, 0x73, 0x70, 0x40};

// Must match FIXED_CHANNEL in master.cpp exactly. Since the master hosts
// its own Wi-Fi network instead of joining a phone hotspot, this value
// is fully under your control and won't drift between sessions.
const uint8_t ESPNOW_CHANNEL = 1;

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

void onDataSent(
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  const wifi_tx_info_t *,
#else
  const uint8_t *,
#endif
  esp_now_send_status_t status
) {
  digitalWrite(LED_PIN, status == ESP_NOW_SEND_SUCCESS ? HIGH : LOW);
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("ESP-NOW send OK");
  } else {
    uint8_t primaryChannel;
    wifi_second_chan_t secondChannel;
    esp_wifi_get_channel(&primaryChannel, &secondChannel);
    Serial.print("ESP-NOW send FAILED (radio currently on channel ");
    Serial.print(primaryChannel);
    Serial.println(")");
  }
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
  delay(100); // let the radio finish initialising before reading the MAC
  WiFi.disconnect(); // make sure we are not associated to anything
  WiFi.setSleep(false); // keep the radio awake to hear the master's ACK
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.println("ESP32 dual sensor - SLAVE (ESP-NOW only, no Wi-Fi join)");
  Serial.print("Slave MAC address: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Fixed ESP-NOW channel: ");
  Serial.println(ESPNOW_CHANNEL);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, MASTER_MAC, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;
  esp_err_t addPeerResult = esp_now_add_peer(&peerInfo);
  if (addPeerResult != ESP_OK) {
    Serial.print("Failed to add master peer, err=");
    Serial.println(addPeerResult);
    return;
  }

  uint8_t primaryChannel;
  wifi_second_chan_t secondChannel;
  esp_wifi_get_channel(&primaryChannel, &secondChannel);
  Serial.print("Radio actually on channel: ");
  Serial.println(primaryChannel);

  Serial.println("Slave ready; sending sensor data over ESP-NOW");
}

void loop() {
  SensorPacket packet;
  packet.sensor1Cm = readSensorDistance(SENSOR_1_TRIG_PIN, SENSOR_1_ECHO_PIN);
  delay(60);
  packet.sensor2Cm = readSensorDistance(SENSOR_2_TRIG_PIN, SENSOR_2_ECHO_PIN);
  esp_now_send(MASTER_MAC, reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
  delay(100);
}
// espnow_test_master.cpp
// Bare-bones ESP-NOW receiver, now printing real ultrasonic sensor
// readings sent by the slave board. Still no WiFi join, no UDP - just
// ESP-NOW on a fixed channel, for isolated testing.
//
// Copy/rename this file into src/ before building (only one file with
// setup()/loop() can live in src/ at a time).

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>

const uint8_t FIXED_CHANNEL = 1; // must match the slave's value

struct SensorPacket {
  float sensor1Cm;
  float sensor2Cm;
};

void printDistanceValue(float distance) {
  if (distance < 0) {
    Serial.print("NaN");
  } else {
    Serial.print(distance, 2);
    Serial.print(" cm");
  }
}

void handleMessage(const uint8_t *incomingData, int length) {
  if (length != sizeof(SensorPacket)) {
    Serial.print("Got a packet with the wrong size: ");
    Serial.println(length);
    return;
  }
  SensorPacket packet;
  memcpy(&packet, incomingData, sizeof(packet));
  Serial.print("Received -> sensor1: ");
  printDistanceValue(packet.sensor1Cm);
  Serial.print(" | sensor2: ");
  printDistanceValue(packet.sensor2Cm);
  Serial.println();
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataReceived(const esp_now_recv_info_t *, const uint8_t *incomingData, int length) {
  handleMessage(incomingData, length);
}
#else
void onDataReceived(const uint8_t *, const uint8_t *incomingData, int length) {
  handleMessage(incomingData, length);
}
#endif

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);
  delay(100); // let the radio finish initialising before reading the MAC
  WiFi.disconnect();
  WiFi.setSleep(false); // keep the radio awake to hear ESP-NOW packets/ACKs
  esp_wifi_set_channel(FIXED_CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.print("Master MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Requested channel: ");
  Serial.println(FIXED_CHANNEL);
  Serial.print("Actual channel: ");
  Serial.println(WiFi.channel());
  Serial.println("^ copy the MAC above into espnow_test_slave.cpp's masterAddress[]");

  if (esp_now_init() != ESP_OK) {
    Serial.println("esp_now_init FAILED");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onDataReceived);

  Serial.println("Master ready, waiting for sensor packets...");
}

void loop() {
  delay(100);
}
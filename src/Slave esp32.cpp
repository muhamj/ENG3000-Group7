// SLAVE ESP32 (Box B)
// Reads its 2 local ultrasonic sensors and sends the distances to the
// master ESP32 over ESP-NOW. The master then forwards everything to the
// PC over Bluetooth.
//
// Flash this onto the ESP32 that is NOT connected to the PC.
// Wiring matches src/maintest.cpp: TRIG/ECHO pairs below.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// ---- Sensor pins (Box B / slave) ----
const int SENSOR_1_TRIG_PIN = 32;
const int SENSOR_1_ECHO_PIN = 35;
const int SENSOR_2_TRIG_PIN = 12;
const int SENSOR_2_ECHO_PIN = 14;
const int LED_PIN = 2;

// Fill this in with the MASTER's MAC address.
// Get it by flashing the master firmware first and reading its printed
// MAC address from the Serial Monitor, then paste it here.
uint8_t masterAddress[] = { 0x00, 0x70, 0x07, 0x7C, 0x72, 0xA4 };

// Packet sent to the master. Keep this struct identical on both boards.
struct SensorPacket {
  float sensor1Cm;
  float sensor2Cm;
};

SensorPacket outgoing;

float readSensorDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);

  if (duration <= 0) {
    return -1.0f;
  }

  return duration * 0.0343f / 2.0f;
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
#else
void onDataSent(const uint8_t *macAddress, esp_now_send_status_t status) {
#endif
  digitalWrite(LED_PIN, status == ESP_NOW_SEND_SUCCESS ? HIGH : LOW);
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

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  delay(1000);

  Serial.println("Slave ESP32 starting (Box B)");
  Serial.print("Slave MAC address: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, masterAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add master as peer");
    return;
  }

  Serial.println("Slave ready, sending to master...");
}

void loop() {
  float sensor1Distance = readSensorDistance(SENSOR_1_TRIG_PIN, SENSOR_1_ECHO_PIN);

  // Prevent the first ultrasonic pulse from interfering with sensor 2.
  delay(60);

  float sensor2Distance = readSensorDistance(SENSOR_2_TRIG_PIN, SENSOR_2_ECHO_PIN);

  outgoing.sensor1Cm = sensor1Distance;
  outgoing.sensor2Cm = sensor2Distance;

  esp_now_send(masterAddress, reinterpret_cast<uint8_t *>(&outgoing), sizeof(outgoing));

  // Keep this loop fast so the master gets fresh data often.
  // The master polls its own sensors at roughly the same rate, so match it.
  delay(80);
}
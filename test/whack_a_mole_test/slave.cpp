// slave-3-sensors.cpp – hardened (same pins, same packet, safer reads)

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>

#define SLAVE_DEBUG_PRINT 1

const uint8_t ESPNOW_BROADCAST_MAC[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
const uint8_t ESPNOW_CHANNEL = 1;

const int SENSOR_1_TRIG_PIN = 32, SENSOR_1_ECHO_PIN = 35;
const int SENSOR_2_TRIG_PIN = 12, SENSOR_2_ECHO_PIN = 14;
const int SENSOR_3_TRIG_PIN = 22, SENSOR_3_ECHO_PIN = 23;

const int LED_PIN = 2, BUZZER_PIN = 18;
const unsigned int BUZZER_FREQUENCY_HZ = 2200;
const unsigned long SENSOR_SETTLE_DELAY_MS = 60;
const uint8_t SLAVE_SAMPLE_REQUEST = 2;

struct SensorPacket {
  float sensor1Cm, sensor2Cm, sensor3Cm;
};

volatile bool buzzerRequested = false;
volatile bool sampleRequested = false;
bool buzzerOutputEnabled = false;

float readSensorDistance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(5);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  noInterrupts();
  long us = pulseIn(echo, HIGH, 30000);
  interrupts();

  return us <= 0 ? -1.0f : us * 0.0343f / 2.0f;
}

void onDataSent(
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  const wifi_tx_info_t *,
#else
  const uint8_t *,
#endif
  esp_now_send_status_t status)
{
  digitalWrite(LED_PIN, status == ESP_NOW_SEND_SUCCESS ? HIGH : LOW);
  if (status != ESP_NOW_SEND_SUCCESS) {
    uint8_t ch; wifi_second_chan_t sec;
    esp_wifi_get_channel(&ch, &sec);
    Serial.printf("ESP-NOW send FAILED (ch %d)\n", ch);
  }
}

void handleMasterCommand(const uint8_t *data, int len) {
  if (len != 1) return;
  if (data[0] <= 1) buzzerRequested = (data[0] == 1);
  else if (data[0] == SLAVE_SAMPLE_REQUEST) sampleRequested = true;
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataReceived(const esp_now_recv_info_t *, const uint8_t *data, int len) {
#else
void onDataReceived(const uint8_t *, const uint8_t *data, int len) {
#endif
  handleMasterCommand(data, len);
}

void updateBuzzerOutput() {
  bool req = buzzerRequested;
  if (req == buzzerOutputEnabled) return;
  if (req) tone(BUZZER_PIN, BUZZER_FREQUENCY_HZ);
  else noTone(BUZZER_PIN);
  buzzerOutputEnabled = req;
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(SENSOR_1_TRIG_PIN, OUTPUT); pinMode(SENSOR_1_ECHO_PIN, INPUT);
  pinMode(SENSOR_2_TRIG_PIN, OUTPUT); pinMode(SENSOR_2_ECHO_PIN, INPUT);
  pinMode(SENSOR_3_TRIG_PIN, OUTPUT); pinMode(SENSOR_3_ECHO_PIN, INPUT);
  noTone(BUZZER_PIN);
  digitalWrite(SENSOR_1_TRIG_PIN, LOW);
  digitalWrite(SENSOR_2_TRIG_PIN, LOW);
  digitalWrite(SENSOR_3_TRIG_PIN, LOW);

  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);
  delay(100);
  WiFi.disconnect();
  WiFi.setSleep(false);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.println("ESP32 triple sensor – SLAVE");
  Serial.print("MAC: "); Serial.println(WiFi.macAddress());
  Serial.printf("Channel %d, packet %d bytes\n", ESPNOW_CHANNEL, sizeof(SensorPacket));

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataReceived);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, ESPNOW_BROADCAST_MAC, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  Serial.println("Slave ready");
}

void loop() {
  updateBuzzerOutput();
  if (!sampleRequested) { delay(1); return; }
  sampleRequested = false;

  SensorPacket pkt;
  pkt.sensor1Cm = readSensorDistance(SENSOR_1_TRIG_PIN, SENSOR_1_ECHO_PIN);
  delay(SENSOR_SETTLE_DELAY_MS);
  pkt.sensor2Cm = readSensorDistance(SENSOR_2_TRIG_PIN, SENSOR_2_ECHO_PIN);
  delay(SENSOR_SETTLE_DELAY_MS);
  pkt.sensor3Cm = readSensorDistance(SENSOR_3_TRIG_PIN, SENSOR_3_ECHO_PIN);

#if SLAVE_DEBUG_PRINT
  Serial.printf("Slave S4=%.1f S5=%.1f S6=%.1f\n",
                pkt.sensor1Cm, pkt.sensor2Cm, pkt.sensor3Cm);
#endif

  esp_now_send(ESPNOW_BROADCAST_MAC, (uint8_t*)&pkt, sizeof(pkt));
}
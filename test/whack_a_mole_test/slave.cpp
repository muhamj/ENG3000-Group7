// SLAVE board: reads two ultrasonic sensors and sends them to the master.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>

const uint8_t ESPNOW_BROADCAST_MAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const uint8_t ESPNOW_CHANNEL = 1;

const int SENSOR_1_TRIG_PIN = 32;
const int SENSOR_1_ECHO_PIN = 35;
const int SENSOR_2_TRIG_PIN = 12;
const int SENSOR_2_ECHO_PIN = 14;
const int LED_PIN = 2;
const int BUZZER_PIN = 18;
const unsigned int BUZZER_FREQUENCY_HZ = 2200;
// Matched to master's 90ms settle delay to avoid ultrasonic crosstalk
// between this board's own two sensors.
const unsigned long SENSOR_SETTLE_DELAY_MS = 90;
const uint8_t SLAVE_SAMPLE_REQUEST = 2;

struct SensorPacket {
  float sensor1Cm;
  float sensor2Cm;
};

volatile bool buzzerRequested = false;
volatile bool sampleRequested = false;
bool buzzerOutputEnabled = false;

float readSensorDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 16000);
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

void handleMasterCommand(const uint8_t *incomingData, int length) {
  if (length != 1) return;
  if (incomingData[0] <= 1) buzzerRequested = incomingData[0] == 1;
  else if (incomingData[0] == SLAVE_SAMPLE_REQUEST) sampleRequested = true;
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataReceived(const esp_now_recv_info_t *, const uint8_t *incomingData, int length) {
#else
void onDataReceived(const uint8_t *, const uint8_t *incomingData, int length) {
#endif
  handleMasterCommand(incomingData, length);
}

void updateBuzzerOutput() {
  bool requested = buzzerRequested;
  if (requested == buzzerOutputEnabled) return;
  if (requested) {
    tone(BUZZER_PIN, BUZZER_FREQUENCY_HZ);
  } else {
    noTone(BUZZER_PIN);
  }
  buzzerOutputEnabled = requested;
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(SENSOR_1_TRIG_PIN, OUTPUT);
  pinMode(SENSOR_1_ECHO_PIN, INPUT);
  pinMode(SENSOR_2_TRIG_PIN, OUTPUT);
  pinMode(SENSOR_2_ECHO_PIN, INPUT);
  noTone(BUZZER_PIN);
  digitalWrite(SENSOR_1_TRIG_PIN, LOW);
  digitalWrite(SENSOR_2_TRIG_PIN, LOW);
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);
  delay(100);
  WiFi.disconnect();
  WiFi.setSleep(false);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.println("ESP32 dual sensor - SLAVE (ESP-NOW only)");
  Serial.print("Slave MAC address: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Fixed ESP-NOW channel: ");
  Serial.println(ESPNOW_CHANNEL);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataReceived);

  esp_now_peer_info_t broadcastPeer = {};
  memcpy(broadcastPeer.peer_addr, ESPNOW_BROADCAST_MAC, 6);
  broadcastPeer.channel = ESPNOW_CHANNEL;
  broadcastPeer.encrypt = false;
  broadcastPeer.ifidx = WIFI_IF_STA;
  esp_err_t addPeerResult = esp_now_add_peer(&broadcastPeer);
  if (addPeerResult != ESP_OK) {
    Serial.print("Failed to add master peer, err=");
    Serial.println(addPeerResult);
    return;
  }

  Serial.println("Slave ready; sending sensor data and listening for buzzer state");
}

void loop() {
  updateBuzzerOutput();
  if (!sampleRequested) {
    delay(1);
    return;
  }
  sampleRequested = false;

  SensorPacket packet;
  packet.sensor1Cm = readSensorDistance(SENSOR_1_TRIG_PIN, SENSOR_1_ECHO_PIN);
  delay(SENSOR_SETTLE_DELAY_MS);
  packet.sensor2Cm = readSensorDistance(SENSOR_2_TRIG_PIN, SENSOR_2_ECHO_PIN);
  esp_now_send(ESPNOW_BROADCAST_MAC, reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
}
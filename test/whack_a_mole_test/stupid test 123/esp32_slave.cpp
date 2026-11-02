#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// Pins — keep the same as your current slave if the hardware is wired this way
const int SENSOR_TRIG_PINS[3] = {12, 32, 22};
const int SENSOR_ECHO_PINS[3] = {14, 35, 23};

// Must match master exactly (12 bytes)
struct SensorPacket {
  float sensor1Cm;
  float sensor2Cm;
  float sensor3Cm;
};

// Master MAC from your log
uint8_t masterAddress[] = {0x90, 0x15, 0x06, 0x73, 0x70, 0x40};

const uint8_t SLAVE_SAMPLE_REQUEST = 2;

float readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  noInterrupts();
  long duration = pulseIn(echoPin, HIGH, 16000);
  interrupts();

  if (duration <= 0) return -1.0f;
  return duration * 0.0343f / 2.0f;
}

void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == 1 && data[0] == SLAVE_SAMPLE_REQUEST) {
    SensorPacket pkt;
    pkt.sensor1Cm = readDistance(SENSOR_TRIG_PINS[0], SENSOR_ECHO_PINS[0]);
    delay(30);
    pkt.sensor2Cm = readDistance(SENSOR_TRIG_PINS[1], SENSOR_ECHO_PINS[1]);
    delay(30);
    pkt.sensor3Cm = readDistance(SENSOR_TRIG_PINS[2], SENSOR_ECHO_PINS[2]);

    esp_now_send(masterAddress, (uint8_t *)&pkt, sizeof(pkt));
  }
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataRecvV3(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  onDataRecv(info->src_addr, data, len);
}
#endif

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Ultrasonic SLAVE (request/response) ===");

  for (int i = 0; i < 3; i++) {
    pinMode(SENSOR_TRIG_PINS[i], OUTPUT);
    pinMode(SENSOR_ECHO_PINS[i], INPUT);
    digitalWrite(SENSOR_TRIG_PINS[i], LOW);
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_now_register_recv_cb(onDataRecvV3);
#else
  esp_now_register_recv_cb(onDataRecv);
#endif

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, masterAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add Master peer");
    while (true) delay(1000);
  }

  Serial.print("Slave MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("Waiting for sample requests from Master...");
}

void loop() {
  // Nothing to do — we only reply when Master asks
  delay(10);
}
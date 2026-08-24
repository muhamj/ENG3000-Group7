// MASTER ESP32 (Box A)
// - Reads its own 2 local ultrasonic sensors
// - Receives 2 more sensor readings from the slave ESP32 via ESP-NOW
// - Sends all 4 distances to the PC over Bluetooth Classic (SPP), one
//   line per update, in the same style the existing Python code expects.
//
// This is the ESP32 that gets paired with the PC over Bluetooth.
// It replaces src/maintest.cpp for the sensor-fusion MVP.
//
// Output line format (PC parses this):
//   distances: s1a=<cm|nan> s1b=<cm|nan> s2a=<cm|nan> s2b=<cm|nan>
//
// s1a/s1b = master's own 2 sensors (Box A)
// s2a/s2b = slave's 2 sensors (Box B), relayed over ESP-NOW

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_arduino_version.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled. Run "idf.py menuconfig" or check board config.
#endif

BluetoothSerial SerialBT;

// ---- Sensor pins (Box A / master) ----
const int SENSOR_1_TRIG_PIN = 32;
const int SENSOR_1_ECHO_PIN = 35;
const int SENSOR_2_TRIG_PIN = 12;
const int SENSOR_2_ECHO_PIN = 14;
const int LED_PIN = 2;

// Must match the slave firmware's struct exactly.
struct SensorPacket {
  float sensor1Cm;
  float sensor2Cm;
};

// Most recent values received from the slave over ESP-NOW.
volatile float slaveSensor1Cm = -1.0f;
volatile float slaveSensor2Cm = -1.0f;
volatile unsigned long lastSlavePacketMs = 0;

// If we haven't heard from the slave in this long, treat it as offline
// and report NaN for its sensors rather than a stale reading.
const unsigned long SLAVE_TIMEOUT_MS = 500;

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

void printDistanceValue(Print &out, float distance) {
  if (distance < 0) {
    out.print("nan");
  } else {
    out.print(distance, 2);
  }
}

void handleSlavePacket(const uint8_t *incomingData, int length) {
  if (length != sizeof(SensorPacket)) {
    return;
  }

  SensorPacket packet;
  memcpy(&packet, incomingData, sizeof(packet));

  slaveSensor1Cm = packet.sensor1Cm;
  slaveSensor2Cm = packet.sensor2Cm;
  lastSlavePacketMs = millis();
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataReceived(const esp_now_recv_info_t *info, const uint8_t *incomingData, int length) {
  handleSlavePacket(incomingData, length);
}
#else
void onDataReceived(const uint8_t *macAddress, const uint8_t *incomingData, int length) {
  handleSlavePacket(incomingData, length);
}
#endif

void setup() {
  pinMode(LED_PIN, OUTPUT);

  pinMode(SENSOR_1_TRIG_PIN, OUTPUT);
  pinMode(SENSOR_1_ECHO_PIN, INPUT);
  pinMode(SENSOR_2_TRIG_PIN, OUTPUT);
  pinMode(SENSOR_2_ECHO_PIN, INPUT);

  digitalWrite(SENSOR_1_TRIG_PIN, LOW);
  digitalWrite(SENSOR_2_TRIG_PIN, LOW);

  Serial.begin(115200);
  delay(500);

  // Bluetooth name the PC will see when pairing/scanning.
  SerialBT.begin("WhackAMole-Master");

  Serial.println("Master ESP32 starting (Box A)");
  Serial.println("Bluetooth device name: WhackAMole-Master");

  // WiFi must be in STA mode for ESP-NOW, but we never actually
  // join a network, so this doesn't conflict with Bluetooth.
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(200);

  Serial.print("Master MAC address (put this in the slave firmware): ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(onDataReceived);

  Serial.println("Master ready. Waiting for slave packets + reading local sensors...");
}

void loop() {
  digitalWrite(LED_PIN, HIGH);

  float local1 = readSensorDistance(SENSOR_1_TRIG_PIN, SENSOR_1_ECHO_PIN);
  delay(60); // avoid the two local ultrasonic pulses interfering with each other
  float local2 = readSensorDistance(SENSOR_2_TRIG_PIN, SENSOR_2_ECHO_PIN);

  digitalWrite(LED_PIN, LOW);

  float remote1 = slaveSensor1Cm;
  float remote2 = slaveSensor2Cm;
  bool slaveOnline = (millis() - lastSlavePacketMs) < SLAVE_TIMEOUT_MS;

  if (!slaveOnline) {
    remote1 = -1.0f;
    remote2 = -1.0f;
  }

  // Build one line and send it to both Serial (for debugging over USB)
  // and Bluetooth (for the PC game).
  for (int i = 0; i < 2; i++) {
    Print &out = (i == 0) ? static_cast<Print &>(Serial) : static_cast<Print &>(SerialBT);

    out.print("distances: s1a=");
    printDistanceValue(out, local1);
    out.print(" s1b=");
    printDistanceValue(out, local2);
    out.print(" s2a=");
    printDistanceValue(out, remote1);
    out.print(" s2b=");
    printDistanceValue(out, remote2);
    out.println();
  }

  delay(80);
}
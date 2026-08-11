#include <Arduino.h>

const int SENSOR_TRIG_PIN = 32;
const int SENSOR_ECHO_PIN = 35;
const int LED_PIN = 2;

float readSingleSensorDistance() {
  digitalWrite(SENSOR_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(SENSOR_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SENSOR_TRIG_PIN, LOW);

  long duration = pulseIn(SENSOR_ECHO_PIN, HIGH, 30000);
  if (duration <= 0) {
    return -1.0;
  }

  return duration * 0.0343f / 2.0f;
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(SENSOR_TRIG_PIN, OUTPUT);
  pinMode(SENSOR_ECHO_PIN, INPUT);
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32 sensor test: TRIG=12, ECHO=13");
}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  float distance = readSingleSensorDistance();
  if (distance < 0) {
    Serial.println("distance: NaN");
  } else {
    Serial.print("distance: ");
    Serial.print(distance, 2);
    Serial.println(" cm");
  }
  digitalWrite(LED_PIN, LOW);
  delay(1000);
}

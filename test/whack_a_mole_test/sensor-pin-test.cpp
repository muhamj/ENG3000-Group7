#include <Arduino.h>

const int SENSOR_1_TRIG_PIN = 12;
const int SENSOR_1_ECHO_PIN = 14;
const int SENSOR_2_TRIG_PIN = 32;
const int SENSOR_2_ECHO_PIN = 35;
const int SENSOR_3_TRIG_PIN = 22;  // unknown - set these before uploading
const int SENSOR_3_ECHO_PIN = 23;  // unknown - set these before uploading

const unsigned long SETTLE_DELAY_MS = 60;
const unsigned long LOOP_DELAY_MS = 1000;

struct Sensor {
  const char *name;
  int trigPin;
  int echoPin;
};

Sensor sensors[] = {
  {"Sensor 1", SENSOR_1_TRIG_PIN, SENSOR_1_ECHO_PIN},
  {"Sensor 2", SENSOR_2_TRIG_PIN, SENSOR_2_ECHO_PIN},
  {"Sensor 3", SENSOR_3_TRIG_PIN, SENSOR_3_ECHO_PIN},
};

float readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 16000);
  return duration <= 0 ? -1.0f : duration * 0.0343f / 2.0f;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== Ultrasonic pin test ===");
  for (Sensor &s : sensors) {
    if (s.trigPin < 0 || s.echoPin < 0) {
      Serial.printf("%s: pins not set (skip)\n", s.name);
      continue;
    }
    pinMode(s.trigPin, OUTPUT);
    pinMode(s.echoPin, INPUT);
    digitalWrite(s.trigPin, LOW);
  }
}

void loop() {
  for (Sensor &s : sensors) {
    if (s.trigPin < 0 || s.echoPin < 0) continue;
    float cm = readDistance(s.trigPin, s.echoPin);
    if (cm < 0) {
      Serial.printf("%s (trig=%d, echo=%d): NO ECHO - check wiring\n",
                    s.name, s.trigPin, s.echoPin);
    } else if (cm > 400) {
      Serial.printf("%s (trig=%d, echo=%d): %.1f cm - out of range\n",
                    s.name, s.trigPin, s.echoPin, cm);
    } else {
      Serial.printf("%s (trig=%d, echo=%d): %.1f cm - looks OK\n",
                    s.name, s.trigPin, s.echoPin, cm);
    }
  }
  Serial.println();
  delay(LOOP_DELAY_MS);
}

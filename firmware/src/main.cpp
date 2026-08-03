#include <Arduino.h>
#include <WiFi.h>

const char* ssid = "Austin's Phone";
const char* password = "123456789";

void blinkOnce(int delayMs) {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(delayMs);
  digitalWrite(LED_BUILTIN, LOW);
  delay(delayMs);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  delay(1000);

  Serial.println("test");

  for (int i = 0; i < 3; ++i) {
    blinkOnce(150);
  }

  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  for (int i = 0; i < 20; ++i) {
    if (WiFi.status() == WL_CONNECTED) {
      break;
    }
    blinkOnce(200);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Wi-Fi connected");
  } else {
    Serial.println("Wi-Fi connection failed");
  }
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    blinkOnce(1000);
  } else {
    blinkOnce(250);
  }
}

#include <WiFi.h>

const char* ssid = "Austin's Phone";
const char* password = "123456789";

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Print MAC address
  Serial.print("ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());

  Serial.println();
  Serial.println("Connecting to Wi-Fi...");

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("Wi-Fi connected!");

  // Print IP address
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Print MAC again after connection
  Serial.print("MAC address: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
}
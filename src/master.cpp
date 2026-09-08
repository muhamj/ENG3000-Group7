#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>

// The master joins the hotspot and forwards all four readings to the PC.
const char *WIFI_SSID = "Austin's Phone";
const char *WIFI_PASSWORD = "123456789";
const char *LAPTOP_IP = "172.20.10.3";
const unsigned int UDP_PORT = 4210;

// Confirmed working MAC of the slave board (physical MAC, doesn't change
// with firmware). Update if you ever swap which physical board is slave.
const uint8_t SLAVE_MAC[] = {0x00, 0x70, 0x07, 0x7C, 0x72, 0xA4};

// Fallback channel ONLY used if Wi-Fi never connects. If Wi-Fi does
// connect, the hotspot's real channel (whatever it turns out to be)
// always wins - see setup(), where the peer is registered using
// WiFi.channel() instead of this constant once connected.
const uint8_t ESPNOW_FALLBACK_CHANNEL = 1;

const int SENSOR_1_TRIG_PIN = 32;
const int SENSOR_1_ECHO_PIN = 35;
const int SENSOR_2_TRIG_PIN = 12;
const int SENSOR_2_ECHO_PIN = 14;
const int LED_PIN = 2;

WiFiUDP udp;

struct SensorPacket {
  float sensor1Cm;
  float sensor2Cm;
};

volatile float slaveSensor1Cm = -1.0f;
volatile float slaveSensor2Cm = -1.0f;
volatile unsigned long lastSlavePacketMs = 0;


const char *wifiStatusName(wl_status_t status) {
  switch (status) {
    case WL_CONNECTED:
      return "connected";
    case WL_NO_SSID_AVAIL:
      return "hotspot not found";
    case WL_CONNECT_FAILED:
      return "connection failed (check password)";
    case WL_CONNECTION_LOST:
      return "connection lost";
    case WL_DISCONNECTED:
      return "disconnected";
    default:
      return "connection timed out";
  }
}


bool connectToWiFi() {
  Serial.print("Connecting to Wi-Fi SSID: ");
  Serial.println(WIFI_SSID);
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  const unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 15000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Wi-Fi failed: ");
    Serial.println(wifiStatusName(WiFi.status()));
    return false;
  }

  Serial.print("Wi-Fi IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Wi-Fi channel: ");
  Serial.println(WiFi.channel());
  Serial.print("UDP destination: ");
  Serial.print(LAPTOP_IP);
  Serial.print(":");
  Serial.println(UDP_PORT);
  udp.begin(UDP_PORT);
  return true;
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
void onDataReceived(const esp_now_recv_info_t *, const uint8_t *incomingData, int length) {
#else
void onDataReceived(const uint8_t *, const uint8_t *incomingData, int length) {
#endif
  handleSlavePacket(incomingData, length);
}


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


float getClosestValidDistance(float sensor1, float sensor2) {
  if (sensor1 < 0 && sensor2 < 0) {
    return -1.0f;
  }

  if (sensor1 < 0) {
    return sensor2;
  }

  if (sensor2 < 0) {
    return sensor1;
  }

  return sensor1 < sensor2 ? sensor1 : sensor2;
}


void printDistanceValue(float distance) {
  if (distance < 0) {
    Serial.print("NaN");
  } else {
    Serial.print(distance, 2);
    Serial.print(" cm");
  }
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
  delay(100); // let the radio finish initialising before reading the MAC
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false); // keep the radio awake so ESP-NOW packets from the
                         // slave aren't missed while modem-sleep is active

  // Safe default in case Wi-Fi never connects at all. If it does connect,
  // WiFi.begin() below will silently move the radio to the hotspot's real
  // channel regardless of this - that's expected and handled further down.
  esp_wifi_set_channel(ESPNOW_FALLBACK_CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.println("ESP32 dual sensor - MASTER");
  Serial.println("Sensor 1: TRIG=32, ECHO=35");
  Serial.println("Sensor 2: TRIG=12, ECHO=14");

  Serial.print("Master MAC Address: ");
  Serial.println(WiFi.macAddress());

  bool wifiConnected = connectToWiFi();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(onDataReceived);

  // Use the channel we actually ended up on, not a hardcoded guess. If
  // Wi-Fi connected, that's the hotspot's real channel (WiFi.channel());
  // otherwise fall back to the constant above.
  uint8_t effectiveChannel = wifiConnected ? (uint8_t)WiFi.channel() : ESPNOW_FALLBACK_CHANNEL;

  esp_now_peer_info_t slavePeer = {};
  memcpy(slavePeer.peer_addr, SLAVE_MAC, 6);
  slavePeer.channel = effectiveChannel;
  slavePeer.encrypt = false;
  slavePeer.ifidx = WIFI_IF_STA;
  if (esp_now_add_peer(&slavePeer) != ESP_OK) {
    Serial.println("Failed to add slave as ESP-NOW peer");
  } else {
    Serial.println("Slave registered as ESP-NOW peer");
  }

  uint8_t primaryChannel;
  wifi_second_chan_t secondChannel;
  esp_wifi_get_channel(&primaryChannel, &secondChannel);
  Serial.print("Master radio actually on channel: ");
  Serial.println(primaryChannel);
  Serial.println("*** If this changes, update ESPNOW_CHANNEL in slave.cpp to match and reflash it. ***");

  Serial.println("ESP-NOW receiver ready");
}


void loop() {
  static unsigned long lastWiFiRetry = 0;
  if (WiFi.status() != WL_CONNECTED && millis() - lastWiFiRetry >= 10000) {
    lastWiFiRetry = millis();
    connectToWiFi();
  }

  digitalWrite(LED_PIN, HIGH);

  float sensor1Distance = readSensorDistance(
    SENSOR_1_TRIG_PIN,
    SENSOR_1_ECHO_PIN
  );

  // Prevent the first ultrasonic pulse from interfering with sensor 2.
  delay(60);

  float sensor2Distance = readSensorDistance(
    SENSOR_2_TRIG_PIN,
    SENSOR_2_ECHO_PIN
  );

  float remote1 = slaveSensor1Cm;
  float remote2 = slaveSensor2Cm;
  if (millis() - lastSlavePacketMs >= 1000) {
    remote1 = -1.0f;
    remote2 = -1.0f;
  }

  float closestDistance = getClosestValidDistance(
    getClosestValidDistance(sensor1Distance, sensor2Distance),
    getClosestValidDistance(remote1, remote2)
  );

  // Packet format matches what the game (game_wifi.py) and the dashboard
  // (sensor_monitor.py / website.html) both expect: sensor1/sensor2 are
  // the master's own two sensors, sensor3/sensor4 are the slave's two
  // (relayed here over ESP-NOW).
  char packet[180];
  snprintf(
    packet,
    sizeof(packet),
    "{\"mac\":\"%s\",\"sensor1\":%.2f,\"sensor2\":%.2f,\"sensor3\":%.2f,\"sensor4\":%.2f,\"closest\":%.2f}",
    WiFi.macAddress().c_str(),
    sensor1Distance,
    sensor2Distance,
    remote1,
    remote2,
    closestDistance
  );

  if (WiFi.status() == WL_CONNECTED) {
    udp.beginPacket(LAPTOP_IP, UDP_PORT);
    udp.write(reinterpret_cast<const uint8_t *>(packet), strlen(packet));
    if (udp.endPacket() == 0) {
      Serial.println("Wi-Fi UDP send failed");
    }
  } else {
    Serial.println("Wi-Fi disconnected; packet not sent");
  }

  // Keep USB serial output available for diagnostics while testing.
  Serial.print("distance: ");
  printDistanceValue(closestDistance);

  Serial.print(" | sensor 1: ");
  printDistanceValue(sensor1Distance);

  Serial.print(" | sensor 2: ");
  printDistanceValue(sensor2Distance);

  Serial.print(" | sensor 3 (slave 1): ");
  printDistanceValue(remote1);

  Serial.print(" | sensor 4 (slave 2): ");
  printDistanceValue(remote2);

  Serial.print(" | ms since last slave packet: ");
  Serial.print(millis() - lastSlavePacketMs);

  Serial.println();

  digitalWrite(LED_PIN, LOW);
  delay(100);
}
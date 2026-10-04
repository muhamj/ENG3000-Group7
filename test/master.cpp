// master.cpp
//
// MASTER board. Responsibilities:
//   1. Hosts its own Wi-Fi network (no phone hotspot, no router needed) -
//      connect your laptop's Wi-Fi directly to it.
//   2. Reads its own two ultrasonic sensors.
//   3. Receives the slave board's two sensor readings over ESP-NOW.
//   4. Broadcasts all four readings as UDP JSON, for the pygame game
//      (game_wifi.py) and the terminal/HTML dashboard (sensor_monitor.py
//      / website.html) to pick up.
//   5. Serves its own built-in web page at http://192.168.4.1/ showing
//      all four readings live, with no laptop script required.
//
// Flash espnow_slave-matching slave.cpp to the OTHER board, with its
// MASTER_MAC set to this board's SoftAP MAC below. The SoftAP MAC is the
// destination because ESP-NOW is registered on WIFI_IF_AP here.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>

// ---------------------------------------------------------------------
// Wi-Fi network the master hosts. Connect your laptop to this directly.
// ---------------------------------------------------------------------
const char *AP_SSID = "WhackAMole-Sensors";
const char *AP_PASSWORD = "molemole1"; // must be 8+ characters

// We host the AP ourselves, so WE decide the channel - no guessing what
// a hotspot happens to use, and no channel drift after connecting. This
// single number is the source of truth: slave.cpp's ESPNOW_CHANNEL must
// match it exactly.
const uint8_t FIXED_CHANNEL = 1;

// ---------------------------------------------------------------------
// UDP broadcast (used by the Python game/dashboard on the laptop)
// ---------------------------------------------------------------------
const unsigned int UDP_PORT = 4210;
// SoftAP default subnet is 192.168.4.0/24 (gateway 192.168.4.1).
// Broadcasting to .255 means no laptop IP address needs to be hardcoded
// or kept up to date - whoever is listening on UDP_PORT gets it.
IPAddress BROADCAST_IP(192, 168, 4, 255);

// ---------------------------------------------------------------------
// ESP-NOW: receiving the slave's two sensor readings
// ---------------------------------------------------------------------
// Physical MAC address of the slave board (burned into its hardware,
// doesn't change with firmware). Update if you swap which physical
// board is the slave.
const uint8_t SLAVE_MAC[] = {0x00, 0x70, 0x07, 0x7C, 0x72, 0xA4};

struct SensorPacket {
  float sensor1Cm;
  float sensor2Cm;
};

volatile float slaveSensor1Cm = -1.0f;
volatile float slaveSensor2Cm = -1.0f;
volatile unsigned long lastSlavePacketMs = 0;

// ---------------------------------------------------------------------
// Master's own two ultrasonic sensors
// ---------------------------------------------------------------------
const int SENSOR_1_TRIG_PIN = 32;
const int SENSOR_1_ECHO_PIN = 35;
const int SENSOR_2_TRIG_PIN = 12;
const int SENSOR_2_ECHO_PIN = 14;
const int LED_PIN = 2;
const unsigned long SENSOR_SETTLE_DELAY_MS = 60;
const float MAX_TRACKING_DISTANCE_CM = 200.0f;
const float POSITION_SMOOTHING_ALPHA = 0.35f;

WiFiUDP udp;
WebServer webServer(80);

// =======================================================================
// Sensor reading
// =======================================================================

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

bool isValidDistance(float distance) {
  return isfinite(distance) && distance > 0.0f && distance <= MAX_TRACKING_DISTANCE_CM;
}

float getClosestValidDistance(float a, float b) {
  if (!isValidDistance(a) && !isValidDistance(b)) return -1.0f;
  if (!isValidDistance(a)) return b;
  if (!isValidDistance(b)) return a;
  return a < b ? a : b;
}

void printDistanceValue(float distance) {
  if (distance < 0) {
    Serial.print("NaN");
  } else {
    Serial.print(distance, 2);
    Serial.print(" cm");
  }
}

struct PlayerPosition {
  bool valid;
  float x;
  float y;
};

float closestValid(float a, float b) {
  return getClosestValidDistance(a, b);
}

PlayerPosition calculatePlayerPosition(
  float sensor1,
  float sensor2,
  float sensor3,
  float sensor4
) {
  static bool hasSmoothedPosition = false;
  static float smoothedX = 0.5f;
  static float smoothedY = 0.5f;

  float leftTotal = 0.0f;
  float rightTotal = 0.0f;
  int leftCount = 0;
  int rightCount = 0;

  if (isValidDistance(sensor1)) { leftTotal += sensor1; leftCount++; }
  if (isValidDistance(sensor3)) { leftTotal += sensor3; leftCount++; }
  if (isValidDistance(sensor2)) { rightTotal += sensor2; rightCount++; }
  if (isValidDistance(sensor4)) { rightTotal += sensor4; rightCount++; }

  if (leftCount == 0 && rightCount == 0) {
    hasSmoothedPosition = false;
    return {false, -1.0f, -1.0f};
  }

  float leftDistance = leftCount == 0 ? MAX_TRACKING_DISTANCE_CM : leftTotal / leftCount;
  float rightDistance = rightCount == 0 ? MAX_TRACKING_DISTANCE_CM : rightTotal / rightCount;
  float leftCloseness = MAX_TRACKING_DISTANCE_CM - leftDistance;
  float rightCloseness = MAX_TRACKING_DISTANCE_CM - rightDistance;
  float totalCloseness = leftCloseness + rightCloseness;
  float x = totalCloseness <= 0 ? 0.5f : rightCloseness / totalCloseness;

  float nearestDistance = closestValid(leftDistance, rightDistance);
  float y = 1.0f - nearestDistance / MAX_TRACKING_DISTANCE_CM;
  if (!hasSmoothedPosition) {
    smoothedX = x;
    smoothedY = y;
    hasSmoothedPosition = true;
  } else {
    smoothedX += POSITION_SMOOTHING_ALPHA * (x - smoothedX);
    smoothedY += POSITION_SMOOTHING_ALPHA * (y - smoothedY);
  }
  return {true, smoothedX, smoothedY};
}

// =======================================================================
// ESP-NOW receive callback (from the slave board)
// =======================================================================

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

// =======================================================================
// Shared JSON builder - used for both the UDP broadcast and the web
// dashboard's /data endpoint, so the two never drift out of sync.
// =======================================================================

void buildSensorJson(char *buffer, size_t bufferSize) {
  float sensor1 = readSensorDistance(SENSOR_1_TRIG_PIN, SENSOR_1_ECHO_PIN);
  delay(SENSOR_SETTLE_DELAY_MS); // let sensor 1's echo die down before sensor 2
  float sensor2 = readSensorDistance(SENSOR_2_TRIG_PIN, SENSOR_2_ECHO_PIN);

  float remote1 = slaveSensor1Cm;
  float remote2 = slaveSensor2Cm;
  if (millis() - lastSlavePacketMs >= 1000) {
    remote1 = -1.0f;
    remote2 = -1.0f;
  }

  float closest = getClosestValidDistance(
    getClosestValidDistance(sensor1, sensor2),
    getClosestValidDistance(remote1, remote2)
  );
  PlayerPosition position = calculatePlayerPosition(sensor1, sensor2, remote1, remote2);

  snprintf(
    buffer,
    bufferSize,
    "{\"mac\":\"%s\",\"sensor1\":%.2f,\"sensor2\":%.2f,\"sensor3\":%.2f,\"sensor4\":%.2f,\"closest\":%.2f,\"x\":%.3f,\"y\":%.3f}",
    WiFi.macAddress().c_str(),
    sensor1,
    sensor2,
    remote1,
    remote2,
    closest,
    position.x,
    position.y
  );

  Serial.print("distance: ");
  printDistanceValue(closest);
  Serial.print(" | sensor 1: ");
  printDistanceValue(sensor1);
  Serial.print(" | sensor 2: ");
  printDistanceValue(sensor2);
  Serial.print(" | sensor 3 (slave 1): ");
  printDistanceValue(remote1);
  Serial.print(" | sensor 4 (slave 2): ");
  printDistanceValue(remote2);
  Serial.print(" | ms since last slave packet: ");
  Serial.print(millis() - lastSlavePacketMs);
  Serial.print(" | x: ");
  if (position.valid) Serial.print(position.x, 3); else Serial.print("NaN");
  Serial.print(" | y: ");
  if (position.valid) Serial.println(position.y, 3); else Serial.println("NaN");
}

// =======================================================================
// Built-in web dashboard
// =======================================================================

const char DASHBOARD_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Sensor Dashboard</title>
<style>
  body {
    font-family: Consolas, "Courier New", monospace;
    background: #111;
    color: #eee;
    margin: 0;
    padding: 24px;
  }
  h1 { font-size: 18px; margin: 0 0 16px 0; }
  .grid {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 14px;
    max-width: 620px;
  }
  .box {
    background: #1a1a1a;
    border: 1px solid #333;
    border-radius: 10px;
    padding: 18px;
    text-align: center;
  }
  .box-label {
    font-size: 13px;
    color: #888;
    text-transform: uppercase;
    letter-spacing: 0.05em;
    margin-bottom: 8px;
  }
  .box-value {
    font-size: 30px;
    font-weight: bold;
  }
  .status {
    margin-top: 16px;
    color: #888;
    font-size: 12px;
  }
  .position-map {
    margin-top: 18px;
    width: min(100%, 620px);
    background: #1a1a1a;
    border: 1px solid #333;
    border-radius: 10px;
    padding: 12px;
  }
  canvas { display: block; width: 100%; height: auto; background: #101820; }
</style>
</head>
<body>
  <h1>ESP32 Sensor Dashboard</h1>
  <div class="grid">
    <div class="box"><div class="box-label">Sensor 1</div><div class="box-value" id="s1">--</div></div>
    <div class="box"><div class="box-label">Sensor 2</div><div class="box-value" id="s2">--</div></div>
    <div class="box"><div class="box-label">Sensor 3</div><div class="box-value" id="s3">--</div></div>
    <div class="box"><div class="box-label">Sensor 4</div><div class="box-value" id="s4">--</div></div>
    <div class="box"><div class="box-label">X coordinate</div><div class="box-value" id="x">--</div></div>
    <div class="box"><div class="box-label">Y coordinate</div><div class="box-value" id="y">--</div></div>
  </div>
  <div class="position-map"><canvas id="positionMap" width="600" height="360"></canvas></div>
  <div class="status" id="status">Loading...</div>
  <script>
    function fmt(v) {
      return (v === null || v < 0) ? "--" : v.toFixed(1) + " cm";
    }
    function fmtCoordinate(v) {
      return (v === null || v < 0) ? "--" : v.toFixed(3);
    }
    function drawPosition(x, y) {
      const canvas = document.getElementById("positionMap");
      const ctx = canvas.getContext("2d");
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.strokeStyle = "#425466";
      ctx.lineWidth = 2;
      for (let i = 1; i < 3; i++) {
        ctx.beginPath(); ctx.moveTo(canvas.width * i / 3, 0);
        ctx.lineTo(canvas.width * i / 3, canvas.height); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(0, canvas.height * i / 3);
        ctx.lineTo(canvas.width, canvas.height * i / 3); ctx.stroke();
      }
      if (x === null || y === null || x < 0 || y < 0) return;
      ctx.fillStyle = "#ffcc33";
      ctx.beginPath();
      ctx.arc(x * canvas.width, y * canvas.height, 12, 0, Math.PI * 2);
      ctx.fill();
    }
    async function poll() {
      try {
        const res = await fetch("/data", { cache: "no-store" });
        const d = await res.json();
        document.getElementById("s1").textContent = fmt(d.sensor1);
        document.getElementById("s2").textContent = fmt(d.sensor2);
        document.getElementById("s3").textContent = fmt(d.sensor3);
        document.getElementById("s4").textContent = fmt(d.sensor4);
        document.getElementById("x").textContent = fmtCoordinate(d.x);
        document.getElementById("y").textContent = fmtCoordinate(d.y);
        drawPosition(d.x, d.y);
        document.getElementById("status").textContent =
          "Updated " + new Date().toLocaleTimeString();
      } catch (e) {
        document.getElementById("status").textContent = "Connection lost";
      }
    }
    poll();
    setInterval(poll, 500);
  </script>
</body>
</html>
)HTML";

void handleRoot() {
  webServer.send(200, "text/html", DASHBOARD_HTML);
}

void handleData() {
  char json[180];
  buildSensorJson(json, sizeof(json));
  webServer.send(200, "application/json", json);
}

// =======================================================================
// Setup / loop
// =======================================================================

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(SENSOR_1_TRIG_PIN, OUTPUT);
  pinMode(SENSOR_1_ECHO_PIN, INPUT);
  pinMode(SENSOR_2_TRIG_PIN, OUTPUT);
  pinMode(SENSOR_2_ECHO_PIN, INPUT);
  digitalWrite(SENSOR_1_TRIG_PIN, LOW);
  digitalWrite(SENSOR_2_TRIG_PIN, LOW);

  Serial.begin(115200);
  delay(1000);

  // AP_STA mode: hosting our own softAP fixes the channel; the STA side
  // stays available so ESP-NOW can run alongside it.
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false); // keep the radio awake so ESP-NOW packets from
                         // the slave aren't missed during modem-sleep
  WiFi.softAP(AP_SSID, AP_PASSWORD, FIXED_CHANNEL);

  Serial.println("ESP32 dual sensor - MASTER (hosting its own Wi-Fi network)");
  Serial.println("Sensor 1: TRIG=32, ECHO=35");
  Serial.println("Sensor 2: TRIG=12, ECHO=14");
  Serial.print("Master MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Master SoftAP MAC (copy this into slave.cpp): ");
  Serial.println(WiFi.softAPmacAddress());
  Serial.print("Wi-Fi network name: ");
  Serial.println(AP_SSID);
  Serial.print("Wi-Fi password: ");
  Serial.println(AP_PASSWORD);
  Serial.print("Fixed channel: ");
  Serial.println(FIXED_CHANNEL);
  Serial.print("Dashboard: http://");
  Serial.print(WiFi.softAPIP());
  Serial.println("/");
  Serial.println("Connect your laptop's Wi-Fi to the network above.");

  udp.begin(UDP_PORT);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(onDataReceived);

  esp_now_peer_info_t slavePeer = {};
  memcpy(slavePeer.peer_addr, SLAVE_MAC, 6);
  slavePeer.channel = FIXED_CHANNEL;
  slavePeer.encrypt = false;
  slavePeer.ifidx = WIFI_IF_AP;
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

  webServer.on("/", handleRoot);
  webServer.on("/data", handleData);
  webServer.begin();
  Serial.println("Web dashboard started");

  Serial.println("ESP-NOW receiver ready");
}

void loop() {
  webServer.handleClient();

  digitalWrite(LED_PIN, HIGH);

  char packet[180];
  buildSensorJson(packet, sizeof(packet));

  udp.beginPacket(BROADCAST_IP, UDP_PORT);
  udp.write(reinterpret_cast<const uint8_t *>(packet), strlen(packet));
  if (udp.endPacket() == 0) {
    Serial.println("UDP broadcast send failed");
  }

  digitalWrite(LED_PIN, LOW);
  delay(100);
}
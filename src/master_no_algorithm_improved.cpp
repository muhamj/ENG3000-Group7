// Position-stable master firmware variant.
// Copy this file over src/master_no_algorithm_improved.cpp when ready to use it.
// Keep this file outside src/ until then; src/ must contain only one setup()/loop().

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>

const char *AP_SSID = "WhackAMole-Sensors";
const char *AP_PASSWORD = "molemole1";
const uint8_t FIXED_CHANNEL = 1;
const unsigned int UDP_PORT = 4210;
IPAddress BROADCAST_IP(192, 168, 4, 255);

const uint8_t SLAVE_MAC[] = {0x00, 0x70, 0x07, 0x7C, 0x72, 0xA4};

const unsigned long SLAVE_REPLY_TIMEOUT_MS = 150;

struct SampleRequest {
  uint8_t marker;
};

struct SensorPacket {
  float sensor1Cm;
  float sensor2Cm;
};

volatile float slaveSensor1Cm = -1.0f;
volatile float slaveSensor2Cm = -1.0f;
volatile bool slaveReplyReady = false;

const int SENSOR_1_TRIG_PIN = 32;
const int SENSOR_1_ECHO_PIN = 35;
const int SENSOR_2_TRIG_PIN = 12;
const int SENSOR_2_ECHO_PIN = 14;
const int LED_PIN = 2;
const unsigned long SENSOR_SETTLE_DELAY_MS = 60;
const float MAX_DISTANCE_CM = 350.0f;
const float POSITION_SMOOTHING_ALPHA = 0.35f;
const float BOX_BASELINE_M = 1.5f;
const float DEAD_ZONE_DEPTH_M = 0.6f;
const float PLAY_AREA_DEPTH_M = 1.4f;

// Ignore the less reliable outer 10% on each side, then map the remaining
// center region across the complete dashboard grid.
const float PLAY_AREA_X_MARGIN_FRACTION = 0.1f;
const float PLAY_AREA_DEPTH_MARGIN_FRACTION = 0.1f;

WiFiUDP udp;
WebServer webServer(80);

struct PlayerPosition {
  bool valid;
  float x;
  float y;
};

float readSensorDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration <= 0) return -1.0f;
  return duration * 0.0343f / 2.0f;
}

bool isValidDistance(float distance) {
  return isfinite(distance) && distance > 0.0f && distance <= MAX_DISTANCE_CM;
}

float nearestValidDepth(float sensor1, float sensor2, float sensor3, float sensor4) {
  float validDistances[4];
  int count = 0;
  float readings[] = {sensor1, sensor2, sensor3, sensor4};

  for (float reading : readings) {
    if (isValidDistance(reading)) validDistances[count++] = reading;
  }
  if (count == 0) return -1.0f;

  float nearest = validDistances[0];
  for (int i = 1; i < count; i++) {
    if (validDistances[i] < nearest) nearest = validDistances[i];
  }
  return nearest;
}

float pickBoxRange(float sensorA, float sensorB) {
  bool validA = isValidDistance(sensorA);
  bool validB = isValidDistance(sensorB);
  if (!validA && !validB) return -1.0f;
  if (!validA) return sensorB;
  if (!validB) return sensorA;
  return sensorA < sensorB ? sensorA : sensorB;
}

PlayerPosition calculatePlayerPosition(
  float sensor1,
  float sensor2,
  float sensor3,
  float sensor4
) {
  static bool hasPosition = false;
  static float smoothedX = 0.5f;
  static float smoothedY = 0.5f;

  // Hold the last coordinate whenever any sensor is blank or invalid.
  if (!isValidDistance(sensor1) || !isValidDistance(sensor2) ||
      !isValidDistance(sensor3) || !isValidDistance(sensor4)) {
    if (hasPosition) return {true, smoothedX, smoothedY};
    return {false, -1.0f, -1.0f};
  }

  float leftRangeCm = pickBoxRange(sensor1, sensor2);
  float rightRangeCm = pickBoxRange(sensor3, sensor4);

  bool frameValid = true;
  float xM = 0.0f, yFromWallM = 0.0f;

  if (leftRangeCm < 0.0f || rightRangeCm < 0.0f) {
    frameValid = false;
  } else {
    float leftRangeM = leftRangeCm / 100.0f;
    float rightRangeM = rightRangeCm / 100.0f;
    xM = (BOX_BASELINE_M * BOX_BASELINE_M + leftRangeM * leftRangeM
      - rightRangeM * rightRangeM) / (2.0f * BOX_BASELINE_M);
    float ySquared = leftRangeM * leftRangeM - xM * xM;
    if (ySquared < 0.0f) {
      frameValid = false;
    } else {
      yFromWallM = sqrtf(ySquared);
      float xMarginM = BOX_BASELINE_M * PLAY_AREA_X_MARGIN_FRACTION;
      float depthMarginM = PLAY_AREA_DEPTH_M * PLAY_AREA_DEPTH_MARGIN_FRACTION;
      float minDepthM = DEAD_ZONE_DEPTH_M + depthMarginM;
      float maxDepthM = DEAD_ZONE_DEPTH_M + PLAY_AREA_DEPTH_M - depthMarginM;
      if (xM < xMarginM || xM > BOX_BASELINE_M - xMarginM ||
          yFromWallM < minDepthM || yFromWallM > maxDepthM) {
        frameValid = false;
      }
    }
  }

  if (!frameValid) {
    if (hasPosition) return {true, smoothedX, smoothedY};
    return {false, -1.0f, -1.0f};
  }

  float xMinM = BOX_BASELINE_M * PLAY_AREA_X_MARGIN_FRACTION;
  float xMaxM = BOX_BASELINE_M - xMinM;
  float depthMarginM = PLAY_AREA_DEPTH_M * PLAY_AREA_DEPTH_MARGIN_FRACTION;
  float minDepthM = DEAD_ZONE_DEPTH_M + depthMarginM;
  float trackedDepthM = PLAY_AREA_DEPTH_M - 2.0f * depthMarginM;
  float x = (xM - xMinM) / (xMaxM - xMinM);
  // y=1 is the near edge; the dashboard flips canvas y to place it at the top.
  float y = 1.0f - (yFromWallM - minDepthM) / trackedDepthM;

  if (!hasPosition) {
    smoothedX = x;
    smoothedY = y;
    hasPosition = true;
  } else {
    smoothedX += POSITION_SMOOTHING_ALPHA * (x - smoothedX);
    smoothedY += POSITION_SMOOTHING_ALPHA * (y - smoothedY);
  }
  return {true, smoothedX, smoothedY};
}

void handleSlavePacket(const uint8_t *incomingData, int length) {
  if (length != sizeof(SensorPacket)) return;
  SensorPacket packet;
  memcpy(&packet, incomingData, sizeof(packet));
  slaveSensor1Cm = packet.sensor1Cm;
  slaveSensor2Cm = packet.sensor2Cm;
  slaveReplyReady = true;
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataReceived(const esp_now_recv_info_t *, const uint8_t *data, int length) {
#else
void onDataReceived(const uint8_t *, const uint8_t *data, int length) {
#endif
  handleSlavePacket(data, length);
}

bool requestSlaveSample() {
  slaveReplyReady = false;
  SampleRequest request{1};
  esp_now_send(SLAVE_MAC, reinterpret_cast<uint8_t *>(&request), sizeof(request));

  unsigned long start = millis();
  while (!slaveReplyReady && millis() - start < SLAVE_REPLY_TIMEOUT_MS) {
    delay(1);
  }
  return slaveReplyReady;
}

void buildSensorJson(char *buffer, size_t bufferSize) {
  float sensor1 = readSensorDistance(SENSOR_1_TRIG_PIN, SENSOR_1_ECHO_PIN);
  delay(SENSOR_SETTLE_DELAY_MS);
  float sensor2 = readSensorDistance(SENSOR_2_TRIG_PIN, SENSOR_2_ECHO_PIN);

  bool slaveFresh = requestSlaveSample();
  float sensor3 = slaveFresh ? slaveSensor1Cm : -1.0f;
  float sensor4 = slaveFresh ? slaveSensor2Cm : -1.0f;

  PlayerPosition position = calculatePlayerPosition(sensor1, sensor2, sensor3, sensor4);
  float depth = nearestValidDepth(sensor1, sensor2, sensor3, sensor4);

  snprintf(
    buffer,
    bufferSize,
    "{\"mac\":\"%s\",\"sensor1\":%.2f,\"sensor2\":%.2f,\"sensor3\":%.2f,\"sensor4\":%.2f,\"closest\":%.2f,\"x\":%.3f,\"y\":%.3f}",
    WiFi.macAddress().c_str(),
    sensor1,
    sensor2,
    sensor3,
    sensor4,
    depth,
    position.valid ? position.x : -1.0f,
    position.valid ? position.y : -1.0f
  );
}

const char DASHBOARD_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Sensor Dashboard</title>
<style>
  body { font-family: Consolas, "Courier New", monospace; background: #111; color: #eee; margin: 0; padding: 24px; }
  h1 { font-size: 18px; margin: 0 0 16px 0; }
  .grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 14px; max-width: 620px; }
  .box { background: #1a1a1a; border: 1px solid #333; border-radius: 10px; padding: 18px; text-align: center; }
  .box-label { font-size: 13px; color: #888; text-transform: uppercase; letter-spacing: 0.05em; margin-bottom: 8px; }
  .box-value { font-size: 30px; font-weight: bold; }
  .status { margin-top: 16px; color: #888; font-size: 12px; }
  .position-map { margin-top: 18px; width: min(100%, 620px); background: #1a1a1a; border: 1px solid #333; border-radius: 10px; padding: 12px; }
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
    function fmt(v) { return (v === null || v < 0) ? "--" : v.toFixed(1) + " cm"; }
    function fmtCoordinate(v) { return (v === null || v < 0) ? "--" : v.toFixed(3); }
    function drawPosition(x, y) {
      const canvas = document.getElementById("positionMap");
      const ctx = canvas.getContext("2d");
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.strokeStyle = "#425466"; ctx.lineWidth = 2;
      for (let i = 1; i < 3; i++) {
        ctx.beginPath(); ctx.moveTo(canvas.width * i / 3, 0);
        ctx.lineTo(canvas.width * i / 3, canvas.height); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(0, canvas.height * i / 3);
        ctx.lineTo(canvas.width, canvas.height * i / 3); ctx.stroke();
      }
      if (x === null || y === null || x < 0 || y < 0) return;
      ctx.fillStyle = "#ffcc33"; ctx.beginPath();
      ctx.arc(x * canvas.width, (1 - y) * canvas.height, 12, 0, Math.PI * 2); ctx.fill();
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
        document.getElementById("status").textContent = "Updated " + new Date().toLocaleTimeString();
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
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  WiFi.softAP(AP_SSID, AP_PASSWORD, FIXED_CHANNEL);
  udp.begin(UDP_PORT);

  Serial.print("Master SoftAP MAC: ");
  Serial.println(WiFi.softAPmacAddress());
  Serial.print("Master station MAC (NOT what the slave should target): ");
  Serial.println(WiFi.macAddress());

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
  }

  Serial.println("Master position-tracking firmware ready");
  Serial.print("Dashboard/network IP: ");
  Serial.println(WiFi.softAPIP());

  webServer.on("/", handleRoot);
  webServer.on("/data", handleData);
  webServer.begin();
  Serial.println("Web dashboard started");
}

void loop() {
  webServer.handleClient();

  digitalWrite(LED_PIN, HIGH);
  char packet[180];
  buildSensorJson(packet, sizeof(packet));
  udp.beginPacket(BROADCAST_IP, UDP_PORT);
  udp.write(reinterpret_cast<const uint8_t *>(packet), strlen(packet));
  udp.endPacket();
  digitalWrite(LED_PIN, LOW);

  webServer.handleClient();
}

// master.cpp
//
// MASTER board.
//
// Responsibilities:
//   1. Hosts its own Wi-Fi network.
//   2. Reads its own two ultrasonic sensors.
//   3. Receives the slave board's two sensor readings over ESP-NOW.
//   4. Broadcasts all four readings as UDP JSON.
//   5. Serves its built-in web dashboard.
//   6. Calculates player position.
//
// POSITIONING (this version):
//
//   Physical layout:
//     S1 (master) and S3 (slave) point 70 deg from the wall: mostly
//     forward, angled inward. These are the TRACKING sensors.
//     S2 (master) and S4 (slave) point 20 deg from the wall: they skim
//     along the wall toward the other box. They are NOT used for
//     position. Their raw values are still sent to the dashboard.
//
//   Algorithm:
//     - Two-circle trilateration from S1 (master range) and S3 (slave range).
//     - If only one of S1/S3 sees the player, short-term one-box recovery
//       keeps the previous X and re-estimates depth.
//     - The old lateral-difference correction has been removed.
//

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>

// ---------------------------------------------------------------------
// Wi-Fi network the master hosts
// ---------------------------------------------------------------------

const char *AP_SSID = "WhackAMole-Sensors";
const char *AP_PASSWORD = "molemole1";
const uint8_t FIXED_CHANNEL = 1;

// ---------------------------------------------------------------------
// UDP
// ---------------------------------------------------------------------

const unsigned int UDP_PORT = 4210;
IPAddress BROADCAST_IP(192, 168, 4, 255);

// ---------------------------------------------------------------------
// ESP-NOW
// ---------------------------------------------------------------------

const uint8_t ESPNOW_BROADCAST_MAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct SensorPacket {
  float sensor1Cm;
  float sensor2Cm;
};

volatile float slaveSensor1Cm = -1.0f;
volatile float slaveSensor2Cm = -1.0f;
volatile unsigned long lastSlavePacketMs = 0;
volatile bool slaveReplyReady = false;

const uint8_t SLAVE_SAMPLE_REQUEST = 2;

// ---------------------------------------------------------------------
// Pins
// ---------------------------------------------------------------------

const int SENSOR_1_TRIG_PIN = 32;
const int SENSOR_1_ECHO_PIN = 35;
const int SENSOR_2_TRIG_PIN = 12;
const int SENSOR_2_ECHO_PIN = 14;

const int LED_PIN = 2;
const int BUZZER_PIN = 18;
const unsigned int BUZZER_FREQUENCY_HZ = 2200;

// ---------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------

const unsigned long SENSOR_SETTLE_DELAY_MS = 90;

// ---------------------------------------------------------------------
// Position configuration
// ---------------------------------------------------------------------

const float MAX_TRACKING_DISTANCE_CM = 200.0f;

// Distance between the two sensor boxes.
const float BOX_BASELINE_M = 1.5f;

// Tolerance outside the physical left/right bounds before a
// trilateration result is rejected.
const float POSITION_X_MARGIN_M = 0.35f;

// Dead zone near the sensor wall. Matches the dashboard (60 cm).
const float DEAD_ZONE_DEPTH_M = 0.60f;
const float DEAD_ZONE_RELEASE_DEPTH_M = 0.65f;

// Depth represented by the graph.
const float POSITION_MAP_DEPTH_M = 1.8f;

// ---------------------------------------------------------------------
// Smoothing
// ---------------------------------------------------------------------

const float POSITION_SMOOTHING_ALPHA = 0.70f;
const size_t POSITION_HISTORY_SIZE = 3;
const size_t SENSOR_HISTORY_SIZE = 3;

// ---------------------------------------------------------------------
// One-box recovery
// ---------------------------------------------------------------------

const unsigned long ONE_BOX_RECOVERY_TIMEOUT_MS = 1200;

// ---------------------------------------------------------------------
// State
// ---------------------------------------------------------------------

bool playerInDeadZone = false;

WiFiUDP udp;
WebServer webServer(80);

char cachedJson[256] = "{}";

// =====================================================================
// Sensor reading
// =====================================================================

float readSensorDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 16000);

  if (duration <= 0) {
    return -1.0f;
  }

  return duration * 0.0343f / 2.0f;
}

// =====================================================================
// Distance helpers
// =====================================================================

bool isValidDistance(float distance) {
  return isfinite(distance) &&
         distance > 0.0f &&
         distance <= MAX_TRACKING_DISTANCE_CM;
}

void printDistanceValue(float distance) {
  if (!isValidDistance(distance)) {
    Serial.print("NaN");
  } else {
    Serial.print(distance, 2);
    Serial.print(" cm");
  }
}

// =====================================================================
// Structures
// =====================================================================

struct PlayerPosition {
  bool valid;
  float x;
  float y;
};

struct PositionFilterState {
  bool hasPosition;

  float smoothedX;
  float smoothedY;

  float xHistory[POSITION_HISTORY_SIZE];
  float yHistory[POSITION_HISTORY_SIZE];

  size_t historyCount;
  size_t historyIndex;

  unsigned long lastFullPositionMs;
  unsigned long lastRecoveryMs;
};

// Filter state for the two TRACKING sensors only (S1 and S3).
struct SensorDistanceFilterState {
  float history[2][SENSOR_HISTORY_SIZE];
  size_t historyCount[2];
  size_t historyIndex[2];
};

// =====================================================================
// Median (insertion sort, sorts the array in place)
// =====================================================================

float medianOf(float *values, int count) {
  for (int i = 1; i < count; i++) {
    float value = values[i];
    int j = i - 1;

    while (j >= 0 && values[j] > value) {
      values[j + 1] = values[j];
      j--;
    }

    values[j + 1] = value;
  }

  if (count % 2 == 0) {
    return (values[count / 2 - 1] + values[count / 2]) * 0.5f;
  }

  return values[count / 2];
}

// =====================================================================
// Individual sensor filtering (median of last few valid readings)
// =====================================================================

float filterSensorDistance(
  float distance,
  size_t sensorIndex,
  SensorDistanceFilterState &state
) {
  state.history[sensorIndex][state.historyIndex[sensorIndex]] =
    isValidDistance(distance) ? distance : -1.0f;

  state.historyIndex[sensorIndex] =
    (state.historyIndex[sensorIndex] + 1) % SENSOR_HISTORY_SIZE;

  if (state.historyCount[sensorIndex] < SENSOR_HISTORY_SIZE) {
    state.historyCount[sensorIndex]++;
  }

  if (!isValidDistance(distance)) {
    return -1.0f;
  }

  float samples[SENSOR_HISTORY_SIZE];
  int sampleCount = 0;

  for (size_t i = 0; i < state.historyCount[sensorIndex]; i++) {
    float sample = state.history[sensorIndex][i];

    if (isValidDistance(sample)) {
      samples[sampleCount++] = sample;
    }
  }

  if (sampleCount < 3) {
    return distance;
  }

  return medianOf(samples, sampleCount);
}

// =====================================================================
// Position helpers
// =====================================================================

PlayerPosition retainPosition(const PositionFilterState &state) {
  if (!state.hasPosition) {
    return {false, -1.0f, -1.0f};
  }

  return {true, state.smoothedX, state.smoothedY};
}

PlayerPosition addPositionSample(
  float x,
  float y,
  PositionFilterState &state
) {
  state.xHistory[state.historyIndex] = x;
  state.yHistory[state.historyIndex] = y;

  state.historyIndex = (state.historyIndex + 1) % POSITION_HISTORY_SIZE;

  if (state.historyCount < POSITION_HISTORY_SIZE) {
    state.historyCount++;
  }

  float xSamples[POSITION_HISTORY_SIZE];
  float ySamples[POSITION_HISTORY_SIZE];

  for (size_t i = 0; i < state.historyCount; i++) {
    xSamples[i] = state.xHistory[i];
    ySamples[i] = state.yHistory[i];
  }

  float measuredX = medianOf(xSamples, state.historyCount);
  float measuredY = medianOf(ySamples, state.historyCount);

  if (!state.hasPosition) {
    state.smoothedX = measuredX;
    state.smoothedY = measuredY;
    state.hasPosition = true;
  } else {
    state.smoothedX += POSITION_SMOOTHING_ALPHA * (measuredX - state.smoothedX);
    state.smoothedY += POSITION_SMOOTHING_ALPHA * (measuredY - state.smoothedY);
  }

  return {true, state.smoothedX, state.smoothedY};
}

void getPreviousPositionMeters(
  const PositionFilterState &state,
  float &xMeters,
  float &depthMeters
) {
  if (!state.hasPosition) {
    xMeters = BOX_BASELINE_M * 0.5f;
    depthMeters = POSITION_MAP_DEPTH_M * 0.5f;
    return;
  }

  xMeters = state.smoothedX * BOX_BASELINE_M;
  depthMeters = (1.0f - state.smoothedY) * POSITION_MAP_DEPTH_M;
}

// Hysteresis for the dead zone so the buzzer doesn't flicker.
bool updateDeadZone(bool latched, float depthMeters) {
  if (latched) {
    return depthMeters <= DEAD_ZONE_RELEASE_DEPTH_M;
  }

  return depthMeters <= DEAD_ZONE_DEPTH_M;
}

// =====================================================================
// One-box recovery
// =====================================================================
//
// Used when only ONE tracking sensor (S1 or S3) sees the player.
// Keeps the previous X and estimates depth from the one range:
//   depth = sqrt(range^2 - (previousX - boxX)^2)
//

PlayerPosition calculateOneBoxRecovery(
  float rangeCm,
  bool isMasterBox,
  PositionFilterState &state
) {
  if (!state.hasPosition || !isValidDistance(rangeCm)) {
    return retainPosition(state);
  }

  float previousX;
  float previousDepth;

  getPreviousPositionMeters(state, previousX, previousDepth);

  float boxX = isMasterBox ? 0.0f : BOX_BASELINE_M;
  float rangeM = rangeCm / 100.0f;
  float dx = previousX - boxX;

  float depthSquared = rangeM * rangeM - dx * dx;

  float recoveredDepth =
    depthSquared >= 0.0f ? sqrtf(depthSquared) : previousDepth;

  recoveredDepth = fminf(POSITION_MAP_DEPTH_M, fmaxf(0.0f, recoveredDepth));

  static bool deadZoneLatchedRecovery = false;
  deadZoneLatchedRecovery = updateDeadZone(deadZoneLatchedRecovery, recoveredDepth);
  playerInDeadZone = deadZoneLatchedRecovery;

  float x = fminf(1.0f, fmaxf(0.0f, previousX / BOX_BASELINE_M));
  float y = 1.0f - recoveredDepth / POSITION_MAP_DEPTH_M;

  state.lastRecoveryMs = millis();

  return addPositionSample(x, y, state);
}

// =====================================================================
// Main position calculation
// =====================================================================
//
// sensor1 = master S1 (70 deg, tracking)
// sensor3 = slave  S3 (70 deg, tracking)
//
// S2 and S4 are intentionally NOT passed in.
//

PlayerPosition calculatePlayerPosition(float sensor1, float sensor3) {
  static PositionFilterState filterState = {};
  static SensorDistanceFilterState distanceFilterState = {};
  static bool deadZoneLatched = false;

  playerInDeadZone = false;

  // 1. Filter the two tracking sensors.
  float f1 = filterSensorDistance(sensor1, 0, distanceFilterState);
  float f3 = filterSensorDistance(sensor3, 1, distanceFilterState);

  bool masterHasReading = isValidDistance(f1);
  bool slaveHasReading = isValidDistance(f3);

  // 2. Two-circle trilateration.
  if (masterHasReading && slaveHasReading) {
    float rm = f1 / 100.0f;
    float rs = f3 / 100.0f;
    float D = BOX_BASELINE_M;

    float xMeters = (rm * rm - rs * rs + D * D) / (2.0f * D);
    float depthSquared = rm * rm - xMeters * xMeters;

    // Allow a small negative value from noise, clamp it to zero.
    if (depthSquared >= -0.0025f) {
      if (depthSquared < 0.0f) {
        depthSquared = 0.0f;
      }

      float depthMeters = sqrtf(depthSquared);

      if (xMeters >= -POSITION_X_MARGIN_M &&
          xMeters <= D + POSITION_X_MARGIN_M &&
          depthMeters <= POSITION_MAP_DEPTH_M) {

        float rawXNormalised = xMeters / D;

        xMeters = fminf(D, fmaxf(0.0f, xMeters));

        deadZoneLatched = updateDeadZone(deadZoneLatched, depthMeters);
        playerInDeadZone = deadZoneLatched;

        float x = xMeters / D;
        float y = 1.0f - depthMeters / POSITION_MAP_DEPTH_M;

        filterState.lastFullPositionMs = millis();

        Serial.println();
        Serial.println("========== POSITION DEBUG ==========");
        Serial.print("S1 (master, tracking): ");
        printDistanceValue(f1);
        Serial.print(" | S3 (slave, tracking): ");
        printDistanceValue(f3);
        Serial.println();
        Serial.print("Raw trilateration X (0-1): ");
        Serial.println(rawXNormalised, 3);
        Serial.print("Depth: ");
        Serial.print(depthMeters, 3);
        Serial.println(" m");
        Serial.print("Final X: ");
        Serial.print(x, 3);
        Serial.print(" | Y: ");
        Serial.println(y, 3);
        Serial.println("====================================");

        return addPositionSample(x, y, filterState);
      }
    }
  }

  // 3. One-box recovery (short timeout after last good fix).
  if (filterState.hasPosition) {
    unsigned long now = millis();
    bool recent =
      now - filterState.lastFullPositionMs <= ONE_BOX_RECOVERY_TIMEOUT_MS;

    if (recent && masterHasReading && !slaveHasReading) {
      return calculateOneBoxRecovery(f1, true, filterState);
    }

    if (recent && slaveHasReading && !masterHasReading) {
      return calculateOneBoxRecovery(f3, false, filterState);
    }
  }

  // 4. Nothing usable: hold the last position.
  playerInDeadZone = false;
  return retainPosition(filterState);
}

// =====================================================================
// Buzzer
// =====================================================================

void setBuzzerOutput(bool enabled) {
  static bool outputEnabled = false;

  if (enabled == outputEnabled) {
    return;
  }

  if (enabled) {
    tone(BUZZER_PIN, BUZZER_FREQUENCY_HZ);
  } else {
    noTone(BUZZER_PIN);
  }

  outputEnabled = enabled;
}

// =====================================================================
// ESP-NOW receive callback
// =====================================================================

void handleSlavePacket(const uint8_t *incomingData, int length) {
  if (length != sizeof(SensorPacket)) {
    Serial.print("Got ESP-NOW packet with wrong size: ");
    Serial.println(length);
    return;
  }

  SensorPacket packet;
  memcpy(&packet, incomingData, sizeof(packet));

  slaveSensor1Cm = packet.sensor1Cm;
  slaveSensor2Cm = packet.sensor2Cm;
  lastSlavePacketMs = millis();
  slaveReplyReady = true;
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataReceived(
  const esp_now_recv_info_t *,
  const uint8_t *incomingData,
  int length
) {
#else
void onDataReceived(
  const uint8_t *,
  const uint8_t *incomingData,
  int length
) {
#endif
  handleSlavePacket(incomingData, length);
}

// =====================================================================
// Request a sample from the slave
// =====================================================================

bool requestSlaveSample() {
  slaveReplyReady = false;

  uint8_t request = SLAVE_SAMPLE_REQUEST;

  delay(SENSOR_SETTLE_DELAY_MS);

  if (esp_now_send(ESPNOW_BROADCAST_MAC, &request, sizeof(request)) != ESP_OK) {
    return false;
  }

  unsigned long requestStartedMs = millis();

  while (!slaveReplyReady && millis() - requestStartedMs < 250) {
    delay(1);
  }

  return slaveReplyReady;
}

// =====================================================================
// Sample sensors, compute position, build JSON
// =====================================================================

void buildSensorJson(char *buffer, size_t bufferSize) {

  // Master sensors.
  float sensor1 = readSensorDistance(SENSOR_1_TRIG_PIN, SENSOR_1_ECHO_PIN);
  delay(SENSOR_SETTLE_DELAY_MS);

  float sensor2 = readSensorDistance(SENSOR_2_TRIG_PIN, SENSOR_2_ECHO_PIN);
  delay(SENSOR_SETTLE_DELAY_MS);

  // Slave sensors.
  bool slaveSampleReceived = requestSlaveSample();

  float remote1 = slaveSampleReceived ? slaveSensor1Cm : -1.0f;
  float remote2 = slaveSampleReceived ? slaveSensor2Cm : -1.0f;

  // Closest raw distance (dashboard / UDP only).
  float closest = -1.0f;
  float all[4] = {sensor1, sensor2, remote1, remote2};

  for (int i = 0; i < 4; i++) {
    if (isValidDistance(all[i]) &&
        (!isValidDistance(closest) || all[i] < closest)) {
      closest = all[i];
    }
  }

  // Position: ONLY S1 (master) and S3 (slave).
  PlayerPosition position = calculatePlayerPosition(sensor1, remote1);

  // Buzzer on master and slave.
  setBuzzerOutput(playerInDeadZone);

  uint8_t buzzerCommand = playerInDeadZone ? 1 : 0;
  esp_now_send(ESPNOW_BROADCAST_MAC, &buzzerCommand, sizeof(buzzerCommand));

  // JSON.
  snprintf(
    buffer,
    bufferSize,
    "{\"mac\":\"%s\","
    "\"sensor1\":%.2f,"
    "\"sensor2\":%.2f,"
    "\"sensor3\":%.2f,"
    "\"sensor4\":%.2f,"
    "\"closest\":%.2f,"
    "\"x\":%.3f,"
    "\"y\":%.3f}",
    WiFi.macAddress().c_str(),
    sensor1,
    sensor2,
    remote1,
    remote2,
    closest,
    position.valid ? position.x : -1.0f,
    position.valid ? position.y : -1.0f
  );

  // Serial summary.
  Serial.println();
  Serial.println("---------------- POSITION ----------------");
  Serial.print("S1: ");
  printDistanceValue(sensor1);
  Serial.print(" | S2 (unused): ");
  printDistanceValue(sensor2);
  Serial.print(" | S3: ");
  printDistanceValue(remote1);
  Serial.print(" | S4 (unused): ");
  printDistanceValue(remote2);
  Serial.print(" | x: ");

  if (position.valid) {
    Serial.print(position.x, 3);
  } else {
    Serial.print("NaN");
  }

  Serial.print(" | y: ");

  if (position.valid) {
    Serial.print(position.y, 3);
  } else {
    Serial.print("NaN");
  }

  Serial.print(" | dead zone: ");
  Serial.println(playerInDeadZone ? "YES" : "no");
}

// =====================================================================
// Built-in web dashboard
// =====================================================================

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
.box-value { font-size: 30px; font-weight: bold; }
.unused .box-value { color: #666; }
.status { margin-top: 16px; color: #888; font-size: 12px; }
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
  <div class="box"><div class="box-label">Sensor 1 (track)</div><div class="box-value" id="s1">--</div></div>
  <div class="box unused"><div class="box-label">Sensor 2 (unused)</div><div class="box-value" id="s2">--</div></div>
  <div class="box"><div class="box-label">Sensor 3 (track)</div><div class="box-value" id="s3">--</div></div>
  <div class="box unused"><div class="box-label">Sensor 4 (unused)</div><div class="box-value" id="s4">--</div></div>
  <div class="box"><div class="box-label">X coordinate</div><div class="box-value" id="x">--</div></div>
  <div class="box"><div class="box-label">Y coordinate</div><div class="box-value" id="y">--</div></div>
</div>

<div class="position-map">
  <canvas id="positionMap" width="600" height="360"></canvas>
</div>

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

  const deadZoneDepth = 0.6;
  const totalDepth = 1.8;

  const depthToY = depth => canvas.height * (1 - depth / totalDepth);
  const playAreaBottom = depthToY(deadZoneDepth);

  const columnLines = [0.35 / 1.5, 1.15 / 1.5];

  ctx.fillStyle = "#38272c";
  ctx.fillRect(0, playAreaBottom, canvas.width, canvas.height - playAreaBottom);

  ctx.strokeStyle = "#425466";
  ctx.lineWidth = 2;

  for (const boundary of columnLines) {
    ctx.beginPath();
    ctx.moveTo(canvas.width * boundary, 0);
    ctx.lineTo(canvas.width * boundary, playAreaBottom);
    ctx.stroke();
  }

  for (const depth of [1.0, 1.4]) {
    const rowY = depthToY(depth);
    ctx.beginPath();
    ctx.moveTo(0, rowY);
    ctx.lineTo(canvas.width, rowY);
    ctx.stroke();
  }

  ctx.strokeRect(0, 0, canvas.width, playAreaBottom);

  ctx.strokeStyle = "#a85b62";
  ctx.beginPath();
  ctx.moveTo(0, playAreaBottom);
  ctx.lineTo(canvas.width, playAreaBottom);
  ctx.stroke();

  ctx.fillStyle = "#e3a0a4";
  ctx.font = "14px Consolas, monospace";
  ctx.fillText(
    "60 cm DEAD ZONE",
    12,
    playAreaBottom + (canvas.height - playAreaBottom) / 2 + 5
  );

  ctx.fillStyle = "#9eacb8";
  ctx.fillText("ROW 3: 140-180 cm", 12, depthToY(1.6) + 5);
  ctx.fillText("ROW 2: 100-140 cm", 12, depthToY(1.2) + 5);
  ctx.fillText("ROW 1: 60-100 cm", 12, depthToY(0.8) + 5);

  if (x === null || y === null || x < 0 || y < 0) {
    return;
  }

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

// =====================================================================
// Web handlers
// =====================================================================

void handleRoot() {
  webServer.send(200, "text/html", DASHBOARD_HTML);
}

void handleData() {
  webServer.send(200, "application/json", cachedJson);
}

// =====================================================================
// Setup
// =====================================================================

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  pinMode(SENSOR_1_TRIG_PIN, OUTPUT);
  pinMode(SENSOR_1_ECHO_PIN, INPUT);
  pinMode(SENSOR_2_TRIG_PIN, OUTPUT);
  pinMode(SENSOR_2_ECHO_PIN, INPUT);

  digitalWrite(SENSOR_1_TRIG_PIN, LOW);
  digitalWrite(SENSOR_2_TRIG_PIN, LOW);

  Serial.begin(115200);
  delay(1000);

  // Wi-Fi AP + ESP-NOW
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  WiFi.softAP(AP_SSID, AP_PASSWORD, FIXED_CHANNEL);

  Serial.println();
  Serial.println("ESP32 dual sensor - MASTER");
  Serial.println("Position algorithm: two-box trilateration using S1 + S3 only");
  Serial.println("Sensor 1: TRIG=32, ECHO=35 (70 deg, tracking)");
  Serial.println("Sensor 2: TRIG=12, ECHO=14 (20 deg, unused for position)");

  Serial.print("Master MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Master SoftAP MAC: ");
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

  // UDP
  udp.begin(UDP_PORT);

  // ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(onDataReceived);

  esp_now_peer_info_t broadcastPeer = {};
  memcpy(broadcastPeer.peer_addr, ESPNOW_BROADCAST_MAC, 6);
  broadcastPeer.channel = FIXED_CHANNEL;
  broadcastPeer.encrypt = false;
  broadcastPeer.ifidx = WIFI_IF_AP;

  if (esp_now_add_peer(&broadcastPeer) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW broadcast peer");
  } else {
    Serial.println("ESP-NOW broadcast peer registered");
  }

  uint8_t primaryChannel;
  wifi_second_chan_t secondChannel;
  esp_wifi_get_channel(&primaryChannel, &secondChannel);

  Serial.print("Master radio actually on channel: ");
  Serial.println(primaryChannel);

  // Web dashboard
  webServer.on("/", handleRoot);
  webServer.on("/data", handleData);
  webServer.begin();

  Serial.println("Web dashboard started");
  Serial.println("ESP-NOW receiver ready");
  Serial.println("----------------------------------------");
}

// =====================================================================
// Main loop
// =====================================================================

void loop() {
  webServer.handleClient();

  digitalWrite(LED_PIN, HIGH);

  buildSensorJson(cachedJson, sizeof(cachedJson));

  udp.beginPacket(BROADCAST_IP, UDP_PORT);
  udp.write(reinterpret_cast<const uint8_t *>(cachedJson), strlen(cachedJson));

  if (udp.endPacket() == 0) {
    Serial.println("UDP broadcast send failed");
  }

  digitalWrite(LED_PIN, LOW);

  delay(20);
}
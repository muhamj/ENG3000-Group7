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
// Flash the matching slave.cpp to the other board. Both boards use the
// fixed channel and ESP-NOW broadcast, so no MAC address pairing is needed.
//
// CHANGE LOG (timing fix for ~50% sensor miss rate):
//   - SENSOR_SETTLE_DELAY_MS raised from 40ms to 90ms. With both boxes
//     powered, master's and slave's ultrasonic pulses were firing close
//     enough together that one box's echo was sometimes still bouncing
//     around the room when the next sensor (on either box) fired,
//     causing that sensor to catch a stray echo or nothing at all
//     (classic multi-sensor ultrasonic crosstalk).
//   - Added an extra settle delay after master's own two sensors and
//     BEFORE requesting the slave to fire, so master's echoes are fully
//     clear of the room before slave's pulses go out.
//   - Added raw duration_us logging in readSensorDistance() so the
//     actual pulseIn() result is visible per-read, not just the
//     filtered/converted distance.

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

// ESP-NOW broadcast reaches the slave without relying on a hard-coded MAC.
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
// Master's own two ultrasonic sensors
// ---------------------------------------------------------------------
const int SENSOR_1_TRIG_PIN = 32;
const int SENSOR_1_ECHO_PIN = 35;
const int SENSOR_2_TRIG_PIN = 12;
const int SENSOR_2_ECHO_PIN = 14;
const int LED_PIN = 2;
const int BUZZER_PIN = 18;
const unsigned int BUZZER_FREQUENCY_HZ = 2200;

// Raised from 40ms to 90ms - see CHANGE LOG above. This is used as the
// gap between every pair of ultrasonic pulses in the whole 4-sensor
// chain (master sensor 1 -> 2, and again before the slave is asked to
// fire), so every sensor's echo has fully died out before the next
// pulse goes out anywhere in the room.
const unsigned long SENSOR_SETTLE_DELAY_MS = 90;

const float MAX_TRACKING_DISTANCE_CM = 200.0f;
const float BOX_BASELINE_M = 1.5f;
const float POSITION_X_MARGIN_M = 0.35f;
const float DEAD_ZONE_DEPTH_M = 0.5f;
const float DEAD_ZONE_RELEASE_DEPTH_M = 0.55f;
const float POSITION_MAP_DEPTH_M = 1.8f;
const size_t POSITION_HISTORY_SIZE = 3;
const float POSITION_SMOOTHING_ALPHA = 0.85f;
bool playerInDeadZone = false;

WiFiUDP udp;
WebServer webServer(80);

// Cached JSON from the most recent sensor cycle. The /data web handler
// returns this instantly instead of triggering its own sensor read,
// which was causing double-fire ultrasonic crosstalk.
char cachedJson[256] = "{}";

// =======================================================================
// Sensor reading
// =======================================================================

float readSensorDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 16000);

  // Raw diagnostic - shows the actual pulseIn() result before any
  // filtering or conversion. Leave this in for now while you confirm
  // the timing fix helped; comment it out or remove it once you're
  // happy with the hit rate, since it adds Serial traffic every cycle.
  Serial.print("  [raw] trig=");
  Serial.print(trigPin);
  Serial.print(" echo=");
  Serial.print(echoPin);
  Serial.print(" duration_us=");
  Serial.println(duration);

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

struct PositionFilterState {
  bool hasPosition;
  float smoothedX;
  float smoothedY;
  float xHistory[POSITION_HISTORY_SIZE];
  float yHistory[POSITION_HISTORY_SIZE];
  size_t historyCount;
  size_t historyIndex;
};

struct SensorDistanceFilterState {
  float history[4][POSITION_HISTORY_SIZE];
  size_t historyCount[4];
  size_t historyIndex[4];
};

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

float filterSensorDistance(float distance, size_t sensorIndex, SensorDistanceFilterState &state) {
  state.history[sensorIndex][state.historyIndex[sensorIndex]] =
    isValidDistance(distance) ? distance : -1.0f;
  state.historyIndex[sensorIndex] =
    (state.historyIndex[sensorIndex] + 1) % POSITION_HISTORY_SIZE;
  if (state.historyCount[sensorIndex] < POSITION_HISTORY_SIZE) {
    state.historyCount[sensorIndex]++;
  }

  if (!isValidDistance(distance)) return -1.0f;

  float samples[POSITION_HISTORY_SIZE];
  int sampleCount = 0;
  for (size_t i = 0; i < state.historyCount[sensorIndex]; i++) {
    float sample = state.history[sensorIndex][i];
    if (isValidDistance(sample)) samples[sampleCount++] = sample;
  }
  return sampleCount < 3 ? distance : medianOf(samples, sampleCount);
}

// ---------------------------------------------------------------------
// Physical layout (for reference — the mounting angles determine which
// part of the play area each sensor can "see", but the measured range
// is always the straight-line distance from sensor to target regardless
// of beam direction, so the angles are NOT used in the position math).
//
//   Master box at X = 0 (left corner):
//     sensor 1: 20 deg anticlockwise from perpendicular (toward slave)
//     sensor 2: 70 deg anticlockwise from perpendicular (toward slave)
//   Slave box at X = BOX_BASELINE_M (right corner):
//     sensor 3: 20 deg clockwise from perpendicular (toward master)
//     sensor 4: 70 deg clockwise from perpendicular (toward master)
//   Both boxes' sensors point inward, covering the play area between them.
// ---------------------------------------------------------------------

PlayerPosition retainPosition(const PositionFilterState &state) {
  return state.hasPosition ? PlayerPosition{true, state.smoothedX, state.smoothedY}
                           : PlayerPosition{false, -1.0f, -1.0f};
}

PlayerPosition addPositionSample(float x, float y, PositionFilterState &state) {
  state.xHistory[state.historyIndex] = x;
  state.yHistory[state.historyIndex] = y;
  state.historyIndex = (state.historyIndex + 1) % POSITION_HISTORY_SIZE;
  if (state.historyCount < POSITION_HISTORY_SIZE) state.historyCount++;

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

// =======================================================================
// Trilateration-based position calculation.
//
// Each ultrasonic sensor measures the straight-line DISTANCE to the
// target (regardless of beam direction). With the master box at (0, 0)
// and the slave box at (D, 0), two range readings define two circles
// whose intersection gives the player's (x, depth) position:
//
//   x     = (Rm² - Rs² + D²) / (2D)
//   depth = sqrt(Rm² - x²)
//
// Each box has two sensors for redundancy / wider angular coverage.
// We combine each box's readings into one "best range" (closest valid
// reading), then trilaterate between the two boxes.
// =======================================================================

PlayerPosition calculatePlayerPosition(
  float sensor1,
  float sensor2,
  float sensor3,
  float sensor4
) {
  static PositionFilterState filterState = {};
  static SensorDistanceFilterState distanceFilterState = {};
  static bool deadZoneLatched = false;
  playerInDeadZone = false;

  // 1. Noise-reject each sensor's reading (median filter)
  float f1 = filterSensorDistance(sensor1, 0, distanceFilterState);
  float f2 = filterSensorDistance(sensor2, 1, distanceFilterState);
  float f3 = filterSensorDistance(sensor3, 2, distanceFilterState);
  float f4 = filterSensorDistance(sensor4, 3, distanceFilterState);

  // 2. Combine each box's two sensors into one best range.
  //    Both sensors on the same box measure distance to the same target;
  //    the closer valid reading is preferred (longer readings are more
  //    likely to be stray echoes from walls/ceiling).
  float masterRangeCm = getClosestValidDistance(f1, f2);
  float slaveRangeCm  = getClosestValidDistance(f3, f4);

  // 3. Trilateration requires a range from EACH box.
  if (masterRangeCm < 0.0f || slaveRangeCm < 0.0f) {
    return retainPosition(filterState);
  }

  float rm = masterRangeCm / 100.0f;   // metres
  float rs = slaveRangeCm  / 100.0f;   // metres
  float D  = BOX_BASELINE_M;           // 1.5 m

  // Trilateration:
  //   master at (0,0):  rm² = x² + depth²
  //   slave  at (D,0):  rs² = (x-D)² + depth²
  //   subtract → x = (rm² - rs² + D²) / (2D)
  float xMeters = (rm * rm - rs * rs + D * D) / (2.0f * D);
  float depthSquared = rm * rm - xMeters * xMeters;

  // If depthSquared < 0 the two range circles don't intersect
  // (geometrically inconsistent readings). Retain last known position.
  if (depthSquared < 0.0f) {
    return retainPosition(filterState);
  }
  float depthMeters = sqrtf(depthSquared);

  // Bounds check
  if (xMeters < -POSITION_X_MARGIN_M ||
      xMeters > D + POSITION_X_MARGIN_M) {
    return retainPosition(filterState);
  }
  if (depthMeters > POSITION_MAP_DEPTH_M) {
    return retainPosition(filterState);
  }

  // Dead zone (too close to the sensor wall)
  if (deadZoneLatched) {
    deadZoneLatched = depthMeters <= DEAD_ZONE_RELEASE_DEPTH_M;
  } else {
    deadZoneLatched = depthMeters <= DEAD_ZONE_DEPTH_M;
  }
  playerInDeadZone = deadZoneLatched;

  // Normalise to [0, 1] for the game / dashboard
  float x = fminf(1.0f, fmaxf(0.0f, xMeters / D));
  float y = 1.0f - depthMeters / POSITION_MAP_DEPTH_M;
  return addPositionSample(x, y, filterState);
}

void setBuzzerOutput(bool enabled) {
  static bool outputEnabled = false;
  if (enabled == outputEnabled) return;
  if (enabled) {
    tone(BUZZER_PIN, BUZZER_FREQUENCY_HZ);
  } else {
    noTone(BUZZER_PIN);
  }
  outputEnabled = enabled;
}

// =======================================================================
// ESP-NOW receive callback (from the slave board)
// =======================================================================

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
  Serial.println("Slave reply received by master");
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataReceived(const esp_now_recv_info_t *, const uint8_t *incomingData, int length) {
#else
void onDataReceived(const uint8_t *, const uint8_t *incomingData, int length) {
#endif
  handleSlavePacket(incomingData, length);
}

bool requestSlaveSample() {
  slaveReplyReady = false;
  uint8_t request = SLAVE_SAMPLE_REQUEST;
  delay(SENSOR_SETTLE_DELAY_MS);
  if (esp_now_send(ESPNOW_BROADCAST_MAC, &request, sizeof(request)) != ESP_OK) return false;

  unsigned long requestStartedMs = millis();
  while (!slaveReplyReady && millis() - requestStartedMs < 250) {
    delay(1);
  }
  return slaveReplyReady;
}

// =======================================================================
// Shared JSON builder - used for both the UDP broadcast and the web
// dashboard's /data endpoint, so the two never drift out of sync.
// =======================================================================

void buildSensorJson(char *buffer, size_t bufferSize) {
  float sensor1 = readSensorDistance(SENSOR_1_TRIG_PIN, SENSOR_1_ECHO_PIN);
  delay(SENSOR_SETTLE_DELAY_MS); // let sensor 1's echo die down before sensor 2
  float sensor2 = readSensorDistance(SENSOR_2_TRIG_PIN, SENSOR_2_ECHO_PIN);

  // Extra settle gap before asking the slave to fire - gives sensor 2's
  // echo (and any lingering reflections from sensor 1) a full clear
  // window before two MORE ultrasonic pulses go out from across the room.
  delay(SENSOR_SETTLE_DELAY_MS);

  bool slaveSampleReceived = requestSlaveSample();
  float remote1 = slaveSampleReceived ? slaveSensor1Cm : -1.0f;
  float remote2 = slaveSampleReceived ? slaveSensor2Cm : -1.0f;

  float closest = getClosestValidDistance(
    getClosestValidDistance(sensor1, sensor2),
    getClosestValidDistance(remote1, remote2)
  );
  PlayerPosition position = calculatePlayerPosition(sensor1, sensor2, remote1, remote2);
  setBuzzerOutput(playerInDeadZone);
  uint8_t buzzerCommand = playerInDeadZone ? 1 : 0;
  esp_now_send(ESPNOW_BROADCAST_MAC, &buzzerCommand, sizeof(buzzerCommand));

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
  Serial.print(" | dead zone: ");
  Serial.println(playerInDeadZone ? "YES" : "no");
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
        ctx.beginPath(); ctx.moveTo(canvas.width * boundary, 0);
        ctx.lineTo(canvas.width * boundary, playAreaBottom); ctx.stroke();
      }
      for (const depth of [1.0, 1.4]) {
        const rowY = depthToY(depth);
        ctx.beginPath(); ctx.moveTo(0, rowY);
        ctx.lineTo(canvas.width, rowY); ctx.stroke();
      }
      ctx.strokeRect(0, 0, canvas.width, playAreaBottom);
      ctx.strokeStyle = "#a85b62";
      ctx.beginPath(); ctx.moveTo(0, playAreaBottom);
      ctx.lineTo(canvas.width, playAreaBottom); ctx.stroke();
      ctx.fillStyle = "#e3a0a4";
      ctx.font = "14px Consolas, monospace";
      ctx.fillText("60 cm DEAD ZONE", 12, playAreaBottom + (canvas.height - playAreaBottom) / 2 + 5);
      ctx.fillStyle = "#9eacb8";
      ctx.fillText("ROW 3: 140-180 cm", 12, depthToY(1.6) + 5);
      ctx.fillText("ROW 2: 100-140 cm", 12, depthToY(1.2) + 5);
      ctx.fillText("ROW 1: 60-100 cm", 12, depthToY(0.8) + 5);
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
  webServer.send(200, "application/json", cachedJson);
}

// =======================================================================
// Setup / loop
// =======================================================================

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

  webServer.on("/", handleRoot);
  webServer.on("/data", handleData);
  webServer.begin();
  Serial.println("Web dashboard started");

  Serial.println("ESP-NOW receiver ready");
}

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
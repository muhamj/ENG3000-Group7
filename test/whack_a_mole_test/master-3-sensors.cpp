// master-3-sensors.cpp
// Fixed version — Sensor 3 (and the others) now reliable with Wi-Fi active
//
// Key fix: noInterrupts() / interrupts() around pulseIn()
// + slightly longer settle time (80 ms)

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>

// =====================================================================
// Debug / test flags
// =====================================================================

#define DEBUG_RAW_PULSES   0   // 1 = print every raw pulse duration
#define SELF_TEST_AT_BOOT  1   // 1 = run 8-round self-test after Wi-Fi up
#define BENCH_NO_WIFI      0   // 1 = sensors only, no radio (isolation test)

// =====================================================================
// Wi-Fi network the master hosts
// =====================================================================

const char *AP_SSID     = "WhackAMole-Sensors";
const char *AP_PASSWORD = "molemole1";
const uint8_t FIXED_CHANNEL = 1;

// =====================================================================
// UDP
// =====================================================================

const unsigned int UDP_PORT = 4210;
IPAddress BROADCAST_IP(192, 168, 4, 255);

// =====================================================================
// Pins — identical wiring on both boards
// =====================================================================

constexpr int SENSORS_PER_BOX = 3;
constexpr int NUM_SENSORS     = 6;

const int SENSOR_TRIG_PINS[SENSORS_PER_BOX] = {32, 12, 22};  // S1, S2, S3
const int SENSOR_ECHO_PINS[SENSORS_PER_BOX] = {35, 14, 23};

const int LED_PIN    = 2;
const int BUZZER_PIN = 18;
const unsigned int BUZZER_FREQUENCY_HZ = 2200;

// =====================================================================
// Sensor timing
// =====================================================================

const unsigned long SENSOR_SETTLE_DELAY_MS = 80;   // slightly longer than original
const unsigned long SLAVE_REPLY_TIMEOUT_MS = 500;

// =====================================================================
// ESP-NOW
// =====================================================================

const uint8_t ESPNOW_BROADCAST_MAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const uint8_t SLAVE_SAMPLE_REQUEST   = 2;

// 12 bytes: 3 floats, identical struct on slave
struct SensorPacket {
  float sensor1Cm;
  float sensor2Cm;
  float sensor3Cm;
};

volatile float slaveSensorCm[SENSORS_PER_BOX] = {-1.0f, -1.0f, -1.0f};
volatile unsigned long lastSlavePacketMs = 0;
volatile bool slaveReplyReady = false;

// =====================================================================
// Physical geometry
// =====================================================================

constexpr float BOX_BASELINE_CM = 150.0f;
constexpr float BOX_BASELINE_M  = BOX_BASELINE_CM / 100.0f;

constexpr float POSITION_MAP_DEPTH_CM = 180.0f;
constexpr float POSITION_MAP_DEPTH_M  = POSITION_MAP_DEPTH_CM / 100.0f;

const float MAX_TRACKING_DISTANCE_CM = 300.0f;

const float DEAD_ZONE_DEPTH_CM    = 55.0f;
const float DEAD_ZONE_DEPTH_M     = DEAD_ZONE_DEPTH_CM / 100.0f;
const float DEAD_ZONE_RELEASE_CM  = 60.0f;
const float DEAD_ZONE_RELEASE_M   = DEAD_ZONE_RELEASE_CM / 100.0f;

// =====================================================================
// Sensor mounting angles
// =====================================================================

const float SENSOR_ANGLE_DEG[NUM_SENSORS] = {
  90.0f, 45.0f, 70.0f,   // master S1, S2, S3
  90.0f, 45.0f, 70.0f    // slave  S4, S5, S6
};

// =====================================================================
// Grid-scoring algorithm parameters
// =====================================================================

constexpr float GRID_STEP_M = 0.05f;
constexpr int GRID_COLS  = (int)(BOX_BASELINE_M / GRID_STEP_M + 0.5f) + 1;
constexpr int GRID_ROWS  = (int)(POSITION_MAP_DEPTH_M / GRID_STEP_M + 0.5f) + 1;
constexpr int GRID_CELLS = GRID_COLS * GRID_ROWS;

const float BEAM_SIGMA_DEG   = 12.0f;
const float BEAM_CUTOFF_DEG  = 40.0f;

const float RANGE_SIGMA_BASE_CM = 5.0f;
const float RANGE_SIGMA_PER_CM  = 0.03f;

const float P_OUT_OF_BEAM    = 0.30f;
const float P_RANGE_MISMATCH = 0.02f;
const float P_MISS_IN_BEAM   = 0.25f;

const float MIN_SENSOR_RANGE_CM = 3.0f;
const float RANGE_GATE_MARGIN_CM = 25.0f;

const float PRIOR_SIGMA_M      = 0.45f;
const unsigned long PRIOR_MAX_AGE_MS = 1500;

const float PEAK_RADIUS_M = 0.15f;
const float SUPPORT_MIN_BEAM = 0.35f;
const unsigned long POSITION_HOLD_MS = 2500;

// =====================================================================
// Position smoothing
// =====================================================================

const float POSITION_SMOOTHING_ALPHA = 0.70f;
const size_t POSITION_HISTORY_SIZE   = 3;
const size_t SENSOR_HISTORY_SIZE     = 3;

// =====================================================================
// State
// =====================================================================

bool playerInDeadZone = false;

WiFiUDP udp;
WebServer webServer(80);
char cachedJson[384] = "{}";

uint16_t cellRangeCm[GRID_CELLS][NUM_SENSORS];
uint8_t  cellOffAxisDeg[GRID_CELLS][NUM_SENSORS];
uint16_t sensorBoundaryCm[NUM_SENSORS];

float cellScore[GRID_CELLS];

float lastFilteredCm[NUM_SENSORS];
uint8_t lastSupportMask = 0;
unsigned long lastFitMicros = 0;

// =====================================================================
// Robust sensor read  ← CRITICAL FIX
// =====================================================================

float readSensorDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Protect the timing-critical measurement from Wi-Fi / ESP-NOW interrupts
  noInterrupts();
  long duration = pulseIn(echoPin, HIGH, 16000);
  interrupts();

#if DEBUG_RAW_PULSES
  Serial.printf("  [raw] trig=%d echo=%d us=%ld\n", trigPin, echoPin, duration);
#endif

  if (duration <= 0) return -1.0f;
  return duration * 0.0343f / 2.0f;
}

// =====================================================================
// Distance helpers
// =====================================================================

bool isValidDistance(float d) {
  return isfinite(d) && d > 0.0f && d <= MAX_TRACKING_DISTANCE_CM;
}

void printDistanceValue(float d) {
  if (!isValidDistance(d)) Serial.print("  ---  ");
  else { Serial.print(d, 1); Serial.print("cm"); }
}

// =====================================================================
// Geometry tables
// =====================================================================

float sensorOriginX(int s) {
  return s < SENSORS_PER_BOX ? 0.0f : BOX_BASELINE_M;
}

void sensorAxis(int s, float &ax, float &ay) {
  float rad = SENSOR_ANGLE_DEG[s] * DEG_TO_RAD;
  float dir = s < SENSORS_PER_BOX ? 1.0f : -1.0f;
  ax = dir * cosf(rad);
  ay = sinf(rad);
  if (fabsf(ax) < 1e-4f) ax = 0.0f;
}

void buildGeometryTables() {
  for (int s = 0; s < NUM_SENSORS; s++) {
    float ox = sensorOriginX(s);
    float ax, ay;
    sensorAxis(s, ax, ay);

    float t = 1e9f;
    if (ay > 1e-4f)  t = fminf(t, POSITION_MAP_DEPTH_M / ay);
    if (ax > 1e-4f)  t = fminf(t, (BOX_BASELINE_M - ox) / ax);
    else if (ax < -1e-4f) t = fminf(t, (0.0f - ox) / ax);
    sensorBoundaryCm[s] = (uint16_t)fminf(65000.0f, t * 100.0f + 0.5f);

    for (int row = 0; row < GRID_ROWS; row++) {
      for (int col = 0; col < GRID_COLS; col++) {
        int idx = row * GRID_COLS + col;
        float vx = col * GRID_STEP_M - ox;
        float vy = row * GRID_STEP_M;
        float r  = sqrtf(vx * vx + vy * vy);
        if (r < 0.001f) r = 0.001f;
        float c = (vx * ax + vy * ay) / r;
        c = fminf(1.0f, fmaxf(-1.0f, c));
        cellRangeCm[idx][s]    = (uint16_t)(r * 100.0f + 0.5f);
        cellOffAxisDeg[idx][s] = (uint8_t)(acosf(c) * RAD_TO_DEG + 0.5f);
      }
    }
  }
}

// =====================================================================
// Median filter
// =====================================================================

float medianOf(float *v, int n) {
  for (int i = 1; i < n; i++) {
    float val = v[i];
    int j = i - 1;
    while (j >= 0 && v[j] > val) { v[j + 1] = v[j]; j--; }
    v[j + 1] = val;
  }
  if (n % 2 == 0) return (v[n / 2 - 1] + v[n / 2]) * 0.5f;
  return v[n / 2];
}

// =====================================================================
// Per-sensor distance filtering
// =====================================================================

struct SensorDistanceFilterState {
  float history[NUM_SENSORS][SENSOR_HISTORY_SIZE];
  size_t historyCount[NUM_SENSORS];
  size_t historyIndex[NUM_SENSORS];
};

float gateReading(int s, float cm) {
  if (!isValidDistance(cm)) return -1.0f;
  if (cm < MIN_SENSOR_RANGE_CM) return -1.0f;
  if (cm > sensorBoundaryCm[s] + RANGE_GATE_MARGIN_CM) return -1.0f;
  return cm;
}

float filterSensorDistance(float distance, size_t s, SensorDistanceFilterState &st) {
  st.history[s][st.historyIndex[s]] = isValidDistance(distance) ? distance : -1.0f;
  st.historyIndex[s] = (st.historyIndex[s] + 1) % SENSOR_HISTORY_SIZE;
  if (st.historyCount[s] < SENSOR_HISTORY_SIZE) st.historyCount[s]++;

  if (!isValidDistance(distance)) return -1.0f;

  float samples[SENSOR_HISTORY_SIZE];
  int cnt = 0;
  for (size_t i = 0; i < st.historyCount[s]; i++) {
    if (isValidDistance(st.history[s][i])) samples[cnt++] = st.history[s][i];
  }
  if (cnt < 3) return distance;
  return medianOf(samples, cnt);
}

// =====================================================================
// Position structures
// =====================================================================

struct PlayerPosition {
  bool valid;
  float x, y;
  int support;
};

struct PositionFilterState {
  bool hasPosition;
  float smoothedX, smoothedY;
  float xHistory[POSITION_HISTORY_SIZE];
  float yHistory[POSITION_HISTORY_SIZE];
  size_t historyCount, historyIndex;
  unsigned long lastFixMs;
};

// =====================================================================
// Add / hold position
// =====================================================================

PlayerPosition addPositionSample(float x, float y, int support, PositionFilterState &st) {
  st.xHistory[st.historyIndex] = x;
  st.yHistory[st.historyIndex] = y;
  st.historyIndex = (st.historyIndex + 1) % POSITION_HISTORY_SIZE;
  if (st.historyCount < POSITION_HISTORY_SIZE) st.historyCount++;

  float xs[POSITION_HISTORY_SIZE], ys[POSITION_HISTORY_SIZE];
  for (size_t i = 0; i < st.historyCount; i++) {
    xs[i] = st.xHistory[i];
    ys[i] = st.yHistory[i];
  }
  float mx = medianOf(xs, st.historyCount);
  float my = medianOf(ys, st.historyCount);

  if (!st.hasPosition) {
    st.smoothedX = mx;
    st.smoothedY = my;
    st.hasPosition = true;
  } else {
    st.smoothedX += POSITION_SMOOTHING_ALPHA * (mx - st.smoothedX);
    st.smoothedY += POSITION_SMOOTHING_ALPHA * (my - st.smoothedY);
  }
  st.lastFixMs = millis();
  return {true, st.smoothedX, st.smoothedY, support};
}

PlayerPosition holdPosition(PositionFilterState &st) {
  if (st.hasPosition && millis() - st.lastFixMs <= POSITION_HOLD_MS)
    return {true, st.smoothedX, st.smoothedY, 0};
  st.hasPosition = false;
  st.historyCount = st.historyIndex = 0;
  return {false, -1.0f, -1.0f, 0};
}

// =====================================================================
// Beam / likelihood models
// =====================================================================

float beamSensitivity(uint8_t offAxisDeg) {
  if (offAxisDeg > BEAM_CUTOFF_DEG) return 0.0f;
  float a = (float)offAxisDeg / BEAM_SIGMA_DEG;
  return expf(-0.5f * a * a);
}

float rangeSigmaCm(float r) {
  return RANGE_SIGMA_BASE_CM + RANGE_SIGMA_PER_CM * r;
}

float sensorLogLikelihood(float reading, uint16_t cellR, uint8_t offDeg) {
  float b = beamSensitivity(offDeg);

  if (isValidDistance(reading)) {
    if (b <= 0.0f) return logf(P_OUT_OF_BEAM);
    float e = (reading - (float)cellR) / rangeSigmaCm(reading);
    float rangeWeight = expf(-0.5f * e * e);
    float pIn = P_RANGE_MISMATCH + (1.0f - P_RANGE_MISMATCH) * rangeWeight;
    return logf((1.0f - b) * P_OUT_OF_BEAM + b * pIn);
  }

  if (cellR < MIN_SENSOR_RANGE_CM) return 0.0f;
  if (b <= 0.0f) return 0.0f;
  return logf((1.0f - b) + b * P_MISS_IN_BEAM);
}

// =====================================================================
// Main position calculation
// =====================================================================

PlayerPosition calculatePlayerPosition(const float rawCm[NUM_SENSORS]) {
  static PositionFilterState filterState = {};
  static SensorDistanceFilterState distState = {};
  static bool deadZoneLatched = false;

  unsigned long startUs = micros();
  playerInDeadZone = false;
  lastSupportMask = 0;

  float f[NUM_SENSORS];
  int validCount = 0;
  for (int s = 0; s < NUM_SENSORS; s++) {
    float gated = gateReading(s, rawCm[s]);
    f[s] = filterSensorDistance(gated, s, distState);
    lastFilteredCm[s] = f[s];
    if (isValidDistance(f[s])) validCount++;
  }

  if (validCount == 0) return holdPosition(filterState);

  unsigned long now = millis();
  bool usePrior = filterState.hasPosition &&
                  (now - filterState.lastFixMs <= PRIOR_MAX_AGE_MS);
  float priorX     = filterState.smoothedX * BOX_BASELINE_M;
  float priorDepth = (1.0f - filterState.smoothedY) * POSITION_MAP_DEPTH_M;
  float invTwoSigmaSq = 1.0f / (2.0f * PRIOR_SIGMA_M * PRIOR_SIGMA_M);

  float best = -1e30f;
  int bestIdx = -1;
  for (int row = 0; row < GRID_ROWS; row++) {
    for (int col = 0; col < GRID_COLS; col++) {
      int idx = row * GRID_COLS + col;
      float score = 0.0f;
      for (int s = 0; s < NUM_SENSORS; s++) {
        score += sensorLogLikelihood(f[s], cellRangeCm[idx][s], cellOffAxisDeg[idx][s]);
      }
      if (usePrior) {
        float dx = col * GRID_STEP_M - priorX;
        float dy = row * GRID_STEP_M - priorDepth;
        score -= (dx * dx + dy * dy) * invTwoSigmaSq;
      }
      cellScore[idx] = score;
      if (score > best) { best = score; bestIdx = idx; }
    }
  }

  int support = 0;
  for (int s = 0; s < NUM_SENSORS; s++) {
    if (!isValidDistance(f[s])) continue;
    float b   = beamSensitivity(cellOffAxisDeg[bestIdx][s]);
    float err = fabsf(f[s] - (float)cellRangeCm[bestIdx][s]);
    if (b >= SUPPORT_MIN_BEAM && err <= 3.0f * rangeSigmaCm(f[s])) {
      support++;
      lastSupportMask |= (1 << s);
    }
  }

  if (support == 0) {
    lastFitMicros = micros() - startUs;
    return holdPosition(filterState);
  }

  float bestX = (bestIdx % GRID_COLS) * GRID_STEP_M;
  float bestY = (bestIdx / GRID_COLS) * GRID_STEP_M;
  float sumW = 0.0f, sumX = 0.0f, sumY = 0.0f;
  float radiusSq = PEAK_RADIUS_M * PEAK_RADIUS_M;

  for (int row = 0; row < GRID_ROWS; row++) {
    for (int col = 0; col < GRID_COLS; col++) {
      float cx = col * GRID_STEP_M;
      float cy = row * GRID_STEP_M;
      float dx = cx - bestX;
      float dy = cy - bestY;
      if (dx * dx + dy * dy > radiusSq) continue;
      float w = expf(cellScore[row * GRID_COLS + col] - best);
      sumW += w;
      sumX += w * cx;
      sumY += w * cy;
    }
  }

  float xMeters     = sumW > 0.0f ? sumX / sumW : bestX;
  float depthMeters = sumW > 0.0f ? sumY / sumW : bestY;

  if (deadZoneLatched) {
    deadZoneLatched = depthMeters <= DEAD_ZONE_RELEASE_M;
  } else {
    deadZoneLatched = depthMeters <= DEAD_ZONE_DEPTH_M;
  }
  playerInDeadZone = deadZoneLatched;

  float x = fminf(1.0f, fmaxf(0.0f, xMeters / BOX_BASELINE_M));
  float y = fminf(1.0f, fmaxf(0.0f, 1.0f - depthMeters / POSITION_MAP_DEPTH_M));

  lastFitMicros = micros() - startUs;
  return addPositionSample(x, y, support, filterState);
}

// =====================================================================
// Buzzer
// =====================================================================

void setBuzzerOutput(bool on) {
  static bool enabled = false;
  if (on == enabled) return;
  if (on) tone(BUZZER_PIN, BUZZER_FREQUENCY_HZ);
  else noTone(BUZZER_PIN);
  enabled = on;
}

// =====================================================================
// ESP-NOW receive callback
// =====================================================================

void handleSlavePacket(const uint8_t *data, int len) {
  if (len != (int)sizeof(SensorPacket)) {
    Serial.printf("ESP-NOW wrong size %d (expect %d)\n", len, sizeof(SensorPacket));
    return;
  }
  SensorPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));
  slaveSensorCm[0] = pkt.sensor1Cm;
  slaveSensorCm[1] = pkt.sensor2Cm;
  slaveSensorCm[2] = pkt.sensor3Cm;
  lastSlavePacketMs = millis();
  slaveReplyReady = true;
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataReceived(const esp_now_recv_info_t *, const uint8_t *data, int len) {
#else
void onDataReceived(const uint8_t *, const uint8_t *data, int len) {
#endif
  handleSlavePacket(data, len);
}

// =====================================================================
// Non-blocking wait
// =====================================================================

void waitMs(unsigned long ms) {
  unsigned long start = millis();
  do {
    webServer.handleClient();
    delay(1);
  } while (millis() - start < ms);
}

// =====================================================================
// Request a sample from the slave
// =====================================================================

bool requestSlaveSample() {
  slaveReplyReady = false;
  uint8_t req = SLAVE_SAMPLE_REQUEST;
  waitMs(SENSOR_SETTLE_DELAY_MS);

  if (esp_now_send(ESPNOW_BROADCAST_MAC, &req, 1) != ESP_OK) {
    Serial.println("ESP-NOW send request FAILED");
    return false;
  }

  unsigned long t0 = millis();
  while (!slaveReplyReady && millis() - t0 < SLAVE_REPLY_TIMEOUT_MS) {
    webServer.handleClient();
    delay(1);
  }
  return slaveReplyReady;
}

// =====================================================================
// Sample cycle
// =====================================================================

void buildSensorJson(char *buf, size_t bufSize) {
  float s[NUM_SENSORS];

  // Master sensors (S1, S2, S3)
  for (int i = 0; i < SENSORS_PER_BOX; i++) {
    s[i] = readSensorDistance(SENSOR_TRIG_PINS[i], SENSOR_ECHO_PINS[i]);
    waitMs(SENSOR_SETTLE_DELAY_MS);
  }

  // Slave sensors (S4, S5, S6)
  bool slaveOk = requestSlaveSample();
  for (int i = 0; i < SENSORS_PER_BOX; i++) {
    s[SENSORS_PER_BOX + i] = slaveOk ? slaveSensorCm[i] : -1.0f;
  }
  if (!slaveOk) Serial.println("!! No reply from slave");

  float closest = -1.0f;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (isValidDistance(s[i]) && (!isValidDistance(closest) || s[i] < closest))
      closest = s[i];
  }

  PlayerPosition pos = calculatePlayerPosition(s);

  setBuzzerOutput(playerInDeadZone);
  uint8_t buzzCmd = playerInDeadZone ? 1 : 0;
  esp_now_send(ESPNOW_BROADCAST_MAC, &buzzCmd, 1);

  snprintf(buf, bufSize,
    "{\"mac\":\"%s\","
    "\"sensor1\":%.2f,\"sensor2\":%.2f,\"sensor3\":%.2f,"
    "\"sensor4\":%.2f,\"sensor5\":%.2f,\"sensor6\":%.2f,"
    "\"closest\":%.2f,\"x\":%.3f,\"y\":%.3f,\"support\":%d,"
    "\"deadZone\":%s}",
    WiFi.macAddress().c_str(),
    s[0], s[1], s[2], s[3], s[4], s[5],
    closest,
    pos.valid ? pos.x : -1.0f,
    pos.valid ? pos.y : -1.0f,
    pos.valid ? pos.support : 0,
    playerInDeadZone ? "true" : "false");

  Serial.println();
  Serial.println("---------------- POSITION ----------------");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.printf("S%d%s=", i + 1, (lastSupportMask & (1 << i)) ? "*" : "");
    printDistanceValue(s[i]);
    Serial.print(i == 2 ? "\n" : "  ");
  }
  Serial.printf("x: %s | y: %s | support: %d/6 | dead: %s | fit: %lu us\n",
    pos.valid ? String(pos.x, 3).c_str() : "NaN",
    pos.valid ? String(pos.y, 3).c_str() : "NaN",
    pos.valid ? pos.support : 0,
    playerInDeadZone ? "YES" : "no",
    lastFitMicros);
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
<title>Whack-a-Mole — 6-Sensor Dashboard</title>
<style>
*{box-sizing:border-box}
body{font-family:Consolas,"Courier New",monospace;background:#111;color:#eee;margin:0;padding:24px}
h1{font-size:18px;margin:0 0 16px}
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:14px;max-width:660px}
.box{background:#1a1a1a;border:1px solid #333;border-radius:10px;padding:14px;text-align:center}
.box-label{font-size:12px;color:#888;text-transform:uppercase;letter-spacing:.05em;margin-bottom:8px}
.box-value{font-size:26px;font-weight:bold}
.box.master .box-label{color:#4aa3ff}
.box.slave .box-label{color:#5ed39a}
.box.dead-active{border-color:#ff4444;box-shadow:0 0 8px #ff4444aa}
.status{margin-top:16px;color:#888;font-size:12px}
.position-map{margin-top:18px;width:min(100%,660px);background:#1a1a1a;border:1px solid #333;border-radius:10px;padding:12px;box-sizing:border-box}
canvas{display:block;width:100%;height:auto;background:#101820}
.legend{margin-top:10px;font-size:11px;color:#888}
.legend span{display:inline-block;width:12px;height:12px;border-radius:3px;vertical-align:middle;margin-right:4px}
</style>
</head>
<body>
<h1>🕹️ Whack-a-Mole — 6-Sensor Dashboard</h1>
<div class="grid">
  <div class="box master"><div class="box-label">S1 · Master 90°</div><div class="box-value" id="s1">--</div></div>
  <div class="box master"><div class="box-label">S2 · Master 45°</div><div class="box-value" id="s2">--</div></div>
  <div class="box master"><div class="box-label">S3 · Master 70°</div><div class="box-value" id="s3">--</div></div>
  <div class="box slave"><div class="box-label">S4 · Slave 90°</div><div class="box-value" id="s4">--</div></div>
  <div class="box slave"><div class="box-label">S5 · Slave 45°</div><div class="box-value" id="s5">--</div></div>
  <div class="box slave"><div class="box-label">S6 · Slave 70°</div><div class="box-value" id="s6">--</div></div>
  <div class="box"><div class="box-label">X Position</div><div class="box-value" id="x">--</div></div>
  <div class="box"><div class="box-label">Y Position</div><div class="box-value" id="y">--</div></div>
  <div class="box"><div class="box-label">Support</div><div class="box-value" id="sup">--</div></div>
</div>
<div class="position-map"><canvas id="positionMap" width="500" height="600"></canvas></div>
<div class="legend">
  <span style="background:#4aa3ff"></span>Master beams (S1–S3)
  &nbsp;&nbsp;
  <span style="background:#5ed39a"></span>Slave beams (S4–S6)
  &nbsp;&nbsp;
  <span style="background:#ffcc33"></span>Player position
</div>
<div class="status" id="status">Loading...</div>
<div class="status" id="deadStatus"></div>

<script>
const ANGLES = [90, 45, 70, 90, 45, 70];
const BEAM_HALF_DEG = 15;
const AREA_W = 1.5;
const AREA_H = 1.8;
const BOX_COLORS = ["#4aa3ff", "#5ed39a"];

const DEAD_ZONE_CM = 55;
const ROW1_END_CM  = 100;
const ROW2_END_CM  = 140;
const COL_LEFT_CM  = 35;
const COL_RIGHT_CM = 115;

function fmt(v) { return (v == null || v < 0) ? "--" : v.toFixed(1) + " cm"; }
function fmtC(v) { return (v == null || v < 0) ? "--" : v.toFixed(3); }

function drawPosition(x, y, sensors) {
  const c = document.getElementById("positionMap");
  const ctx = c.getContext("2d");
  const W = c.width, H = c.height;
  const toPx = (xm, dm) => [xm / AREA_W * W, H * (1 - dm / AREA_H)];

  ctx.clearRect(0, 0, W, H);

  const deadY = H * (1 - (DEAD_ZONE_CM / 100) / AREA_H);
  ctx.fillStyle = "#38272c";
  ctx.fillRect(0, deadY, W, H - deadY);

  const colLeftPx  = (COL_LEFT_CM / 100) / AREA_W * W;
  const colRightPx = (COL_RIGHT_CM / 100) / AREA_W * W;
  ctx.fillStyle = "#1a1520";
  ctx.fillRect(colLeftPx, 0, colRightPx - colLeftPx, deadY);

  for (let i = 0; i < 6; i++) {
    const slave = i >= 3;
    const ox = slave ? AREA_W : 0;
    const dir = slave ? -1 : 1;
    const col = BOX_COLORS[slave ? 1 : 0];
    const a  = ANGLES[i] * Math.PI / 180;
    const a0 = (ANGLES[i] - BEAM_HALF_DEG) * Math.PI / 180;
    const a1 = (ANGLES[i] + BEAM_HALF_DEG) * Math.PI / 180;
    const L = 2.6;
    const o  = toPx(ox, 0);
    const p0 = toPx(ox + dir * Math.cos(a0) * L, Math.sin(a0) * L);
    const p1 = toPx(ox + dir * Math.cos(a1) * L, Math.sin(a1) * L);
    ctx.fillStyle = col + "1a";
    ctx.beginPath();
    ctx.moveTo(o[0], o[1]);
    ctx.lineTo(p0[0], p0[1]);
    ctx.lineTo(p1[0], p1[1]);
    ctx.closePath();
    ctx.fill();

    const r = sensors[i];
    if (r != null && r > 0) {
      const rm = r / 100;
      const pt = toPx(ox + dir * Math.cos(a) * rm, Math.sin(a) * rm);
      ctx.fillStyle = col;
      ctx.beginPath();
      ctx.arc(pt[0], pt[1], 4, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  ctx.strokeStyle = "#425466";
  ctx.lineWidth = 2;

  [COL_LEFT_CM, COL_RIGHT_CM].forEach(cm => {
    const px = (cm / 100) / AREA_W * W;
    ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, deadY); ctx.stroke();
  });

  [ROW1_END_CM, ROW2_END_CM].forEach(cm => {
    const py = H * (1 - (cm / 100) / AREA_H);
    ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(W, py); ctx.stroke();
  });

  ctx.strokeRect(0, 0, W, deadY);

  ctx.strokeStyle = "#a85b62";
  ctx.beginPath(); ctx.moveTo(0, deadY); ctx.lineTo(W, deadY); ctx.stroke();

  ctx.fillStyle = "#e3a0a4";
  ctx.font = "13px Consolas, monospace";
  ctx.fillText("55 cm DEAD ZONE", 12, deadY + (H - deadY) / 2 + 5);

  ctx.fillStyle = "#9eacb8";
  ctx.font = "11px Consolas, monospace";

  const rowLabel = (text, fromCm, toCm) => {
    const midCm = (fromCm + toCm) / 2;
    const py = H * (1 - (midCm / 100) / AREA_H);
    ctx.fillText(text, 12, py + 4);
  };
  rowLabel("NEAR: 55–100 cm",  55, 100);
  rowLabel("MID:  100–140 cm", 100, 140);
  rowLabel("FAR:  140–180 cm", 140, 180);

  if (x == null || y == null || x < 0 || y < 0) return;
  ctx.fillStyle = "#ffcc33";
  ctx.shadowColor = "#ffcc33";
  ctx.shadowBlur = 12;
  ctx.beginPath();
  ctx.arc(x * W, y * H, 12, 0, Math.PI * 2);
  ctx.fill();
  ctx.shadowBlur = 0;
}

async function poll() {
  try {
    const res = await fetch("/data", {cache: "no-store"});
    const d = await res.json();
    const sensors = [d.sensor1, d.sensor2, d.sensor3, d.sensor4, d.sensor5, d.sensor6];
    for (let i = 0; i < 6; i++) {
      document.getElementById("s" + (i + 1)).textContent = fmt(sensors[i]);
    }
    document.getElementById("x").textContent   = fmtC(d.x);
    document.getElementById("y").textContent   = fmtC(d.y);
    document.getElementById("sup").textContent = (d.x < 0) ? "--" : d.support + " / 6";

    drawPosition(d.x, d.y, sensors);

    const deadEl = document.getElementById("deadStatus");
    if (d.deadZone) {
      deadEl.textContent = "⚠️  DEAD ZONE — player too close to sensors";
      deadEl.style.color = "#ff6666";
    } else {
      deadEl.textContent = "";
    }

    document.getElementById("status").textContent =
      "Updated " + new Date().toLocaleTimeString() +
      (d.support > 0 ? "  ·  " + d.support + "/6 sensors supporting" : "");
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

void handleRoot() { webServer.send(200, "text/html", DASHBOARD_HTML); }
void handleData() { webServer.send(200, "application/json", cachedJson); }

// =====================================================================
// Sensor self-test
// =====================================================================

void runSensorSelfTest() {
  Serial.println("\n=== SENSOR SELF-TEST (Wi-Fi already up) ===");
  Serial.println("Hold hand ~20 cm in front of each sensor in turn.");
  for (int round = 0; round < 8; round++) {
    for (int i = 0; i < SENSORS_PER_BOX; i++) {
      float cm = readSensorDistance(SENSOR_TRIG_PINS[i], SENSOR_ECHO_PINS[i]);
      Serial.printf("  S%d pins %d/%d  -> ", i + 1, SENSOR_TRIG_PINS[i], SENSOR_ECHO_PINS[i]);
      if (cm < 0) Serial.println("NO ECHO");
      else Serial.printf("%.1f cm\n", cm);
      delay(80);
    }
    Serial.println();
  }
  Serial.println("=== SELF-TEST DONE ===\n");
}

// =====================================================================
// Setup
// =====================================================================

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  for (int i = 0; i < SENSORS_PER_BOX; i++) {
    pinMode(SENSOR_TRIG_PINS[i], OUTPUT);
    pinMode(SENSOR_ECHO_PINS[i], INPUT);
    digitalWrite(SENSOR_TRIG_PINS[i], LOW);
  }

  Serial.begin(115200);
  delay(1000);

  buildGeometryTables();

#if BENCH_NO_WIFI
  Serial.println("=== BENCH MODE — Wi-Fi OFF, master sensors only ===");
  while (true) {
    for (int i = 0; i < SENSORS_PER_BOX; i++) {
      float cm = readSensorDistance(SENSOR_TRIG_PINS[i], SENSOR_ECHO_PINS[i]);
      Serial.printf("S%d(%d/%d)=", i + 1, SENSOR_TRIG_PINS[i], SENSOR_ECHO_PINS[i]);
      if (cm < 0) Serial.print("  ---   ");
      else Serial.printf("%6.1fcm ", cm);
      delay(SENSOR_SETTLE_DELAY_MS);
    }
    Serial.println();
    delay(200);
  }
#endif

  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  WiFi.softAP(AP_SSID, AP_PASSWORD, FIXED_CHANNEL);

  Serial.println("\n========================================");
  Serial.println("ESP32 6-sensor Whack-a-Mole — MASTER");
  Serial.println("========================================");
  Serial.printf("Grid %dx%d, step %.0f cm, %d cells\n",
                GRID_COLS, GRID_ROWS, GRID_STEP_M * 100, GRID_CELLS);
  Serial.println("Pins: S1 32/35, S2 12/14, S3 22/23");
  Serial.printf("Angles: S1=%.0f° S2=%.0f° S3=%.0f°  S4=%.0f° S5=%.0f° S6=%.0f°\n",
                SENSOR_ANGLE_DEG[0], SENSOR_ANGLE_DEG[1], SENSOR_ANGLE_DEG[2],
                SENSOR_ANGLE_DEG[3], SENSOR_ANGLE_DEG[4], SENSOR_ANGLE_DEG[5]);
  Serial.printf("Dead zone: %.0f cm (release %.0f cm)\n",
                DEAD_ZONE_DEPTH_CM, DEAD_ZONE_RELEASE_CM);
  Serial.print("Master MAC: "); Serial.println(WiFi.macAddress());
  Serial.print("SoftAP MAC: "); Serial.println(WiFi.softAPmacAddress());
  Serial.printf("AP: %s / %s  ch %d\n", AP_SSID, AP_PASSWORD, FIXED_CHANNEL);
  Serial.print("Dashboard: http://"); Serial.print(WiFi.softAPIP()); Serial.println("/");

  udp.begin(UDP_PORT);

#if SELF_TEST_AT_BOOT
  runSensorSelfTest();
#endif

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAILED");
    return;
  }
  esp_now_register_recv_cb(onDataReceived);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, ESPNOW_BROADCAST_MAC, 6);
  peer.channel = FIXED_CHANNEL;
  peer.encrypt = false;
  peer.ifidx   = WIFI_IF_AP;
  if (esp_now_add_peer(&peer) != ESP_OK)
    Serial.println("Failed to add broadcast peer");
  else
    Serial.println("ESP-NOW broadcast peer OK");

  uint8_t ch;
  wifi_second_chan_t sec;
  esp_wifi_get_channel(&ch, &sec);
  Serial.printf("Radio actually on channel %d\n", ch);

  webServer.on("/", handleRoot);
  webServer.on("/data", handleData);
  webServer.begin();
  Serial.println("Web + ESP-NOW ready");
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
  udp.write((const uint8_t *)cachedJson, strlen(cachedJson));
  if (udp.endPacket() == 0) Serial.println("UDP send failed");

  digitalWrite(LED_PIN, LOW);
  waitMs(SENSOR_SETTLE_DELAY_MS);
}
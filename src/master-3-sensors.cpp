// master-3-sensors.cpp
//
// MASTER board, six-sensor version.
//
// Responsibilities:
//   1. Hosts its own Wi-Fi network.
//   2. Reads its own three ultrasonic sensors (S1, S2, S3).
//   3. Receives the slave's three readings (S4, S5, S6) over ESP-NOW.
//   4. Broadcasts all six readings + position as UDP JSON.
//   5. Serves the web dashboard.
//   6. Calculates player position from all six sensors.
//
// SENSOR NUMBERING AND ANGLES
//   Angles are measured from the sensor wall: 0 = along the wall toward the
//   other box, 90 = straight out into the play area.
//
//   Master box (x = 0):   S1 45 deg   S2 90 deg   S3 70 deg
//   Slave  box (x = 1.5): S4 45 deg   S5 90 deg   S6 70 deg
//
//   Both boxes use the same wiring, so the slave's first/second/third sensor
//   (TRIG 32 / 12 / 22) is reported as S4 / S5 / S6.
//
// POSITIONING ALGORITHM: "beam-aware grid fusion"
//   The old code collapsed each box into one range and intersected two
//   circles. That fails when the player is beside a box or in a back corner,
//   because the beams barely overlap there. The new approach:
//
//   - The play area is split into 5 cm cells. For every cell and every sensor
//     we precompute (once, at boot) the range from that sensor to the cell and
//     how far the cell is off the sensor's axis.
//   - Each cycle, every sensor "votes" on every cell:
//       * reading, cell inside the beam, range matches  -> strong support
//       * reading, cell inside the beam, range differs  -> strong penalty
//       * reading, cell outside the beam                -> mild penalty
//         (that sensor was probably looking at something else)
//       * no echo, cell inside the beam                 -> penalty
//         (a player there would normally have been seen)
//       * no echo, cell outside the beam                -> neutral
//   - The best-scoring cell (refined to a weighted centroid) is the position.
//   - A weak motion prior toward the previous position resolves the arc
//     ambiguity when only one sensor can see the player, so one-box recovery
//     falls out of the same maths instead of being a special case.
//   - Readings longer than the distance from a sensor to the edge of the play
//     area (plus a margin) are treated as "no echo": they are the back wall,
//     not the player.
//
//   No sensor angle is treated as an exact ray: each beam is modelled as a
//   Gaussian cone, so a player slightly outside the nominal axis still counts.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>

// 1 = print the raw trig/echo pulse for each sensor (very noisy).
#define DEBUG_RAW_PULSES 0

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
// Sensors
// ---------------------------------------------------------------------

constexpr int NUM_SENSORS = 6;
constexpr int SENSORS_PER_BOX = 3;

// Master's own sensors (global S1..S3). The slave uses the same pins.
const int SENSOR_TRIG_PINS[SENSORS_PER_BOX] = {32, 12, 22};
const int SENSOR_ECHO_PINS[SENSORS_PER_BOX] = {35, 14, 23};

const int LED_PIN = 2;
const int BUZZER_PIN = 18;
const unsigned int BUZZER_FREQUENCY_HZ = 2200;

// Gap between firing one sensor and the next, to avoid crosstalk.
const unsigned long SENSOR_SETTLE_DELAY_MS = 40;

// The slave needs ~170 ms to fire three sensors (2 x 60 ms gaps + listening),
// so the reply timeout must be well above that.
const unsigned long SLAVE_REPLY_TIMEOUT_MS = 450;

// ---------------------------------------------------------------------
// ESP-NOW
// ---------------------------------------------------------------------

const uint8_t ESPNOW_BROADCAST_MAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Must be identical to the struct in slave-3-sensors.cpp (12 bytes).
struct SensorPacket {
  float sensor1Cm;
  float sensor2Cm;
  float sensor3Cm;
};

volatile float slaveSensorCm[SENSORS_PER_BOX] = {-1.0f, -1.0f, -1.0f};
volatile unsigned long lastSlavePacketMs = 0;
volatile bool slaveReplyReady = false;

const uint8_t SLAVE_SAMPLE_REQUEST = 2;

// ---------------------------------------------------------------------
// Play area geometry
// ---------------------------------------------------------------------

constexpr float BOX_BASELINE_M = 1.5f;       // distance between the boxes
constexpr float POSITION_MAP_DEPTH_M = 1.8f; // depth of the play area

// Accept ranges long enough to see the far corners (diagonal ~2.34 m).
const float MAX_TRACKING_DISTANCE_CM = 300.0f;

// Dead zone near the sensor wall (buzzer).
const float DEAD_ZONE_DEPTH_M = 0.10f;
const float DEAD_ZONE_RELEASE_DEPTH_M = 0.15f;

// Angle of each sensor axis from the sensor wall, toward the other box.
const float SENSOR_ANGLE_DEG[NUM_SENSORS] = {
  45.0f, 90.0f, 70.0f,   // master: S1, S2, S3
  45.0f, 90.0f, 70.0f    // slave:  S4, S5, S6
};

// ---------------------------------------------------------------------
// Grid + sensor model (tuning knobs)
// ---------------------------------------------------------------------

constexpr float GRID_STEP_M = 0.05f;
constexpr int GRID_COLS = (int)(BOX_BASELINE_M / GRID_STEP_M + 0.5f) + 1;
constexpr int GRID_ROWS = (int)(POSITION_MAP_DEPTH_M / GRID_STEP_M + 0.5f) + 1;
constexpr int GRID_CELLS = GRID_COLS * GRID_ROWS;

// Beam: Gaussian sensitivity vs off-axis angle. HC-SR04 is roughly +/-15 deg,
// so sigma ~12 deg gives about half sensitivity at the nominal edge.
// Raise it if the player is being "lost" between beams; lower it if
// neighbouring sensors fight each other.
const float BEAM_SIGMA_DEG = 12.0f;
const float BEAM_CUTOFF_DEG = 40.0f;   // beyond this, sensitivity = 0

// Range noise: sigma = BASE + PER_CM * reading.
const float RANGE_SIGMA_BASE_CM = 5.0f;
const float RANGE_SIGMA_PER_CM = 0.03f;

// Probabilities used in the per-cell vote (see header comment).
const float P_OUT_OF_BEAM = 0.30f;     // reading exists but cell is outside beam
const float P_RANGE_MISMATCH = 0.02f;  // reading exists, cell in beam, wrong range
const float P_MISS_IN_BEAM = 0.25f;    // no echo although player would be in beam
                                       // (raise if steep sensors often miss)

const float MIN_SENSOR_RANGE_CM = 3.0f;

// Readings beyond (distance to play-area edge along the beam + this margin)
// are ignored as wall/background.
const float RANGE_GATE_MARGIN_CM = 25.0f;

// Motion prior toward the previous position.
const float PRIOR_SIGMA_M = 0.45f;
const unsigned long PRIOR_MAX_AGE_MS = 1500;

// Refine the best cell with a weighted centroid over this radius.
const float PEAK_RADIUS_M = 0.15f;

// A sensor "supports" the final position if the position lies in its beam
// (sensitivity >= this) and the range matches within 3 sigma.
const float SUPPORT_MIN_BEAM = 0.35f;

// If nothing supports a position, keep the last one for this long, then drop.
const unsigned long POSITION_HOLD_MS = 2500;

// ---------------------------------------------------------------------
// Smoothing
// ---------------------------------------------------------------------

const float POSITION_SMOOTHING_ALPHA = 0.70f;
const size_t POSITION_HISTORY_SIZE = 3;
const size_t SENSOR_HISTORY_SIZE = 3;

// ---------------------------------------------------------------------
// State
// ---------------------------------------------------------------------

bool playerInDeadZone = false;

WiFiUDP udp;
WebServer webServer(80);

char cachedJson[384] = "{}";

// Precomputed geometry tables (about 21 KB).
uint16_t cellRangeCm[GRID_CELLS][NUM_SENSORS];
uint8_t cellOffAxisDeg[GRID_CELLS][NUM_SENSORS];
uint16_t sensorBoundaryCm[NUM_SENSORS];
float cellScore[GRID_CELLS];

// Last-cycle diagnostics.
float lastFilteredCm[NUM_SENSORS];
uint8_t lastSupportMask = 0;
unsigned long lastFitMicros = 0;

// =====================================================================
// Waiting that keeps the dashboard responsive
// =====================================================================

void waitMs(unsigned long ms) {
  unsigned long start = millis();
  do {
    webServer.handleClient();
    delay(1);
  } while (millis() - start < ms);
}

// =====================================================================
// Sensor reading
// =====================================================================

float readSensorDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 20000);

#if DEBUG_RAW_PULSES
  Serial.printf("  [raw] trig=%d echo=%d duration_us=%ld\n", trigPin, echoPin, duration);
#endif

  return duration <= 0 ? -1.0f : duration * 0.0343f / 2.0f;
}

bool isValidDistance(float distance) {
  return isfinite(distance) && distance > 0.0f && distance <= MAX_TRACKING_DISTANCE_CM;
}

void printDistanceValue(float distance) {
  if (!isValidDistance(distance)) {
    Serial.print("  ---  ");
  } else {
    Serial.print(distance, 1);
    Serial.print("cm");
  }
}

// =====================================================================
// Geometry tables
// =====================================================================

float sensorOriginX(int s) {
  return s < SENSORS_PER_BOX ? 0.0f : BOX_BASELINE_M;
}

void sensorAxis(int s, float &ax, float &ay) {
  float rad = SENSOR_ANGLE_DEG[s] * DEG_TO_RAD;
  float dir = s < SENSORS_PER_BOX ? 1.0f : -1.0f;  // slave looks back toward master
  ax = dir * cosf(rad);
  ay = sinf(rad);
  if (fabsf(ax) < 1e-4f) ax = 0.0f;
}

void buildGeometryTables() {
  for (int s = 0; s < NUM_SENSORS; s++) {
    float ox = sensorOriginX(s);
    float ax, ay;
    sensorAxis(s, ax, ay);

    // Distance from the sensor along its axis to the edge of the play area.
    float t = 1e9f;
    if (ay > 1e-4f) t = fminf(t, POSITION_MAP_DEPTH_M / ay);
    if (ax > 1e-4f) t = fminf(t, (BOX_BASELINE_M - ox) / ax);
    else if (ax < -1e-4f) t = fminf(t, (0.0f - ox) / ax);
    sensorBoundaryCm[s] = (uint16_t)fminf(65000.0f, t * 100.0f + 0.5f);

    for (int row = 0; row < GRID_ROWS; row++) {
      for (int col = 0; col < GRID_COLS; col++) {
        int idx = row * GRID_COLS + col;
        float vx = col * GRID_STEP_M - ox;
        float vy = row * GRID_STEP_M;
        float r = sqrtf(vx * vx + vy * vy);
        if (r < 0.001f) r = 0.001f;

        float c = (vx * ax + vy * ay) / r;
        c = fminf(1.0f, fmaxf(-1.0f, c));

        cellRangeCm[idx][s] = (uint16_t)(r * 100.0f + 0.5f);
        cellOffAxisDeg[idx][s] = (uint8_t)(acosf(c) * RAD_TO_DEG + 0.5f);
      }
    }
  }
}

// =====================================================================
// Median + per-sensor filtering
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
  if (count % 2 == 0) return (values[count / 2 - 1] + values[count / 2]) * 0.5f;
  return values[count / 2];
}

struct SensorDistanceFilterState {
  float history[NUM_SENSORS][SENSOR_HISTORY_SIZE];
  size_t historyCount[NUM_SENSORS];
  size_t historyIndex[NUM_SENSORS];
};

// Drop readings that are impossible for this sensor: too close, or farther
// than the edge of the play area along its beam (that is the back wall).
float gateReading(int s, float cm) {
  if (!isValidDistance(cm)) return -1.0f;
  if (cm < MIN_SENSOR_RANGE_CM) return -1.0f;
  if (cm > sensorBoundaryCm[s] + RANGE_GATE_MARGIN_CM) return -1.0f;
  return cm;
}

float filterSensorDistance(float distance, size_t s, SensorDistanceFilterState &state) {
  state.history[s][state.historyIndex[s]] = isValidDistance(distance) ? distance : -1.0f;
  state.historyIndex[s] = (state.historyIndex[s] + 1) % SENSOR_HISTORY_SIZE;
  if (state.historyCount[s] < SENSOR_HISTORY_SIZE) state.historyCount[s]++;

  if (!isValidDistance(distance)) return -1.0f;

  float samples[SENSOR_HISTORY_SIZE];
  int sampleCount = 0;
  for (size_t i = 0; i < state.historyCount[s]; i++) {
    float sample = state.history[s][i];
    if (isValidDistance(sample)) samples[sampleCount++] = sample;
  }
  if (sampleCount < 3) return distance;
  return medianOf(samples, sampleCount);
}

// =====================================================================
// Position smoothing
// =====================================================================

struct PlayerPosition {
  bool valid;
  float x;       // 0..1 across the baseline
  float y;       // 1 at the sensor wall, 0 at the far edge
  int support;   // number of sensors that agree with this position
};

struct PositionFilterState {
  bool hasPosition;
  float smoothedX;
  float smoothedY;
  float xHistory[POSITION_HISTORY_SIZE];
  float yHistory[POSITION_HISTORY_SIZE];
  size_t historyCount;
  size_t historyIndex;
  unsigned long lastFixMs;
};

PlayerPosition addPositionSample(float x, float y, int support, PositionFilterState &state) {
  state.xHistory[state.historyIndex] = x;
  state.yHistory[state.historyIndex] = y;
  state.historyIndex = (state.historyIndex + 1) % POSITION_HISTORY_SIZE;
  if (state.historyCount < POSITION_HISTORY_SIZE) state.historyCount++;

  float xs[POSITION_HISTORY_SIZE];
  float ys[POSITION_HISTORY_SIZE];
  for (size_t i = 0; i < state.historyCount; i++) {
    xs[i] = state.xHistory[i];
    ys[i] = state.yHistory[i];
  }
  float mx = medianOf(xs, state.historyCount);
  float my = medianOf(ys, state.historyCount);

  if (!state.hasPosition) {
    state.smoothedX = mx;
    state.smoothedY = my;
    state.hasPosition = true;
  } else {
    state.smoothedX += POSITION_SMOOTHING_ALPHA * (mx - state.smoothedX);
    state.smoothedY += POSITION_SMOOTHING_ALPHA * (my - state.smoothedY);
  }
  state.lastFixMs = millis();
  return {true, state.smoothedX, state.smoothedY, support};
}

// No fresh information: keep the last position for a while, then drop it.
PlayerPosition holdPosition(PositionFilterState &state) {
  if (state.hasPosition && millis() - state.lastFixMs <= POSITION_HOLD_MS) {
    return {true, state.smoothedX, state.smoothedY, 0};
  }
  state.hasPosition = false;
  state.historyCount = 0;
  state.historyIndex = 0;
  return {false, -1.0f, -1.0f, 0};
}

// =====================================================================
// Sensor model: per-sensor, per-cell log-likelihood
// =====================================================================

float beamSensitivity(uint8_t offAxisDeg) {
  if (offAxisDeg > BEAM_CUTOFF_DEG) return 0.0f;
  float a = offAxisDeg / BEAM_SIGMA_DEG;
  return expf(-0.5f * a * a);
}

float rangeSigmaCm(float readingCm) {
  return RANGE_SIGMA_BASE_CM + RANGE_SIGMA_PER_CM * readingCm;
}

float sensorLogLikelihood(float readingCm, uint16_t cellRange, uint8_t offAxisDeg) {
  float b = beamSensitivity(offAxisDeg);

  if (isValidDistance(readingCm)) {
    if (b <= 0.0f) return logf(P_OUT_OF_BEAM);
    float e = (readingCm - (float)cellRange) / rangeSigmaCm(readingCm);
    float wr = expf(-0.5f * e * e);
    float pIn = P_RANGE_MISMATCH + (1.0f - P_RANGE_MISMATCH) * wr;
    return logf((1.0f - b) * P_OUT_OF_BEAM + b * pIn);
  }

  // No echo. A cell too close to the sensor to be measurable proves nothing.
  if (cellRange < MIN_SENSOR_RANGE_CM) return 0.0f;
  if (b <= 0.0f) return 0.0f;
  return logf((1.0f - b) + b * P_MISS_IN_BEAM);
}

// =====================================================================
// Six-sensor positioning
// =====================================================================

PlayerPosition calculatePlayerPosition(const float rawCm[NUM_SENSORS]) {
  static PositionFilterState filterState = {};
  static SensorDistanceFilterState distanceFilterState = {};
  static bool deadZoneLatched = false;

  unsigned long startUs = micros();
  playerInDeadZone = false;
  lastSupportMask = 0;

  // 1. Gate and filter each sensor.
  float f[NUM_SENSORS];
  int validCount = 0;
  for (int s = 0; s < NUM_SENSORS; s++) {
    float gated = gateReading(s, rawCm[s]);
    f[s] = filterSensorDistance(gated, s, distanceFilterState);
    lastFilteredCm[s] = f[s];
    if (isValidDistance(f[s])) validCount++;
  }

  if (validCount == 0) return holdPosition(filterState);

  // 2. Motion prior, only if we have a recent position.
  unsigned long now = millis();
  bool usePrior = filterState.hasPosition && (now - filterState.lastFixMs <= PRIOR_MAX_AGE_MS);
  float priorX = filterState.smoothedX * BOX_BASELINE_M;
  float priorDepth = (1.0f - filterState.smoothedY) * POSITION_MAP_DEPTH_M;
  float invTwoSigmaSq = 1.0f / (2.0f * PRIOR_SIGMA_M * PRIOR_SIGMA_M);

  // 3. Score every cell.
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
      if (score > best) {
        best = score;
        bestIdx = idx;
      }
    }
  }

  // 4. Does any sensor actually support the best cell?
  int support = 0;
  for (int s = 0; s < NUM_SENSORS; s++) {
    if (!isValidDistance(f[s])) continue;
    float b = beamSensitivity(cellOffAxisDeg[bestIdx][s]);
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

  // 5. Refine: weighted centroid around the best cell.
  float bestX = (bestIdx % GRID_COLS) * GRID_STEP_M;
  float bestY = (bestIdx / GRID_COLS) * GRID_STEP_M;
  float sumW = 0.0f, sumX = 0.0f, sumY = 0.0f;
  float radiusSq = PEAK_RADIUS_M * PEAK_RADIUS_M;
  for (int row = 0; row < GRID_ROWS; row++) {
    for (int col = 0; col < GRID_COLS; col++) {
      float cx = col * GRID_STEP_M;
      float cy = row * GRID_STEP_M;
      float dx = cx - bestX, dy = cy - bestY;
      if (dx * dx + dy * dy > radiusSq) continue;
      float w = expf(cellScore[row * GRID_COLS + col] - best);
      sumW += w;
      sumX += w * cx;
      sumY += w * cy;
    }
  }
  float xMeters = sumW > 0.0f ? sumX / sumW : bestX;
  float depthMeters = sumW > 0.0f ? sumY / sumW : bestY;

  // 6. Dead zone (latched, with hysteresis).
  if (deadZoneLatched) {
    deadZoneLatched = depthMeters <= DEAD_ZONE_RELEASE_DEPTH_M;
  } else {
    deadZoneLatched = depthMeters <= DEAD_ZONE_DEPTH_M;
  }
  playerInDeadZone = deadZoneLatched;

  // 7. Normalise and smooth.
  float x = fminf(1.0f, fmaxf(0.0f, xMeters / BOX_BASELINE_M));
  float y = fminf(1.0f, fmaxf(0.0f, 1.0f - depthMeters / POSITION_MAP_DEPTH_M));

  lastFitMicros = micros() - startUs;
  return addPositionSample(x, y, support, filterState);
}

// =====================================================================
// Buzzer
// =====================================================================

void setBuzzerOutput(bool enabled) {
  static bool outputEnabled = false;
  if (enabled == outputEnabled) return;
  if (enabled) tone(BUZZER_PIN, BUZZER_FREQUENCY_HZ);
  else noTone(BUZZER_PIN);
  outputEnabled = enabled;
}

// =====================================================================
// ESP-NOW receive callback
// =====================================================================

void handleSlavePacket(const uint8_t *incomingData, int length) {
  if (length != (int)sizeof(SensorPacket)) {
    Serial.print("Got ESP-NOW packet with wrong size: ");
    Serial.print(length);
    Serial.print(" (expected ");
    Serial.print(sizeof(SensorPacket));
    Serial.println(") - is the slave running slave-3-sensors.cpp?");
    return;
  }

  SensorPacket packet;
  memcpy(&packet, incomingData, sizeof(packet));

  slaveSensorCm[0] = packet.sensor1Cm;
  slaveSensorCm[1] = packet.sensor2Cm;
  slaveSensorCm[2] = packet.sensor3Cm;

  lastSlavePacketMs = millis();
  slaveReplyReady = true;
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

  waitMs(SENSOR_SETTLE_DELAY_MS);

  if (esp_now_send(ESPNOW_BROADCAST_MAC, &request, sizeof(request)) != ESP_OK) {
    return false;
  }

  unsigned long startedMs = millis();
  while (!slaveReplyReady && millis() - startedMs < SLAVE_REPLY_TIMEOUT_MS) {
    webServer.handleClient();
    delay(1);
  }
  return slaveReplyReady;
}

// =====================================================================
// Sample all six sensors, compute position, build JSON
// =====================================================================

void buildSensorJson(char *buffer, size_t bufferSize) {
  float s[NUM_SENSORS];

  // Master sensors S1..S3, one after another.
  for (int i = 0; i < SENSORS_PER_BOX; i++) {
    s[i] = readSensorDistance(SENSOR_TRIG_PINS[i], SENSOR_ECHO_PINS[i]);
    waitMs(SENSOR_SETTLE_DELAY_MS);
  }

  // Slave sensors S4..S6.
  bool slaveOk = requestSlaveSample();
  for (int i = 0; i < SENSORS_PER_BOX; i++) {
    s[SENSORS_PER_BOX + i] = slaveOk ? slaveSensorCm[i] : -1.0f;
  }
  if (!slaveOk) Serial.println("!! No reply from slave this cycle");

  // Closest raw reading (dashboard).
  float closest = -1.0f;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (isValidDistance(s[i]) && (!isValidDistance(closest) || s[i] < closest)) closest = s[i];
  }

  PlayerPosition position = calculatePlayerPosition(s);

  // Buzzer.
  setBuzzerOutput(playerInDeadZone);
  uint8_t buzzerCommand = playerInDeadZone ? 1 : 0;
  esp_now_send(ESPNOW_BROADCAST_MAC, &buzzerCommand, sizeof(buzzerCommand));

  // JSON.
  snprintf(
    buffer, bufferSize,
    "{\"mac\":\"%s\","
    "\"sensor1\":%.2f,\"sensor2\":%.2f,\"sensor3\":%.2f,"
    "\"sensor4\":%.2f,\"sensor5\":%.2f,\"sensor6\":%.2f,"
    "\"closest\":%.2f,\"x\":%.3f,\"y\":%.3f,\"support\":%d}",
    WiFi.macAddress().c_str(),
    s[0], s[1], s[2], s[3], s[4], s[5],
    closest,
    position.valid ? position.x : -1.0f,
    position.valid ? position.y : -1.0f,
    position.valid ? position.support : 0
  );

  // Serial: '*' marks sensors that agree with the final position.
  Serial.println();
  Serial.println("---------------- POSITION ----------------");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("S");
    Serial.print(i + 1);
    Serial.print((lastSupportMask & (1 << i)) ? "*=" : "=");
    printDistanceValue(s[i]);
    Serial.print(i == SENSORS_PER_BOX - 1 ? "\n" : "  ");
  }
  Serial.println();
  Serial.print("x: ");
  if (position.valid) Serial.print(position.x, 3); else Serial.print("NaN");
  Serial.print(" | y: ");
  if (position.valid) Serial.print(position.y, 3); else Serial.print("NaN");
  Serial.print(" | support: ");
  Serial.print(position.valid ? position.support : 0);
  Serial.print("/6 | dead zone: ");
  Serial.print(playerInDeadZone ? "YES" : "no");
  Serial.print(" | fit: ");
  Serial.print(lastFitMicros);
  Serial.println(" us");
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
body { font-family: Consolas, "Courier New", monospace; background: #111; color: #eee; margin: 0; padding: 24px; }
h1 { font-size: 18px; margin: 0 0 16px 0; }
.grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 14px; max-width: 620px; }
.box { background: #1a1a1a; border: 1px solid #333; border-radius: 10px; padding: 14px; text-align: center; }
.box-label { font-size: 12px; color: #888; text-transform: uppercase; letter-spacing: 0.05em; margin-bottom: 8px; }
.box-value { font-size: 26px; font-weight: bold; }
.status { margin-top: 16px; color: #888; font-size: 12px; }
.position-map { margin-top: 18px; width: min(100%, 620px); background: #1a1a1a; border: 1px solid #333; border-radius: 10px; padding: 12px; box-sizing: border-box; }
canvas { display: block; width: 100%; height: auto; background: #101820; }
</style>
</head>
<body>
<h1>ESP32 Sensor Dashboard (6 sensors)</h1>

<div class="grid">
  <div class="box"><div class="box-label">S1 &middot; master 45&deg;</div><div class="box-value" id="s1">--</div></div>
  <div class="box"><div class="box-label">S2 &middot; master 90&deg;</div><div class="box-value" id="s2">--</div></div>
  <div class="box"><div class="box-label">S3 &middot; master 70&deg;</div><div class="box-value" id="s3">--</div></div>
  <div class="box"><div class="box-label">S4 &middot; slave 45&deg;</div><div class="box-value" id="s4">--</div></div>
  <div class="box"><div class="box-label">S5 &middot; slave 90&deg;</div><div class="box-value" id="s5">--</div></div>
  <div class="box"><div class="box-label">S6 &middot; slave 70&deg;</div><div class="box-value" id="s6">--</div></div>
  <div class="box"><div class="box-label">X coordinate</div><div class="box-value" id="x">--</div></div>
  <div class="box"><div class="box-label">Y coordinate</div><div class="box-value" id="y">--</div></div>
  <div class="box"><div class="box-label">Sensors agreeing</div><div class="box-value" id="sup">--</div></div>
</div>

<div class="position-map">
  <canvas id="positionMap" width="500" height="600"></canvas>
</div>

<div class="status" id="status">Loading...</div>

<script>
// Keep these in sync with SENSOR_ANGLE_DEG in the firmware.
const ANGLES = [45, 90, 70, 45, 90, 70];
const BEAM_HALF_DEG = 15;
const AREA_W = 1.5;
const AREA_H = 1.8;
const BOX_COLORS = ["#4aa3ff", "#5ed39a"];

function fmt(v) {
  return (v === null || v < 0) ? "--" : v.toFixed(1);
}

function fmtCoordinate(v) {
  return (v === null || v < 0) ? "--" : v.toFixed(3);
}

function drawPosition(x, y, sensors) {
  const canvas = document.getElementById("positionMap");
  const ctx = canvas.getContext("2d");
  const W = canvas.width;
  const H = canvas.height;
  const toPx = (xm, dm) => [xm / AREA_W * W, H * (1 - dm / AREA_H)];

  ctx.clearRect(0, 0, W, H);

  const deadZoneDepth = 0.10;
  const row1End = 0.6;
  const row2End = 1.0;
  const depthToY = depth => H * (1 - depth / AREA_H);
  const playAreaBottom = depthToY(deadZoneDepth);
  const columnLines = [0.35 / AREA_W, 1.15 / AREA_W];

  ctx.fillStyle = "#38272c";
  ctx.fillRect(0, playAreaBottom, W, H - playAreaBottom);

  // Beam cones (faint) and the raw reading along each axis.
  for (let i = 0; i < 6; i++) {
    const slave = i >= 3;
    const ox = slave ? AREA_W : 0;
    const dir = slave ? -1 : 1;
    const col = BOX_COLORS[slave ? 1 : 0];
    const a = ANGLES[i] * Math.PI / 180;
    const a0 = (ANGLES[i] - BEAM_HALF_DEG) * Math.PI / 180;
    const a1 = (ANGLES[i] + BEAM_HALF_DEG) * Math.PI / 180;
    const L = 2.6;
    const o = toPx(ox, 0);
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
    if (r !== null && r > 0) {
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
  for (const boundary of columnLines) {
    ctx.beginPath();
    ctx.moveTo(W * boundary, 0);
    ctx.lineTo(W * boundary, playAreaBottom);
    ctx.stroke();
  }
  for (const depth of [row1End, row2End]) {
    const rowY = depthToY(depth);
    ctx.beginPath();
    ctx.moveTo(0, rowY);
    ctx.lineTo(W, rowY);
    ctx.stroke();
  }
  ctx.strokeRect(0, 0, W, playAreaBottom);

  ctx.strokeStyle = "#a85b62";
  ctx.beginPath();
  ctx.moveTo(0, playAreaBottom);
  ctx.lineTo(W, playAreaBottom);
  ctx.stroke();

  ctx.fillStyle = "#e3a0a4";
  ctx.font = "14px Consolas, monospace";
  ctx.fillText("10 cm DEAD ZONE", 12, playAreaBottom + (H - playAreaBottom) / 2 + 5);

  ctx.fillStyle = "#9eacb8";
  ctx.fillText("FAR ROW: " + Math.round(row2End * 100) + "-" + Math.round(AREA_H * 100) + " cm", 12, depthToY((row2End + AREA_H) / 2) + 5);
  ctx.fillText("MIDDLE ROW: " + Math.round(row1End * 100) + "-" + Math.round(row2End * 100) + " cm", 12, depthToY((row1End + row2End) / 2) + 5);
  ctx.fillText("NEAR ROW: 10-" + Math.round(row1End * 100) + " cm", 12, depthToY((deadZoneDepth + row1End) / 2) + 5);

  if (x === null || y === null || x < 0 || y < 0) return;

  ctx.fillStyle = "#ffcc33";
  ctx.beginPath();
  ctx.arc(x * W, y * H, 12, 0, Math.PI * 2);
  ctx.fill();
}

async function poll() {
  try {
    const res = await fetch("/data", { cache: "no-store" });
    const d = await res.json();
    const sensors = [d.sensor1, d.sensor2, d.sensor3, d.sensor4, d.sensor5, d.sensor6];

    for (let i = 0; i < 6; i++) {
      document.getElementById("s" + (i + 1)).textContent = fmt(sensors[i]);
    }
    document.getElementById("x").textContent = fmtCoordinate(d.x);
    document.getElementById("y").textContent = fmtCoordinate(d.y);
    document.getElementById("sup").textContent = (d.x < 0) ? "--" : d.support + " / 6";

    drawPosition(d.x, d.y, sensors);

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

  for (int i = 0; i < SENSORS_PER_BOX; i++) {
    pinMode(SENSOR_TRIG_PINS[i], OUTPUT);
    pinMode(SENSOR_ECHO_PINS[i], INPUT);
    digitalWrite(SENSOR_TRIG_PINS[i], LOW);
  }

  Serial.begin(115200);
  delay(1000);

  buildGeometryTables();

  // Wi-Fi AP + ESP-NOW
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  WiFi.softAP(AP_SSID, AP_PASSWORD, FIXED_CHANNEL);

  Serial.println();
  Serial.println("ESP32 triple sensor - MASTER");
  Serial.println("Position algorithm: six-sensor beam-aware grid fusion");
  Serial.printf("Grid: %d x %d cells (%d), step %.0f cm\n",
                GRID_COLS, GRID_ROWS, GRID_CELLS, GRID_STEP_M * 100.0f);
  Serial.println("Sensor layout (angle from wall):");
  for (int s = 0; s < NUM_SENSORS; s++) {
    Serial.printf("  S%d: %.0f deg, area edge along beam at %u cm\n",
                  s + 1, SENSOR_ANGLE_DEG[s], (unsigned)sensorBoundaryCm[s]);
  }
  Serial.println("Master pins: S1 32/35, S2 12/14, S3 22/23 (TRIG/ECHO)");

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

  // Let the slave's last pulse die away before the next cycle fires S1.
  waitMs(SENSOR_SETTLE_DELAY_MS);
}
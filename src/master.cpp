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
//   6. Calculates player position from all four ultrasonic readings.
//
// POSITIONING UPDATE:
//
//   - Uses two-box trilateration as the PRIMARY position estimate.
//   - Uses all four sensors individually instead of immediately collapsing
//     each box into one averaged distance.
//   - Uses the difference between the two sensors on each box as a
//     LEFT/RIGHT signal.
//   - The lateral signal is used to correct X, while the trilateration
//     remains responsible for depth.
//   - Forward/backward behaviour is therefore preserved as much as possible.
//   - One-box recovery is retained for far corners.
//   - Sensor angles are NOT treated as exact rays.
//   - Extensive serial diagnostics are included to help tune the system.
//

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>

// ---------------------------------------------------------------------
// Wi-Fi network the master hosts.
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

const uint8_t ESPNOW_BROADCAST_MAC[] = {
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

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
// Master's ultrasonic sensors
// ---------------------------------------------------------------------

const int SENSOR_1_TRIG_PIN = 32;
const int SENSOR_1_ECHO_PIN = 35;

const int SENSOR_2_TRIG_PIN = 12;
const int SENSOR_2_ECHO_PIN = 14;

const int LED_PIN = 2;

const int BUZZER_PIN = 18;
const unsigned int BUZZER_FREQUENCY_HZ = 2200;

// ---------------------------------------------------------------------
// Sensor timing
// ---------------------------------------------------------------------

const unsigned long SENSOR_SETTLE_DELAY_MS = 90;

// ---------------------------------------------------------------------
// Position configuration
// ---------------------------------------------------------------------

const float MAX_TRACKING_DISTANCE_CM = 200.0f;

// Distance between sensor boxes.
const float BOX_BASELINE_M = 1.5f;

// Small tolerance outside physical left/right bounds.
const float POSITION_X_MARGIN_M = 0.35f;

// Dead zone near sensor wall.
const float DEAD_ZONE_DEPTH_M = 0.5f;
const float DEAD_ZONE_RELEASE_DEPTH_M = 0.55f;

// Depth represented by graph.
const float POSITION_MAP_DEPTH_M = 1.8f;

const float X_GAIN = 2.0f;

// ---------------------------------------------------------------------
// Position smoothing
// ---------------------------------------------------------------------

const float POSITION_SMOOTHING_ALPHA = 0.70f;

const size_t POSITION_HISTORY_SIZE = 3;

// ---------------------------------------------------------------------
// Individual sensor filtering
// ---------------------------------------------------------------------

// Number of historical readings used by each ultrasonic sensor.
const size_t SENSOR_HISTORY_SIZE = 3;

// ---------------------------------------------------------------------
// LEFT / RIGHT POSITIONING
// ---------------------------------------------------------------------

// Minimum difference between two sensors before we consider the
// difference meaningful.
//
// Example:
//   S1 = 105 cm
//   S2 = 108 cm
//
// Difference = 3 cm.
//
// That is probably just ultrasonic noise.
//
// But:
//
//   S1 = 90 cm
//   S2 = 130 cm
//
// Difference = 40 cm.
//
// That is much more likely to contain directional information.
const float LATERAL_MIN_DIFFERENCE_CM = 10.0f;

// Difference at which the lateral signal becomes strong.
const float LATERAL_FULL_DIFFERENCE_CM = 60.0f;

// How strongly the sensor difference affects X.
//
// Start conservatively.
//
// Increase if left/right is still too weak.
// Decrease if X becomes unstable.
const float LATERAL_CORRECTION_STRENGTH = 0.55f;

// Maximum X correction in metres per update.
//
// This prevents one bad ultrasonic reading from teleporting the player.
const float MAX_LATERAL_CORRECTION_M = 0.18f;

// If the master and slave lateral estimates strongly disagree,
// reduce their influence rather than allowing them to fight.
const float LATERAL_CONFLICT_THRESHOLD = 0.45f;

// ---------------------------------------------------------------------
// Sensor mounting angles
// ---------------------------------------------------------------------
//
// These are now ONLY diagnostic information.
// They are NOT used as exact geometric rays.
//
// They can still be useful when tuning the physical sensor setup.
//

const float SENSOR_1_ANGLE_DEG = 20.0f;
const float SENSOR_2_ANGLE_DEG = 70.0f;
const float SENSOR_3_ANGLE_DEG = 20.0f;
const float SENSOR_4_ANGLE_DEG = 70.0f;

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

  long duration = pulseIn(
    echoPin,
    HIGH,
    16000
  );

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
// Position structures
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

struct SensorDistanceFilterState {

  float history[4][SENSOR_HISTORY_SIZE];

  size_t historyCount[4];
  size_t historyIndex[4];
};

// =====================================================================
// Median
// =====================================================================

float medianOf(
  float *values,
  int count
) {

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

    return (
      values[count / 2 - 1] +
      values[count / 2]
    ) * 0.5f;
  }

  return values[count / 2];
}

// =====================================================================
// Individual sensor filtering
// =====================================================================

float filterSensorDistance(
  float distance,
  size_t sensorIndex,
  SensorDistanceFilterState &state
) {

  state.history[sensorIndex][
    state.historyIndex[sensorIndex]
  ] =
    isValidDistance(distance)
      ? distance
      : -1.0f;

  state.historyIndex[sensorIndex] =
    (
      state.historyIndex[sensorIndex] + 1
    ) % SENSOR_HISTORY_SIZE;

  if (
    state.historyCount[sensorIndex]
    < SENSOR_HISTORY_SIZE
  ) {

    state.historyCount[sensorIndex]++;
  }

  if (!isValidDistance(distance)) {
    return -1.0f;
  }

  float samples[SENSOR_HISTORY_SIZE];

  int sampleCount = 0;

  for (
    size_t i = 0;
    i < state.historyCount[sensorIndex];
    i++
  ) {

    float sample =
      state.history[sensorIndex][i];

    if (isValidDistance(sample)) {

      samples[sampleCount++] = sample;
    }
  }

  if (sampleCount < 3) {

    return distance;
  }

  return medianOf(
    samples,
    sampleCount
  );
}

// =====================================================================
// Position history
// =====================================================================

PlayerPosition retainPosition(
  const PositionFilterState &state
) {

  if (!state.hasPosition) {

    return {
      false,
      -1.0f,
      -1.0f
    };
  }

  return {
    true,
    state.smoothedX,
    state.smoothedY
  };
}

// =====================================================================
// Add position sample
// =====================================================================

PlayerPosition addPositionSample(
  float x,
  float y,
  PositionFilterState &state
) {

  state.xHistory[state.historyIndex] = x;
  state.yHistory[state.historyIndex] = y;

  state.historyIndex =
    (
      state.historyIndex + 1
    ) % POSITION_HISTORY_SIZE;

  if (
    state.historyCount
    < POSITION_HISTORY_SIZE
  ) {

    state.historyCount++;
  }

  float xSamples[POSITION_HISTORY_SIZE];
  float ySamples[POSITION_HISTORY_SIZE];

  for (
    size_t i = 0;
    i < state.historyCount;
    i++
  ) {

    xSamples[i] =
      state.xHistory[i];

    ySamples[i] =
      state.yHistory[i];
  }

  float measuredX =
    medianOf(
      xSamples,
      state.historyCount
    );

  float measuredY =
    medianOf(
      ySamples,
      state.historyCount
    );

  if (!state.hasPosition) {

    state.smoothedX = measuredX;
    state.smoothedY = measuredY;

    state.hasPosition = true;

  } else {

    state.smoothedX +=
      POSITION_SMOOTHING_ALPHA *
      (
        measuredX -
        state.smoothedX
      );

    state.smoothedY +=
      POSITION_SMOOTHING_ALPHA *
      (
        measuredY -
        state.smoothedY
      );
  }

  return {
    true,
    state.smoothedX,
    state.smoothedY
  };
}

// =====================================================================
// Get previous position in metres
// =====================================================================

void getPreviousPositionMeters(
  const PositionFilterState &state,
  float &xMeters,
  float &depthMeters
) {

  if (!state.hasPosition) {

    xMeters =
      BOX_BASELINE_M * 0.5f;

    depthMeters =
      POSITION_MAP_DEPTH_M * 0.5f;

    return;
  }

  xMeters =
    state.smoothedX *
    BOX_BASELINE_M;

  depthMeters =
    (
      1.0f -
      state.smoothedY
    ) *
    POSITION_MAP_DEPTH_M;
}

// =====================================================================
// Calculate lateral signal from a sensor pair
// =====================================================================
//
// Returns:
//
//   -1 = player appears toward sensor A
//    0 = no meaningful lateral information
//   +1 = player appears toward sensor B
//
// The signal is based only on the relative distance.
//
// This deliberately does NOT assume that the sensor mounting angle
// creates an exact geometric ray.
//
// =====================================================================

float calculatePairLateralSignal(
  float sensorA,
  float sensorB
) {

  bool validA =
    isValidDistance(sensorA);

  bool validB =
    isValidDistance(sensorB);

  if (!validA && !validB) {

    return 0.0f;
  }

  if (validA && !validB) {

    // Only A sees the player.
    //
    // Treat this as a moderate directional hint.
    return -0.55f;
  }

  if (!validA && validB) {

    // Only B sees the player.
    return 0.55f;
  }

  float difference =
    sensorB - sensorA;

  float absoluteDifference =
    fabsf(difference);

  if (
    absoluteDifference
    < LATERAL_MIN_DIFFERENCE_CM
  ) {

    return 0.0f;
  }

  float strength =
    (
      absoluteDifference -
      LATERAL_MIN_DIFFERENCE_CM
    ) /
    (
      LATERAL_FULL_DIFFERENCE_CM -
      LATERAL_MIN_DIFFERENCE_CM
    );

  strength =
    fminf(
      1.0f,
      fmaxf(
        0.0f,
        strength
      )
    );

  if (difference > 0.0f) {

    return strength;

  } else {

    return -strength;
  }
}

// =====================================================================
// Convert pair lateral signal into X correction
// =====================================================================

float lateralSignalToCorrection(
  float signal
) {

  return
    signal *
    LATERAL_CORRECTION_STRENGTH *
    MAX_LATERAL_CORRECTION_M;
}

// =====================================================================
// Combine lateral information from master and slave
// =====================================================================

float calculateCombinedLateralSignal(
  float sensor1,
  float sensor2,
  float sensor3,
  float sensor4
) {

  float masterSignal =
    calculatePairLateralSignal(
      sensor1,
      sensor2
    );

  float slaveSignal =
    calculatePairLateralSignal(
      sensor3,
      sensor4
    );

  bool masterUseful =
    fabsf(masterSignal) > 0.01f;

  bool slaveUseful =
    fabsf(slaveSignal) > 0.01f;

  if (!masterUseful && !slaveUseful) {

    return 0.0f;
  }

  if (masterUseful && !slaveUseful) {

    return masterSignal;
  }

  if (!masterUseful && slaveUseful) {

    return slaveSignal;
  }

  // If both boxes agree on direction,
  // strongly trust the result.

  if (
    (masterSignal > 0.0f &&
     slaveSignal > 0.0f) ||

    (masterSignal < 0.0f &&
     slaveSignal < 0.0f)
  ) {

    return (
      masterSignal +
      slaveSignal
    ) * 0.5f;
  }

  // The boxes disagree.
  //
  // Don't let them fight the position.
  //
  // The stronger signal gets a reduced influence.

  float masterStrength =
    fabsf(masterSignal);

  float slaveStrength =
    fabsf(slaveSignal);

  if (
    masterStrength >
    slaveStrength
  ) {

    return masterSignal * 0.35f;

  } else {

    return slaveSignal * 0.35f;
  }
}

// =====================================================================
// Choose a usable range from one box
// =====================================================================
//
// This is now primarily used by one-box recovery.
//
// Normal two-box positioning does NOT depend on this.
//

float chooseBoxRange(
  float sensorA,
  float sensorB
) {

  bool validA =
    isValidDistance(sensorA);

  bool validB =
    isValidDistance(sensorB);

  if (!validA && !validB) {

    return -1.0f;
  }

  if (validA && !validB) {

    return sensorA;
  }

  if (!validA && validB) {

    return sensorB;
  }

  // Average for depth/recovery.
  return (
    sensorA +
    sensorB
  ) * 0.5f;
}

// =====================================================================
// One-box recovery
// =====================================================================
//
// Used when only one physical box can see the player.
//
// This retains the previous X and estimates depth from the available
// range. The pair's lateral difference is used as a small directional
// correction.
//
// =====================================================================

PlayerPosition calculateOneBoxRecovery(
  float sensor1,
  float sensor2,
  bool isMasterBox,
  PositionFilterState &state
) {

  if (!state.hasPosition) {

    return {
      false,
      -1.0f,
      -1.0f
    };
  }

  float previousX;
  float previousDepth;

  getPreviousPositionMeters(
    state,
    previousX,
    previousDepth
  );

  float boxX =
    isMasterBox
      ? 0.0f
      : BOX_BASELINE_M;

  float rangeCm =
    chooseBoxRange(
      sensor1,
      sensor2
    );

  if (!isValidDistance(rangeCm)) {

    return retainPosition(state);
  }

  float rangeM =
    rangeCm / 100.0f;

  // ---------------------------------------------------------------
  // Estimate depth from previous X.
  // ---------------------------------------------------------------

  float dx =
    previousX -
    boxX;

  float depthSquared =
    rangeM * rangeM -
    dx * dx;

  float recoveredDepth;

  if (depthSquared >= 0.0f) {

    recoveredDepth =
      sqrtf(depthSquared);

  } else {

    recoveredDepth =
      previousDepth;
  }

  // ---------------------------------------------------------------
  // Use sensor-pair difference as a weak X correction.
  // ---------------------------------------------------------------

  float lateralSignal =
    calculatePairLateralSignal(
      sensor1,
      sensor2
    );

  float correction =
    lateralSignalToCorrection(
      lateralSignal
    );

  // Sensor pair orientation:
  //
  // Master:
  //   negative = toward sensor 1 side
  //   positive = toward sensor 2 side
  //
  // Slave is physically mirrored across the play area, so reverse it.

  if (!isMasterBox) {

    correction *= -1.0f;
  }

  float recoveredX =
    previousX +
    correction;

  recoveredX =
    fminf(
      BOX_BASELINE_M,
      fmaxf(
        0.0f,
        recoveredX
      )
    );

  recoveredDepth =
    fminf(
      POSITION_MAP_DEPTH_M,
      fmaxf(
        0.0f,
        recoveredDepth
      )
    );

  // ---------------------------------------------------------------
  // Dead zone
  // ---------------------------------------------------------------

  static bool deadZoneLatchedRecovery = false;

  if (deadZoneLatchedRecovery) {

    deadZoneLatchedRecovery =
      recoveredDepth <=
      DEAD_ZONE_RELEASE_DEPTH_M;

  } else {

    deadZoneLatchedRecovery =
      recoveredDepth <=
      DEAD_ZONE_DEPTH_M;
  }

  playerInDeadZone =
    deadZoneLatchedRecovery;

  // ---------------------------------------------------------------
  // Convert to graph coordinates.
  // ---------------------------------------------------------------

  float x =
    fminf(
      1.0f,
      fmaxf(
        0.0f,
        recoveredX /
        BOX_BASELINE_M
      )
    );

  float y =
    1.0f -
    recoveredDepth /
    POSITION_MAP_DEPTH_M;

  state.lastRecoveryMs =
    millis();

  return addPositionSample(
    x,
    y,
    state
  );
}

// =====================================================================
// Normal two-box positioning
// =====================================================================
//
// The important change:
//
// We calculate the normal trilateration position from the individual
// sensor groups, but then ALSO inspect the individual sensor differences
// for lateral information.
//
// The lateral information is used to correct X.
//
// =====================================================================

PlayerPosition calculatePlayerPosition(
  float sensor1,
  float sensor2,
  float sensor3,
  float sensor4
) {

  static PositionFilterState filterState = {};

  static SensorDistanceFilterState
    distanceFilterState = {};

  static bool deadZoneLatched = false;

  playerInDeadZone = false;

  // -------------------------------------------------------------------
  // 1. Filter each individual ultrasonic sensor.
  // -------------------------------------------------------------------

  float f1 =
    filterSensorDistance(
      sensor1,
      0,
      distanceFilterState
    );

  float f2 =
    filterSensorDistance(
      sensor2,
      1,
      distanceFilterState
    );

  float f3 =
    filterSensorDistance(
      sensor3,
      2,
      distanceFilterState
    );

  float f4 =
    filterSensorDistance(
      sensor4,
      3,
      distanceFilterState
    );

  // -------------------------------------------------------------------
  // 2. Determine whether each box has information.
  // -------------------------------------------------------------------

  bool masterHasReading =
    isValidDistance(f1) ||
    isValidDistance(f2);

  bool slaveHasReading =
    isValidDistance(f3) ||
    isValidDistance(f4);

  // -------------------------------------------------------------------
  // 3. Normal two-box positioning.
  // -------------------------------------------------------------------

  if (
    masterHasReading &&
    slaveHasReading
  ) {

    // ---------------------------------------------------------------
    // Instead of blindly averaging the pair, use the MEDIAN/average
    // only as the depth/range representation.
    //
    // The lateral difference remains available separately.
    // ---------------------------------------------------------------

    float masterRangeCm =
      chooseBoxRange(
        f1,
        f2
      );

    float slaveRangeCm =
      chooseBoxRange(
        f3,
        f4
      );

    if (
      isValidDistance(masterRangeCm) &&
      isValidDistance(slaveRangeCm)
    ) {

      float rm =
        masterRangeCm /
        100.0f;

      float rs =
        slaveRangeCm /
        100.0f;

      float D =
        BOX_BASELINE_M;

      // -------------------------------------------------------------
      // Normal two-circle trilateration.
      // -------------------------------------------------------------

      float xMeters =
        (
          rm * rm -
          rs * rs +
          D * D
        ) /
        (
          2.0f * D
        );

      float depthSquared =
        rm * rm -
        xMeters * xMeters;

      if (
        depthSquared >=
        -0.0025f
      ) {

        if (
          depthSquared < 0.0f
        ) {

          depthSquared = 0.0f;
        }

        float depthMeters =
          sqrtf(
            depthSquared
          );

        // -----------------------------------------------------------
        // Geometric bounds.
        // -----------------------------------------------------------

        if (
          xMeters >=
          -POSITION_X_MARGIN_M &&

          xMeters <=
          D + POSITION_X_MARGIN_M &&

          depthMeters >= 0.0f &&

          depthMeters <=
          POSITION_MAP_DEPTH_M
        ) {

          // ---------------------------------------------------------
          // NEW:
          //
          // Calculate lateral signal from all four sensors.
          // ---------------------------------------------------------

          float lateralSignal =
            calculateCombinedLateralSignal(
              f1,
              f2,
              f3,
              f4
            );

          float lateralCorrection =
            lateralSignalToCorrection(
              lateralSignal
            );

          // ---------------------------------------------------------
          // Apply correction.
          //
          // This is deliberately limited.
          // ---------------------------------------------------------

          xMeters =
  D * 0.5f +
  X_GAIN * (xMeters - D * 0.5f);

          // ---------------------------------------------------------
          // Keep X inside physical area.
          // ---------------------------------------------------------

          xMeters =
            fminf(
              D,
              fmaxf(
                0.0f,
                xMeters
              )
            );

          // ---------------------------------------------------------
          // Dead zone.
          // ---------------------------------------------------------

          if (deadZoneLatched) {

            deadZoneLatched =
              depthMeters <=
              DEAD_ZONE_RELEASE_DEPTH_M;

          } else {

            deadZoneLatched =
              depthMeters <=
              DEAD_ZONE_DEPTH_M;
          }

          playerInDeadZone =
            deadZoneLatched;

          // ---------------------------------------------------------
          // Normalise X and Y.
          // ---------------------------------------------------------

          float x =
            fminf(
              1.0f,
              fmaxf(
                0.0f,
                xMeters / D
              )
            );

          float y =
            1.0f -
            depthMeters /
            POSITION_MAP_DEPTH_M;

          filterState.lastFullPositionMs =
            millis();

          // ---------------------------------------------------------
          // Diagnostics.
          // ---------------------------------------------------------

          Serial.println();

          Serial.println(
            "========== POSITION DEBUG =========="
          );

          Serial.print(
            "Filtered S1: "
          );
          printDistanceValue(f1);

          Serial.print(
            " | S2: "
          );
          printDistanceValue(f2);

          Serial.print(
            " | S3: "
          );
          printDistanceValue(f3);

          Serial.print(
            " | S4: "
          );
          printDistanceValue(f4);

          Serial.println();

          Serial.print(
            "Master difference: "
          );
          Serial.print(
            f2 - f1,
            2
          );
          Serial.println(
            " cm"
          );

          Serial.print(
            "Slave difference: "
          );
          Serial.print(
            f4 - f3,
            2
          );
          Serial.println(
            " cm"
          );

          Serial.print(
            "Lateral signal: "
          );
          Serial.println(
            lateralSignal,
            3
          );

          Serial.print(
            "Lateral correction: "
          );
          Serial.print(
            lateralCorrection,
            3
          );
          Serial.println(
            " m"
          );

          Serial.print(
            "Trilateration X before correction: "
          );

          float rawX =
            (
              rm * rm -
              rs * rs +
              D * D
            ) /
            (
              2.0f * D
            );

          Serial.println(
            rawX / D,
            3
          );

          Serial.print(
            "Final X: "
          );
          Serial.println(
            x,
            3
          );

          Serial.print(
            "Y: "
          );
          Serial.println(
            y,
            3
          );

          Serial.println(
            "===================================="
          );

          return addPositionSample(
            x,
            y,
            filterState
          );
        }
      }
    }
  }

  // -------------------------------------------------------------------
  // 4. One-box recovery.
  // -------------------------------------------------------------------

  if (
    masterHasReading &&
    !slaveHasReading
  ) {

    if (filterState.hasPosition) {

      unsigned long now =
        millis();

      if (
        now -
        filterState.lastFullPositionMs
        <=
        ONE_BOX_RECOVERY_TIMEOUT_MS
      ) {

        return calculateOneBoxRecovery(
          f1,
          f2,
          true,
          filterState
        );
      }
    }
  }

  if (
    slaveHasReading &&
    !masterHasReading
  ) {

    if (filterState.hasPosition) {

      unsigned long now =
        millis();

      if (
        now -
        filterState.lastFullPositionMs
        <=
        ONE_BOX_RECOVERY_TIMEOUT_MS
      ) {

        return calculateOneBoxRecovery(
          f3,
          f4,
          false,
          filterState
        );
      }
    }
  }

  // -------------------------------------------------------------------
  // 5. No usable information.
  // -------------------------------------------------------------------

  playerInDeadZone = false;

  return retainPosition(
    filterState
  );
}

// =====================================================================
// Buzzer
// =====================================================================

void setBuzzerOutput(
  bool enabled
) {

  static bool outputEnabled = false;

  if (
    enabled ==
    outputEnabled
  ) {

    return;
  }

  if (enabled) {

    tone(
      BUZZER_PIN,
      BUZZER_FREQUENCY_HZ
    );

  } else {

    noTone(
      BUZZER_PIN
    );
  }

  outputEnabled =
    enabled;
}

// =====================================================================
// ESP-NOW receive callback
// =====================================================================

void handleSlavePacket(
  const uint8_t *incomingData,
  int length
) {

  if (
    length !=
    sizeof(SensorPacket)
  ) {

    Serial.print(
      "Got ESP-NOW packet with wrong size: "
    );

    Serial.println(
      length
    );

    return;
  }

  SensorPacket packet;

  memcpy(
    &packet,
    incomingData,
    sizeof(packet)
  );

  slaveSensor1Cm =
    packet.sensor1Cm;

  slaveSensor2Cm =
    packet.sensor2Cm;

  lastSlavePacketMs =
    millis();

  slaveReplyReady =
    true;

  Serial.println(
    "Slave reply received by master"
  );
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

  handleSlavePacket(
    incomingData,
    length
  );
}

// =====================================================================
// Request a sample from slave
// =====================================================================

bool requestSlaveSample() {

  slaveReplyReady =
    false;

  uint8_t request =
    SLAVE_SAMPLE_REQUEST;

  delay(
    SENSOR_SETTLE_DELAY_MS
  );

  if (
    esp_now_send(
      ESPNOW_BROADCAST_MAC,
      &request,
      sizeof(request)
    ) != ESP_OK
  ) {

    return false;
  }

  unsigned long requestStartedMs =
    millis();

  while (
    !slaveReplyReady &&
    millis() -
    requestStartedMs < 250
  ) {

    delay(1);
  }

  return slaveReplyReady;
}

// =====================================================================
// JSON builder
// =====================================================================

void buildSensorJson(
  char *buffer,
  size_t bufferSize
) {

  // ---------------------------------------------------------------
  // Master sensor 1
  // ---------------------------------------------------------------

  float sensor1 =
    readSensorDistance(
      SENSOR_1_TRIG_PIN,
      SENSOR_1_ECHO_PIN
    );

  delay(
    SENSOR_SETTLE_DELAY_MS
  );

  // ---------------------------------------------------------------
  // Master sensor 2
  // ---------------------------------------------------------------

  float sensor2 =
    readSensorDistance(
      SENSOR_2_TRIG_PIN,
      SENSOR_2_ECHO_PIN
    );

  delay(
    SENSOR_SETTLE_DELAY_MS
  );

  // ---------------------------------------------------------------
  // Ask slave to fire its sensors.
  // ---------------------------------------------------------------

  bool slaveSampleReceived =
    requestSlaveSample();

  float remote1 =
    slaveSampleReceived
      ? slaveSensor1Cm
      : -1.0f;

  float remote2 =
    slaveSampleReceived
      ? slaveSensor2Cm
      : -1.0f;

  // ---------------------------------------------------------------
  // Closest raw distance for dashboard.
  // ---------------------------------------------------------------

  float closest =
    -1.0f;

  if (
    isValidDistance(sensor1)
  ) {

    closest =
      sensor1;
  }

  if (
    isValidDistance(sensor2)
  ) {

    if (
      !isValidDistance(closest) ||
      sensor2 < closest
    ) {

      closest =
        sensor2;
    }
  }

  if (
    isValidDistance(remote1)
  ) {

    if (
      !isValidDistance(closest) ||
      remote1 < closest
    ) {

      closest =
        remote1;
    }
  }

  if (
    isValidDistance(remote2)
  ) {

    if (
      !isValidDistance(closest) ||
      remote2 < closest
    ) {

      closest =
        remote2;
    }
  }

  // ---------------------------------------------------------------
  // Calculate position.
  // ---------------------------------------------------------------

  PlayerPosition position =
    calculatePlayerPosition(
      sensor1,
      sensor2,
      remote1,
      remote2
    );

  // ---------------------------------------------------------------
  // Buzzer.
  // ---------------------------------------------------------------

  setBuzzerOutput(
    playerInDeadZone
  );

  uint8_t buzzerCommand =
    playerInDeadZone
      ? 1
      : 0;

  esp_now_send(
    ESPNOW_BROADCAST_MAC,
    &buzzerCommand,
    sizeof(buzzerCommand)
  );

  // ---------------------------------------------------------------
  // JSON.
  // ---------------------------------------------------------------

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

    position.valid
      ? position.x
      : -1.0f,

    position.valid
      ? position.y
      : -1.0f
  );

  // ---------------------------------------------------------------
  // General serial diagnostics.
  // ---------------------------------------------------------------

  Serial.println();

  Serial.println(
    "---------------- POSITION ----------------"
  );

  Serial.print(
    "Sensor 1: "
  );

  printDistanceValue(
    sensor1
  );

  Serial.print(
    " | Sensor 2: "
  );

  printDistanceValue(
    sensor2
  );

  Serial.print(
    " | Sensor 3: "
  );

  printDistanceValue(
    remote1
  );

  Serial.print(
    " | Sensor 4: "
  );

  printDistanceValue(
    remote2
  );

  Serial.print(
    " | closest: "
  );

  printDistanceValue(
    closest
  );

  Serial.print(
    " | x: "
  );

  if (position.valid) {

    Serial.print(
      position.x,
      3
    );

  } else {

    Serial.print(
      "NaN"
    );
  }

  Serial.print(
    " | y: "
  );

  if (position.valid) {

    Serial.print(
      position.y,
      3
    );

  } else {

    Serial.print(
      "NaN"
    );
  }

  Serial.print(
    " | dead zone: "
  );

  Serial.println(
    playerInDeadZone
      ? "YES"
      : "no"
  );
}

// =====================================================================
// Built-in web dashboard
// =====================================================================

const char DASHBOARD_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>

<html>

<head>

<meta charset="UTF-8">

<meta
  name="viewport"
  content="width=device-width, initial-scale=1"
>

<title>Sensor Dashboard</title>

<style>

body {

  font-family:
    Consolas,
    "Courier New",
    monospace;

  background: #111;

  color: #eee;

  margin: 0;

  padding: 24px;
}

h1 {

  font-size: 18px;

  margin: 0 0 16px 0;
}

.grid {

  display: grid;

  grid-template-columns:
    repeat(3, 1fr);

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

canvas {

  display: block;

  width: 100%;

  height: auto;

  background: #101820;
}

</style>

</head>

<body>

<h1>ESP32 Sensor Dashboard</h1>

<div class="grid">

  <div class="box">

    <div class="box-label">
      Sensor 1
    </div>

    <div
      class="box-value"
      id="s1"
    >
      --
    </div>

  </div>

  <div class="box">

    <div class="box-label">
      Sensor 2
    </div>

    <div
      class="box-value"
      id="s2"
    >
      --
    </div>

  </div>

  <div class="box">

    <div class="box-label">
      Sensor 3
    </div>

    <div
      class="box-value"
      id="s3"
    >
      --
    </div>

  </div>

  <div class="box">

    <div class="box-label">
      Sensor 4
    </div>

    <div
      class="box-value"
      id="s4"
    >
      --
    </div>

  </div>

  <div class="box">

    <div class="box-label">
      X coordinate
    </div>

    <div
      class="box-value"
      id="x"
    >
      --
    </div>

  </div>

  <div class="box">

    <div class="box-label">
      Y coordinate
    </div>

    <div
      class="box-value"
      id="y"
    >
      --
    </div>

  </div>

</div>

<div class="position-map">

  <canvas
    id="positionMap"
    width="600"
    height="360"
  ></canvas>

</div>

<div
  class="status"
  id="status"
>
  Loading...
</div>

<script>

function fmt(v) {

  return (
    v === null ||
    v < 0
  )
    ? "--"
    : v.toFixed(1) + " cm";
}

function fmtCoordinate(v) {

  return (
    v === null ||
    v < 0
  )
    ? "--"
    : v.toFixed(3);
}

function drawPosition(x, y) {

  const canvas =
    document.getElementById(
      "positionMap"
    );

  const ctx =
    canvas.getContext("2d");

  ctx.clearRect(
    0,
    0,
    canvas.width,
    canvas.height
  );

  const deadZoneDepth = 0.6;

  const totalDepth = 1.8;

  const depthToY =
    depth =>
      canvas.height *
      (1 - depth / totalDepth);

  const playAreaBottom =
    depthToY(
      deadZoneDepth
    );

  const columnLines = [
    0.35 / 1.5,
    1.15 / 1.5
  ];

  ctx.fillStyle = "#38272c";

  ctx.fillRect(
    0,
    playAreaBottom,
    canvas.width,
    canvas.height -
      playAreaBottom
  );

  ctx.strokeStyle = "#425466";

  ctx.lineWidth = 2;

  for (
    const boundary
    of columnLines
  ) {

    ctx.beginPath();

    ctx.moveTo(
      canvas.width *
        boundary,
      0
    );

    ctx.lineTo(
      canvas.width *
        boundary,
      playAreaBottom
    );

    ctx.stroke();
  }

  for (
    const depth
    of [1.0, 1.4]
  ) {

    const rowY =
      depthToY(
        depth
      );

    ctx.beginPath();

    ctx.moveTo(
      0,
      rowY
    );

    ctx.lineTo(
      canvas.width,
      rowY
    );

    ctx.stroke();
  }

  ctx.strokeRect(
    0,
    0,
    canvas.width,
    playAreaBottom
  );

  ctx.strokeStyle = "#a85b62";

  ctx.beginPath();

  ctx.moveTo(
    0,
    playAreaBottom
  );

  ctx.lineTo(
    canvas.width,
    playAreaBottom
  );

  ctx.stroke();

  ctx.fillStyle = "#e3a0a4";

  ctx.font =
    "14px Consolas, monospace";

  ctx.fillText(
    "60 cm DEAD ZONE",
    12,
    playAreaBottom +
      (canvas.height -
        playAreaBottom) / 2 +
      5
  );

  ctx.fillStyle = "#9eacb8";

  ctx.fillText(
    "ROW 3: 140-180 cm",
    12,
    depthToY(1.6) + 5
  );

  ctx.fillText(
    "ROW 2: 100-140 cm",
    12,
    depthToY(1.2) + 5
  );

  ctx.fillText(
    "ROW 1: 60-100 cm",
    12,
    depthToY(0.8) + 5
  );

  if (
    x === null ||
    y === null ||
    x < 0 ||
    y < 0
  ) {

    return;
  }

  ctx.fillStyle = "#ffcc33";

  ctx.beginPath();

  ctx.arc(
    x * canvas.width,
    y * canvas.height,
    12,
    0,
    Math.PI * 2
  );

  ctx.fill();
}

async function poll() {

  try {

    const res =
      await fetch(
        "/data",
        {
          cache: "no-store"
        }
      );

    const d =
      await res.json();

    document.getElementById(
      "s1"
    ).textContent =
      fmt(d.sensor1);

    document.getElementById(
      "s2"
    ).textContent =
      fmt(d.sensor2);

    document.getElementById(
      "s3"
    ).textContent =
      fmt(d.sensor3);

    document.getElementById(
      "s4"
    ).textContent =
      fmt(d.sensor4);

    document.getElementById(
      "x"
    ).textContent =
      fmtCoordinate(d.x);

    document.getElementById(
      "y"
    ).textContent =
      fmtCoordinate(d.y);

    drawPosition(
      d.x,
      d.y
    );

    document.getElementById(
      "status"
    ).textContent =
      "Updated " +
      new Date()
        .toLocaleTimeString();

  } catch (e) {

    document.getElementById(
      "status"
    ).textContent =
      "Connection lost";
  }
}

poll();

setInterval(
  poll,
  500
);

</script>

</body>

</html>
)HTML";

// =====================================================================
// Web handlers
// =====================================================================

void handleRoot() {

  webServer.send(
    200,
    "text/html",
    DASHBOARD_HTML
  );
}

void handleData() {

  webServer.send(
    200,
    "application/json",
    cachedJson
  );
}

// =====================================================================
// Setup
// =====================================================================

void setup() {

  pinMode(
    LED_PIN,
    OUTPUT
  );

  pinMode(
    BUZZER_PIN,
    OUTPUT
  );

  noTone(
    BUZZER_PIN
  );

  pinMode(
    SENSOR_1_TRIG_PIN,
    OUTPUT
  );

  pinMode(
    SENSOR_1_ECHO_PIN,
    INPUT
  );

  pinMode(
    SENSOR_2_TRIG_PIN,
    OUTPUT
  );

  pinMode(
    SENSOR_2_ECHO_PIN,
    INPUT
  );

  digitalWrite(
    SENSOR_1_TRIG_PIN,
    LOW
  );

  digitalWrite(
    SENSOR_2_TRIG_PIN,
    LOW
  );

  Serial.begin(
    115200
  );

  delay(1000);

  // ---------------------------------------------------------------
  // Wi-Fi AP + ESP-NOW
  // ---------------------------------------------------------------

  WiFi.mode(
    WIFI_AP_STA
  );

  WiFi.setSleep(
    false
  );

  WiFi.softAP(
    AP_SSID,
    AP_PASSWORD,
    FIXED_CHANNEL
  );

  Serial.println();

  Serial.println(
    "ESP32 dual sensor - MASTER"
  );

  Serial.println(
    "Position algorithm: four-sensor lateral-aware trilateration"
  );

  Serial.println(
    "Sensor 1: TRIG=32, ECHO=35"
  );

  Serial.println(
    "Sensor 2: TRIG=12, ECHO=14"
  );

  Serial.print(
    "Master MAC Address: "
  );

  Serial.println(
    WiFi.macAddress()
  );

  Serial.print(
    "Master SoftAP MAC: "
  );

  Serial.println(
    WiFi.softAPmacAddress()
  );

  Serial.print(
    "Wi-Fi network name: "
  );

  Serial.println(
    AP_SSID
  );

  Serial.print(
    "Wi-Fi password: "
  );

  Serial.println(
    AP_PASSWORD
  );

  Serial.print(
    "Fixed channel: "
  );

  Serial.println(
    FIXED_CHANNEL
  );

  Serial.print(
    "Dashboard: http://"
  );

  Serial.print(
    WiFi.softAPIP()
  );

  Serial.println(
    "/"
  );

  Serial.println(
    "Connect your laptop's Wi-Fi to the network above."
  );

  // ---------------------------------------------------------------
  // UDP
  // ---------------------------------------------------------------

  udp.begin(
    UDP_PORT
  );

  // ---------------------------------------------------------------
  // ESP-NOW
  // ---------------------------------------------------------------

  if (
    esp_now_init() != ESP_OK
  ) {

    Serial.println(
      "ESP-NOW init failed"
    );

    return;
  }

  esp_now_register_recv_cb(
    onDataReceived
  );

  esp_now_peer_info_t broadcastPeer = {};

  memcpy(
    broadcastPeer.peer_addr,
    ESPNOW_BROADCAST_MAC,
    6
  );

  broadcastPeer.channel =
    FIXED_CHANNEL;

  broadcastPeer.encrypt =
    false;

  broadcastPeer.ifidx =
    WIFI_IF_AP;

  if (
    esp_now_add_peer(
      &broadcastPeer
    ) != ESP_OK
  ) {

    Serial.println(
      "Failed to add ESP-NOW broadcast peer"
    );

  } else {

    Serial.println(
      "ESP-NOW broadcast peer registered"
    );
  }

  uint8_t primaryChannel;

  wifi_second_chan_t secondChannel;

  esp_wifi_get_channel(
    &primaryChannel,
    &secondChannel
  );

  Serial.print(
    "Master radio actually on channel: "
  );

  Serial.println(
    primaryChannel
  );

  // ---------------------------------------------------------------
  // Web dashboard
  // ---------------------------------------------------------------

  webServer.on(
    "/",
    handleRoot
  );

  webServer.on(
    "/data",
    handleData
  );

  webServer.begin();

  Serial.println(
    "Web dashboard started"
  );

  Serial.println(
    "ESP-NOW receiver ready"
  );

  Serial.println(
    "----------------------------------------"
  );
}

// =====================================================================
// Main loop
// =====================================================================

void loop() {

  webServer.handleClient();

  digitalWrite(
    LED_PIN,
    HIGH
  );

  buildSensorJson(
    cachedJson,
    sizeof(cachedJson)
  );

  udp.beginPacket(
    BROADCAST_IP,
    UDP_PORT
  );

  udp.write(
    reinterpret_cast<
      const uint8_t *
    >(
      cachedJson
    ),
    strlen(cachedJson)
  );

  if (
    udp.endPacket() == 0
  ) {

    Serial.println(
      "UDP broadcast send failed"
    );
  }

  digitalWrite(
    LED_PIN,
    LOW
  );

  delay(20);
}
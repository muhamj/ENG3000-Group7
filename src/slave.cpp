#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>

// Confirmed working MAC of the master board (physical MAC, doesn't
// change with firmware). Update if you ever swap which physical board
// is master.
const uint8_t MASTER_MAC[] = {0x90, 0x15, 0x06, 0x73, 0x70, 0x40};

const int SENSOR_1_TRIG_PIN = 32;
const int SENSOR_1_ECHO_PIN = 35;
const int SENSOR_2_TRIG_PIN = 12;
const int SENSOR_2_ECHO_PIN = 14;
const int LED_PIN = 2;

// The master joins this same hotspot; joining an access point silently
// moves the master's radio onto whatever channel that hotspot actually
// broadcasts on, which is not necessarily channel 1 and can change
// between hotspot sessions. Instead of hardcoding that channel here and
// keeping it in sync by hand, the slave scans for this SSID at boot
// (and periodically thereafter) and reads the channel directly off the
// AP's beacon - no manual sync step required. Keep this in sync with
// WIFI_SSID in master.cpp.
const char *HOTSPOT_SSID = "Austin's Phone";

// Used only if a startup scan can't find the hotspot at all (e.g. the
// phone hotspot isn't turned on yet when the slave boots). Once a real
// scan succeeds, its result always takes over.
const uint8_t ESPNOW_FALLBACK_CHANNEL = 1;

// If sends keep failing, the hotspot's channel may have changed (e.g. it
// was restarted). Re-scan after this many consecutive failures rather
// than waiting for a reflash.
const uint8_t RESCAN_AFTER_CONSECUTIVE_FAILURES = 8;

// Don't hammer the radio with scans back-to-back if the hotspot is
// genuinely down or out of range; wait this long between scan attempts.
const unsigned long RESCAN_RETRY_INTERVAL_MS = 5000;

struct SensorPacket {
  float sensor1Cm;
  float sensor2Cm;
};

uint8_t currentEspNowChannel = ESPNOW_FALLBACK_CHANNEL;
volatile uint8_t consecutiveSendFailures = 0;
bool peerAdded = false;

float readSensorDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  return duration <= 0 ? -1.0f : duration * 0.0343f / 2.0f;
}

void onDataSent(
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  const wifi_tx_info_t *,
#else
  const uint8_t *,
#endif
  esp_now_send_status_t status
) {
  digitalWrite(LED_PIN, status == ESP_NOW_SEND_SUCCESS ? HIGH : LOW);
  if (status == ESP_NOW_SEND_SUCCESS) {
    consecutiveSendFailures = 0;
    Serial.println("ESP-NOW send OK");
  } else {
    if (consecutiveSendFailures < 255) {
      consecutiveSendFailures++;
    }
    uint8_t primaryChannel;
    wifi_second_chan_t secondChannel;
    esp_wifi_get_channel(&primaryChannel, &secondChannel);
    Serial.print("ESP-NOW send FAILED (radio currently on channel ");
    Serial.print(primaryChannel);
    Serial.print(", ");
    Serial.print(consecutiveSendFailures);
    Serial.println(" in a row)");
  }
}

// Scans for HOTSPOT_SSID and returns the Wi-Fi channel it's broadcasting
// on, or 0 if it wasn't found in this scan. Uses the synchronous scan
// (blocks briefly, typically a few hundred ms), which is fine here since
// it only runs at boot and on the rare rescan.
uint8_t scanForHotspotChannel() {
  Serial.print("Scanning for hotspot \"");
  Serial.print(HOTSPOT_SSID);
  Serial.println("\"...");

  int networkCount = WiFi.scanNetworks();
  uint8_t foundChannel = 0;

  for (int i = 0; i < networkCount; i++) {
    if (WiFi.SSID(i) == HOTSPOT_SSID) {
      foundChannel = (uint8_t)WiFi.channel(i);
      Serial.print("Found hotspot on channel ");
      Serial.println(foundChannel);
      break;
    }
  }

  WiFi.scanDelete();

  if (foundChannel == 0) {
    Serial.println("Hotspot not found in scan");
  }
  return foundChannel;
}

// Sets the radio to the given channel and (re)registers the master as an
// ESP-NOW peer on it. ESP-NOW peers are pinned to a channel at add time,
// so changing channel later means removing and re-adding the peer.
void applyEspNowChannel(uint8_t channel) {
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  currentEspNowChannel = channel;

  if (peerAdded) {
    esp_now_del_peer(MASTER_MAC);
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, MASTER_MAC, 6);
  peerInfo.channel = channel;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;
  esp_err_t addPeerResult = esp_now_add_peer(&peerInfo);
  if (addPeerResult != ESP_OK) {
    Serial.print("Failed to add master peer, err=");
    Serial.println(addPeerResult);
    peerAdded = false;
    return;
  }
  peerAdded = true;

  uint8_t primaryChannel;
  wifi_second_chan_t secondChannel;
  esp_wifi_get_channel(&primaryChannel, &secondChannel);
  Serial.print("ESP-NOW now using channel ");
  Serial.println(primaryChannel);

  consecutiveSendFailures = 0;
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
  WiFi.disconnect();  // make sure we are not associated to anything
  WiFi.setSleep(false); // keep the radio awake to hear the master's ACK

  Serial.println("ESP32 dual sensor - SLAVE (ESP-NOW only, not joining Wi-Fi)");
  Serial.print("Slave MAC address: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_send_cb(onDataSent);

  // Retry the scan a few times at boot in case the hotspot hasn't
  // finished starting up yet, then fall back to the hardcoded channel
  // rather than blocking forever.
  uint8_t scannedChannel = 0;
  for (int attempt = 0; attempt < 3 && scannedChannel == 0; attempt++) {
    scannedChannel = scanForHotspotChannel();
    if (scannedChannel == 0) {
      delay(1000);
    }
  }

  if (scannedChannel != 0) {
    applyEspNowChannel(scannedChannel);
  } else {
    Serial.println("Falling back to hardcoded channel; will keep retrying in the background");
    applyEspNowChannel(ESPNOW_FALLBACK_CHANNEL);
  }

  Serial.println("Slave ready; sending sensor data over ESP-NOW");
}

void loop() {
  static unsigned long lastRescanAttempt = 0;

  // If sends have been failing repeatedly, the hotspot's channel likely
  // changed (e.g. it was restarted) - re-scan and re-pin ESP-NOW to
  // wherever it is now, instead of requiring a manual reflash.
  if (consecutiveSendFailures >= RESCAN_AFTER_CONSECUTIVE_FAILURES &&
      millis() - lastRescanAttempt >= RESCAN_RETRY_INTERVAL_MS) {
    lastRescanAttempt = millis();
    uint8_t scannedChannel = scanForHotspotChannel();
    if (scannedChannel != 0 && scannedChannel != currentEspNowChannel) {
      applyEspNowChannel(scannedChannel);
    } else if (scannedChannel == 0) {
      // Hotspot still not visible; reset the failure count so we wait a
      // full RESCAN_RETRY_INTERVAL_MS before trying again rather than
      // rescanning on every loop iteration.
      consecutiveSendFailures = 0;
    }
  }

  SensorPacket packet;
  packet.sensor1Cm = readSensorDistance(SENSOR_1_TRIG_PIN, SENSOR_1_ECHO_PIN);
  delay(60);
  packet.sensor2Cm = readSensorDistance(SENSOR_2_TRIG_PIN, SENSOR_2_ECHO_PIN);
  esp_now_send(MASTER_MAC, reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
  delay(100);
}
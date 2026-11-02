// =============================================================================
//  6slave.cpp  -  Whack-a-Mole 6-sensor positioning system  (SLAVE board)
//
//  Roles : ESP-NOW only (STA mode on channel 1, no AP connection), 3 local
//          HC-SR04 sensors, buzzer.
//   - Request byte 2  -> measure 3 sensors sequentially, reply with 12 bytes
//                        (3 x float, centimetres, -1 = no valid reading).
//   - Command byte 1/0 -> buzzer on/off (failsafe: off if master goes silent).
//  Works on Arduino-ESP32 core 2.x and 3.x (classic ESP32).
// =============================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>
#include <math.h>

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
#define CORE_V3 1
#else
#define CORE_V3 0
#endif

static const uint8_t  WIFI_CHANNEL = 1;

static const uint8_t LED_PIN    = 2;
static const uint8_t BUZZER_PIN = 18;
static const uint8_t TRIG_PIN[3] = {12, 32, 22};
static const uint8_t ECHO_PIN[3] = {14, 35, 23};
static const uint16_t BUZZER_HZ  = 2200;

static const uint32_t SETTLE_MS         = 60;
static const uint32_t PULSE_TIMEOUT_US  = 30000;
static const uint32_t BUZZ_FAILSAFE_MS  = 2000;
static const float    MIN_VALID_CM      = 3.0f;
static const float    MAX_VALID_CM      = 400.0f;

static const uint8_t CMD_BUZZ_OFF = 0;
static const uint8_t CMD_BUZZ_ON  = 1;
static const uint8_t CMD_SAMPLE   = 2;

static portMUX_TYPE sensorMux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE rxMux     = portMUX_INITIALIZER_UNLOCKED;

static volatile bool     g_reqPending = false;
static uint8_t           g_reqMac[6];
static volatile bool     g_buzzWanted = false;
static volatile uint32_t g_lastCmdMs  = 0;

static uint8_t peerMac[6];
static bool    havePeer  = false;
static bool    buzzerOn  = false;

// ---------------------------------------------------------------------------
static void setBuzzer(bool on) {
  if (on == buzzerOn) return;
  buzzerOn = on;
#if CORE_V3
  ledcWriteTone(BUZZER_PIN, on ? BUZZER_HZ : 0);
#else
  ledcWriteTone(0, on ? BUZZER_HZ : 0);
#endif
}

static void applyBuzzer() {
  bool want = g_buzzWanted && (millis() - g_lastCmdMs < BUZZ_FAILSAFE_MS);
  setBuzzer(want);
}

static void waitMs(uint32_t ms) {
  uint32_t t0 = millis();
  do { applyBuzzer(); delay(1); } while (millis() - t0 < ms);
}

static float readSensorCm(uint8_t i) {
  const uint8_t trig = TRIG_PIN[i], echo = ECHO_PIN[i];
  digitalWrite(trig, LOW);
  delayMicroseconds(5);
  unsigned long us;
  portENTER_CRITICAL(&sensorMux);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  us = pulseIn(echo, HIGH, PULSE_TIMEOUT_US);
  portEXIT_CRITICAL(&sensorMux);
  if (us == 0) return -1.0f;
  float cm = (float)us * 0.01715f;
  if (cm < MIN_VALID_CM || cm > MAX_VALID_CM) return -1.0f;
  return cm;
}

// ---------------------------------------------------------------------------
static void handleRx(const uint8_t* mac, const uint8_t* data, int len) {
  if (len != 1 || !data || !mac) return;
  uint8_t c = data[0];
  if (c == CMD_SAMPLE) {
    portENTER_CRITICAL(&rxMux);
    memcpy(g_reqMac, mac, 6);
    g_reqPending = true;
    portEXIT_CRITICAL(&rxMux);
  } else if (c == CMD_BUZZ_ON || c == CMD_BUZZ_OFF) {
    g_buzzWanted = (c == CMD_BUZZ_ON);
    g_lastCmdMs = millis();
  }
}
#if CORE_V3
static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (info) handleRx(info->src_addr, data, len);
}
#else
static void onRecv(const uint8_t* mac, const uint8_t* data, int len) { handleRx(mac, data, len); }
#endif

static bool ensurePeer(const uint8_t* mac) {
  if (havePeer && memcmp(mac, peerMac, 6) == 0 && esp_now_is_peer_exist(mac)) return true;
  if (havePeer && esp_now_is_peer_exist(peerMac)) esp_now_del_peer(peerMac);
  esp_now_peer_info_t p;
  memset(&p, 0, sizeof(p));
  memcpy(p.peer_addr, mac, 6);
  p.channel = 0;
  p.ifidx   = WIFI_IF_STA;
  p.encrypt = false;
  if (esp_now_is_peer_exist(mac) || esp_now_add_peer(&p) == ESP_OK) {
    memcpy(peerMac, mac, 6);
    havePeer = true;
    return true;
  }
  havePeer = false;
  return false;
}

// ---------------------------------------------------------------------------
static void doSample() {
  uint8_t mac[6];
  portENTER_CRITICAL(&rxMux);
  memcpy(mac, g_reqMac, 6);
  g_reqPending = false;
  portEXIT_CRITICAL(&rxMux);

  float v[3];
  for (int i = 0; i < 3; i++) {
    v[i] = readSensorCm(i);
    if (i < 2) waitMs(SETTLE_MS);   // master waits out the final settle itself
  }

  if (!ensurePeer(mac)) { Serial.println("[NOW] peer add failed"); return; }
  esp_err_t e = esp_now_send(mac, (const uint8_t*)v, sizeof(v));   // 12 bytes
  if (e != ESP_OK) {
    delay(5);
    e = esp_now_send(mac, (const uint8_t*)v, sizeof(v));
  }
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  Serial.printf("sample %.1f %.1f %.1f -> %s\n", v[0], v[1], v[2], e == ESP_OK ? "sent" : "SEND FAIL");
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Whack-a-Mole SLAVE ===");

  pinMode(LED_PIN, OUTPUT);
  for (int i = 0; i < 3; i++) {
    pinMode(TRIG_PIN[i], OUTPUT);
    digitalWrite(TRIG_PIN[i], LOW);
    pinMode(ECHO_PIN[i], INPUT);
  }
#if CORE_V3
  ledcAttach(BUZZER_PIN, BUZZER_HZ, 8);
  ledcWriteTone(BUZZER_PIN, 0);
#else
  ledcSetup(0, BUZZER_HZ, 8);
  ledcAttachPin(BUZZER_PIN, 0);
  ledcWriteTone(0, 0);
#endif

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed, restarting");
    delay(1000);
    ESP.restart();
  }
  esp_now_register_recv_cb(onRecv);
  Serial.println("Ready, waiting for master requests");
}

void loop() {
  if (g_reqPending) doSample();
  applyBuzzer();
  delay(1);
}
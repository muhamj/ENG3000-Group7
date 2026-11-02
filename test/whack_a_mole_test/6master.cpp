// =============================================================================
//  6master.cpp  -  Whack-a-Mole 6-sensor positioning system  (MASTER board)
//
//  Roles : Wi-Fi soft-AP + ESP-NOW + web dashboard + UDP stream + 3 local
//          HC-SR04 sensors + sensor fusion + buzzer.
//  Works on Arduino-ESP32 core 2.x and 3.x (classic ESP32).
//
//  Units used everywhere on the wire / dashboard:
//      sensorN, x, y, closest  ->  centimetres   (-1 = no valid reading / none)
//  Coordinates: origin (0,0) = Master sensor cluster, +x towards the Slave box
//  (x = 150 cm at the Slave), +y straight out into the play area (0..180 cm).
//
//  UDP/HTTP JSON:
//   {"sensor1":..,...,"sensor6":..,"x":..,"y":..,"support":n,"closest":..,
//    "present":0|1,"dead":0|1,"slave":0|1,"uptime":ms}
// =============================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>
#include <string.h>

// ----------------------------- build options --------------------------------
#define BOOT_PROBE         1   // bisect boot: runs your sensor-pin-test logic after each setup step
#define BOOT_SELFTEST      0   // fire each local sensor 8x at boot (Wi-Fi already up)
#define PARALLEL_SAMPLING  0   // 0 = sequential (no cross-talk, safest), 1 = master+slave ping together

#define VERBOSE_SERIAL     1   // 1 = labelled multi-line report every cycle, 0 = one compact line

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
#define CORE_V3 1
#else
#define CORE_V3 0
#endif

// ------------------------------- network ------------------------------------
static const char*    AP_SSID    = "WhackAMole-Sensors";
static const char*    AP_PASS    = "molemole1";
static const uint8_t  AP_CHANNEL = 1;
static const uint16_t UDP_PORT   = 4210;

// -------------------------------- pins --------------------------------------
static const uint8_t LED_PIN    = 2;
static const uint8_t BUZZER_PIN = 18;
static const uint8_t TRIG_PIN[3] = {12, 32, 22};
static const uint8_t ECHO_PIN[3] = {14, 35, 23};
static const uint16_t BUZZER_HZ  = 2200;

// ------------------------------ timing --------------------------------------
static const uint32_t SETTLE_MS         = 60;     // between pings
static const uint32_t PULSE_TIMEOUT_US  = 30000;  // ~5 m
static const uint32_t REPLY_TIMEOUT_MS  = 600;    // wait for slave packet
static const uint32_t EDGE_WAIT_US      = 30000;  // same patience as the original pulseIn(30 ms) version
static const float    MIN_VALID_CM      = 3.0f;
static const float    MAX_VALID_CM      = 400.0f;

// ------------------------------ protocol ------------------------------------
static const uint8_t CMD_BUZZ_OFF = 0;
static const uint8_t CMD_BUZZ_ON  = 1;
static const uint8_t CMD_SAMPLE   = 2;

// ------------------------------ geometry ------------------------------------
static const float FIELD_W = 1.50f;   // m, baseline between the boxes
static const float FIELD_D = 1.80f;   // m, play depth
static const float CELL    = 0.05f;   // m, grid resolution
static const int   NX = (int)(FIELD_W / CELL + 0.5f) + 1;   // 31
static const int   NY = (int)(FIELD_D / CELL + 0.5f) + 1;   // 37
static const int   NCELL_MAX = 31 * 37;
static const float SENSOR_ANGLE_DEG[6] = {90, 45, 70, 90, 45, 70};

// ---------------------------- fusion tuning ---------------------------------
static const float BEAM_SIGMA_DEG  = 11.0f;  // Gaussian cone width
static const float RANGE_SIGMA     = 0.07f;  // m
static const float PLAYER_RADIUS   = 0.06f;  // m, sensor sees front surface of the body
static const float CLUTTER         = 0.03f;
static const float P_BG            = 0.12f;  // reading explained by something off-beam
static const float P_MISS          = 0.20f;  // in-beam cell but sensor saw nothing
static const float MAX_USEFUL_CM   = 240.0f; // beyond this = "nothing in the field"
static const float MAX_RANGE_M     = 2.40f;
static const float MOTION_SIGMA    = 0.30f;  // m
static const float PRIOR_FLOOR     = 0.15f;
static const float SMOOTH_ALPHA    = 0.55f;
static const int   COAST_MAX       = 5;      // cycles to hold position with no support
static const float SUPPORT_ANGLE   = 0.30f;  // rad
static const float SUPPORT_RANGE   = 0.12f;  // m
static const int   MIN_SUPPORT_START = 2;    // sensors that must agree to START tracking (1 = debug only)

// ------------------------------ dead zone -----------------------------------
static const float    DEAD_CM          = 10.0f;
static const float    DEAD_RELEASE_CM  = 15.0f;
static const uint32_t DEAD_MIN_HOLD_MS = 400;

// ------------------------------ globals -------------------------------------
static WebServer server(80);
static WiFiUDP   udp;
static portMUX_TYPE sensorMux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE rxMux     = portMUX_INITIALIZER_UNLOCKED;

static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint8_t slaveMac[6];
static bool    haveSlave = false;

static volatile bool g_rxFlag = false;
static float         g_rxVals[3];
static uint8_t       g_rxMac[6];

static char jsonBuf[360] = "{}";

// grid tables
static uint16_t g_range_mm[6][NCELL_MAX];
static uint8_t  g_beam[6][NCELL_MAX];
static float    g_post[NCELL_MAX];
static float    g_ox[6], g_dx[6], g_dy[6];

// filter / tracking state
static float   s_hist[6][3];
static uint8_t s_hidx[6];
struct Track { bool active; float x, y; int miss; };
static Track trk = {false, 0, 0, 0};
static bool     deadLatched = false;
static uint32_t deadSince = 0;
static int      deadReleaseCnt = 0;
static bool     slaveOk = false;
static int      slaveMissed = 0;
static bool     buzzerOn = false;

// =============================================================================
//  Dashboard
// =============================================================================
static const char PAGE[] PROGMEM = R"HTML(<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Whack-a-Mole Tracker</title>
<style>
:root{color-scheme:dark light;--bg:#10141a;--fg:#e6edf3;--mut:#8b98a5;--card:#1a212b;--ok:#3fb950;--bad:#f85149;--acc:#58a6ff}
@media(prefers-color-scheme:light){:root{--bg:#f4f6f8;--fg:#1c2329;--mut:#5d6b78;--card:#fff}}
body{margin:0;padding:12px;font:15px system-ui,sans-serif;background:var(--bg);color:var(--fg)}
h1{font-size:18px;margin:0 0 10px}
.wrap{display:flex;flex-wrap:wrap;gap:14px;align-items:flex-start}
canvas{max-width:100%;height:auto;border-radius:10px;background:#0b0f14}
.side{flex:1;min-width:230px;display:flex;flex-direction:column;gap:10px}
.card{background:var(--card);border-radius:10px;padding:10px 12px}
.row{display:flex;justify-content:space-between;padding:2px 0}.row span:first-child{color:var(--mut)}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:4px 12px}
.bad{color:var(--bad)}.ok{color:var(--ok)}.mut{color:var(--mut)}
#dead{display:none;background:var(--bad);color:#fff;text-align:center;font-weight:700;padding:8px;border-radius:10px}
</style></head><body>
<h1>Whack-a-Mole Tracker <span id="link" class="mut">connecting...</span></h1>
<div class="wrap"><canvas id="c"></canvas><div class="side">
<div id="dead">DEAD ZONE - BUZZER ACTIVE</div>
<div class="card"><div class="row"><span>Player</span><b id="pres">-</b></div>
<div class="row"><span>X / Y (cm)</span><b id="xy">-</b></div>
<div class="row"><span>Support</span><b id="sup">-</b></div>
<div class="row"><span>Closest (cm)</span><b id="clo">-</b></div>
<div class="row"><span>Slave link</span><b id="sl">-</b></div>
<div class="row"><span>Uptime</span><b id="up">-</b></div></div>
<div class="card"><div class="grid" id="sens"></div></div>
</div></div>
<script>
const $=id=>document.getElementById(id);
const W=150,D=180,S=2.4,P=24,ANG=[90,45,70,90,45,70];
const cv=$('c'),g=cv.getContext('2d');cv.width=W*S+P*2;cv.height=D*S+P*2;
const X=x=>P+x*S,Y=y=>cv.height-P-y*S,R=a=>a*Math.PI/180;
let trail=[],busy=false,fails=0;
function draw(d){
 g.clearRect(0,0,cv.width,cv.height);
 g.fillStyle='#121a24';g.fillRect(X(0),Y(D),W*S,D*S);
 g.fillStyle='rgba(248,81,73,.25)';g.fillRect(X(0),Y(10),W*S,10*S);
 g.strokeStyle='rgba(139,152,165,.18)';g.lineWidth=1;
 for(let x=0;x<=W;x+=30){g.beginPath();g.moveTo(X(x),Y(0));g.lineTo(X(x),Y(D));g.stroke();}
 for(let y=0;y<=D;y+=30){g.beginPath();g.moveTo(X(0),Y(y));g.lineTo(X(W),Y(y));g.stroke();}
 g.strokeStyle='#58a6ff';g.lineWidth=2;g.strokeRect(X(0),Y(D),W*S,D*S);
 for(let s=0;s<6;s++){
  const ox=s<3?0:W,a=s<3?ANG[s]:180-ANG[s],v=d['sensor'+(s+1)];
  g.strokeStyle='rgba(88,166,255,.25)';g.lineWidth=1;
  for(const da of[-15,15]){g.beginPath();g.moveTo(X(ox),Y(0));
   g.lineTo(X(ox+260*Math.cos(R(a+da))),Y(260*Math.sin(R(a+da))));g.stroke();}
  if(v>0){g.strokeStyle='#d29922';g.lineWidth=3;g.beginPath();
   g.arc(X(ox),Y(0),v*S,-R(a+15),-R(a-15));g.stroke();}
  g.fillStyle='#58a6ff';g.beginPath();g.arc(X(ox),Y(0),5,0,7);g.fill();
 }
 if(d.present){
  trail.push([d.x,d.y]);if(trail.length>25)trail.shift();
  trail.forEach((p,i)=>{g.fillStyle='rgba(63,185,80,'+(i/trail.length*.5)+')';
   g.beginPath();g.arc(X(p[0]),Y(p[1]),4,0,7);g.fill();});
  g.fillStyle=d.dead?'#f85149':'#3fb950';g.beginPath();g.arc(X(d.x),Y(d.y),11,0,7);g.fill();
 }else trail=[];
}
function render(d){
 $('link').textContent='live';$('link').className='ok';
 $('pres').textContent=d.present?'tracked':'none';
 $('xy').textContent=d.present?d.x.toFixed(1)+' / '+d.y.toFixed(1):'-';
 $('sup').textContent=d.support;$('clo').textContent=d.closest>0?d.closest.toFixed(1):'-';
 $('sl').textContent=d.slave?'OK':'LOST';$('sl').className=d.slave?'ok':'bad';
 $('up').textContent=Math.floor(d.uptime/1000)+' s';
 $('dead').style.display=d.dead?'block':'none';
 let h='';for(let i=1;i<=6;i++){const v=d['sensor'+i];
  h+='<div class="row" style="grid-column:span 2"><span>S'+i+(i<4?' (M)':' (S)')+'</span><b class="'+(v>0?'':'mut')+'">'+(v>0?v.toFixed(1)+' cm':'--')+'</b></div>';}
 $('sens').innerHTML=h;draw(d);
}
async function tick(){if(busy)return;busy=true;
 try{const r=await fetch('/data',{cache:'no-store'});render(await r.json());fails=0;}
 catch(e){if(++fails>3){$('link').textContent='offline';$('link').className='bad';}}
 busy=false;}
setInterval(tick,250);tick();
</script></body></html>)HTML";

// =============================================================================
//  Helpers
// =============================================================================
static void serviceWait(uint32_t ms) {
  uint32_t t0 = millis();
  do { server.handleClient(); delay(1); } while (millis() - t0 < ms);
}

static void setBuzzer(bool on) {
  buzzerOn = on;
#if CORE_V3
  ledcWriteTone(BUZZER_PIN, on ? BUZZER_HZ : 0);
#else
  ledcWriteTone(0, on ? BUZZER_HZ : 0);
#endif
}

// Reading codes: >0 = distance in cm, otherwise a negative reason code:
//   -1 = no echo (sensor works, nothing in range)
//   -2 = no pulse at all (nothing in range for modules that stay silent, or wiring)
//   -3 = out of range (<MIN_VALID_CM or >MAX_VALID_CM)
static const float SENSOR_ANGLE_DEG_LBL[6] = {90, 45, 70, 90, 45, 70};

static int whyCode(float v) {
  if (v > 0) return 0;
  if (v <= -2.5f) return -3;
  if (v <= -1.5f) return -2;
  return -1;
}

static void printSensorLine(int s, float v, bool offline) {
  Serial.printf("  S%d %s %2.0f deg  T%02d/E%02d : ", s + 1, s < 3 ? "Master" : "Slave ",
                SENSOR_ANGLE_DEG_LBL[s], TRIG_PIN[s % 3], ECHO_PIN[s % 3]);
  if (offline)             Serial.println("-- slave offline (no reply)");
  else if (v > 0)          Serial.printf("%6.1f cm\n", v);
  else switch (whyCode(v)) {
    case -2: Serial.println("-- no pulse at all (nothing in range - some modules stay silent then; hold a hand 20-50 cm in front to test)"); break;
    case -3: Serial.println("-- out of range (<3 cm or >400 cm)"); break;
    default: Serial.println("-- no echo (nothing in range)"); break;
  }
}

// ---- single HC-SR04 measurement, timing-critical part with interrupts off ----
// Returns distance in cm (>0) or a negative reason code (see above).
static float readSensorCm(uint8_t i) {
  const uint8_t trig = TRIG_PIN[i], echo = ECHO_PIN[i];
  digitalWrite(trig, LOW);
  delayMicroseconds(5);
  bool edge = false, timedOut = false;
  uint32_t width = 0;
  portENTER_CRITICAL(&sensorMux);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  uint32_t t0 = micros();
  while (!digitalRead(echo)) {
    if ((uint32_t)(micros() - t0) > EDGE_WAIT_US) break;
  }
  if (digitalRead(echo)) {
    edge = true;
    uint32_t ts = micros();
    while (digitalRead(echo)) {
      if ((uint32_t)(micros() - ts) > PULSE_TIMEOUT_US) { timedOut = true; break; }
    }
    width = micros() - ts;
  }
  portEXIT_CRITICAL(&sensorMux);
  if (!edge)     return -2.0f;
  if (timedOut)  return -1.0f;
  float cm = (float)width * 0.01715f;
  if (cm < MIN_VALID_CM || cm > MAX_VALID_CM) return -3.0f;
  return cm;
}

// =============================================================================
//  ESP-NOW
// =============================================================================
static void handleRx(const uint8_t* mac, const uint8_t* data, int len) {
  if (len != 12 || !mac || !data) return;
  portENTER_CRITICAL(&rxMux);
  memcpy((void*)g_rxVals, data, 12);
  memcpy(g_rxMac, mac, 6);
  g_rxFlag = true;
  portEXIT_CRITICAL(&rxMux);
}
#if CORE_V3
static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (info) handleRx(info->src_addr, data, len);
}
#else
static void onRecv(const uint8_t* mac, const uint8_t* data, int len) { handleRx(mac, data, len); }
#endif

static void addPeer(const uint8_t* mac) {
  if (esp_now_is_peer_exist(mac)) return;
  esp_now_peer_info_t p;
  memset(&p, 0, sizeof(p));
  memcpy(p.peer_addr, mac, 6);
  p.channel = 0;               // follow current channel
  p.ifidx   = WIFI_IF_AP;
  p.encrypt = false;
  esp_err_t e = esp_now_add_peer(&p);
  if (e != ESP_OK) Serial.printf("[NOW] add_peer failed: %d\n", (int)e);
}

static void sendToSlave(uint8_t cmd) {
  if (haveSlave) {
    if (esp_now_send(slaveMac, &cmd, 1) == ESP_OK) return;
  }
  esp_now_send(BCAST, &cmd, 1);
}

static void clearReply() {
  portENTER_CRITICAL(&rxMux);
  g_rxFlag = false;
  portEXIT_CRITICAL(&rxMux);
}

static bool takeReply(float out[3]) {
  bool got = false;
  uint8_t mac[6];
  portENTER_CRITICAL(&rxMux);
  if (g_rxFlag) {
    memcpy(out, (const void*)g_rxVals, 12);
    memcpy(mac, g_rxMac, 6);
    g_rxFlag = false;
    got = true;
  }
  portEXIT_CRITICAL(&rxMux);
  if (got) {
    if (!haveSlave || memcmp(mac, slaveMac, 6) != 0) {
      memcpy(slaveMac, mac, 6);
      haveSlave = true;
      addPeer(slaveMac);
      Serial.printf("[NOW] slave %02X:%02X:%02X:%02X:%02X:%02X\n",
                    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    for (int i = 0; i < 3; i++) {
      if (!isfinite(out[i])) out[i] = -1.0f;
      else if (out[i] > 0 && (out[i] < MIN_VALID_CM || out[i] > MAX_VALID_CM)) out[i] = -3.0f;
      else if (out[i] <= 0) out[i] = (float)whyCode(out[i]);   // keep reason code
    }
  }
  return got;
}

// =============================================================================
//  Fusion
// =============================================================================
static void initGrid() {
  const float sigma = BEAM_SIGMA_DEG * (float)M_PI / 180.0f;
  for (int s = 0; s < 6; s++) {
    float a = SENSOR_ANGLE_DEG[s] * (float)M_PI / 180.0f;
    bool slaveSide = (s >= 3);
    g_ox[s] = slaveSide ? FIELD_W : 0.0f;
    g_dx[s] = (slaveSide ? -1.0f : 1.0f) * cosf(a);   // slave looks back toward master
    g_dy[s] = sinf(a);
    for (int iy = 0; iy < NY; iy++) {
      for (int ix = 0; ix < NX; ix++) {
        int i = iy * NX + ix;
        float dx = ix * CELL - g_ox[s];
        float dy = iy * CELL;
        float r = sqrtf(dx * dx + dy * dy);
        uint8_t w = 0;
        if (r > 0.03f) {
          float c = (dx * g_dx[s] + dy * g_dy[s]) / r;
          if (c > 1) c = 1;
          if (c < -1) c = -1;
          float ang = acosf(c) / sigma;
          w = (uint8_t)(255.0f * expf(-0.5f * ang * ang) + 0.5f);
        }
        float mm = r * 1000.0f;
        g_range_mm[s][i] = (uint16_t)(mm > 65000.0f ? 65000.0f : mm);
        g_beam[s][i] = w;
      }
    }
  }
}

static int countSupport(const float d[6], float x, float y) {
  int n = 0;
  for (int s = 0; s < 6; s++) {
    if (d[s] < MIN_VALID_CM || d[s] > MAX_USEFUL_CM) continue;
    float dx = x - g_ox[s], dy = y;
    float r = sqrtf(dx * dx + dy * dy);
    if (r < 0.03f) continue;
    float c = (dx * g_dx[s] + dy * g_dy[s]) / r;
    if (c > 1) c = 1;
    if (c < -1) c = -1;
    if (acosf(c) > SUPPORT_ANGLE) continue;
    if (fabsf(r - PLAYER_RADIUS - d[s] / 100.0f) <= SUPPORT_RANGE) n++;
  }
  return n;
}

// d[] = filtered distances in cm (<=0 = invalid). Returns true if an estimate exists.
static bool fuse(const float dcm[6], bool tracking, float lastX, float lastY, int miss,
                 float& estX, float& estY, int& support) {
  float d[6];
  bool valid[6];
  int nValid = 0;
  for (int s = 0; s < 6; s++) {
    valid[s] = (dcm[s] >= MIN_VALID_CM && dcm[s] <= MAX_USEFUL_CM);
    d[s] = dcm[s] / 100.0f;
    if (valid[s]) nValid++;
  }
  if (nValid == 0) return false;

  float sigM = MOTION_SIGMA + 0.10f * (float)miss;
  float best = -1.0f;
  int bi = 0;
  for (int iy = 0; iy < NY; iy++) {
    for (int ix = 0; ix < NX; ix++) {
      int i = iy * NX + ix;
      float p = 1.0f;
      if (tracking) {
        float ex = ix * CELL - lastX, ey = iy * CELL - lastY;
        p = PRIOR_FLOOR + expf(-0.5f * (ex * ex + ey * ey) / (sigM * sigM));
      }
      for (int s = 0; s < 6; s++) {
        float w = g_beam[s][i] * (1.0f / 255.0f);
        float r = g_range_mm[s][i] * 0.001f;
        if (valid[s]) {
          float e = (r - PLAYER_RADIUS - d[s]) / RANGE_SIGMA;
          float e2 = e * e;
          float L = (e2 > 50.0f) ? 0.0f : expf(-0.5f * e2);
          p *= w * (L + CLUTTER) + (1.0f - w) * P_BG;
        } else if (r < MAX_RANGE_M) {
          p *= w * P_MISS + (1.0f - w);
        }
      }
      g_post[i] = p;
      if (p > best) { best = p; bi = i; }
    }
  }

  // soft-argmax around the best cell for sub-grid resolution
  int bx = bi % NX, by = bi / NX;
  float sw = 0, sx = 0, sy = 0;
  const int R = 4;  // cells (20 cm)
  for (int iy = by - R; iy <= by + R; iy++) {
    if (iy < 0 || iy >= NY) continue;
    for (int ix = bx - R; ix <= bx + R; ix++) {
      if (ix < 0 || ix >= NX) continue;
      float w = g_post[iy * NX + ix];
      sw += w; sx += w * ix * CELL; sy += w * iy * CELL;
    }
  }
  if (sw <= 0.0f) return false;
  estX = sx / sw;
  estY = sy / sw;
  if (estX < 0) estX = 0;
  if (estX > FIELD_W) estX = FIELD_W;
  if (estY < 0) estY = 0;
  if (estY > FIELD_D) estY = FIELD_D;
  support = countSupport(dcm, estX, estY);
  return true;
}

// 3-sample median over valid readings only
static float medianFilter(int s, float v) {
  s_hist[s][s_hidx[s]] = v;
  s_hidx[s] = (s_hidx[s] + 1) % 3;
  float a[3];
  int n = 0;
  for (int k = 0; k < 3; k++) if (s_hist[s][k] > 0) a[n++] = s_hist[s][k];
  if (n < 2) return -1.0f;
  if (n == 2) {
    if (fabsf(a[0] - a[1]) > 30.0f) return -1.0f;
    return 0.5f * (a[0] + a[1]);
  }
  if (a[0] > a[1]) { float t = a[0]; a[0] = a[1]; a[1] = t; }
  if (a[1] > a[2]) { float t = a[1]; a[1] = a[2]; a[2] = t; }
  if (a[0] > a[1]) { float t = a[0]; a[0] = a[1]; a[1] = t; }
  return a[1];
}

// =============================================================================
//  One measurement + fusion cycle
// =============================================================================
static void printReport(const float raw[6], bool present, float xcm, float ycm, int sup, uint32_t now) {
  static uint32_t cyc = 0;
  cyc++;
  int nValid = 0;
  for (int s = 0; s < 6; s++) if (raw[s] > 0) nValid++;
#if VERBOSE_SERIAL
  Serial.printf("\n===== cycle %lu  (%lu s) =====\n", (unsigned long)cyc, (unsigned long)(now / 1000));
  for (int s = 0; s < 6; s++) printSensorLine(s, raw[s], s >= 3 && !slaveOk);
  Serial.printf("  Slave link : %s\n", slaveOk ? "OK" : "LOST");
  if (present) Serial.printf("  Position   : x=%.1f cm  y=%.1f cm  (%d sensors agree)\n", xcm, ycm, sup);
  else         Serial.printf("  Position   : NOT TRACKED (%d of 6 sensors have a reading; need %d seeing the same object)\n", nValid, MIN_SUPPORT_START);
  Serial.printf("  Dead zone  : %s   Buzzer: %s\n", deadLatched ? "YES (player < 10 cm from sensors)" : "no", buzzerOn ? "ON" : "off");
#else
  Serial.printf("S1=%.0f S2=%.0f S3=%.0f | S4=%.0f S5=%.0f S6=%.0f | pos=(%.1f,%.1f) sup=%d dead=%d slave=%d\n",
                raw[0], raw[1], raw[2], raw[3], raw[4], raw[5], xcm, ycm, sup, deadLatched ? 1 : 0, slaveOk ? 1 : 0);
#endif
}

static void runCycle() {
  float raw[6];
  for (int i = 0; i < 6; i++) raw[i] = -1.0f;
  float slaveVals[3] = {-1, -1, -1};

  clearReply();
#if PARALLEL_SAMPLING
  sendToSlave(CMD_SAMPLE);
#endif
  for (int i = 0; i < 3; i++) {
    raw[i] = readSensorCm(i);
    serviceWait(SETTLE_MS);
  }
#if !PARALLEL_SAMPLING
  sendToSlave(CMD_SAMPLE);
#endif

  bool got = false;
  uint32_t tw = millis();
  while (millis() - tw < REPLY_TIMEOUT_MS) {
    server.handleClient();
    if (takeReply(slaveVals)) { got = true; break; }
    delay(1);
  }
  if (got) {
    slaveMissed = 0; slaveOk = true;
    for (int i = 0; i < 3; i++) raw[3 + i] = slaveVals[i];
  } else {
    if (++slaveMissed >= 3) slaveOk = false;
  }

  // --- filter + fuse ---
  float filt[6];
  bool any = false;
  for (int s = 0; s < 6; s++) {
    filt[s] = medianFilter(s, raw[s]);
    if (filt[s] > 0) any = true;
  }
  float fx = 0, fy = 0;
  int sup = 0;
  bool fixOK = any && fuse(filt, trk.active, trk.x, trk.y, trk.miss, fx, fy, sup);
  bool accept = fixOK && (sup >= MIN_SUPPORT_START || (trk.active && sup >= 1));
  if (accept) {
    if (!trk.active) { trk.x = fx; trk.y = fy; }
    else { trk.x += SMOOTH_ALPHA * (fx - trk.x); trk.y += SMOOTH_ALPHA * (fy - trk.y); }
    trk.active = true;
    trk.miss = 0;
  } else {
    sup = 0;
    if (trk.active && ++trk.miss > COAST_MAX) trk.active = false;
  }
  bool present = trk.active;
  float xcm = present ? trk.x * 100.0f : -1.0f;
  float ycm = present ? trk.y * 100.0f : -1.0f;

  // --- closest raw reading ---
  float closest = -1.0f;
  for (int s = 0; s < 6; s++) if (raw[s] > 0 && (closest < 0 || raw[s] < closest)) closest = raw[s];

  // --- dead zone with latching + hysteresis (uses raw readings for fast reaction) ---
  uint32_t now = millis();
  // A close reading only counts if two sensors agree this cycle, or the same
  // sensor repeats it on consecutive cycles (rejects one-off 2 cm spikes).
  static uint8_t prevM10 = 0, prevM15 = 0;
  uint8_t m10 = 0, m15 = 0;
  for (int s = 0; s < 6; s++) {
    if (raw[s] > 0 && raw[s] < DEAD_CM)         m10 |= (1u << s);
    if (raw[s] > 0 && raw[s] < DEAD_RELEASE_CM) m15 |= (1u << s);
  }
  bool close10 = (__builtin_popcount(m10) >= 2) || (m10 & prevM10);
  bool close15 = (__builtin_popcount(m15) >= 2) || (m15 & prevM15);
  prevM10 = m10;
  prevM15 = m15;
  bool enter = close10 || (present && ycm < DEAD_CM);
  if (enter) {
    if (!deadLatched) deadSince = now;
    deadLatched = true;
    deadReleaseCnt = 0;
  } else if (deadLatched) {
    bool clear = !close15 && !(present && ycm < DEAD_RELEASE_CM);
    deadReleaseCnt = clear ? deadReleaseCnt + 1 : 0;
    if (deadReleaseCnt >= 2 && now - deadSince >= DEAD_MIN_HOLD_MS) {
      deadLatched = false;
      deadReleaseCnt = 0;
    }
  }
  setBuzzer(deadLatched);
  sendToSlave(deadLatched ? CMD_BUZZ_ON : CMD_BUZZ_OFF);

  // --- publish ---
  int n = snprintf(jsonBuf, sizeof(jsonBuf),
      "{\"sensor1\":%.1f,\"sensor2\":%.1f,\"sensor3\":%.1f,\"sensor4\":%.1f,\"sensor5\":%.1f,\"sensor6\":%.1f,"
      "\"x\":%.1f,\"y\":%.1f,\"support\":%d,\"closest\":%.1f,\"present\":%d,\"dead\":%d,\"slave\":%d,\"uptime\":%lu}",
      raw[0], raw[1], raw[2], raw[3], raw[4], raw[5], xcm, ycm, sup, closest,
      present ? 1 : 0, deadLatched ? 1 : 0, slaveOk ? 1 : 0, (unsigned long)now);
  if (n > 0 && WiFi.softAPgetStationNum() > 0) {
    udp.beginPacket(IPAddress(192, 168, 4, 255), UDP_PORT);
    udp.write((const uint8_t*)jsonBuf, (size_t)n);
    udp.endPacket();
  }
  printReport(raw, present, xcm, ycm, sup, now);

  digitalWrite(LED_PIN, slaveOk ? HIGH : !digitalRead(LED_PIN));

  // let the last slave ping die out before the next cycle starts
#if !PARALLEL_SAMPLING
  serviceWait(SETTLE_MS);
#endif
}

// =============================================================================
//  Boot self-test
// =============================================================================
#if BOOT_SELFTEST
// ---- Boot diagnostic: 3 measurement methods x (Wi-Fi OFF / Wi-Fi ON) --------------
//  A = my polling method inside a critical section (used by the firmware)
//  B = Arduino pulseIn inside a critical section   (first firmware version)
//  C = plain Arduino pulseIn, no critical section  (what your sensor-pin-test does)
static uint8_t g_diag[2][3][3];   // [phase][sensor][method] = valid pings out of 8

static float pingPulseIn(uint8_t i, bool critical) {
  digitalWrite(TRIG_PIN[i], LOW);
  delayMicroseconds(5);
  unsigned long us;
  if (critical) portENTER_CRITICAL(&sensorMux);
  digitalWrite(TRIG_PIN[i], HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN[i], LOW);
  us = pulseIn(ECHO_PIN[i], HIGH, PULSE_TIMEOUT_US);
  if (critical) portEXIT_CRITICAL(&sensorMux);
  if (us == 0) return -1.0f;
  float cm = (float)us * 0.01715f;
  return (cm < MIN_VALID_CM || cm > MAX_VALID_CM) ? -1.0f : cm;
}

static void bootSelfTest(int phase, const char* label) {
  Serial.printf("\n[DIAG] ===== %s : 8 pings per method, hand 20 cm in front of every sensor =====\n", label);
  for (int s = 0; s < 3; s++) {
    int a = 0, b = 0, c = 0;
    float la = -1, lb = -1, lc = -1;
    for (int k = 0; k < 8; k++) {
      float v;
      v = readSensorCm(s);        if (v > 0) { a++; la = v; } delay(SETTLE_MS);
      v = pingPulseIn(s, true);   if (v > 0) { b++; lb = v; } delay(SETTLE_MS);
      v = pingPulseIn(s, false);  if (v > 0) { c++; lc = v; } delay(SETTLE_MS);
    }
    g_diag[phase][s][0] = a; g_diag[phase][s][1] = b; g_diag[phase][s][2] = c;
    Serial.printf("[DIAG] S%d T%02d/E%02d   A(poll+crit)=%d/8 (last %.1f)   B(pulseIn+crit)=%d/8 (last %.1f)   C(pulseIn plain)=%d/8 (last %.1f)\n",
                  s + 1, TRIG_PIN[s], ECHO_PIN[s], a, la, b, lb, c, lc);
  }
}

static void bootDiagVerdict() {
  Serial.println("\n[DIAG] ===== VERDICT (copy this whole [DIAG] block back to me) =====");
  for (int s = 0; s < 3; s++) {
    int offA = g_diag[0][s][0], offB = g_diag[0][s][1], offC = g_diag[0][s][2];
    int onA  = g_diag[1][s][0], onB  = g_diag[1][s][1], onC  = g_diag[1][s][2];
    int best = max(max(max(offA, offB), max(offC, onA)), max(onB, onC));
    Serial.printf("[DIAG] S%d: ", s + 1);
    if (best < 3)                         Serial.println("nothing read in ANY mode (object not in beam, or wiring)");
    else if (offC >= 5 && onC <= 1)       Serial.println("works with Wi-Fi OFF, fails with Wi-Fi ON -> Wi-Fi/power related");
    else if (onC >= 5 && onA <= 1)        Serial.println("plain pulseIn works but my polling method fails -> my measurement code");
    else if (onC >= 5 && onB <= 1)        Serial.println("critical section breaks it -> interrupts-off is the problem");
    else if (onA >= 5)                    Serial.println("OK in the firmware's own method");
    else                                  Serial.println("mixed result - see table above");
  }
}
#endif

// =============================================================================
//  setup / loop
// =============================================================================
#if BOOT_PROBE
// Same logic as sensor-pin-test.cpp (2 us low, 10 us trigger, pulseIn 16 ms, sensors back to back).
static float probeRead(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 16000);
  return duration <= 0 ? -1.0f : duration * 0.0343f / 2.0f;
}

static void bootProbe(const char* stage) {
  static const int T[3] = {12, 32, 22};   // same order as your test sketch
  static const int E[3] = {14, 35, 23};
  for (int r = 0; r < 2; r++) {
    char b[3][12];
    for (int i = 0; i < 3; i++) {
      float v = probeRead(T[i], E[i]);
      if (v < 0) snprintf(b[i], sizeof(b[i]), "NO ECHO");
      else       snprintf(b[i], sizeof(b[i]), "%.1f cm", v);
    }
    Serial.printf("[PROBE %-30s] 12/14=%-8s 32/35=%-8s 22/23=%-8s\n", stage, b[0], b[1], b[2]);
    delay(1000);
  }
}
#endif

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Whack-a-Mole MASTER ===");

  for (int i = 0; i < 3; i++) {
    pinMode(TRIG_PIN[i], OUTPUT);
    digitalWrite(TRIG_PIN[i], LOW);
    pinMode(ECHO_PIN[i], INPUT);
  }
#if BOOT_PROBE
  Serial.println("\n[PROBE] >>> HOLD A HAND ~20 cm IN FRONT OF ALL THREE SENSORS NOW (keep it there ~15 s) <<<");
  for (int i = 4; i > 0; i--) { Serial.printf("[PROBE] starting in %d...\n", i); delay(1000); }
  bootProbe("1 pins only (= your test)");
#endif
  pinMode(LED_PIN, OUTPUT);
#if CORE_V3
  ledcAttach(BUZZER_PIN, BUZZER_HZ, 8);
#else
  ledcSetup(0, BUZZER_HZ, 8);
  ledcAttachPin(BUZZER_PIN, 0);
#endif
  setBuzzer(false);
#if BOOT_PROBE
  bootProbe("2 + LED + buzzer LEDC");
#endif

  for (int s = 0; s < 6; s++) {
    s_hidx[s] = 0;
    for (int k = 0; k < 3; k++) s_hist[s][k] = -1.0f;
  }
  initGrid();
#if BOOT_PROBE
  bootProbe("3 + fusion grid");
#endif

#if BOOT_SELFTEST
  Serial.println("\n[DIAG] >>> HOLD A HAND ~20 cm IN FRONT OF ALL THREE SENSORS (S1, S2, S3) NOW, keep it there ~25 s <<<");
  for (int i = 5; i > 0; i--) { Serial.printf("[DIAG] starting in %d...\n", i); delay(1000); }
  bootSelfTest(0, "Wi-Fi OFF");
#endif

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, 4)) {
    Serial.println("softAP failed, restarting");
    delay(1000);
    ESP.restart();
  }
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed, restarting");
    delay(1000);
    ESP.restart();
  }
  esp_now_register_recv_cb(onRecv);
  addPeer(BCAST);
#if BOOT_PROBE
  bootProbe("4 + Wi-Fi AP + ESP-NOW");
#endif

#if BOOT_SELFTEST
  bootSelfTest(1, "Wi-Fi ON (AP + ESP-NOW running)");
  bootDiagVerdict();
#endif

  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", PAGE); });
  server.on("/data", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", jsonBuf);
  });
  server.on("/favicon.ico", HTTP_GET, []() { server.send(204); });
  server.onNotFound([]() { server.send(404, "text/plain", "not found"); });
  server.begin();
  udp.begin(UDP_PORT);
  Serial.println("Ready: http://192.168.4.1/  UDP broadcast port 4210");
}

void loop() {
  runCycle();
}
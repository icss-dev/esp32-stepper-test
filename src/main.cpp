#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

#include <WiFi.h>
#include <WebServer.h>

#include <FastAccelStepper.h>

#define PIN_STEP  25
#define PIN_DIR   26
#define PIN_EN    27          // EN активен LOW
#define PIN_AL    34

// WiFi (STA)
static const char* WIFI_SSID_C = WIFI_SSID;
static const char* WIFI_PASS_C = WIFI_PASS;

static const uint32_t FREQ_MAX = 400000;

static volatile uint32_t g_userFreq = 10000;     // Hz
static volatile uint32_t g_accel    = 200000;    // Hz/s
static volatile uint8_t  g_dir      = 0;         // 0/1
static volatile uint8_t  g_en       = 1;         // 0/1
static volatile bool     g_alarm    = false;

static volatile bool     g_runReq   = false;
static volatile bool     g_dirPend  = false;
static volatile uint8_t  g_dirNext  = 0;

enum CmdType : uint8_t { CMD_START, CMD_STOP, CMD_FREQ, CMD_DIR, CMD_EN, CMD_RAMP, CMD_STATUS, CMD_ACCEL };

struct Cmd {
  CmdType type;
  uint32_t a;
  uint32_t b;
};

static QueueHandle_t qCmd;

static inline uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline bool readAlarm() {
  return digitalRead(PIN_AL) == HIGH;
}

static FastAccelStepperEngine engine;
static FastAccelStepper* stepper = nullptr;

static void applyEnablePin() {
  digitalWrite(PIN_EN, g_en ? LOW : HIGH);
}

static inline void applyDirPin() {
  digitalWrite(PIN_DIR, g_dir ? HIGH : LOW);
}

static void applyParamsToStepper() {
  if (!stepper) return;
  stepper->setSpeedInHz(clamp_u32(g_userFreq, 1, FREQ_MAX));
  stepper->setAcceleration(clamp_u32(g_accel, 1, 2000000));
}

static void applyRunDirectionToUpdateSpeed() {
  if (!stepper) return;
  if (!g_runReq) return;

  if (g_dir) stepper->runBackward();
  else       stepper->runForward();
}

static void requestStart() {
  if (!stepper || !g_en || g_alarm) return;
  applyParamsToStepper();
  g_runReq = true;
  applyRunDirectionToUpdateSpeed();
}

static void requestStop() {
  g_runReq = false;
  if (stepper) stepper->stopMove();
}

static void requestDir(uint8_t newDir) {
  newDir = newDir ? 1 : 0;
  if (newDir == g_dir) return;

  g_dirNext = newDir;

  if (stepper && stepper->isRunning()) {
    g_dirPend = true;
    stepper->stopMove();
  } else {
    g_dir = newDir;
    applyDirPin();
    if (g_runReq) requestStart();
  }
}

static void StepTask(void* arg) {
  uint32_t lastPollMs = 0;

  while (true) {
    Cmd cmd;
    while (xQueueReceive(qCmd, &cmd, 0) == pdTRUE) {
      switch (cmd.type) {
        case CMD_START:
          requestStart();
          break;

        case CMD_STOP:
          requestStop();
          break;

        case CMD_FREQ:
          g_userFreq = clamp_u32(cmd.a, 1, FREQ_MAX);
          applyParamsToStepper();
          if (stepper && stepper->isRunning()) applyRunDirectionToUpdateSpeed();
          break;

        case CMD_ACCEL:
          g_accel = clamp_u32(cmd.a, 1, 2000000);
          applyParamsToStepper();
          if (stepper && stepper->isRunning()) applyRunDirectionToUpdateSpeed();
          break;

        case CMD_DIR:
          requestDir(cmd.a ? 1 : 0);
          break;

        case CMD_EN:
          g_en = cmd.a ? 1 : 0;
          applyEnablePin();
          if (!g_en) requestStop();
          else if (g_runReq && !g_alarm) requestStart();
          break;

        case CMD_RAMP: {
          uint32_t target = clamp_u32(cmd.a, 1, FREQ_MAX);
          uint32_t ms = clamp_u32(cmd.b, 50, 60000);

          uint32_t cur = g_userFreq;
          uint32_t diff = (target > cur) ? (target - cur) : (cur - target);
          uint32_t acc = (diff == 0) ? g_accel : (uint32_t)((uint64_t)diff * 1000ULL / ms);

          g_userFreq = target;
          g_accel = clamp_u32(acc, 1, 2000000);

          applyParamsToStepper();
          if (stepper && stepper->isRunning()) applyRunDirectionToUpdateSpeed();
          if (g_en && !g_alarm) requestStart();
          break;
        }

        case CMD_STATUS:
          break;
      }
    }

    uint32_t now = millis();
    if ((uint32_t)(now - lastPollMs) >= 10) {
      lastPollMs = now;
      bool al = readAlarm();
      if (al != g_alarm) {
        g_alarm = al;
        if (g_alarm) requestStop();
        else if (g_runReq && g_en) requestStart();
      }
    }

    if (g_dirPend && stepper && !stepper->isRunning()) {
      g_dirPend = false;
      g_dir = g_dirNext;
      applyDirPin();
      if (g_runReq && g_en && !g_alarm) requestStart();
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

static void ConsoleTask(void* arg) {
  Serial.println();
  Serial.println("STEP test (FastAccelStepper + WiFi Web)");

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Web: http://");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Web: no WiFi");
  }
  Serial.println();

  Serial.println("Commands:");
  Serial.println("  start | stop");
  Serial.println("  f <hz>");
  Serial.println("  acc <hz_per_s>");
  Serial.println("  dir <0|1>");
  Serial.println("  en <0|1>");
  Serial.println("  ramp <hz> <ms>");
  Serial.println("  status");
  Serial.println();

  char line[96];
  size_t n = 0;

  auto send = [](Cmd c) {
    xQueueSend(qCmd, &c, portMAX_DELAY);
  };

  while (true) {
    while (Serial.available()) {
      char ch = (char)Serial.read();
      Serial.write(ch);
      if (ch == '\r') continue;

      if (ch == '\n') {
        line[n] = 0;
        n = 0;

        char* p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == 0) continue;

        if (!strcmp(p, "start")) { send({CMD_START,0,0}); Serial.println("ok"); continue; }
        if (!strcmp(p, "stop"))  { send({CMD_STOP,0,0});  Serial.println("ok"); continue; }

        if (!strcmp(p, "status")) {
          Serial.printf("runReq=%d running=%d freq=%lu dir=%u en=%u alarm=%d acc=%lu\n",
                        (int)g_runReq,
                        stepper ? (int)stepper->isRunning() : 0,
                        (unsigned long)g_userFreq,
                        (unsigned)g_dir,
                        (unsigned)g_en,
                        (int)g_alarm,
                        (unsigned long)g_accel);
          continue;
        }

        if (p[0] == 'f' && (p[1] == ' ' || p[1] == '\t')) {
          send({CMD_FREQ, (uint32_t)strtoul(p + 2, nullptr, 10), 0});
          Serial.println("ok");
          continue;
        }

        if (!strncmp(p, "acc ", 4)) {
          send({CMD_ACCEL, (uint32_t)strtoul(p + 4, nullptr, 10), 0});
          Serial.println("ok");
          continue;
        }

        if (!strncmp(p, "dir ", 4)) {
          send({CMD_DIR, (uint32_t)strtoul(p + 4, nullptr, 10), 0});
          Serial.println("ok");
          continue;
        }

        if (!strncmp(p, "en ", 3)) {
          send({CMD_EN, (uint32_t)strtoul(p + 3, nullptr, 10), 0});
          Serial.println("ok");
          continue;
        }

        if (!strncmp(p, "ramp ", 5)) {
          char* a = p + 5;
          char* b = a;
          while (*b && *b != ' ' && *b != '\t') b++;
          if (*b) *b++ = 0;
          while (*b == ' ' || *b == '\t') b++;

          send({CMD_RAMP,
                (uint32_t)strtoul(a, nullptr, 10),
                (uint32_t)strtoul(b, nullptr, 10)});
          Serial.println("ok");
          continue;
        }

        Serial.println("ERR");
      } else {
        if (n < sizeof(line) - 1) line[n++] = ch;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ===== Web =====
static WebServer server(80);

static bool qSend(CmdType t, uint32_t a=0, uint32_t b=0) {
  if (!qCmd) return false;
  Cmd c{t,a,b};
  return xQueueSend(qCmd, &c, 0) == pdTRUE;
}

static void handleStatus() {
  bool running = stepper ? stepper->isRunning() : false;

  char json[256];
  snprintf(json, sizeof(json),
           "{\"runReq\":%d,\"running\":%d,\"freq\":%lu,\"acc\":%lu,\"dir\":%u,\"en\":%u,\"alarm\":%d}",
           (int)g_runReq,
           (int)running,
           (unsigned long)g_userFreq,
           (unsigned long)g_accel,
           (unsigned)g_dir,
           (unsigned)g_en,
           (int)g_alarm);

  server.send(200, "application/json", json);
}

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="ru">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>ESP32 STEP</title>
  <style>
    body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial,sans-serif;margin:16px;max-width:720px}
    .row{display:flex;gap:10px;flex-wrap:wrap;align-items:center;margin:10px 0}
    input{padding:10px;font-size:16px;width:160px}
    button{padding:10px 14px;font-size:16px;cursor:pointer}
    .card{border:1px solid #ddd;border-radius:12px;padding:14px;margin:12px 0}
    .k{display:inline-block;min-width:140px;color:#555}
    .v{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
    .grid{display:grid;grid-template-columns:1fr;gap:8px}
    @media (min-width:560px){ .grid{grid-template-columns:1fr 1fr} }
  </style>
</head>
<body>
  <h2>ESP32 STEP (FastAccelStepper)</h2>

  <div class="card">
    <div class="row">
      <button onclick="api('/api/start')">Start</button>
      <button onclick="api('/api/stop')">Stop</button>
      <button onclick="refresh(true)">Refresh</button>
    </div>

    <div class="row">
      <span class="k">Freq (Hz)</span>
      <input id="freq" type="number" min="1" max="400000" step="1" value="10000">
      <button onclick="setFreq()">Set</button>
    </div>

    <div class="row">
      <span class="k">Accel (Hz/s)</span>
      <input id="acc" type="number" min="1" max="2000000" step="1" value="200000">
      <button onclick="setAcc()">Set</button>
    </div>

    <div class="row">
      <span class="k">Dir (0/1)</span>
      <input id="dir" type="number" min="0" max="1" step="1" value="0">
      <button onclick="setDir()">Set</button>
    </div>

    <div class="row">
      <span class="k">Enable (0/1)</span>
      <input id="en" type="number" min="0" max="1" step="1" value="1">
      <button onclick="setEn()">Set</button>
    </div>

    <div class="row">
      <span class="k">Ramp</span>
      <input id="rhz" type="number" min="1" max="400000" step="1" value="20000" placeholder="Hz">
      <input id="rms" type="number" min="50" max="60000" step="10" value="1000" placeholder="ms">
      <button onclick="ramp()">Go</button>
    </div>
  </div>

  <div class="card">
    <div style="margin-bottom:8px"><b>Статусы</b></div>
    <div class="grid">
      <div><span class="k">runReq</span> <span class="v" id="s_runReq">—</span></div>
      <div><span class="k">running</span> <span class="v" id="s_running">—</span></div>
      <div><span class="k">freq</span> <span class="v" id="s_freq">—</span></div>
      <div><span class="k">acc</span> <span class="v" id="s_acc">—</span></div>
      <div><span class="k">dir</span> <span class="v" id="s_dir">—</span></div>
      <div><span class="k">en</span> <span class="v" id="s_en">—</span></div>
      <div><span class="k">alarm</span> <span class="v" id="s_alarm">—</span></div>
    </div>
  </div>

<script>
async function api(path){
  try{
    const r = await fetch(path, {method:'GET'});
    await refresh(false);
    return r.ok;
  }catch(e){ console.log(e); }
  return false;
}

const $ = (id)=>document.getElementById(id);
const inputs = ['freq','acc','dir','en','rhz','rms'];
const isEditing = () => inputs.some(id => $(id) === document.activeElement);

let last = null;
let initialized = false;

function updateStatus(j){
  $('s_runReq').textContent  = j.runReq;
  $('s_running').textContent = j.running;
  $('s_freq').textContent    = j.freq;
  $('s_acc').textContent     = j.acc;
  $('s_dir').textContent     = j.dir;
  $('s_en').textContent      = j.en;
  $('s_alarm').textContent   = j.alarm;
}

function setInputIfChanged(id, val){
  const el = $(id);
  const cur = el.value;
  const next = String(val);
  if (cur !== next) el.value = next;
}

async function refresh(forceInputs){
  try{
    const r = await fetch('/api/status');
    const j = await r.json();

    updateStatus(j);

    const changed =
      !last ||
      last.freq !== j.freq ||
      last.acc  !== j.acc  ||
      last.dir  !== j.dir  ||
      last.en   !== j.en;

    const shouldUpdateInputs =
      (!initialized) || (forceInputs === true) || (changed && !isEditing());

    if (shouldUpdateInputs){
      setInputIfChanged('freq', j.freq);
      setInputIfChanged('acc',  j.acc);
      setInputIfChanged('dir',  j.dir);
      setInputIfChanged('en',   j.en);
      initialized = true;
    }

    last = j;
  }catch(e){
    $('s_runReq').textContent='ERR';
    $('s_running').textContent='ERR';
    $('s_freq').textContent='ERR';
    $('s_acc').textContent='ERR';
    $('s_dir').textContent='ERR';
    $('s_en').textContent='ERR';
    $('s_alarm').textContent='ERR';
  }
}

function setFreq(){
  const v = parseInt($('freq').value||'0',10);
  return api('/api/f?hz='+encodeURIComponent(v));
}
function setAcc(){
  const v = parseInt($('acc').value||'0',10);
  return api('/api/acc?hz='+encodeURIComponent(v));
}
function setDir(){
  const v = parseInt($('dir').value||'0',10);
  return api('/api/dir?v='+encodeURIComponent(v));
}
function setEn(){
  const v = parseInt($('en').value||'0',10);
  return api('/api/en?v='+encodeURIComponent(v));
}
function ramp(){
  const hz = parseInt($('rhz').value||'0',10);
  const ms = parseInt($('rms').value||'0',10);
  return api('/api/ramp?hz='+encodeURIComponent(hz)+'&ms='+encodeURIComponent(ms));
}

setInterval(()=>refresh(false), 500);
refresh(true);
</script>
</body>
</html>
)HTML";

static void handleRoot() {
  server.send(200, "text/html; charset=utf-8", FPSTR(INDEX_HTML));
}

static void handleStart() { server.send(200, "text/plain", qSend(CMD_START) ? "ok" : "err"); }
static void handleStop()  { server.send(200, "text/plain", qSend(CMD_STOP)  ? "ok" : "err"); }

static void handleSetF() {
  uint32_t hz = server.hasArg("hz") ? (uint32_t)strtoul(server.arg("hz").c_str(), nullptr, 10) : 0;
  hz = clamp_u32(hz, 1, FREQ_MAX);
  server.send(200, "text/plain", qSend(CMD_FREQ, hz, 0) ? "ok" : "err");
}
static void handleSetAcc() {
  uint32_t hz = server.hasArg("hz") ? (uint32_t)strtoul(server.arg("hz").c_str(), nullptr, 10) : 0;
  hz = clamp_u32(hz, 1, 2000000);
  server.send(200, "text/plain", qSend(CMD_ACCEL, hz, 0) ? "ok" : "err");
}
static void handleSetDir() {
  uint32_t v = server.hasArg("v") ? (uint32_t)strtoul(server.arg("v").c_str(), nullptr, 10) : 0;
  v = v ? 1 : 0;
  server.send(200, "text/plain", qSend(CMD_DIR, v, 0) ? "ok" : "err");
}
static void handleSetEn() {
  uint32_t v = server.hasArg("v") ? (uint32_t)strtoul(server.arg("v").c_str(), nullptr, 10) : 0;
  v = v ? 1 : 0;
  server.send(200, "text/plain", qSend(CMD_EN, v, 0) ? "ok" : "err");
}
static void handleRamp() {
  uint32_t hz = server.hasArg("hz") ? (uint32_t)strtoul(server.arg("hz").c_str(), nullptr, 10) : 0;
  uint32_t ms = server.hasArg("ms") ? (uint32_t)strtoul(server.arg("ms").c_str(), nullptr, 10) : 0;
  hz = clamp_u32(hz, 1, FREQ_MAX);
  ms = clamp_u32(ms, 50, 60000);
  server.send(200, "text/plain", qSend(CMD_RAMP, hz, ms) ? "ok" : "err");
}

static void WebTask(void* arg) {
  while (true) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

static void wifiInit() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID_C, WIFI_PASS_C);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 15000) delay(250);

  server.on("/", HTTP_ANY, handleRoot);

  server.on("/api/status", HTTP_ANY, handleStatus);

  server.on("/api/start",  HTTP_ANY, handleStart);
  server.on("/api/stop",   HTTP_ANY, handleStop);
  server.on("/api/f",      HTTP_ANY, handleSetF);
  server.on("/api/acc",    HTTP_ANY, handleSetAcc);
  server.on("/api/dir",    HTTP_ANY, handleSetDir);
  server.on("/api/en",     HTTP_ANY, handleSetEn);
  server.on("/api/ramp",   HTTP_ANY, handleRamp);

  server.onNotFound([](){
    if (server.method() == HTTP_OPTIONS) { server.send(204); return; }
    if (server.uri() == "/favicon.ico")  { server.send(204); return; }
    if (server.uri() == "/robots.txt")   { server.send(204); return; }

    Serial.printf("[HTTP 404] %s %s\n",
                  (server.method() == HTTP_GET) ? "GET" :
                  (server.method() == HTTP_POST) ? "POST" : "OTHER",
                  server.uri().c_str());

    server.send(404, "text/plain", "404");
  });


  server.begin();
}


void setup() {
  Serial.begin(115200);

  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_AL, INPUT);

  digitalWrite(PIN_STEP, LOW);
  g_dir = 0;
  applyDirPin();

  g_en = 1;
  applyEnablePin();

  engine.init();
  stepper = engine.stepperConnectToPin(PIN_STEP);
  if (!stepper) {
    Serial.println("ERR: stepperConnectToPin failed");
    while (true) delay(1000);
  }

  stepper->setDirectionPin(PIN_DIR);
  stepper->setEnablePin(PIN_EN);
  stepper->setAutoEnable(true);

  applyParamsToStepper();

  qCmd = xQueueCreate(16, sizeof(Cmd));

  wifiInit();

  xTaskCreatePinnedToCore(StepTask,    "StepTask", 4096, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(ConsoleTask, "Console",  4096, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(WebTask,     "Web",      4096, nullptr, 2, nullptr, 0);
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}

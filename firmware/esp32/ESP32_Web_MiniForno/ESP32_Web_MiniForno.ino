#include <WiFi.h>
#include <WebServer.h>

#define ESP_RX_PIN 16
#define ESP_TX_PIN 17
#define STM_BAUD 115200
#define HISTORY_MAX 96
#define COMMAND_QUEUE_MAX 16
#define COMMAND_GAP_MS 120
#define COMMAND_TIMEOUT_MS 900

const char *AP_SSID = "MiniForno-ESP32";
const char *AP_PASS = "12345678";

HardwareSerial STMSerial(2);
WebServer server(80);

String stmLine;
String ovenState = "INICIO";
String profileName = "ESTANHO";
String stepName = "MATERIAL";
String menuItem = "MATERIAL";
int profileId = 0;
int stepId = 0;
int tempX10 = -1;
int angXX10 = 0;
int angYX10 = 0;
int setpointX10 = 300;
int heaterDuty = 0;
int kpX100 = 300;
int angleLimitX10 = 150;
int heatingSeconds = 10;
int soakSeconds = 10;
int progressPct = 0;
int stmSeconds = 0;
int ledPwmLevel = 0;
bool heaterOn = false;
bool mpuOk = false;
bool calibrated = false;
bool emergency = false;
bool autoEmergency = false;
bool manualMode = false;
bool doorOpen = false;
bool safetyMode = true;
bool configDirty = false;
bool ledReady = false;
bool ledHeating = false;
bool ledHolding = false;
bool ledDone = false;
bool ledEmergency = false;
bool ledPwm = false;
bool ledLife = false;
unsigned long cfgRev = 0;
unsigned long stateRev = 0;
unsigned long syncRev = 0;
String cfgSource = "BOOT";
String stateSource = "BOOT";
String syncSource = "BOOT";
bool pendingCommand = false;
String lastCommandSent = "";
unsigned long lastCommandMs = 0;
String lastAck = "";
String lastErr = "";
unsigned long lastFrameMs = 0;
unsigned long lastHistorySampleMs = 0;

String historyRows[HISTORY_MAX];
int historyCount = 0;
String commandQueue[COMMAND_QUEUE_MAX];
uint8_t commandHead = 0;
uint8_t commandTail = 0;
uint8_t commandCount = 0;

String valueOf(const String &line, const String &key) {
  int start = line.indexOf(key + "=");
  if (start < 0) return "";
  start += key.length() + 1;
  int end = line.indexOf(',', start);
  if (end < 0) end = line.length();
  return line.substring(start, end);
}

String decimalX10(int value) {
  String sign = value < 0 ? "-" : "";
  int absValue = abs(value);
  return sign + String(absValue / 10) + "." + String(absValue % 10);
}

String decimalX100(int value) {
  String sign = value < 0 ? "-" : "";
  int absValue = abs(value);
  int frac = absValue % 100;
  String tail = frac < 10 ? "0" + String(frac) : String(frac);
  return sign + String(absValue / 100) + "." + tail;
}

void sendCommand(const String &cmd) {
  pendingCommand = true;
  lastCommandSent = cmd;
  lastCommandMs = millis();
  STMSerial.print(cmd);
  STMSerial.print("\r\n");
  STMSerial.flush();
  Serial.print("CMD -> ");
  Serial.println(cmd);
}

bool enqueueCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return true;
  if (commandCount >= COMMAND_QUEUE_MAX) {
    lastErr = "ERR,ESP_QUEUE_FULL";
    Serial.print("QUEUE FULL -> ");
    Serial.println(cmd);
    return false;
  }

  commandQueue[commandTail] = cmd;
  commandTail = (uint8_t)((commandTail + 1U) % COMMAND_QUEUE_MAX);
  commandCount++;
  return true;
}

void processCommandQueue() {
  unsigned long now = millis();

  if (pendingCommand) {
    if ((now - lastCommandMs) < COMMAND_TIMEOUT_MS) return;
    lastErr = "ERR,STM_TIMEOUT:" + lastCommandSent;
    pendingCommand = false;
    Serial.println(lastErr);
  }

  if (commandCount == 0) return;
  if ((now - lastCommandMs) < COMMAND_GAP_MS) return;

  String next = commandQueue[commandHead];
  commandQueue[commandHead] = "";
  commandHead = (uint8_t)((commandHead + 1U) % COMMAND_QUEUE_MAX);
  commandCount--;

  sendCommand(next);
}

void appendHistoryRow(const String &row) {
  if (historyCount < HISTORY_MAX) {
    historyRows[historyCount++] = row;
  } else {
    for (int i = 1; i < HISTORY_MAX; i++) historyRows[i - 1] = historyRows[i];
    historyRows[HISTORY_MAX - 1] = row;
  }
}

bool isHeatingHistoryState(const String &state) {
  return state == "PRE_AQUEC" || state == "AQUECENDO";
}

void addHeatingHistorySample() {
  unsigned long now = millis();

  if (!isHeatingHistoryState(ovenState) || tempX10 < 0) {
    lastHistorySampleMs = 0;
    return;
  }

  if (lastHistorySampleMs != 0 && (now - lastHistorySampleMs) < 4000UL) return;
  lastHistorySampleMs = now;

  String row = String(stmSeconds) + "," + decimalX10(tempX10) + ",";
  row += doorOpen ? "ABERTA" : "FECHADA";
  row += ",";
  row += String(heaterDuty);
  appendHistoryRow(row);
}

void addHistoryFromLog(const String &line) {
  (void)line;
}

void parseStmLine(const String &line) {
  if (line.startsWith("LOG,")) {
    addHistoryFromLog(line);
    return;
  }

  if (line.startsWith("ACK,")) {
    lastAck = line;
    lastErr = "";
    pendingCommand = false;
    return;
  }

  if (line.startsWith("ERR,")) {
    lastErr = line;
    pendingCommand = false;
    return;
  }

  if (!line.startsWith("DATA,")) return;

  String v;
  v = valueOf(line, "TEMP10"); if (v.length()) tempX10 = v.toInt();
  v = valueOf(line, "temp_x10"); if (v.length()) tempX10 = v.toInt();
  v = valueOf(line, "DOOR10"); if (v.length()) angXX10 = v.toInt();
  v = valueOf(line, "DOORY10"); if (v.length()) angYX10 = v.toInt();
  v = valueOf(line, "angx_x10"); if (v.length()) angXX10 = v.toInt();
  v = valueOf(line, "angy_x10"); if (v.length()) angYX10 = v.toInt();
  v = valueOf(line, "DOOROPEN"); if (v.length()) doorOpen = v.toInt() != 0;
  v = valueOf(line, "SAFETY"); if (v.length()) safetyMode = v.toInt() != 0;
  v = valueOf(line, "mpu"); if (v.length()) mpuOk = v.toInt() != 0;
  v = valueOf(line, "cal"); if (v.length()) calibrated = v.toInt() != 0;
  v = valueOf(line, "emg"); if (v.length()) emergency = v.toInt() != 0;
  v = valueOf(line, "auto"); if (v.length()) autoEmergency = v.toInt() != 0;
  v = valueOf(line, "STATE"); if (v.length()) ovenState = v;
  v = valueOf(line, "state"); if (v.length()) ovenState = v;
  v = valueOf(line, "profile"); if (v.length()) profileName = v;
  v = valueOf(line, "profile_id"); if (v.length()) profileId = v.toInt();
  v = valueOf(line, "step"); if (v.length()) stepName = v;
  v = valueOf(line, "step_id"); if (v.length()) stepId = v.toInt();
  v = valueOf(line, "SP10"); if (v.length()) setpointX10 = v.toInt();
  v = valueOf(line, "set_x10"); if (v.length()) setpointX10 = v.toInt();
  v = valueOf(line, "heat_time"); if (v.length()) heatingSeconds = v.toInt();
  v = valueOf(line, "soak"); if (v.length()) soakSeconds = v.toInt();
  v = valueOf(line, "POWER"); if (v.length()) heaterDuty = v.toInt();
  v = valueOf(line, "duty"); if (v.length()) heaterDuty = v.toInt();
  v = valueOf(line, "HEATER"); if (v.length()) heaterOn = v.toInt() != 0;
  v = valueOf(line, "heat"); if (v.length()) heaterOn = v.toInt() != 0;
  v = valueOf(line, "kp"); if (v.length()) kpX100 = v.toInt();
  v = valueOf(line, "angle"); if (v.length()) angleLimitX10 = v.toInt();
  v = valueOf(line, "manual"); if (v.length()) manualMode = v.toInt() != 0;
  v = valueOf(line, "progress"); if (v.length()) progressPct = v.toInt();
  v = valueOf(line, "sec"); if (v.length()) stmSeconds = v.toInt();
  v = valueOf(line, "menu"); if (v.length()) menuItem = v;
  v = valueOf(line, "dirty"); if (v.length()) configDirty = v.toInt() != 0;
  v = valueOf(line, "led_ready"); if (v.length()) ledReady = v.toInt() != 0;
  v = valueOf(line, "led_heat"); if (v.length()) ledHeating = v.toInt() != 0;
  v = valueOf(line, "led_hold"); if (v.length()) ledHolding = v.toInt() != 0;
  v = valueOf(line, "led_done"); if (v.length()) ledDone = v.toInt() != 0;
  v = valueOf(line, "led_emg"); if (v.length()) ledEmergency = v.toInt() != 0;
  v = valueOf(line, "led_pwm"); if (v.length()) ledPwm = v.toInt() != 0;
  v = valueOf(line, "led_life"); if (v.length()) ledLife = v.toInt() != 0;
  v = valueOf(line, "CFG_REV"); if (v.length()) cfgRev = (unsigned long)v.toInt();
  v = valueOf(line, "CFG_SRC"); if (v.length()) cfgSource = v;
  v = valueOf(line, "STATE_REV"); if (v.length()) stateRev = (unsigned long)v.toInt();
  v = valueOf(line, "STATE_SRC"); if (v.length()) stateSource = v;
  v = valueOf(line, "SYNC_REV"); if (v.length()) syncRev = (unsigned long)v.toInt();
  v = valueOf(line, "SYNC_SRC"); if (v.length()) syncSource = v;
  v = valueOf(line, "PWM"); if (v.length()) heaterDuty = v.toInt();
  v = valueOf(line, "LED_PWM_LEVEL"); if (v.length()) ledPwmLevel = v.toInt();
  addHeatingHistorySample();
  lastFrameMs = millis();
}

void readStmSerial() {
  while (STMSerial.available()) {
    char c = (char)STMSerial.read();
    if (c == '\n' || c == '\r') {
      if (stmLine.length() > 0) {
        parseStmLine(stmLine);
        stmLine = "";
      }
    } else if (stmLine.length() < 1100) {
      stmLine += c;
    } else {
      stmLine = "";
    }
  }
}

String jsonEscape(String text) {
  text.replace("\\", "\\\\");
  text.replace("\"", "\\\"");
  return text;
}

String jsonData() {
  bool online = (millis() - lastFrameMs) < 3500;
  String json = "{";
  json += "\"online\":"; json += online ? "true" : "false";
  json += ",\"temp\":"; json += decimalX10(tempX10);
  json += ",\"angX\":"; json += decimalX10(angXX10);
  json += ",\"angY\":"; json += decimalX10(angYX10);
  json += ",\"setpoint\":"; json += decimalX10(setpointX10);
  json += ",\"setpointX10\":"; json += setpointX10;
  json += ",\"heatingSeconds\":"; json += heatingSeconds;
  json += ",\"soakSeconds\":"; json += soakSeconds;
  json += ",\"duty\":"; json += heaterDuty;
  json += ",\"heaterOn\":"; json += heaterOn ? "true" : "false";
  json += ",\"mpu\":"; json += mpuOk ? "true" : "false";
  json += ",\"calibrated\":"; json += calibrated ? "true" : "false";
  json += ",\"emergency\":"; json += emergency ? "true" : "false";
  json += ",\"autoEmergency\":"; json += autoEmergency ? "true" : "false";
  json += ",\"manual\":"; json += manualMode ? "true" : "false";
  json += ",\"doorOpen\":"; json += doorOpen ? "true" : "false";
  json += ",\"safety\":"; json += safetyMode ? "true" : "false";
  json += ",\"dirty\":"; json += configDirty ? "true" : "false";
  json += ",\"ledReady\":"; json += ledReady ? "true" : "false";
  json += ",\"ledHeating\":"; json += ledHeating ? "true" : "false";
  json += ",\"ledHolding\":"; json += ledHolding ? "true" : "false";
  json += ",\"ledDone\":"; json += ledDone ? "true" : "false";
  json += ",\"ledEmergency\":"; json += ledEmergency ? "true" : "false";
  json += ",\"ledPwm\":"; json += ledPwm ? "true" : "false";
  json += ",\"ledLife\":"; json += ledLife ? "true" : "false";
  json += ",\"state\":\""; json += ovenState; json += "\"";
  json += ",\"profile\":\""; json += profileName; json += "\"";
  json += ",\"profileId\":"; json += profileId;
  json += ",\"step\":\""; json += stepName; json += "\"";
  json += ",\"stepId\":"; json += stepId;
  json += ",\"menu\":\""; json += menuItem; json += "\"";
  json += ",\"kp\":"; json += decimalX100(kpX100);
  json += ",\"angle\":"; json += decimalX10(angleLimitX10);
  json += ",\"progress\":"; json += progressPct;
  json += ",\"seconds\":"; json += stmSeconds;
  json += ",\"cfgRev\":"; json += cfgRev;
  json += ",\"stateRev\":"; json += stateRev;
  json += ",\"syncRev\":"; json += syncRev;
  json += ",\"cfgSource\":\""; json += jsonEscape(cfgSource); json += "\"";
  json += ",\"stateSource\":\""; json += jsonEscape(stateSource); json += "\"";
  json += ",\"syncSource\":\""; json += jsonEscape(syncSource); json += "\"";
  json += ",\"pendingCommand\":"; json += pendingCommand ? "true" : "false";
  json += ",\"commandAgeMs\":"; json += pendingCommand ? (millis() - lastCommandMs) : 0;
  json += ",\"commandQueue\":"; json += (int)commandCount;
  json += ",\"lastCommand\":\""; json += jsonEscape(lastCommandSent); json += "\"";
  json += ",\"lastAck\":\""; json += jsonEscape(lastAck); json += "\"";
  json += ",\"lastErr\":\""; json += jsonEscape(lastErr); json += "\"";
  json += ",\"ledPwmLevel\":"; json += ledPwmLevel;
  json += "}";
  return json;
}

const char INDEX_HTML[] PROGMEM = R"MINIFORNO(
<!doctype html>
<html lang="pt-BR">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Controlador de Forno</title>
<style>
:root{--bg:#fff;--warm:#fff7e8;--panel:#fff;--line:#efd9bb;--text:#202633;--muted:#697386;--amber:#f59e0b;--orange:#f97316;--red:#ef4444;--yellow:#facc15;--green:#14b8a6;--blue:#2563eb;--shadow:0 14px 32px rgba(32,38,51,.08)}
*{box-sizing:border-box}body{margin:0;background:linear-gradient(180deg,#fff8e8 0,#fff 360px);font-family:Inter,system-ui,-apple-system,Segoe UI,sans-serif;color:var(--text)}
.top{position:sticky;top:0;z-index:5;background:rgba(255,255,255,.94);backdrop-filter:blur(14px);border-bottom:1px solid var(--line);padding:12px 16px;display:flex;justify-content:space-between;gap:12px;align-items:center}
.brand{display:flex;gap:10px;align-items:center;font-weight:900}.mark{width:34px;height:34px;border-radius:8px;background:linear-gradient(135deg,var(--yellow),var(--orange),var(--red));box-shadow:var(--shadow)}
.pill{border:1px solid var(--line);border-radius:999px;padding:8px 12px;color:var(--muted);font-size:12px;background:#fff}.dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:7px;background:var(--red)}.dot.ok{background:var(--green)}
main{max-width:1120px;margin:0 auto;padding:24px 14px 44px}.hero{display:grid;grid-template-columns:1fr .9fr;gap:18px;align-items:stretch;margin-bottom:14px}
h1{font-size:52px;line-height:1.02;margin:10px 0}.eyebrow,.label{color:var(--orange);font-size:11px;font-weight:900;text-transform:uppercase;letter-spacing:0}.lead{color:var(--muted);line-height:1.55;max-width:620px}
.oven{min-height:270px;border:1px solid var(--line);border-radius:8px;background:linear-gradient(180deg,#fff,#fff3d8);box-shadow:var(--shadow);position:relative;overflow:hidden}
.chamber{position:absolute;left:10%;right:10%;bottom:18%;height:45%;border:7px solid #283142;border-radius:8px;background:#fff}.door{position:absolute;left:16%;right:16%;bottom:25%;height:29%;border:3px solid #a1aab7;border-radius:6px;background:linear-gradient(180deg,#f9fafb,#e5e7eb)}
.res{position:absolute;left:22%;right:22%;bottom:34%;display:flex;gap:8px}.res span{height:8px;flex:1;border-radius:999px;background:linear-gradient(90deg,var(--red),var(--yellow),var(--red));box-shadow:0 0 18px rgba(249,115,22,.8);opacity:.25}.oven.on .res span{opacity:1;animation:pulse .65s infinite alternate}.oven.on{box-shadow:0 0 0 1px rgba(249,115,22,.25),0 20px 55px rgba(249,115,22,.18)}
@keyframes pulse{from{filter:brightness(1)}to{filter:brightness(1.75)}}.actions{display:flex;gap:10px;flex-wrap:wrap;align-items:center}.tabs{display:flex;gap:8px;flex-wrap:wrap;margin:18px 0}.tab{border:1px solid var(--line);background:#fff;border-radius:8px;padding:11px 14px;font-weight:850;cursor:pointer}.tab.active{background:var(--amber);border-color:#e38c06;color:#241400}
.view{display:none}.view.active{display:block}.panel{background:rgba(255,255,255,.92);border:1px solid var(--line);border-radius:8px;padding:16px;box-shadow:var(--shadow)}.stage{display:none}.stage.active{display:block}.stageTitle{font-size:26px;font-weight:950;margin:8px 0 16px}
.grid{display:grid;grid-template-columns:repeat(4,1fr);gap:13px}.metric{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:16px;min-height:104px}.wide{grid-column:span 2}.full{grid-column:1/-1}.value{font-size:30px;font-weight:950;margin-top:12px}.unit{font-size:14px;color:var(--muted)}
.badge{display:inline-flex;align-items:center;border-radius:999px;padding:9px 13px;background:#fff2cc;border:1px solid #fed56b;font-weight:900}.badge.EMERGENCIA{background:#fee2e2;color:#991b1b;border-color:#fecaca}.badge.AQUECENDO,.badge.PRE_AQUEC{background:#ffedd5;color:#9a3412}.badge.PATAMAR{background:#dcfce7;color:#166534}.badge.RESFRIANDO{background:#dbeafe;color:#1e40af}.badge.FINALIZADO{background:#fef9c3;color:#854d0e}
.progress{height:14px;background:#f4eadb;border-radius:999px;overflow:hidden;border:1px solid var(--line)}.bar{height:100%;width:0;background:linear-gradient(90deg,var(--yellow),var(--orange),var(--red));transition:.18s}
.btn{border:0;border-radius:8px;padding:12px 14px;font-weight:900;background:#fff;border:1px solid var(--line);cursor:pointer}.primary{background:var(--amber);border-color:#e38c06}.danger{background:#fff1f1;border-color:#fecaca;color:#991b1b}.danger.hot{background:var(--red);color:#fff}.ok{background:#dcfce7;color:#166534;border-color:#bbf7d0}
.stepper{display:grid;grid-template-columns:repeat(4,1fr);gap:10px;margin:2px 0 18px}.step{border:1px solid var(--line);border-radius:8px;padding:12px;background:#fff;color:var(--muted);font-weight:900}.step.active{background:#fff1d6;border-color:var(--amber);color:#9a3412}
.choiceGrid{display:grid;grid-template-columns:repeat(4,1fr);gap:12px}.choice{border:1px solid var(--line);border-radius:8px;padding:16px;background:#fff;cursor:pointer;text-align:left;min-height:96px}.choice b{font-size:18px}.choice small{display:block;color:var(--muted);margin-top:8px}.choice.active{border-color:var(--orange);box-shadow:inset 0 -4px 0 var(--orange)}
canvas{width:100%;height:250px;display:block}.row{display:flex;gap:10px;flex-wrap:wrap;align-items:center}.field{display:grid;gap:6px;color:var(--muted);font-weight:800}.field input{width:150px;border:1px solid var(--line);border-radius:8px;padding:11px;font-size:16px;font-weight:850}.slider{width:100%;accent-color:var(--orange)}table{width:100%;border-collapse:collapse;font-size:13px}th,td{border-bottom:1px solid var(--line);padding:10px;text-align:left}
.switch{display:inline-flex;align-items:center;gap:10px;font-weight:900}.switch input{display:none}.knob{width:52px;height:28px;border-radius:999px;background:#f1d8d8;border:1px solid #fecaca;position:relative}.knob:after{content:"";position:absolute;width:22px;height:22px;left:2px;top:2px;border-radius:50%;background:#fff;box-shadow:0 2px 8px rgba(0,0,0,.18);transition:.16s}.switch input:checked+.knob{background:var(--red);border-color:var(--red)}.switch input:checked+.knob:after{left:26px}
.summary{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;margin:14px 0}.summary div{border:1px solid var(--line);border-radius:8px;padding:12px;background:#fff}.smallValue{font-size:14px;line-height:1.35;word-break:break-word}.ledStrip{display:flex;gap:8px;flex-wrap:wrap;margin-top:12px}.ledItem{display:flex;align-items:center;gap:7px;border:1px solid var(--line);border-radius:999px;background:#fff;padding:8px 11px;font-size:12px;font-weight:900;color:var(--muted)}.ledDot{width:12px;height:12px;border-radius:50%;background:#d1d5db;box-shadow:inset 0 0 0 1px rgba(0,0,0,.08)}.ledDot.on{background:var(--amber);box-shadow:0 0 12px rgba(245,158,11,.75)}.ledDot.red.on{background:var(--red);box-shadow:0 0 12px rgba(239,68,68,.7)}.ledDot.green.on{background:var(--green);box-shadow:0 0 12px rgba(20,184,166,.65)}.hide{display:none}
@media(max-width:780px){h1{font-size:34px}.hero{grid-template-columns:1fr}.grid{grid-template-columns:1fr 1fr}.wide,.full{grid-column:1/-1}.stepper,.choiceGrid,.summary{grid-template-columns:1fr}.value{font-size:25px}.oven{min-height:220px}}
</style>
</head>
<body>
<div class="top"><div class="brand"><span class="mark"></span><span>Controlador<br>de Forno</span></div><div class="pill"><span id="dot" class="dot"></span><span id="online">Aguardando STM32</span></div></div>
<main>
<section class="hero"><div><div class="eyebrow">Mini forno didatico</div><h1>Escolha o material, confirme e acompanhe o aquecimento.</h1><p class="lead">Estado atual: <span id="stateBadgeHero" class="badge">--</span></p><div class="actions"><button class="btn primary" onclick="tab('processo',document.querySelector('[data-tab=processo]'))">Processo</button><button class="btn" onclick="cmd('STATE=INICIO')">Voltar ao inicio</button><label class="switch"><input id="safetyHeroSwitch" type="checkbox" onchange="setSafetySwitch(this.checked)"><span class="knob"></span><span id="safetyHeroText">Seguranca da porta</span></label><button class="btn danger" onclick="cmd('RESET')">Reset emergencia</button></div><div class="ledStrip"><span class="ledItem"><i id="ledLife" class="ledDot green"></i>Vida</span><span class="ledItem"><i id="ledReady" class="ledDot green"></i>Pronto</span><span class="ledItem"><i id="ledHeating" class="ledDot"></i>Aquecendo</span><span class="ledItem"><i id="ledHolding" class="ledDot"></i>Patamar</span><span class="ledItem"><i id="ledDone" class="ledDot green"></i>Fim</span><span class="ledItem"><i id="ledEmergency" class="ledDot red"></i>Emergencia</span><span class="ledItem"><i id="ledPwm" class="ledDot"></i>Saida</span></div></div><div class="oven" id="oven"><div class="chamber"></div><div class="door"></div><div class="res"><span></span><span></span><span></span><span></span></div></div></section>
<div class="tabs"><button data-tab="processo" class="tab active" onclick="tab('processo',this)">Processo</button><button class="tab" onclick="tab('manual',this)">Manual</button><button class="tab" onclick="tab('historico',this)">Historico</button><button class="tab" onclick="tab('sensores',this)">Sensores</button></div>

<section id="processo" class="view active">
<div class="panel">
<div class="stepper"><div id="s0" class="step">1 Material</div><div id="s1" class="step">2 Temperatura</div><div id="s2" class="step">3 Tempos</div><div id="s3" class="step">4 Confirmar</div></div>

<div id="stageMaterial" class="stage active"><div class="stageTitle">Material</div><div class="choiceGrid"><button id="p0" class="choice" onclick="chooseProfile('ESTANHO')"><b>Estanho</b><small>Teste: 30.0 C</small></button><button id="p1" class="choice" onclick="chooseProfile('LATAO')"><b>Latao</b><small>Teste: 40.0 C</small></button><button id="p2" class="choice" onclick="chooseProfile('ALUMINIO')"><b>Aluminio</b><small>Teste: 35.0 C</small></button><button id="p3" class="choice" onclick="chooseProfile('MANUAL')"><b>Manual</b><small>Setpoint livre</small></button></div></div>

<div id="stageTemp" class="stage"><div class="stageTitle">Temperatura</div><div class="summary"><div><span class="label">Material</span><div class="value" id="profile">--</div></div><div><span class="label">Setpoint</span><div class="value"><span id="setpoint">--</span><span class="unit"> C</span></div></div><div><span class="label">Etapa</span><div class="value" id="stepName">--</div></div></div><div id="manualSetBox" class="row"><label class="field">Temperatura C<input id="wizardTemp" type="number" min="0" max="120" step="0.5"></label><button class="btn primary" onclick="saveTemp()">Salvar temperatura</button></div><div id="presetBox" class="actions" style="margin-top:12px"><button class="btn primary" onclick="cmd('STEP=2')">Avancar para tempos</button></div></div>

<div id="stageTimes" class="stage"><div class="stageTitle">Tempos</div><div class="row"><label class="field">Aquecimento s<input id="wizardHeat" type="number" min="5" max="3600" step="1" value="10"></label><label class="field">Patamar s<input id="wizardSoak" type="number" min="5" max="3600" step="1" value="10"></label><button class="btn primary" onclick="saveTimes()">Salvar tempos</button></div></div>

<div id="stageConfirm" class="stage"><div class="stageTitle">Confirmar</div><div class="summary"><div><span class="label">Material</span><div class="value" id="sumProfile">--</div></div><div><span class="label">Temperatura</span><div class="value"><span id="sumTemp">--</span><span class="unit"> C</span></div></div><div><span class="label">Tempos</span><div class="value"><span id="sumTimes">--</span></div></div></div><div class="actions"><button class="btn" onclick="cmd('BACK')">Voltar</button><button class="btn primary" onclick="cmd('CONFIRM')">Confirmar e iniciar</button><button class="btn" onclick="cmd('SAVE')">Salvar Flash</button></div></div>

<div id="stageRun" class="stage"><div class="stageTitle">Execucao</div><div class="grid"><div class="metric"><div class="label">Temperatura</div><div class="value"><span id="temp">--</span><span class="unit"> C</span></div></div><div class="metric"><div class="label">Estado</div><div class="value"><span id="stateBadge" class="badge">--</span></div></div><div class="metric"><div class="label">PWM proporcional</div><div class="value"><span id="duty">0</span><span class="unit"> %</span></div></div><div class="metric"><div class="label">Saida PWM</div><div class="value" id="relayState">--</div></div><div class="metric"><div class="label">Potencia atual</div><div class="value"><span id="power">0.0</span><span class="unit"> W</span></div></div><div class="metric"><div class="label">Corrente media</div><div class="value"><span id="current">0.00</span><span class="unit"> A</span></div></div><div class="metric"><div class="label">Setpoint</div><div class="value"><span id="runSetpoint">--</span><span class="unit"> C</span></div></div><div class="metric"><div class="label">Porta</div><div class="value" id="doorState">--</div></div><div class="metric wide"><div class="label">Progresso</div><div class="progress" style="margin-top:16px"><div class="bar" id="bar"></div></div></div><div class="metric full"><div class="label">Grafico dos ultimos 60 s</div><canvas id="chart" width="1000" height="250"></canvas></div></div></div>
</div>
</section>

<section id="manual" class="view">
<div class="panel"><div class="stageTitle">Manual</div><div class="grid"><div class="metric wide"><div class="label">PWM da resistencia</div><div class="value"><span id="manualValue">0</span><span class="unit"> %</span></div><input class="slider" id="manualSlider" type="range" min="0" max="100" value="0" oninput="manualPreview()" onchange="manualSend()"><div class="actions" style="margin-top:14px"><button class="btn primary" onclick="manualSend()">Ativar manual</button><button class="btn" onclick="manualOff()">Desligar manual</button></div></div><div class="metric"><div class="label">Potencia maxima</div><div class="value">15.0<span class="unit"> W</span></div></div><div class="metric"><div class="label">Fonte</div><div class="value">12<span class="unit"> V</span></div></div></div></div>
</section>

<section id="historico" class="view">
<div class="panel"><div class="row" style="justify-content:space-between"><div><div class="label">Historico de aquecimento</div></div><button class="btn primary" onclick="downloadCsv()">Exportar CSV</button></div><table><thead><tr><th>Tempo (s)</th><th>Temperatura (C)</th><th>Porta</th><th>PWM (%)</th></tr></thead><tbody id="histBody"></tbody></table></div>
</section>

<section id="sensores" class="view">
<div class="panel"><div class="stageTitle">Sensores e porta</div><div class="grid"><div class="metric"><div class="label">Sensor de temperatura</div><div class="value"><span id="sensorTemp">--</span><span class="unit"> C</span></div></div><div class="metric"><div class="label">Angulo X</div><div class="value"><span id="angX">--</span><span class="unit"> deg</span></div></div><div class="metric"><div class="label">Angulo Y</div><div class="value"><span id="angY">--</span><span class="unit"> deg</span></div></div><div class="metric"><div class="label">Porta</div><div class="value" id="doorSensor">--</div></div><div class="metric"><div class="label">Seguranca</div><div class="value" id="safetyState">--</div></div><div class="metric"><div class="label">MPU6050</div><div class="value" id="mpuState">--</div></div><div class="metric"><div class="label">Calibracao</div><div class="value" id="calState">--</div></div><div class="metric wide"><div class="label">Limite da porta</div><div class="row" style="margin-top:14px"><label class="field">Angulo deg<input id="angleInput" type="number" min="2" max="90" step="0.5"></label><button class="btn primary" onclick="saveAngle()">Salvar limite</button></div></div><div class="metric wide"><div class="label">Modo de seguranca</div><label class="switch" style="margin-top:18px"><input id="safetySwitch" type="checkbox" onchange="setSafetySwitch(this.checked)"><span class="knob"></span><span id="safetyText">Seguranca</span></label></div><div class="metric"><div class="label">Emergencia auto</div><div class="value" id="autoEmg">--</div></div><div class="metric"><div class="label">Sincronizacao</div><div class="value" id="syncStatus">--</div></div><div class="metric"><div class="label">Ultima config</div><div class="value" id="cfgSync">--</div></div><div class="metric"><div class="label">Ultimo estado</div><div class="value" id="stateSync">--</div></div><div class="metric"><div class="label">Rev. geral</div><div class="value" id="syncRevView">--</div></div><div class="metric"><div class="label">PWM real</div><div class="value"><span id="pwmReal">0</span><span class="unit"> %</span></div></div><div class="metric"><div class="label">LED espelho</div><div class="value"><span id="ledMirrorLevel">0</span><span class="unit"> %</span></div></div><div class="metric wide"><div class="label">Ultimo ACK</div><div class="value smallValue" id="ackView">--</div></div><div class="metric wide"><div class="label">Ultimo ERR</div><div class="value smallValue" id="errView">--</div></div><div class="metric full"><button class="btn ok" onclick="cmd('CAL')">Calibrar MPU6050</button></div></div></div>
</section>
</main>

<script>
var points=[];var lastData=null;var histTimer=0;var chartWindowMs=60000;
function $(id){return document.getElementById(id)}
function fmt(v){let n=Number(v);return Number.isFinite(n)?n.toFixed(1):'--'}
function setText(id,v){let e=$(id);if(e)e.textContent=v}
function tab(id,el){document.querySelectorAll('.view').forEach(v=>v.classList.remove('active'));document.querySelectorAll('.tab').forEach(t=>t.classList.remove('active'));$(id).classList.add('active');if(el)el.classList.add('active')}
function wait(ms){return new Promise(r=>setTimeout(r,ms))}
function cmd(c){return sendMany([c])}
function sendMany(list){return fetch('/api/cmd',{method:'POST',headers:{'Content-Type':'text/plain'},body:list.join('|'),cache:'no-store'}).then(r=>r.json()).then(d=>{if(!d.ok)throw new Error(d.error||'Falha ao enviar')}).then(()=>wait(700)).then(refresh).catch(e=>{setText('errView','ESP: '+e.message)})}
function chooseProfile(p){return sendMany(['PROFILE='+p,'STEP=1','GET'])}
function saveTemp(){let v=Math.round(Number($('wizardTemp').value||0)*10);return sendMany(['SET10,'+v,'STEP=2','GET'])}
function saveTimes(){let h=Math.round(Number($('wizardHeat').value||10));let s=Math.round(Number($('wizardSoak').value||10));return sendMany(['HEATTIME='+h,'SOAK='+s,'STEP=3','GET'])}
function saveAngle(){let v=Math.round(Number($('angleInput').value||15)*10);cmd('ANGLE='+v)}
function setEmergencySwitch(v){setSafetySwitch(v)}
function setSafetySwitch(v){cmd(v?'SAFETY,1':'SAFETY,0')}
function manualPreview(){setText('manualValue',$('manualSlider').value)}
function manualSend(){cmd('MANUAL='+$('manualSlider').value)}
function manualOff(){$('manualSlider').value=0;manualPreview();cmd('MANUAL=0')}
function isRunState(s){return ['PRE_AQUEC','AQUECENDO','PATAMAR','RESFRIANDO','FINALIZADO','MANUAL','EMERGENCIA'].includes(s)}
function showStage(id){['stageMaterial','stageTemp','stageTimes','stageConfirm','stageRun'].forEach(s=>$(s).classList.toggle('active',s==id))}
function renderStage(d){let id='stageMaterial';if(isRunState(d.state))id='stageRun';else if(d.state=='CONFIRMAR')id='stageConfirm';else if(d.state=='CONFIG'){id=['stageMaterial','stageTemp','stageTimes','stageConfirm'][d.stepId]||'stageMaterial'}showStage(id);$('manualSetBox').classList.remove('hide');$('presetBox').classList.add('hide')}
function paintSteps(d){for(let i=0;i<4;i++)$('s'+i).classList.toggle('active',!isRunState(d.state)&&i==d.stepId);for(let i=0;i<4;i++)$('p'+i).classList.toggle('active',i==d.profileId)}
function fillInputs(d){if(document.activeElement&&document.activeElement.tagName=='INPUT')return;$('wizardTemp').value=fmt(d.setpoint);$('wizardHeat').value=d.heatingSeconds||10;$('wizardSoak').value=d.soakSeconds||10;$('angleInput').value=fmt(d.angle);if(d.manual){$('manualSlider').value=d.duty;manualPreview()}}
function drawChart(){let c=$('chart'),x=c.getContext('2d'),w=c.width,h=c.height,now=Date.now();x.clearRect(0,0,w,h);x.strokeStyle='#f0dfc7';x.lineWidth=1;for(let i=0;i<5;i++){let y=28+i*(h-50)/4;x.beginPath();x.moveTo(38,y);x.lineTo(w-12,y);x.stroke()}x.fillStyle='#697386';x.font='12px system-ui';x.fillText('Temperatura C',38,18);if(points.length<2)return;let vals=points.map(p=>p.v),mn=Math.min(...vals)-2,mx=Math.max(...vals)+2;if(mx<=mn)mx=mn+1;x.strokeStyle='#f97316';x.lineWidth=3;x.beginPath();points.forEach((p,i)=>{let px=38+((p.t-(now-chartWindowMs))/chartWindowMs)*(w-58);let py=(h-22)-((p.v-mn)/(mx-mn))*(h-58);if(i==0)x.moveTo(px,py);else x.lineTo(px,py)});x.stroke()}
function led(id,on){let e=$(id);if(e)e.classList.toggle('on',!!on)}
function updateLeds(d){led('ledLife',d.ledLife);led('ledReady',d.ledReady);led('ledHeating',d.ledHeating);led('ledHolding',d.ledHolding);led('ledDone',d.ledDone);led('ledEmergency',d.ledEmergency);led('ledPwm',d.ledPwm)}
function updateCards(d){let duty=Number(d.duty||0),power=15*duty/100,current=1.25*duty/100;setText('temp',d.temp>=0?fmt(d.temp):'--');setText('sensorTemp',d.temp>=0?fmt(d.temp):'--');setText('setpoint',fmt(d.setpoint));setText('runSetpoint',fmt(d.setpoint));setText('duty',duty);setText('relayState',d.heaterOn?'ON':'OFF');setText('power',power.toFixed(1));setText('current',current.toFixed(2));setText('profile',d.profile);setText('stepName',d.step);setText('sumProfile',d.profile);setText('sumTemp',fmt(d.setpoint));setText('sumTimes',(d.heatingSeconds||10)+'s / '+(d.soakSeconds||10)+'s');setText('angX',fmt(d.angX));setText('angY',fmt(d.angY));setText('doorState',d.doorOpen?'ABERTA':'FECHADA');setText('doorSensor',d.doorOpen?'ABERTA':'FECHADA');setText('safetyState',d.safety?'ON':'OFF');setText('mpuState',d.mpu?'OK':'OFF');setText('calState',d.calibrated?'OK':'PEND');setText('angleLimit',fmt(d.angle));setText('autoEmg',d.autoEmergency?'SIM':'NAO');setText('syncStatus',Number(d.commandQueue||0)>0?'FILA '+d.commandQueue:(d.pendingCommand?(Number(d.commandAgeMs||0)>2500?'SEM CONF.':'AGUARDANDO'):'OK'));setText('cfgSync',(d.cfgSource||'--')+' / '+(d.cfgRev||0));setText('stateSync',(d.stateSource||'--')+' / '+(d.stateRev||0));setText('syncRevView',(d.syncSource||'--')+' / '+(d.syncRev||0));setText('pwmReal',d.duty||0);setText('ledMirrorLevel',d.ledPwmLevel||0);setText('ackView',d.lastAck||'--');setText('errView',d.lastErr||'--');$('bar').style.width=d.progress+'%'}
function updateState(d){$('dot').className='dot '+(d.online?'ok':'');setText('online',d.online?'STM32 online':'Sem dados do STM32');$('oven').classList.toggle('on',d.heaterOn);['stateBadge','stateBadgeHero'].forEach(id=>{let e=$(id);if(e){e.textContent=d.state;e.className='badge '+d.state}});$('safetyHeroSwitch').checked=!!d.safety;setText('safetyHeroText',d.safety?'Porta gera emergencia':'Porta liberada');$('safetySwitch').checked=!!d.safety;setText('safetyText',d.safety?'Porta gera emergencia':'Porta liberada');updateLeds(d)}
function refresh(){fetch('/api/data',{cache:'no-store'}).then(r=>r.json()).then(d=>{lastData=d;updateState(d);updateCards(d);paintSteps(d);renderStage(d);fillInputs(d);if(d.online&&d.temp>=0){let now=Date.now();points.push({t:now,v:Number(d.temp)});points=points.filter(p=>now-p.t<=chartWindowMs);drawChart()}let now=Date.now();if(now-histTimer>4000){histTimer=now;refreshHistory()}}).catch(()=>{})}
function refreshHistory(){fetch('/api/export',{cache:'no-store'}).then(r=>r.text()).then(t=>{let rows=t.trim().split(/\r?\n/).slice(1);$('histBody').innerHTML=rows.map(l=>{let c=l.split(',');return `<tr><td>${c[0]||''}</td><td>${c[1]||''}</td><td>${c[2]||''}</td><td>${c[3]||''}</td></tr>`}).join('')})}
function downloadCsv(){location.href='/api/export'}
setInterval(refresh,200);refresh();refreshHistory();
</script>
</body>
</html>
)MINIFORNO";

void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleData() {
  server.send(200, "application/json", jsonData());
}

int enqueueCommandBatch(String commands) {
  commands.trim();
  int start = 0;
  int queued = 0;

  while (start < commands.length()) {
    int end = commands.indexOf('|', start);
    if (end < 0) end = commands.length();

    String one = commands.substring(start, end);
    one.trim();
    if (one.length() > 0 && enqueueCommand(one)) queued++;

    start = end + 1;
  }

  return queued;
}

void handleCommand() {
  String cmd = server.arg("c");
  if (cmd.length() == 0) cmd = server.arg("cmd");
  if (cmd.length() == 0) cmd = server.arg("command");
  if (cmd.length() == 0 && server.hasArg("plain")) cmd = server.arg("plain");
  int queued = 0;
  bool ok = true;

  if (cmd.length() > 0) {
    queued = enqueueCommandBatch(cmd);
    ok = queued > 0;
  } else {
    ok = false;
    lastErr = "ERR,EMPTY_COMMAND";
  }

  server.sendHeader("Cache-Control", "no-store");
  String json = "{\"ok\":";
  json += ok ? "true" : "false";
  json += ",\"queued\":";
  json += queued;
  json += ",\"queueDepth\":";
  json += (int)commandCount;
  json += ",\"error\":\"";
  json += jsonEscape(lastErr);
  json += "\"}";
  server.send(ok ? 200 : 400, "application/json", json);
}

void handleExport() {
  String csv = "tempo_s,temperatura_c,porta,pwm_percent\n";
  for (int i = 0; i < historyCount; i++) {
    csv += historyRows[i];
    csv += "\n";
  }
  server.send(200, "text/csv; charset=utf-8", csv);
}

void setup() {
  Serial.begin(115200);
  STMSerial.begin(STM_BAUD, SERIAL_8N1, ESP_RX_PIN, ESP_TX_PIN);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/data", HTTP_GET, handleData);
  server.on("/api/cmd", HTTP_GET, handleCommand);
  server.on("/api/cmd", HTTP_POST, handleCommand);
  server.on("/api/export", HTTP_GET, handleExport);
  server.begin();

  Serial.println("Controlador de Forno ESP32 pronto");
  Serial.print("Wi-Fi: ");
  Serial.println(AP_SSID);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
}

void loop() {
  readStmSerial();
  processCommandQueue();
  server.handleClient();
}

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <WiFiUdp.h>
#include <ESP8266mDNS.h>
#include <EEPROM.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <anchor_logic.h>
#include <ais_encoder.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ======== PIN CONFIGURATION ========
// GPS: connect GPS TX → D5 (GPIO14), GPS RX → D6 (GPIO12)
static const int GPS_RX_PIN = 14;  // D5 - receives data FROM GPS TX
static const int GPS_TX_PIN = 12;  // D6 - sends data TO GPS RX
static const int BUZZER_PIN = 13;  // D7 - buzzer/alarm
static const int BUTTON_PIN = 0;   // D3 (GPIO0) - toggle anchor watch / silence alarm
static const int STATUS_LED_PIN = 16; // D0 (GPIO16) - button LED: on when anchor watch is armed

// ======== SETTINGS ========
static const uint32_t GPS_BAUD = 9600;
static const unsigned long DEBOUNCE_MS = 300;
static const uint16_t TRACCAR_PORT = 5055;            // Traccar OsmAnd protocol port
static const unsigned long REPORT_INTERVAL_MS = 30000;
static const unsigned long AIS_POS_INTERVAL_MS = 30000;
static const unsigned long AIS_STATIC_INTERVAL_MS = 360000;  // 6 min

// ---- TEMPORARY: fake GPS for testing without a working GPS module ----
// Set USE_FAKE_GPS = false once the real GPS is connected and working.
static const bool USE_FAKE_GPS = false;
static const double FAKE_LAT = 63.33138594195133;
static const double FAKE_LNG = 10.072873222653783;

// ---- WiFi station reconnect behaviour ----
static const unsigned long WIFI_RETRY_MS = 10000;     // retry every 10 s...
static const int WIFI_FAST_RETRIES = 10;              // ...for the first 10 tries
static const unsigned long WIFI_BACKOFF_MS = 60000;   // then back off to every 60 s

// ======== WiFi AP (for the local config page) ========
const char* AP_SSID = "AnchorWatch";
const char* AP_PASS = "ankervagt";  // min 8 chars

// ======== PERSISTENT CONFIG (stored in EEPROM, editable on /settings) ========
struct AppConfig {
  uint32_t magic;
  char staSsid[33];      // upstream WiFi (internet)
  char staPass[65];
  uint32_t aisMmsi;      // vessel MMSI
  char aisName[21];      // vessel name
  char aisCallsign[8];   // call sign
  uint16_t aisShipType;  // 37 = pleasure craft
  char aisHost[64];      // MarineTraffic roaming / AISHub host
  uint16_t aisPort;      // UDP port for AIS feed
  char traccarHost[64];  // Traccar host ("" = off)
  char traccarId[33];    // Traccar device id
  float radius;          // default anchor radius (m)
  char aisHost2[64];     // secondary AIS host (AISHub) -- appended in v3
  uint16_t aisPort2;     // secondary AIS UDP port
};
static const uint32_t CONFIG_MAGIC = 0x414E4B33;     // "ANK3"
static const uint32_t CONFIG_MAGIC_V2 = 0x414E4B32;  // previous layout (migrated)
AppConfig cfg;

// ======== GLOBALS ========
TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
ESP8266WebServer server(80);
AnchorState anchor;
WiFiUDP aisUdp;
Adafruit_SSD1306 display(128, 64, &Wire, -1);
static const uint8_t OLED_ADDR = 0x3C;  // try 0x3D if the screen stays blank
bool oledOK = false;
unsigned long lastDisplay = 0;

// Anchor swing track: recent boat offsets (metres East/North) from the anchor.
static const int TRACK_N = 120;
float trackE[TRACK_N];
float trackN[TRACK_N];
int trackCount = 0;
int trackHead = 0;
unsigned long lastTrackPt = 0;
double trackAnchorLat = 0, trackAnchorLng = 0;

double currentLat = 0.0;
double currentLng = 0.0;
unsigned long lastButtonPress = 0;
unsigned long lastGPSUpdate = 0;
uint32_t satelliteCount = 0;
unsigned long lastTraccarReport = 0;
bool lastReportedAlarm = false;
unsigned long lastAisPos = 0;
unsigned long lastAisStatic = 0;

// ======== CONFIG STORAGE ========
void saveConfig();  // forward declaration (used by loadConfig migration)

void loadConfig() {
  EEPROM.begin(512);
  EEPROM.get(0, cfg);
  if (cfg.magic == CONFIG_MAGIC_V2) {
    // Migrate previous layout: keep all settings, init the new AISHub fields.
    cfg.aisHost2[0] = '\0';
    cfg.aisPort2 = 0;
    cfg.magic = CONFIG_MAGIC;
    saveConfig();
    Serial.println("Config migrated to v3 (AISHub fields added)");
  } else if (cfg.magic != CONFIG_MAGIC) {
    memset(&cfg, 0, sizeof(cfg));
    cfg.magic = CONFIG_MAGIC;
    cfg.aisMmsi = 258010710;            // default MMSI (editable on /settings)
    strcpy(cfg.aisName, "ANCHORWATCH");
    cfg.aisShipType = 37;               // pleasure craft
    strcpy(cfg.aisHost, "5.9.207.224"); // MarineTraffic roaming listener
    cfg.radius = 30.0;
  }
  if (cfg.radius >= 5.0 && cfg.radius <= 500.0) anchor.setRadius(cfg.radius);
}

void saveConfig() {
  cfg.magic = CONFIG_MAGIC;
  EEPROM.put(0, cfg);
  EEPROM.commit();
}

void applyWifi() {
  if (strlen(cfg.staSsid) > 0) {
    WiFi.begin(cfg.staSsid, cfg.staPass);
    Serial.print("Connecting to WiFi: ");
    Serial.println(cfg.staSsid);
  }
}

// Keep the upstream WiFi connection alive: retry on a short interval, then back
// off to a slower one. The config AP (AnchorWatch) stays up the whole time as a
// fallback, and this also re-homes the device when moved to a different network.
void manageWifi() {
  if (strlen(cfg.staSsid) == 0) return;
  static unsigned long lastTry = 0;
  static int attempts = 0;
  if (WiFi.status() == WL_CONNECTED) { attempts = 0; return; }
  unsigned long interval = (attempts < WIFI_FAST_RETRIES) ? WIFI_RETRY_MS : WIFI_BACKOFF_MS;
  if (millis() - lastTry > interval) {
    lastTry = millis();
    attempts++;
    Serial.print("WiFi not connected, retry #");
    Serial.println(attempts);
    WiFi.begin(cfg.staSsid, cfg.staPass);
  }
}

// True if we have a usable position (real GPS fix, or the temporary fake one).
bool gpsValid() { return USE_FAKE_GPS || gps.location.isValid(); }

String jsonEscape(const char* s) {
  String o;
  for (const char* p = s; *p; p++) {
    if (*p == '"' || *p == '\\') o += '\\';
    o += *p;
  }
  return o;
}

// ======== WEB: DASHBOARD ========
void handleRoot() {
  String html = R"rawhtml(
<!DOCTYPE html>
<html><head>
<meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>Ankervakt</title>
<style>
  body{font-family:sans-serif;margin:0;padding:20px;background:#1a1a2e;color:#eee;text-align:center}
  .card{background:#16213e;border-radius:12px;padding:20px;margin:10px auto;max-width:400px}
  .ok{border:2px solid #0f0} .off{border:2px solid #555}
  .warn{border:2px solid #f00;animation:pulse 1s infinite}
  @keyframes pulse{0%,100%{opacity:1}50%{opacity:0.4}}
  h1{color:#e94560;margin:0 0 10px} .big{font-size:2.2em;font-weight:bold;margin:6px 0}
  button{background:#e94560;color:#fff;border:none;padding:12px 24px;border-radius:8px;font-size:1em;margin:5px;cursor:pointer}
  button:hover{background:#c73550}
  .info{color:#aaa;font-size:0.9em}
  a{color:#e94560}
  input{padding:8px;border-radius:6px;border:1px solid #555;background:#0f3460;color:#eee;width:80px;text-align:center}
</style>
</head><body>
<h1>&#9875; Ankervakt</h1>
<div id='d'></div>
<div class="card"><div class="info">Svingm&oslash;nster rundt anker</div><canvas id="sw" width="280" height="280" style="background:#0f1a30;border-radius:8px;max-width:100%;margin-top:8px"></canvas></div>
<p><a href='/settings'>&#9881; Innstillinger</a></p>
<script>
function u(){fetch('/status').then(r=>r.json()).then(d=>{
  let on=d.anchorSet;
  let cls=d.alarm?'warn':(on?'ok':'off');
  let h='<div class="card '+cls+'"><div>Ankervakt</div>';
  h+='<div class="big">'+(on?(d.alarm?'ALARM!':'P&Aring;'):'AV')+'</div>';
  if(on){h+='<div>'+d.distance.toFixed(1)+' m fra anker</div>';}
  h+='<div class="info">Radius: '+d.radius.toFixed(0)+' m</div></div>';
  h+='<div class="card"><div>Satellitter: '+d.sats+'</div>';
  h+='<div>Posisjon: '+d.lat.toFixed(6)+', '+d.lng.toFixed(6)+'</div>';
  if(on){h+='<div>Anker: '+d.anchorLat.toFixed(6)+', '+d.anchorLng.toFixed(6)+'</div>';}
  h+='<div class="info">Internett: '+(d.online?'tilkoblet':'frakoblet')+'</div>';
  h+='</div><div class="card">';
  if(on){h+='<button onclick="dis()">Sl&aring; av ankervakt</button>';}
  else{h+='<button onclick="arm()">Sl&aring; p&aring; (sett anker her)</button>';}
  if(d.alarm){h+='<button onclick="sl()">Stopp alarm</button>';}
  h+='<br><br>Radius: <input id="r" type="number" value="'+d.radius.toFixed(0)+'"> m ';
  h+='<button onclick="sr()">Oppdater</button></div>';
  document.getElementById('d').innerHTML=h;
})}
function arm(){fetch('/arm').then(r=>r.text()).then(t=>{if(t=='NO_FIX'){alert('Ingen GPS-posisjon enda. Vent paa GPS-fix.');}u();})}
function dis(){fetch('/disarm').then(()=>u())}
function sl(){fetch('/silence').then(()=>u())}
function sr(){fetch('/radius?r='+document.getElementById('r').value).then(()=>u())}
function dt(){fetch('/track').then(r=>r.json()).then(d=>{
  let c=document.getElementById('sw'),x=c.getContext('2d'),W=c.width,H=c.height,cx=W/2,cy=H/2;
  x.clearRect(0,0,W,H);x.textAlign='center';
  if(!d.set){x.fillStyle='#888';x.fillText('Ankervakt av',cx,cy);return;}
  let maxm=d.r*1.15;
  d.pts.forEach(p=>{let m=Math.hypot(p[0],p[1]);if(m>maxm)maxm=m*1.1;});
  let sc=(Math.min(W,H)/2-12)/maxm;
  x.strokeStyle='#0f0';x.lineWidth=1;x.beginPath();x.arc(cx,cy,d.r*sc,0,7);x.stroke();
  x.strokeStyle='#39f';x.beginPath();
  d.pts.forEach((p,i)=>{let px=cx+p[0]*sc,py=cy-p[1]*sc;i?x.lineTo(px,py):x.moveTo(px,py);});x.stroke();
  x.fillStyle='#39f';d.pts.forEach(p=>{x.beginPath();x.arc(cx+p[0]*sc,cy-p[1]*sc,2,0,7);x.fill();});
  x.fillStyle='#e94560';x.beginPath();x.arc(cx,cy,4,0,7);x.fill();
  if(d.pts.length){let p=d.pts[d.pts.length-1];x.fillStyle='#fff';x.beginPath();x.arc(cx+p[0]*sc,cy-p[1]*sc,5,0,7);x.fill();}
})}
setInterval(u,2000);u();setInterval(dt,3000);dt();
</script>
</body></html>
)rawhtml";
  server.send(200, "text/html", html);
}

// ======== WEB: SETTINGS PAGE ========
void handleSettings() {
  String html = R"rawhtml(
<!DOCTYPE html>
<html><head>
<meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>Innstillinger</title>
<style>
  body{font-family:sans-serif;margin:0;padding:20px;background:#1a1a2e;color:#eee}
  .card{background:#16213e;border-radius:12px;padding:16px;margin:12px auto;max-width:440px}
  h1,h2{color:#e94560} h1{text-align:center} h2{font-size:1.1em;margin:0 0 10px}
  label{display:block;margin:8px 0 2px;font-size:.9em;color:#ccc}
  input{width:100%;box-sizing:border-box;padding:9px;border-radius:6px;border:1px solid #555;background:#0f3460;color:#eee}
  button{background:#e94560;color:#fff;border:none;padding:11px 20px;border-radius:8px;font-size:1em;margin-top:12px;cursor:pointer}
  button:hover{background:#c73550}
  .info{color:#aaa;font-size:.85em} a{color:#e94560} .pill{font-weight:bold}
  ul{padding-left:18px} li{margin:6px 0}
</style>
</head><body>
<h1>&#9881; Innstillinger</h1>

<div class="card">
  <h2>1. Internett (WiFi)</h2>
  <div class="info">Koble enheten til b&aring;tens internett (mobil-hotspot / 4G / marina) for AIS og Traccar.</div>
  <label>Nettverksnavn (SSID)</label><input id="ssid">
  <label>Passord</label><input id="pass" type="password">
  <button onclick="saveWifi()">Lagre &amp; koble til</button>
  <div class="info" style="margin-top:8px">Status: <span class="pill" id="wifistat">...</span></div>
</div>

<div class="card" id="netnote">
  <div class="info">&#9888; AIS- og Traccar-valg vises n&aring;r enheten er <b>tilkoblet internett</b> over.</div>
</div>

<div class="card" id="aiscard" style="display:none">
  <h2>2. AIS (MarineTraffic / AISHub)</h2>
  <div class="info">Vis b&aring;ten p&aring; AIS-kart. Krever din MMSI + en UDP-host/port fra en <b>roaming AIS-stasjon</b>.</div>
  <div class="info"><b>Slik f&aring;r du host + port (MarineTraffic):</b>
  <ol style="margin:6px 0 6px 18px;padding:0">
    <li>Lag konto p&aring; <a href="https://www.marinetraffic.com" target="_blank">marinetraffic.com</a>.</li>
    <li>G&aring; til <b>My Account &rarr; My Station</b> &rarr; <b>Add Receiving Station</b>.</li>
    <li>Fyll ut stasjonsnavn, kontaktnavn, posisjon (ankerplassen i desimalgrader) og <b>Roaming MMSI = din MMSI</b>.</li>
    <li>Send inn &rarr; du f&aring;r en <b>IP</b> og en <b>port</b> (vises p&aring; siden + p&aring; e-post).</li>
    <li>Lim IP-en inn i <b>AIS host</b> og porten i <b>AIS UDP-port</b> under, og trykk Lagre AIS.</li>
    <li>Start sending innen <b>10 dager</b>, ellers slettes stasjonen.</li>
  </ol></div>
  <label>MMSI (9 siffer)</label><input id="mmsi" type="number">
  <label>B&aring;tnavn (maks 20)</label><input id="name" maxlength="20">
  <label>Kallesignal (maks 7)</label><input id="call" maxlength="7">
  <label>Skipstype (37 = fritidsb&aring;t)</label><input id="type" type="number">
  <label>AIS host (MarineTraffic)</label><input id="aishost">
  <label>AIS UDP-port (MarineTraffic)</label><input id="aisport" type="number">
  <div class="info" style="margin-top:8px">Valgfritt: send ogs&aring; til <a href="https://stations.vesselfinder.com/become-partner" target="_blank">VesselFinder</a> (registrer som AIS-partner &rarr; du f&aring;r egen host + port). Feltet virker for hvilket som helst ekstra AIS-m&aring;l.</div>
  <label>VesselFinder host</label><input id="aishost2">
  <label>VesselFinder UDP-port</label><input id="aisport2" type="number">
  <button onclick="saveAis()">Lagre AIS</button>
</div>

<div class="card" id="traccarcard" style="display:none">
  <h2>3. Traccar (egen sporing, valgfritt)</h2>
  <div class="info">Egen live-sporing + fjern-ankervarsel. La st&aring; tomt for &aring; sl&aring; av.</div>
  <label>Traccar host</label><input id="traccarhost">
  <label>Enhets-ID (device id)</label><input id="traccarid">
  <button onclick="saveTraccar()">Lagre Traccar</button>
</div>

<div class="card">
  <h2>Generelt</h2>
  <label>Standard ankerradius (m)</label><input id="radius" type="number">
  <button onclick="saveRadius()">Lagre</button>
</div>

<div class="card">
  <h2>Oppsett-guide</h2>
  <ul>
    <li><b>MMSI:</b> finn din i VHF-en (DSC/MMSI-meny) eller p&aring; radiolisensen fra <a href="https://www.telenor.no/kystradio/" target="_blank">Telenor Kystradio</a>.</li>
    <li><b>AIS (MarineTraffic):</b> registrer en <a href="https://support.marinetraffic.com/en/articles/9552981-how-to-add-your-ais-station-and-share-data" target="_blank">roaming AIS-stasjon</a> &rarr; du f&aring;r host + UDP-port &rarr; lim inn over.</li>
    <li><b>AIS (AISHub, alternativ):</b> se <a href="https://www.aishub.net/" target="_blank">aishub.net</a>.</li>
    <li><b>Traccar:</b> lag konto/server &rarr; legg til en enhet med en ID &rarr; lim inn host + ID.</li>
  </ul>
  <div class="info">Tips: bekreft AIS-meldingene i seriemonitoren (linjer som starter med <code>AIS&gt;</code>) f&oslash;r du stoler p&aring; det.</div>
</div>

<p style="text-align:center"><a href="/">&larr; Tilbake til ankervakt</a></p>
<script>
function g(id){return document.getElementById(id)}
function setOnline(o){
  g('wifistat').textContent=o?'tilkoblet':'frakoblet';
  g('aiscard').style.display=o?'block':'none';
  g('traccarcard').style.display=o?'block':'none';
  g('netnote').style.display=o?'none':'block';
}
function load(){fetch('/getconfig').then(r=>r.json()).then(c=>{
  g('ssid').value=c.ssid; if(c.passSet){g('pass').placeholder='(lagret - la stå tomt)';}
  g('mmsi').value=c.mmsi||''; g('name').value=c.name; g('call').value=c.call;
  g('type').value=c.type; g('aishost').value=c.aishost; g('aisport').value=c.aisport||'';
  g('aishost2').value=c.aishost2; g('aisport2').value=c.aisport2||'';
  g('traccarhost').value=c.traccarhost; g('traccarid').value=c.traccarid; g('radius').value=c.radius;
  setOnline(c.online);
})}
function saveWifi(){fetch('/saveconfig?ssid='+encodeURIComponent(g('ssid').value)+'&pass='+encodeURIComponent(g('pass').value)).then(()=>{alert('Lagret. Kobler til...');setTimeout(poll,2500);})}
function saveAis(){fetch('/saveconfig?mmsi='+encodeURIComponent(g('mmsi').value)+'&name='+encodeURIComponent(g('name').value)+'&call='+encodeURIComponent(g('call').value)+'&type='+encodeURIComponent(g('type').value)+'&aishost='+encodeURIComponent(g('aishost').value)+'&aisport='+encodeURIComponent(g('aisport').value)+'&aishost2='+encodeURIComponent(g('aishost2').value)+'&aisport2='+encodeURIComponent(g('aisport2').value)).then(()=>alert('AIS lagret'))}
function saveTraccar(){fetch('/saveconfig?traccarhost='+encodeURIComponent(g('traccarhost').value)+'&traccarid='+encodeURIComponent(g('traccarid').value)).then(()=>alert('Traccar lagret'))}
function saveRadius(){fetch('/saveconfig?radius='+encodeURIComponent(g('radius').value)).then(()=>alert('Lagret'))}
function poll(){fetch('/status').then(r=>r.json()).then(d=>setOnline(d.online))}
load();setInterval(poll,3000);
</script>
</body></html>
)rawhtml";
  server.send(200, "text/html", html);
}

void handleStatus() {
  String json = "{";
  json += "\"lat\":" + String(currentLat, 6) + ",";
  json += "\"lng\":" + String(currentLng, 6) + ",";
  json += "\"anchorLat\":" + String(anchor.anchorLat, 6) + ",";
  json += "\"anchorLng\":" + String(anchor.anchorLng, 6) + ",";
  json += "\"anchorSet\":" + String(anchor.anchorSet ? "true" : "false") + ",";
  json += "\"distance\":" + String(anchor.currentDistance, 1) + ",";
  json += "\"radius\":" + String(anchor.alarmRadius, 1) + ",";
  json += "\"alarm\":" + String(anchor.alarmActive ? "true" : "false") + ",";
  json += "\"sats\":" + String(satelliteCount) + ",";
  json += "\"online\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

void handleGetConfig() {
  String j = "{";
  j += "\"ssid\":\"" + jsonEscape(cfg.staSsid) + "\",";
  j += "\"passSet\":" + String(strlen(cfg.staPass) > 0 ? "true" : "false") + ",";
  j += "\"mmsi\":" + String(cfg.aisMmsi) + ",";
  j += "\"name\":\"" + jsonEscape(cfg.aisName) + "\",";
  j += "\"call\":\"" + jsonEscape(cfg.aisCallsign) + "\",";
  j += "\"type\":" + String(cfg.aisShipType) + ",";
  j += "\"aishost\":\"" + jsonEscape(cfg.aisHost) + "\",";
  j += "\"aisport\":" + String(cfg.aisPort) + ",";
  j += "\"aishost2\":\"" + jsonEscape(cfg.aisHost2) + "\",";
  j += "\"aisport2\":" + String(cfg.aisPort2) + ",";
  j += "\"traccarhost\":\"" + jsonEscape(cfg.traccarHost) + "\",";
  j += "\"traccarid\":\"" + jsonEscape(cfg.traccarId) + "\",";
  j += "\"radius\":" + String(cfg.radius, 0) + ",";
  j += "\"online\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false");
  j += "}";
  server.send(200, "application/json", j);
}

void handleSaveConfig() {
  if (server.hasArg("ssid")) strlcpy(cfg.staSsid, server.arg("ssid").c_str(), sizeof(cfg.staSsid));
  if (server.hasArg("pass") && server.arg("pass").length() > 0)
    strlcpy(cfg.staPass, server.arg("pass").c_str(), sizeof(cfg.staPass));
  if (server.hasArg("mmsi")) cfg.aisMmsi = strtoul(server.arg("mmsi").c_str(), NULL, 10);
  if (server.hasArg("name")) strlcpy(cfg.aisName, server.arg("name").c_str(), sizeof(cfg.aisName));
  if (server.hasArg("call")) strlcpy(cfg.aisCallsign, server.arg("call").c_str(), sizeof(cfg.aisCallsign));
  if (server.hasArg("type")) cfg.aisShipType = (uint16_t)server.arg("type").toInt();
  if (server.hasArg("aishost")) strlcpy(cfg.aisHost, server.arg("aishost").c_str(), sizeof(cfg.aisHost));
  if (server.hasArg("aisport")) cfg.aisPort = (uint16_t)server.arg("aisport").toInt();
  if (server.hasArg("aishost2")) strlcpy(cfg.aisHost2, server.arg("aishost2").c_str(), sizeof(cfg.aisHost2));
  if (server.hasArg("aisport2")) cfg.aisPort2 = (uint16_t)server.arg("aisport2").toInt();
  if (server.hasArg("traccarhost")) strlcpy(cfg.traccarHost, server.arg("traccarhost").c_str(), sizeof(cfg.traccarHost));
  if (server.hasArg("traccarid")) strlcpy(cfg.traccarId, server.arg("traccarid").c_str(), sizeof(cfg.traccarId));
  if (server.hasArg("radius")) {
    cfg.radius = server.arg("radius").toFloat();
    anchor.setRadius(cfg.radius);
  }
  saveConfig();
  applyWifi();
  Serial.println("Config saved");
  server.send(200, "text/plain", "OK");
}

void handleSetAnchor() {
  if (gpsValid()) {
    anchor.setAnchor(currentLat, currentLng);
    Serial.println("Anchor set at: " + String(anchor.anchorLat, 6) + ", " + String(anchor.anchorLng, 6));
  }
  server.send(200, "text/plain", "OK");
}

void handleSilence() {
  anchor.silenceAlarm();
  digitalWrite(BUZZER_PIN, LOW);
  server.send(200, "text/plain", "OK");
}

void handleRadius() {
  if (server.hasArg("r")) {
    double r = server.arg("r").toDouble();
    if (anchor.setRadius(r)) {
      cfg.radius = r;
      saveConfig();
      Serial.println("Radius set to: " + String(anchor.alarmRadius, 1) + " m");
    }
  }
  server.send(200, "text/plain", "OK");
}

void handleArm() {
  if (gpsValid()) {
    anchor.setAnchor(currentLat, currentLng);
    Serial.println("Anchor watch ON (web) at: " + String(anchor.anchorLat, 6) + ", " + String(anchor.anchorLng, 6));
    server.send(200, "text/plain", "OK");
  } else {
    server.send(200, "text/plain", "NO_FIX");
  }
}

void handleDisarm() {
  anchor.disarm();
  digitalWrite(BUZZER_PIN, LOW);
  Serial.println("Anchor watch OFF (web)");
  server.send(200, "text/plain", "OK");
}

// ======== BUTTON HANDLING ========
void checkButton() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    unsigned long now = millis();
    if (now - lastButtonPress > DEBOUNCE_MS) {
      lastButtonPress = now;

      if (anchor.shouldBuzzerSound()) {
        anchor.silenceAlarm();
        digitalWrite(BUZZER_PIN, LOW);
        Serial.println("Alarm silenced");
      } else if (anchor.anchorSet) {
        anchor.disarm();
        digitalWrite(BUZZER_PIN, LOW);
        Serial.println("Anchor watch OFF");
      } else if (gpsValid()) {
        anchor.setAnchor(currentLat, currentLng);
        Serial.println("Anchor watch ON at: " + String(anchor.anchorLat, 6) + ", " + String(anchor.anchorLng, 6));
        digitalWrite(BUZZER_PIN, HIGH);
        delay(80);
        digitalWrite(BUZZER_PIN, LOW);
      } else {
        Serial.println("Cannot arm: waiting for GPS fix");
      }
    }
  }
}

// Status LED in the button: off = watch disabled, on = armed, fast blink = alarm
void updateStatusLed() {
  if (anchor.shouldBuzzerSound()) {
    digitalWrite(STATUS_LED_PIN, (millis() / 200) % 2 == 0 ? HIGH : LOW);
  } else {
    digitalWrite(STATUS_LED_PIN, anchor.anchorSet ? HIGH : LOW);
  }
}

// Map satellite count to a 0..5 signal level.
int signalBars(uint32_t sats) {
  if (sats >= 10) return 5;
  if (sats >= 8) return 4;
  if (sats >= 6) return 3;
  if (sats >= 4) return 2;
  if (sats >= 1) return 1;
  return 0;
}

// Draw a 5-bar signal meter with its top-left corner at (x, y).
void drawSignalBars(int x, int y, int bars) {
  const int barW = 4, gap = 2, maxH = 12;
  for (int i = 0; i < 5; i++) {
    int h = (maxH * (i + 1)) / 5;
    int bx = x + i * (barW + gap);
    int by = y + (maxH - h);
    if (i < bars) display.fillRect(bx, by, barW, h, SSD1306_WHITE);
    else          display.drawRect(bx, by, barW, h, SSD1306_WHITE);
  }
}

// Draw live status on the OLED: anchor watch on/off, GPS strength, radius.
void renderDisplay() {
  if (!oledOK) return;
  uint32_t sats = satelliteCount;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  // Anchor watch state
  display.setCursor(0, 0);
  display.print("Ankervakt: ");
  display.print(anchor.anchorSet ? (anchor.alarmActive ? "ALARM" : "PAA") : "AV");

  // GPS strength (satellites + signal bars)
  display.setCursor(0, 16);
  display.print("GPS: ");
  display.print(sats);
  display.print(" sat");
  drawSignalBars(128 - 30, 15, signalBars(sats));

  // Anchor radius
  display.setCursor(0, 30);
  display.print("Radius: ");
  display.print(anchor.alarmRadius, 0);
  display.print(" m");

  // Distance from anchor when armed
  display.setCursor(0, 42);
  if (anchor.anchorSet) {
    display.print("Avstand: ");
    display.print(anchor.currentDistance, 0);
    display.print(" m");
  }

  // Internet status / IP (so you know where to reach the config page)
  display.setCursor(0, 54);
  if (WiFi.status() == WL_CONNECTED) display.print(WiFi.localIP().toString());
  else display.print("Nett: offline");

  display.display();
}

// Log one point of the anchor swing track (offset in metres from the anchor).
void addTrackPoint() {
  if (!anchor.anchorSet || !gpsValid()) return;
  // Reset the track if the anchor moved (re-armed at a new spot).
  if (anchor.anchorLat != trackAnchorLat || anchor.anchorLng != trackAnchorLng) {
    trackAnchorLat = anchor.anchorLat;
    trackAnchorLng = anchor.anchorLng;
    trackCount = 0;
    trackHead = 0;
  }
  float north = (float)((currentLat - anchor.anchorLat) * 111320.0);
  float east = (float)((currentLng - anchor.anchorLng) * 111320.0 * cos(radians(anchor.anchorLat)));
  trackE[trackHead] = east;
  trackN[trackHead] = north;
  trackHead = (trackHead + 1) % TRACK_N;
  if (trackCount < TRACK_N) trackCount++;
}

// Web: the swing track as JSON (radius + offsets in metres, oldest first).
void handleTrack() {
  String j = "{\"r\":" + String(anchor.alarmRadius, 1);
  j += ",\"set\":" + String(anchor.anchorSet ? "true" : "false") + ",\"pts\":[";
  int startIdx = (trackHead - trackCount + TRACK_N) % TRACK_N;
  for (int i = 0; i < trackCount; i++) {
    int idx = (startIdx + i) % TRACK_N;
    if (i) j += ",";
    j += "[" + String(trackE[idx], 1) + "," + String(trackN[idx], 1) + "]";
  }
  j += "]}";
  server.send(200, "application/json", j);
}

// ======== TRACCAR REPORTING ========
void reportToTraccar(bool alarm) {
  if (WiFi.status() != WL_CONNECTED || !gpsValid()) return;

  String url = "http://";
  url += cfg.traccarHost;
  url += ":" + String(TRACCAR_PORT);
  url += "/?id=" + String(cfg.traccarId);
  url += "&lat=" + String(currentLat, 6);
  url += "&lon=" + String(currentLng, 6);
  url += "&speed=" + String(gps.speed.knots(), 1);   // OsmAnd expects knots
  if (gps.course.isValid()) url += "&bearing=" + String(gps.course.deg(), 0);
  if (alarm) url += "&alarm=sos";

  WiFiClient client;
  HTTPClient http;
  http.setTimeout(2000);  // keep under the ~3s software watchdog
  if (http.begin(client, url)) {
    int code = http.GET();
    Serial.print("Traccar (");
    Serial.print(alarm ? "ALARM" : "ok");
    Serial.print("): HTTP ");
    Serial.println(code);
    http.end();
  }
}

// ======== AIS REPORTING ========
// Cached resolved IPs for the AIS hosts. DNS is done once per host (not on every
// send), so a slow lookup can never block the loop long enough to trip the
// software watchdog.
IPAddress aisIp1, aisIp2;
bool aisIp1Ok = false, aisIp2Ok = false;

void sendAisTo(const char* host, uint16_t port, IPAddress& ip, bool& ipOk, const String& sentence) {
  if (strlen(host) == 0 || port == 0) return;
  if (!ipOk) {
    // The first DNS lookup can take a few seconds; disable the soft watchdog
    // around it so the resolution can't trip a reset (hardware WDT still guards).
    ESP.wdtDisable();
    int r = WiFi.hostByName(host, ip);
    ESP.wdtEnable(0);
    if (r != 1) {
      Serial.printf("AIS: DNS lookup failed for %s\n", host);
      return;  // try again next time; never block repeatedly
    }
    ipOk = true;
    Serial.printf("AIS: resolved %s -> %s\n", host, ip.toString().c_str());
  }
  int b = aisUdp.beginPacket(ip, port);  // IP -> no DNS, non-blocking
  aisUdp.print(sentence);
  aisUdp.print("\r\n");
  int e = aisUdp.endPacket();
  Serial.printf("AIS> %s  [-> %s:%u begin=%d end=%d]\n", sentence.c_str(), host, port, b, e);
}

void sendAis(const String& sentence) {
  sendAisTo(cfg.aisHost, cfg.aisPort, aisIp1, aisIp1Ok, sentence);    // MarineTraffic
  sendAisTo(cfg.aisHost2, cfg.aisPort2, aisIp2, aisIp2Ok, sentence);  // VesselFinder (if set)
}

bool reportAisPosition() {
  if (WiFi.status() != WL_CONNECTED || !gpsValid()) return false;
  // Use real GPS speed/course when available; in fake-GPS test mode send a
  // plausible stationary value (SOG 0, COG 0) instead of "not available".
  double sog = gps.speed.isValid() ? gps.speed.knots() : (USE_FAKE_GPS ? 0.0 : -1);
  double cog = gps.course.isValid() ? gps.course.deg() : (USE_FAKE_GPS ? 0.0 : -1);
  sendAis(AisEncoder::positionReport(cfg.aisMmsi, currentLat, currentLng, sog, cog, 511));
  return true;
}

bool reportAisStatic() {
  if (WiFi.status() != WL_CONNECTED) return false;
  sendAis(AisEncoder::staticA(cfg.aisMmsi, cfg.aisName));
  sendAis(AisEncoder::staticB(cfg.aisMmsi, cfg.aisShipType, cfg.aisCallsign));
  return true;
}

// ======== SETUP ========
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== AnchorWatch Starting ===");

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(STATUS_LED_PIN, LOW);

  loadConfig();
  if (USE_FAKE_GPS) Serial.println("** USING FAKE GPS POSITION (test mode) **");

  gpsSerial.begin(GPS_BAUD);
  Serial.println("GPS serial started");

  // OLED (I2C: SDA=D2/GPIO4, SCL=D1/GPIO5)
  Wire.begin();
  oledOK = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oledOK) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("Anchor");
    display.println("Watch");
    display.display();
    Serial.println("OLED OK");
  } else {
    Serial.println("OLED not found (check wiring / try 0x3D)");
  }

  // AP for local config; AP+STA so we can also reach the internet for AIS/Traccar
  // AP for local config; AP+STA so we can also reach the internet for AIS/Traccar.
  WiFi.mode(WIFI_AP_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
  applyWifi();

  // mDNS so the config page is reachable at http://anchorwatch.local on the LAN
  if (MDNS.begin("anchorwatch")) {
    MDNS.addService("http", "tcp", 80);
    Serial.println("mDNS started: http://anchorwatch.local");
  }

  server.on("/", handleRoot);
  server.on("/settings", handleSettings);
  server.on("/status", handleStatus);
  server.on("/getconfig", handleGetConfig);
  server.on("/saveconfig", handleSaveConfig);
  server.on("/setanchor", handleSetAnchor);
  server.on("/silence", handleSilence);
  server.on("/radius", handleRadius);
  server.on("/arm", handleArm);
  server.on("/disarm", handleDisarm);
  server.on("/track", handleTrack);
  server.begin();
  Serial.println("Web server started");
}

// ======== MAIN LOOP ========
void loop() {
  // Drain GPS bytes, but cap per loop so RX noise can't spin here forever.
  int gpsGuard = 0;
  while (gpsSerial.available() > 0 && gpsGuard++ < 1200) {
    gps.encode(gpsSerial.read());
  }

  if (USE_FAKE_GPS) {
    // Temporary: pretend we have a fix at the configured fake position.
    currentLat = FAKE_LAT;
    currentLng = FAKE_LNG;
    satelliteCount = 12;
    anchor.updatePosition(currentLat, currentLng);
    digitalWrite(BUZZER_PIN, anchor.shouldBuzzerSound() ? HIGH : LOW);
  } else if (gps.location.isUpdated()) {
    currentLat = gps.location.lat();
    currentLng = gps.location.lng();
    satelliteCount = gps.satellites.value();
    lastGPSUpdate = millis();

    anchor.updatePosition(currentLat, currentLng);
    digitalWrite(BUZZER_PIN, anchor.shouldBuzzerSound() ? HIGH : LOW);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  // blink on GPS fix
  }

  updateStatusLed();
  checkButton();
  manageWifi();

  // Refresh OLED ~2x/sec
  if (millis() - lastDisplay > 500) {
    lastDisplay = millis();
    renderDisplay();
  }

  // Log a swing-track point every 15s (when armed + has a fix)
  if (millis() - lastTrackPt > 15000) {
    lastTrackPt = millis();
    addTrackPoint();
  }

  // [TEMP] GPS diagnostic: is data arriving + being parsed?
  static unsigned long lastGpsDbg = 0;
  if (millis() - lastGpsDbg > 5000) {
    lastGpsDbg = millis();
    Serial.printf("GPS dbg: chars=%lu sentences=%lu csumErr=%lu fix=%d sats=%lu\n",
                  gps.charsProcessed(), gps.sentencesWithFix(), gps.failedChecksum(),
                  (int)gps.location.isValid(), gps.satellites.value());
  }

  // Traccar: periodic + immediately on alarm change
  if (strlen(cfg.staSsid) > 0 && strlen(cfg.traccarHost) > 0) {
    bool alarmNow = anchor.alarmActive;
    if (millis() - lastTraccarReport > REPORT_INTERVAL_MS || alarmNow != lastReportedAlarm) {
      lastTraccarReport = millis();
      lastReportedAlarm = alarmNow;
      reportToTraccar(alarmNow);
    }
  }

  // AIS roaming-station reporting
  if (strlen(cfg.staSsid) > 0 && cfg.aisMmsi != 0 && (strlen(cfg.aisHost) > 0 || strlen(cfg.aisHost2) > 0)) {
    if (lastAisStatic == 0 || millis() - lastAisStatic > AIS_STATIC_INTERVAL_MS) {
      if (reportAisStatic()) lastAisStatic = millis();
    }
    if (lastAisPos == 0 || millis() - lastAisPos > AIS_POS_INTERVAL_MS) {
      if (reportAisPosition()) lastAisPos = millis();
    }
  }

  MDNS.update();
  server.handleClient();
}

// LitPin ESP32 - SOLID ONLY (UI + firmware together)
// --------------------------------------------------
// WIFI BEHAVIOR (fixed):
// 1) Out of box (no creds): AP-only (Litpin / 12345678), UI at 192.168.4.1
// 2) WIFI tab can SCAN local networks (works while AP clients connected)
// 3) Press CONNECT -> stays AP+STA while joining (UI stays reachable)
//    If success: AP turns OFF, runs on LAN.
//    - mDNS hostname: litpin-XXXX.local (XXXX from MAC)
// 4) If moved / network changes / STA drops too long: falls back to AP-only so setup works again.
// 5) Peer discovery: units announce over UDP on LAN; WIFI tab shows "Found units" list.
//
// ===== LIBS YOU MUST INSTALL =====
//   - FastLED
//   - ArduinoJson
//   - ESPmDNS (comes with Arduino-ESP32 core)

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <FastLED.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <cstring>
#include <cmath>

// ---------------- USER PIN CONFIG ----------------
static const int PIN_A = 2;   // OUT1
static const int PIN_B = 4;   // OUT2
static const int PIN_C = 18;  // OUT3
static const int PIN_D = 19;  // OUT4
static const int PIN_E = 21;  // OUT5 placeholder (not populated)
static const int OUT5_PRESENT = 0; // 0 = force OUT5 OFF, 1 = enable OUT5

static const int S1_PIN = 34; // ADC
static const int S2_PIN = 35; // ADC
static const int S3_PIN = 32; // ADC

static const char* DEFAULT_UNIT_NAME = "My Pin 1";

static const char* AP_SSID = "Litpin";
static const char* AP_PASS = "12345678";

#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
static const int MAX_MA = 4000;
// -------------------------------------------------

// ---------------- constants ----------------
static const int OUTS_TOTAL = 5;
static const int SENS_TOTAL = 3;

enum OutType : uint8_t { OUT_OFF=0, OUT_STR6=1, OUT_STR12=2, OUT_STR24=3, OUT_MX4=4 };
enum SensorMode : uint8_t { SENSE=0, FORCEON=1, FORCEOFF=2 };
enum SensorAssign : uint8_t { AS_GI=0, AS_FLASHER=1 };

// ---------------- debug ----------------
#define WIFI_DEBUG 1
#if WIFI_DEBUG
  #define DBG(...) do { Serial.printf(__VA_ARGS__); Serial.printf("\n"); } while(0)
#else
  #define DBG(...) do{}while(0)
#endif

static volatile uint32_t lastWifiEvent = 0;
static volatile uint32_t lastDiscReason = 0;

// ---------------- state ----------------
struct OutCfg { uint8_t type=OUT_MX4; uint8_t rot=0; uint8_t flip=0; }; // rot used for MX4, flip used for strips
struct SensorCfg {
  uint8_t mode=SENSE;
  uint16_t threshold=1200;
  uint8_t assign=AS_FLASHER;
  uint16_t value=0;
  bool active=false;
};
struct State {
  uint16_t ma = 1500;
  uint16_t shue = 30;
  uint8_t  cbr  = 50;
  uint8_t  wbr  = 0;
  uint16_t wtmp = 6500;
  uint8_t  rat  = 0;

  OutCfg out[OUTS_TOTAL];
  SensorCfg s[SENS_TOTAL];

  char unitName[24] = {0};
};
static State st;

// ---------------- wifi cfg ----------------
struct WifiCfg {
  bool hasSTA=false;
  char ssid[33]={0};
  char pass[65]={0};
};
static WifiCfg wifiCfg;

// ---------------- core objects ----------------
static Preferences prefs;
static WebServer server(80);

// ---------------- FastLED buffers ----------------
static const int MAX_PIX = 24; // max supported per OUT buffer
static CRGB ledsA[MAX_PIX], ledsB[MAX_PIX], ledsC[MAX_PIX], ledsD[MAX_PIX], ledsE[MAX_PIX];
static CRGB* ledBufs[OUTS_TOTAL] = { ledsA, ledsB, ledsC, ledsD, ledsE };

// ---------------- helpers ----------------
static int clamp16(int v,int lo,int hi){ if(v<lo) return lo; if(v>hi) return hi; return v; }
static uint16_t clamp16u(int v,int lo,int hi){ return (uint16_t)clamp16(v,lo,hi); }
static float lerpf(float a,float b,float t){ return a + (b-a)*t; }

static void safeStrcpy(char* dst, size_t dstSz, const char* src){
  if(!dst || dstSz==0) return;
  if(!src){ dst[0]=0; return; }
  strncpy(dst, src, dstSz-1);
  dst[dstSz-1]=0;
}

static int pixelsForType(uint8_t t){
  switch(t){
    case OUT_STR6:  return 6;
    case OUT_STR12: return 12;
    case OUT_STR24: return 24;
    case OUT_MX4:   return 16;
    default:        return 0;
  }
}

static void clearAll(){
  for(int o=0;o<OUTS_TOTAL;o++) for(int i=0;i<MAX_PIX;i++) ledBufs[o][i]=CRGB::Black;
}

static CRGB hsv(uint16_t h, uint8_t s, uint8_t v){
  CHSV c((uint8_t)(h*255/360), s, v);
  CRGB r; hsv2rgb_rainbow(c, r); return r;
}

static CRGB whiteTemp(uint16_t kelvin, uint8_t bright){
  float t = (float)(clamp16(kelvin,1000,20000)-1000) / 19000.0f;
  uint8_t wr=(uint8_t)lerpf(255.0f,210.0f,t);
  uint8_t wg=(uint8_t)lerpf(210.0f,235.0f,t);
  uint8_t wb=(uint8_t)lerpf(140.0f,255.0f,t);
  wr=(uint8_t)((wr*bright)/255);
  wg=(uint8_t)((wg*bright)/255);
  wb=(uint8_t)((wb*bright)/255);
  return CRGB(wr,wg,wb);
}

// ---- 4x4 serp + rotation helpers ----
static uint8_t xy4(uint8_t x, uint8_t y){
  return (y & 1) ? (uint8_t)(y*4 + (3-x)) : (uint8_t)(y*4 + x);
}
static void rotCoord4(uint8_t x, uint8_t y, uint8_t rot, uint8_t &xo, uint8_t &yo){
  switch(rot & 3){
    default:
    case 0: xo=x;                  yo=y;                  break;
    case 1: xo=(uint8_t)(3-y);     yo=x;                  break;
    case 2: xo=(uint8_t)(3-x);     yo=(uint8_t)(3-y);     break;
    case 3: xo=y;                  yo=(uint8_t)(3-x);     break;
  }
}
static void setMatrixPix(int outIdx, uint8_t x, uint8_t y, const CRGB& c){
  uint8_t xr, yr;
  rotCoord4(x, y, st.out[outIdx].rot, xr, yr);
  uint8_t idx = xy4(xr, yr);
  if(idx < MAX_PIX) ledBufs[outIdx][idx] = c;
}

// ---------------- persistence ----------------
static void loadWifi(){
  prefs.begin("litpin", true);
  wifiCfg.hasSTA = prefs.getBool("hasSTA", false);
  String s = prefs.getString("ssid","");
  String p = prefs.getString("pass","");
  prefs.end();
  safeStrcpy(wifiCfg.ssid,sizeof(wifiCfg.ssid),s.c_str());
  safeStrcpy(wifiCfg.pass,sizeof(wifiCfg.pass),p.c_str());
}

static void saveWifi(){
  prefs.begin("litpin", false);
  prefs.putBool("hasSTA", wifiCfg.hasSTA);
  prefs.putString("ssid", wifiCfg.ssid);
  prefs.putString("pass", wifiCfg.pass);
  prefs.end();
}

static void loadState(){
  prefs.begin("litpin", true);
  st.ma   = prefs.getUInt("ma", 1500);
  st.shue = prefs.getUInt("shue", 30);
  st.cbr  = prefs.getUChar("cbr", 50);
  st.wbr  = prefs.getUChar("wbr", 0);
  st.wtmp = prefs.getUInt("wtmp", 6500);
  st.rat  = prefs.getUChar("rat", 0);
  String nm = prefs.getString("name", DEFAULT_UNIT_NAME);
  prefs.end();
  safeStrcpy(st.unitName,sizeof(st.unitName),nm.c_str());

  prefs.begin("litpin_o", true);
  for(int i=0;i<OUTS_TOTAL;i++){
    char k1[8],k2[8],k3[8];
    snprintf(k1,sizeof(k1),"t%d",i);
    snprintf(k2,sizeof(k2),"r%d",i);
    snprintf(k3,sizeof(k3),"f%d",i);
    st.out[i].type = prefs.getUChar(k1, OUT_MX4);
    st.out[i].rot  = prefs.getUChar(k2, 0) & 3;
    st.out[i].flip = prefs.getUChar(k3, 0) ? 1 : 0;
  }
  prefs.end();

  prefs.begin("litpin_s", true);
  for(int i=0;i<SENS_TOTAL;i++){
    char km[8],kt[8],ka[8];
    snprintf(km,sizeof(km),"m%d",i);
    snprintf(kt,sizeof(kt),"t%d",i);
    snprintf(ka,sizeof(ka),"a%d",i);
    st.s[i].mode = prefs.getUChar(km, (i==0)?FORCEON:SENSE);
    st.s[i].threshold = prefs.getUInt(kt, 1200);
    st.s[i].assign = prefs.getUChar(ka, (i==0)?AS_GI:AS_FLASHER);
  }
  prefs.end();

  if(!OUT5_PRESENT) st.out[4].type = OUT_OFF;
}

static void saveState(){
  prefs.begin("litpin", false);
  prefs.putUInt("ma", st.ma);
  prefs.putUInt("shue", st.shue);
  prefs.putUChar("cbr", st.cbr);
  prefs.putUChar("wbr", st.wbr);
  prefs.putUInt("wtmp", st.wtmp);
  prefs.putUChar("rat", st.rat);
  prefs.putString("name", st.unitName);
  prefs.end();

  prefs.begin("litpin_o", false);
  for(int i=0;i<OUTS_TOTAL;i++){
    char k1[8],k2[8],k3[8];
    snprintf(k1,sizeof(k1),"t%d",i);
    snprintf(k2,sizeof(k2),"r%d",i);
    snprintf(k3,sizeof(k3),"f%d",i);
    prefs.putUChar(k1, st.out[i].type);
    prefs.putUChar(k2, (uint8_t)(st.out[i].rot & 3));
    prefs.putUChar(k3, (uint8_t)(st.out[i].flip ? 1 : 0));
  }
  prefs.end();

  prefs.begin("litpin_s", false);
  for(int i=0;i<SENS_TOTAL;i++){
    char km[8],kt[8],ka[8];
    snprintf(km,sizeof(km),"m%d",i);
    snprintf(kt,sizeof(kt),"t%d",i);
    snprintf(ka,sizeof(ka),"a%d",i);
    prefs.putUChar(km, st.s[i].mode);
    prefs.putUInt(kt, st.s[i].threshold);
    prefs.putUChar(ka, st.s[i].assign);
  }
  prefs.end();
}

// ---------------- sensors ----------------
static int sensorPin(int i){ return (i==0)?S1_PIN:(i==1)?S2_PIN:S3_PIN; }

static void updateSensors(){
  for(int i=0;i<SENS_TOTAL;i++){
    int v = analogRead(sensorPin(i));
    st.s[i].value = (uint16_t)v;
    bool act=false;
    if(st.s[i].mode==FORCEON) act=true;
    else if(st.s[i].mode==FORCEOFF) act=false;
    else act = (v >= st.s[i].threshold);
    st.s[i].active = act;
  }
}

static void sensorAggregate(bool &giOn){
  bool anyGI=false;
  giOn=false;
  for(int i=0;i<SENS_TOTAL;i++){
    if(st.s[i].assign==AS_GI){
      anyGI=true;
      if(st.s[i].active) giOn=true;
    }
  }
  if(!anyGI) giOn=true; // bench-friendly
}

// ---------------- SOLID render (respects rot/flip) ----------------
static void renderSolidOut(int outIdx){
  int n = pixelsForType(st.out[outIdx].type);
  if(n<=0) return;

  CRGB hueRGB = hsv(st.shue, 255, (uint8_t)(st.cbr*255/100));
  CRGB wRGB   = whiteTemp(st.wtmp, (uint8_t)(st.wbr*255/100));

  if(st.out[outIdx].type == OUT_MX4){
    for(uint8_t y=0;y<4;y++) for(uint8_t x=0;x<4;x++){
      setMatrixPix(outIdx, x, y, hueRGB);
    }

    int whiteCount = (16*(int)st.rat + 50)/100;
    if(whiteCount>0){
      const uint8_t ring0[12][2] = {
        {0,0},{1,0},{2,0},{3,0},
        {3,1},{3,2},{3,3},
        {2,3},{1,3},{0,3},
        {0,2},{0,1}
      };
      const uint8_t ring1[4][2] = { {1,1},{2,1},{2,2},{1,2} };
      int placed=0;
      for(int k=0;k<12 && placed<whiteCount;k++){
        setMatrixPix(outIdx, ring0[k][0], ring0[k][1], wRGB); placed++;
      }
      for(int k=0;k<4 && placed<whiteCount;k++){
        setMatrixPix(outIdx, ring1[k][0], ring1[k][1], wRGB); placed++;
      }
    }
  } else {
    for(int i=0;i<n;i++){
      int phys = st.out[outIdx].flip ? (n-1-i) : i;
      if(phys>=0 && phys<MAX_PIX) ledBufs[outIdx][phys]=hueRGB;
    }

    int whiteCount = (n*(int)st.rat + 50)/100;
    if(whiteCount>0){
      int l=0,r=n-1,placed=0;
      while(placed<whiteCount && l<=r){
        int pl = st.out[outIdx].flip ? (n-1-l) : l;
        if(pl>=0 && pl<MAX_PIX) ledBufs[outIdx][pl]=wRGB;
        placed++;
        if(placed>=whiteCount) break;
        if(r!=l){
          int pr = st.out[outIdx].flip ? (n-1-r) : r;
          if(pr>=0 && pr<MAX_PIX) ledBufs[outIdx][pr]=wRGB;
          placed++;
        }
        l++; r--;
      }
    }
  }
}

// ---------------- LED Scan Test (only active mapped outs) ----------------
struct ScanTest {
  bool active=false;
  uint8_t out=0;
  uint8_t idx=0;
  uint32_t nextMs=0;
};
static ScanTest scan;

static uint8_t scanLenForOut(uint8_t outIdx){
  if(!OUT5_PRESENT && outIdx==4) return 0;
  int n = pixelsForType(st.out[outIdx].type);
  if(n<=0) return 0;
  if(n>MAX_PIX) n = MAX_PIX;
  return (uint8_t)n;
}

static void startScan(){
  scan.active=true;
  scan.out=0;
  scan.idx=0;
  scan.nextMs=0;
}

static void runScan(){
  if(!scan.active) return;
  uint32_t now=millis();
  if(scan.nextMs!=0 && (int32_t)(now - scan.nextMs) < 0) return;

  while(scan.out < OUTS_TOTAL){
    uint8_t n = scanLenForOut(scan.out);
    if(n==0){ scan.out++; scan.idx=0; continue; }
    break;
  }

  if(scan.out >= OUTS_TOTAL){
    scan.active=false;
    clearAll();
    FastLED.show();
    return;
  }

  uint8_t n = scanLenForOut(scan.out);
  if(scan.idx >= n){
    scan.out++;
    scan.idx=0;
    scan.nextMs = now + 10;
    return;
  }

  clearAll();
  ledBufs[scan.out][scan.idx] = CRGB::White;
  FastLED.show();

  scan.idx++;
  scan.nextMs = now + 18;
}

static void showLeds(){
  if(scan.active){
    FastLED.setMaxPowerInVoltsAndMilliamps(5, clamp16(st.ma,0,MAX_MA));
    runScan();
    return;
  }

  uint32_t mA = clamp16(st.ma, 0, MAX_MA);
  if(mA==0) mA=1500;

  bool giOn=false;
  sensorAggregate(giOn);

  clearAll();
  if(giOn){
    for(int o=0;o<OUTS_TOTAL;o++){
      if(!OUT5_PRESENT && o==4) st.out[4].type = OUT_OFF;
      renderSolidOut(o);
    }
  }

  FastLED.setMaxPowerInVoltsAndMilliamps(5, mA);
  FastLED.show();
}

// ---------------- web helpers ----------------
static bool readJsonBody(JsonDocument& doc){
  if(!server.hasArg("plain")) return false;
  return deserializeJson(doc, server.arg("plain")) == DeserializationError::Ok;
}

// =====================================================
// WIFI MANAGER (fixed: keep AP during connect)
// =====================================================
enum WifiRunMode : uint8_t { WM_AP_ONLY=0, WM_CONNECTING=1, WM_STA_OK=2 };
static WifiRunMode wm = WM_AP_ONLY;

static uint32_t staAttemptStartMs = 0;
static uint32_t staLastGoodMs = 0;
static uint32_t nextRetryMs = 0;
static uint8_t  failCount = 0;

static const uint32_t STA_CONNECT_TIMEOUT_MS = 20000;
static const uint32_t STA_DROP_GRACE_MS      = 12000;
static const uint32_t RETRY_BASE_MS          = 15000;

static char mdnsHost[32] = {0};

static WiFiUDP udp;
static const uint16_t UDP_PORT = 4210;
static uint32_t nextAnnounceMs = 0;

struct Peer {
  char host[32];
  char ip[16];
  char name[24];
  uint32_t lastSeenMs;
};
static Peer peers[12];

static void peersClear(){
  for(auto &p: peers){
    p.host[0]=0; p.ip[0]=0; p.name[0]=0; p.lastSeenMs=0;
  }
}
static void peersUpsert(const char* host,const char* ip,const char* name){
  if(!host || !host[0] || !ip || !ip[0]) return;
  if(mdnsHost[0] && strcmp(host, mdnsHost)==0) return;

  for(auto &p: peers){
    if(p.host[0] && strcmp(p.host, host)==0){
      safeStrcpy(p.ip,sizeof(p.ip),ip);
      safeStrcpy(p.name,sizeof(p.name),name?name:"");
      p.lastSeenMs = millis();
      return;
    }
  }
  int best=-1;
  uint32_t oldest=0xFFFFFFFF;
  for(int i=0;i<(int)(sizeof(peers)/sizeof(peers[0]));i++){
    if(peers[i].host[0]==0){ best=i; break; }
    if(peers[i].lastSeenMs < oldest){ oldest=peers[i].lastSeenMs; best=i; }
  }
  if(best>=0){
    safeStrcpy(peers[best].host,sizeof(peers[best].host),host);
    safeStrcpy(peers[best].ip,sizeof(peers[best].ip),ip);
    safeStrcpy(peers[best].name,sizeof(peers[best].name),name?name:"");
    peers[best].lastSeenMs = millis();
  }
}

static void makeMdnsHost(){
  uint8_t mac[6]; WiFi.macAddress(mac);
  snprintf(mdnsHost, sizeof(mdnsHost), "litpin-%02X%02X", mac[4], mac[5]);
}

static void ensureAP(){
  // Ensure AP exists and is stable at 192.168.4.1
  IPAddress ip(192,168,4,1), gw(192,168,4,1), sn(255,255,255,0);

  wifi_mode_t m = WiFi.getMode();
  bool apOn = (m == WIFI_AP) || (m == WIFI_AP_STA);

  if(!apOn){
    WiFi.mode(WIFI_AP);
  }
  WiFi.softAPConfig(ip, gw, sn);

  // softAP() is safe to call repeatedly; it’ll keep it alive.
  WiFi.softAP(AP_SSID, AP_PASS);
}

static void startAPOnly(){
  WiFi.mode(WIFI_AP);
  ensureAP();
  wm = WM_AP_ONLY;
  MDNS.end();
}

static void startSTAConnect(){
  if(!wifiCfg.hasSTA || wifiCfg.ssid[0]==0) {
    startAPOnly();
    return;
  }

  // IMPORTANT: keep AP alive while attempting to connect
  WiFi.mode(WIFI_AP_STA);
  ensureAP();

  WiFi.setSleep(false);
  makeMdnsHost();
  WiFi.setHostname(mdnsHost);

  // Clear any ongoing scan before begin (prevents weird join failures)
  WiFi.scanDelete();

  // Disconnect only STA side; keep AP alive
  WiFi.disconnect(false /*wifioff*/, true /*eraseAP*/);
  delay(80);

  DBG("[WIFI] begin ssid='%s'", wifiCfg.ssid);
  WiFi.begin(wifiCfg.ssid, wifiCfg.pass);

  staAttemptStartMs = millis();
  wm = WM_CONNECTING;
}

static void onStaConnected(){
  wm = WM_STA_OK;
  staLastGoodMs = millis();
  failCount = 0;
  staAttemptStartMs = 0;
  nextRetryMs = 0;

  // Now that STA is up, we can shut AP off if you want “LAN-only”
  WiFi.softAPdisconnect(true);

  if(mdnsHost[0]==0) makeMdnsHost();
  MDNS.end();
  if(MDNS.begin(mdnsHost)){
    MDNS.addService("http","tcp",80);
  }

  udp.stop();
  udp.begin(UDP_PORT);
  peersClear();
  nextAnnounceMs = 0;

  DBG("[WIFI] STA OK: ip=%s host=%s.local", WiFi.localIP().toString().c_str(), mdnsHost);
}

static void scheduleRetry(){
  uint32_t now = millis();
  uint32_t backoff = RETRY_BASE_MS + (uint32_t)failCount * 8000UL;
  nextRetryMs = now + backoff;
}

static void wifiTick(){
  uint32_t now = millis();
  wl_status_t s = WiFi.status();

  if(wm == WM_STA_OK){
    if(s == WL_CONNECTED){
      staLastGoodMs = now;

      if((int32_t)(now - nextAnnounceMs) >= 0){
        nextAnnounceMs = now + 2000;
        String msg = String("LITPIN|") + mdnsHost + "|" + WiFi.localIP().toString() + "|" + st.unitName;
        udp.beginPacket(IPAddress(255,255,255,255), UDP_PORT);
        udp.write((const uint8_t*)msg.c_str(), msg.length());
        udp.endPacket();
      }

      int psize = udp.parsePacket();
      if(psize > 0){
        char buf[180]; int n = udp.read(buf, sizeof(buf)-1);
        if(n>0){
          buf[n]=0;
          if(strncmp(buf,"LITPIN|",7)==0){
            char *a = buf+7;
            char *b = strchr(a,'|'); if(!b) return;
            *b++=0;
            char *c = strchr(b,'|'); if(!c) return;
            *c++=0;
            peersUpsert(a, b, c);
          }
        }
      }
      return;
    }

    // dropped
    if(now - staLastGoodMs > STA_DROP_GRACE_MS){
      DBG("[WIFI] STA dropped -> AP fallback");
      startAPOnly();
      failCount++;
      scheduleRetry();
    }
    return;
  }

  if(wm == WM_CONNECTING){
    if(s == WL_CONNECTED){
      onStaConnected();
      return;
    }
    if(now - staAttemptStartMs > STA_CONNECT_TIMEOUT_MS){
      DBG("[WIFI] connect timeout (failCount=%u)", failCount+1);
      failCount++;
      startAPOnly();
      scheduleRetry();
      return;
    }
    return;
  }

  // AP only background retry if creds exist
  if(wm == WM_AP_ONLY){
    if(wifiCfg.hasSTA && wifiCfg.ssid[0]!=0 && nextRetryMs!=0 && (int32_t)(now - nextRetryMs) >= 0){
      DBG("[WIFI] background retry...");
      startSTAConnect();
    }
  }
}

// =====================================================
// Async WiFi Scan (works even with AP clients connected)
// =====================================================
static uint32_t scanStartMs = 0;
static bool scanRequested = false;

static void handleWifiScan(){
  // always allow scanning while AP clients are connected
  WiFi.mode(WIFI_AP_STA);
  ensureAP();
  WiFi.setSleep(false);

  int done = WiFi.scanComplete(); // -1 running, -2 not started, >=0 count
  if(done == WIFI_SCAN_RUNNING){
    if(scanRequested && (millis() - scanStartMs) > 15000){
      WiFi.scanDelete();
      scanRequested = false;
      StaticJsonDocument<256> doc;
      doc["msg"]="scan timeout (try again)";
      doc["status"]="timeout";
      doc.createNestedArray("nets");
      String out; serializeJson(doc,out);
      server.sendHeader("Cache-Control","no-store");
      server.send(200,"application/json",out);
      return;
    }
    StaticJsonDocument<256> doc;
    doc["msg"]="scanning...";
    doc["status"]="running";
    doc.createNestedArray("nets");
    String out; serializeJson(doc,out);
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json",out);
    return;
  }

  if(done >= 0){
    StaticJsonDocument<4096> doc;
    JsonArray arr = doc.createNestedArray("nets");

    int limit = done;
    if(limit > 25) limit = 25;

    for(int i=0;i<limit;i++){
      JsonObject o = arr.createNestedObject();
      o["ssid"] = WiFi.SSID(i);
      o["rssi"] = WiFi.RSSI(i);
      o["enc"]  = (int)WiFi.encryptionType(i);
    }

    WiFi.scanDelete();
    scanRequested=false;

    doc["msg"]="";
    doc["status"]="done";

    String out; serializeJson(doc,out);
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json",out);
    return;
  }

  // start new async scan
  WiFi.scanDelete();
  scanRequested=true;
  scanStartMs = millis();
  WiFi.scanNetworks(true /*async*/, true /*hidden*/);

  StaticJsonDocument<256> doc;
  doc["msg"]="scanning...";
  doc["status"]="started";
  doc.createNestedArray("nets");
  String out; serializeJson(doc,out);
  server.sendHeader("Cache-Control","no-store");
  server.send(200,"application/json",out);
}

// =====================================================
// UI HTML
// =====================================================
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html><head>
<meta name=viewport content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>LitPin</title>
<style>
:root{--g:#00ff2a;--fg:#00ff2a;--dim:#00ff2a;--card:rgba(5,5,5,.78);--brd:#033;--btnbg:rgba(0,26,10,.85);--shadow:0 0 0 1px #021 inset}
*{box-sizing:border-box}
body{margin:14px;font-family:system-ui,Segoe UI,Roboto,Arial;color:var(--fg);max-width:920px}
h1{margin:0 0 10px 0;color:var(--g);letter-spacing:.6px}
.small{opacity:.92;font-size:12px;color:var(--dim);line-height:1.4}
.card{padding:14px;border:1px solid var(--brd);border-radius:18px;margin:12px 0;background:var(--card);box-shadow:var(--shadow);backdrop-filter:blur(8px);position:relative;z-index:10}
.row{display:flex;gap:12px;align-items:center;margin:12px 0}
label{flex:1;min-width:220px}
input[type=range]{flex:3;height:46px;min-width:0}
.val{width:140px;text-align:right;font-variant-numeric:tabular-nums;color:var(--g);font-weight:900;letter-spacing:.3px}
.btnrow{display:flex;flex-wrap:wrap;gap:10px;align-items:center}
button{padding:14px 16px;border-radius:14px;border:1px solid #0a3;background:var(--btnbg);color:var(--g);font-weight:900;letter-spacing:.4px;min-width:120px;box-shadow:0 0 18px rgba(0,255,42,.10)}
button:active{transform:scale(.98)}
.pill{padding:8px 12px;border-radius:999px;border:1px solid #053;background:rgba(0,16,8,.85);color:var(--g);font-weight:900;letter-spacing:.3px;display:inline-block}
.ok{color:var(--g)}.bad{color:#00ff2a}.hide{display:none}.sep{height:1px;background:#022;margin:14px 0}
.bg{position:fixed;inset:0;z-index:-6;background:radial-gradient(circle at 18% 18%, rgba(0,255,50,.55), transparent 58%),radial-gradient(circle at 82% 24%, rgba(0,255,160,.34), transparent 64%),radial-gradient(circle at 50% 85%, rgba(0,140,30,.70), transparent 72%),linear-gradient(180deg,#000 0%,#001a06 45%,#000 100%)}
.overlay{position:fixed;inset:0;z-index:-5;background:radial-gradient(circle at 50% 10%, rgba(0,0,0,.25), rgba(0,0,0,.88)),linear-gradient(180deg, rgba(0,0,0,.22), rgba(0,0,0,.92))}
body::after{content:"";position:fixed;inset:0;pointer-events:none;background:repeating-linear-gradient(0deg,rgba(255,255,255,.02) 0,rgba(255,255,255,.02) 1px,transparent 1px,transparent 3px);opacity:.08;z-index:0}
.tabs{display:flex;gap:10px;flex-wrap:wrap;margin:10px 0}
.tabs button{min-width:140px;flex:1 1 140px}
canvas#mon{width:100%;max-width:860px;height:240px;border-radius:16px;border:1px solid #033;background:rgba(0,0,0,.35);display:block}
input[type=range]{-webkit-appearance:none;appearance:none;height:46px;background:transparent;--p:0%;}
input[type=range]::-webkit-slider-runnable-track{height:12px;border-radius:999px;border:1px solid #053;background:linear-gradient(90deg, rgba(0,255,42,.40) var(--p), rgba(0,0,0,.65) var(--p));box-shadow:inset 0 0 0 1px rgba(0,255,42,.08);}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:30px;height:30px;border-radius:50%;margin-top:-10px;background:var(--g);border:2px solid rgba(0,0,0,.85);box-shadow:0 0 14px rgba(0,255,42,.25);}
input[type=range]::-moz-range-track{height:12px;border-radius:999px;border:1px solid #053;background:rgba(0,0,0,.65);}
input[type=range]::-moz-range-progress{height:12px;border-radius:999px;background:rgba(0,255,42,.40);}
input[type=range]::-moz-range-thumb{width:30px;height:30px;border-radius:50%;background:var(--g);border:2px solid rgba(0,0,0,.85);box-shadow:0 0 14px rgba(0,255,42,.25);}
input[type=range]:focus{outline:none}
.mapGrid{display:flex;flex-direction:column;gap:10px}
.mapRow{border:1px solid var(--brd);background:rgba(0,0,0,.25);border-radius:14px;padding:10px;display:grid;grid-template-columns: 70px 1fr 110px 90px;gap:10px;align-items:center;}
.mapTag{font-weight:900;color:var(--g);letter-spacing:.4px}
.mapRow select{height:42px;border-radius:12px;border:1px solid #053;background:rgba(0,0,0,.35);color:var(--g);padding:0 12px;}
.mapRow button{min-width:0;padding:12px 12px}
a{color:var(--g);text-decoration:none;border-bottom:1px solid rgba(0,255,42,.35)}
a:hover{border-bottom-color:rgba(0,255,42,.9)}
@media (max-width:720px){
  body{margin:10px}
  .tabs button{min-width:0;flex:1 1 45%}
  .row{flex-direction:column;align-items:stretch;gap:8px}
  label{min-width:0}
  .val{width:auto;text-align:left}
  button{min-width:0;flex:1 1 auto}
  .btnrow button{flex:1 1 100%}
  canvas#mon{height:180px}
  .pill{display:block;width:100%;text-align:center}
  select, input[type=text], input[type=password]{width:100%}
  .mapRow{grid-template-columns: 60px 1fr;grid-template-areas:"tag sel" "rot test";}
  .mapRow .tag{grid-area:tag}
  .mapRow .sel{grid-area:sel}
  .mapRow .rot{grid-area:rot}
  .mapRow .test{grid-area:test}
}
</style></head><body>
<div class=bg></div><div class=overlay></div>

<h1>LitPin</h1>

<div class=tabs>
  <button id=tHome class=active onclick="showTab('home')">HOME</button>
  <button id=tMap onclick="showTab('map')">MAPPING</button>
  <button id=tSet onclick="showTab('set')">SETTINGS</button>
  <button id=tWifi onclick="showTab('wifi')">WIFI</button>
</div>

<div id=tabHome class=card>
  <div class=btnrow style="justify-content:space-between">
    <span class=small></span>
    <div class=btnrow>
      <button onclick=save()>SAVE</button>
      <span id=status class=small>...</span>
    </div>
  </div>

  <div class=sep></div>
  <span class=pill>OUTPUT MONITORS</span>
  <div style="margin-top:10px"><canvas id=mon width=860 height=240></canvas></div>

  <div class=sep></div>
  <div class=row><label>Hue</label><input id=hue type=range min=0 max=360 step=1 oninput="paintRange(this); applySoon()"><div class=val id=hueV>0</div></div>
  <div class=row><label>Hue Brightness</label><input id=cbr type=range min=0 max=100 step=1 oninput="paintRange(this); applySoon()"><div class=val id=cbrV>0%</div></div>
  <div class=row><label>White Brightness</label><input id=wbr type=range min=0 max=100 step=1 oninput="paintRange(this); applySoon()"><div class=val id=wbrV>0%</div></div>
  <div class=row><label>White Temp</label><input id=wtmp type=range min=1000 max=20000 step=50 oninput="paintRange(this); applySoon()"><div class=val id=wtmpV>6500</div></div>
  <div class=row><label>Ratio</label><input id=rat type=range min=0 max=100 step=1 oninput="paintRange(this); applySoon()"><div class=val id=ratV>0%</div></div>
</div>

<div id=tabMap class="card hide">
  <div class=small>Pick package per OUT. ROTATE = matrix 90deg; strip FLIP.</div>
  <div class=sep></div>
  <div id=mapRows class=mapGrid></div>
</div>

<div id=tabSet class="card hide">
  <div class=small>
    Unit: <b id=uName class=ok>...</b> | AP: <b id=apSsid class=ok>...</b> / <b id=apPass class=ok>...</b>
  </div>
  <div class=sep></div>

  <div class=row>
    <label>Unit Name</label>
    <input id=nameBox style="flex:3;height:44px;border-radius:12px;border:1px solid #053;background:rgba(0,0,0,.35);color:var(--g);padding:0 12px">
    <div class=val><button onclick=saveName()>SAVE NAME</button></div>
  </div>

  <div class=sep></div>
  <div class=small>Sensors: Mode = SENSE / ON / OFF. Assign = GI or FLASHER.</div>
  <div id=sensorRows></div>
</div>

<div id=tabWifi class="card hide">
  <div class=small>
    Mode: <b id=wm class=ok>...</b><br>
    AP IP: <b id=apIP class=ok>...</b><br>
    STA: <b id=staOK class=ok>...</b> | STA IP: <b id=staIP class=ok>...</b><br>
    Host: <b id=host class=ok>...</b> |
    URL: <a id=hostUrl href="#" target="_blank">...</a>
  </div>

  <div class=btnrow style="margin-top:10px">
    <button onclick="copyLink()">COPY LINK</button>
    <button onclick="openLink()">OPEN LINK</button>
    <span class=small id=copyMsg></span>
  </div>

  <div class=sep></div>
  <div class=row><label>WiFi SSID</label><input id=ssid style="flex:3;height:44px;border-radius:12px;border:1px solid #053;background:rgba(0,0,0,.35);color:var(--g);padding:0 12px"><div class=val></div></div>
  <div class=row><label>WiFi Password</label><input id=pass type=password style="flex:3;height:44px;border-radius:12px;border:1px solid #053;background:rgba(0,0,0,.35);color:var(--g);padding:0 12px"><div class=val></div></div>

  <div class=btnrow>
    <button onclick=saveWifi()>CONNECT</button>
    <button onclick=scanWifi()>SCAN</button>
    <select id=netList onchange=pickNet() style="flex:1;height:44px;border-radius:12px;border:1px solid #053;background:rgba(0,0,0,.35);color:var(--g);padding:0 12px">
      <option value="">(scan for networks)</option>
    </select>
    <span class=small id=scanMsg></span>
  </div>

  <div class=sep></div>
  <span class=pill>FOUND UNITS (LAN)</span>
  <div id=peers class=small style="margin-top:10px">...</div>

  <div class=sep></div>
  <span class=pill>DIAG</span>
  <div id=diag class=small style="margin-top:10px">...</div>
</div>

<script>
let t=null;
const OUT_TYPES=[{id:0,name:'OFF'},{id:1,name:'6 PACK STRIP'},{id:2,name:'12 PACK STRIP'},{id:3,name:'24 PACK STRIP'},{id:4,name:'4x4 MATRIX'}];
const qs=(id)=>document.getElementById(id);

function paintRange(el){
  if(!el) return;
  const min = (el.min !== '') ? +el.min : 0;
  const max = (el.max !== '') ? +el.max : 100;
  const val = +el.value;
  const p = (max > min) ? ((val - min) * 100 / (max - min)) : 0;
  el.style.setProperty('--p', p.toFixed(1) + '%');
}
function paintAllRanges(){ document.querySelectorAll('input[type=range]').forEach(paintRange); }

let cache=null;
let ssidDirty=false;
let nameDirty=false;

async function fetchState(){
  try{
    const r=await fetch('/state',{cache:'no-store'});
    if(!r.ok) throw 0;
    cache=await r.json();
    applyStateToUI(cache);
  }catch(e){}
}

async function fetchDiag(){
  try{
    const r=await fetch('/diag',{cache:'no-store'});
    if(!r.ok) throw 0;
    const j=await r.json();
    qs('diag').textContent =
      `status=${j.wl} rssi=${j.rssi} failCount=${j.fail} lastEvent=${j.ev} discReason=${j.reason}`;
  }catch(e){}
}

function showTab(n){
  qs('tabHome').classList.toggle('hide',n!=='home');
  qs('tabMap').classList.toggle('hide',n!=='map');
  qs('tabSet').classList.toggle('hide',n!=='set');
  qs('tabWifi').classList.toggle('hide',n!=='wifi');
  qs('tHome').classList.toggle('active',n==='home');
  qs('tMap').classList.toggle('active',n==='map');
  qs('tSet').classList.toggle('active',n==='set');
  qs('tWifi').classList.toggle('active',n==='wifi');
}

function syncVals(){
  qs('hueV').textContent=qs('hue').value;
  qs('cbrV').textContent=qs('cbr').value+'%';
  qs('wbrV').textContent=qs('wbr').value+'%';
  qs('wtmpV').textContent=qs('wtmp').value;
  qs('ratV').textContent=qs('rat').value+'%';
}
function applySoon(){syncVals(); if(t) clearTimeout(t); t=setTimeout(apply,120);}

async function apply(){
  const st=qs('status');
  try{
    const body={
      hue:+qs('hue').value,
      cbr:+qs('cbr').value,
      wbr:+qs('wbr').value,
      wtmp:+qs('wtmp').value,
      rat:+qs('rat').value,
      outs: currentOutPayload(),
      sensors: currentSensorPayload(),
    };
    const r=await fetch('/set',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
    if(!r.ok) throw 0;
    st.innerHTML='<span class=ok>APPLIED</span>';
    await fetchState();
  }catch(e){st.innerHTML='<span class=bad>APPLY FAILED</span>'}
}

async function save(){
  try{
    const r=await fetch('/save',{method:'POST'});
    qs('status').innerHTML=r.ok?'<span class=ok>SAVED</span>':'<span class=bad>SAVE FAIL</span>';
  }catch(e){qs('status').innerHTML='<span class=bad>SAVE FAIL</span>'}
}

async function saveWifi(){
  try{
    const body={ssid:qs('ssid').value||'',pass:qs('pass').value||''};
    const r=await fetch('/wifi',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
    qs('status').innerHTML=r.ok?'<span class=ok>CONNECTING...</span>':'<span class=bad>WIFI FAIL</span>';
    if(r.ok) ssidDirty=false;
  }catch(e){qs('status').innerHTML='<span class=bad>WIFI FAIL</span>'}
}

// ASYNC scan poller
async function scanWifi(){
  const msg=qs('scanMsg');
  const sel=qs('netList');

  msg.textContent='scanning...';
  sel.innerHTML='<option value="">(scanning...)</option>';

  const t0 = Date.now();
  while(true){
    try{
      const r=await fetch('/wifiscan',{cache:'no-store'});
      if(!r.ok) throw 0;
      const j=await r.json();

      const nets=(j.nets||[]);
      const status=j.status||'';
      const m=j.msg||'';

      if(!nets.length && (status==='running' || status==='started')){
        msg.textContent = m || 'scanning...';
        if(Date.now() - t0 > 16000){
          msg.textContent = 'scan timed out';
          sel.innerHTML='<option value="">(scan timed out)</option>';
          return;
        }
        await new Promise(res=>setTimeout(res,500));
        continue;
      }

      nets.sort((a,b)=>(b.rssi||-999)-(a.rssi||-999));
      sel.innerHTML='<option value="">(select one)</option>';
      for(const n of nets){
        const opt=document.createElement('option');
        opt.value=n.ssid||'';
        opt.textContent=`${n.ssid||'(hidden)'} (${n.rssi} dBm)`;
        sel.appendChild(opt);
      }
      msg.textContent = m ? m : (nets.length ? '' : 'none found');

      if(sel.options.length>1){
        sel.selectedIndex=1;
        pickNet();
      }
      return;

    }catch(e){
      msg.textContent='scan failed';
      sel.innerHTML='<option value="">(scan failed)</option>';
      return;
    }
  }
}

function pickNet(){
  const sel=qs('netList');
  if(sel && sel.value){
    qs('ssid').value=sel.value;
    ssidDirty=true;
    qs('pass').focus();
  }
}

async function saveName(){
  try{
    const body={name:qs('nameBox').value||''};
    const r=await fetch('/name',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
    qs('status').innerHTML=r.ok?'<span class=ok>NAME SAVED</span>':'<span class=bad>NAME FAIL</span>';
    if(r.ok) nameDirty=false;
    await fetchState();
  }catch(e){qs('status').innerHTML='<span class=bad>NAME FAIL</span>'}
}

function bestLink(){
  if(!cache) return '';
  if(cache.sta && cache.host) return `http://${cache.host}.local/`;
  if(cache.ap_ip) return `http://${cache.ap_ip}/`;
  return '';
}
async function copyLink(){
  const msg=qs('copyMsg');
  const link=bestLink();
  if(!link){ msg.textContent='(no link yet)'; return; }
  try{
    await navigator.clipboard.writeText(link);
    msg.textContent='copied!';
    setTimeout(()=>msg.textContent='',1500);
  }catch(e){
    msg.textContent='copy blocked (browser)';
  }
}
function openLink(){
  const link=bestLink();
  if(link) window.open(link,'_blank');
}

// ----- monitor drawing (same as your build) -----
const mon=qs('mon'); const mctx=mon.getContext('2d');
function hsvToRgb(h,s,v){h=((h%360)+360)%360; s=Math.max(0,Math.min(1,s)); v=Math.max(0,Math.min(1,v));
  const c=v*s,x=c*(1-Math.abs(((h/60)%2)-1)),m=v-c; let r=0,g=0,b=0;
  if(h<60){r=c;g=x;} else if(h<120){r=x;g=c;} else if(h<180){g=c;b=x;} else if(h<240){g=x;b=c;} else if(h<300){r=x;b=c;} else {r=c;b=x;}
  return [Math.round((r+m)*255),Math.round((g+m)*255),Math.round((b+m)*255)];}
function lerp(a,b,t){return a+(b-a)*t;}
function whiteTempRgb(t01){t01=Math.max(0,Math.min(1,t01)); const warm=[255,210,140], cool=[210,235,255];
  return [Math.round(lerp(warm[0],cool[0],t01)),Math.round(lerp(warm[1],cool[1],t01)),Math.round(lerp(warm[2],cool[2],t01))];}
function xy4(x,y){return (y&1)?(y*4+(3-x)):(y*4+x);}
function rotCoord4(x,y,rot){
  rot&=3;
  if(rot===0) return [x,y];
  if(rot===1) return [3-y,x];
  if(rot===2) return [3-x,3-y];
  return [y,3-x];
}
function setPixMx(pixPhys,x,y,rot,color){
  const rc=rotCoord4(x,y,rot);
  pixPhys[xy4(rc[0],rc[1])] = color;
}
const ringCoords4=[
  [[0,0],[1,0],[2,0],[3,0],[3,1],[3,2],[3,3],[2,3],[1,3],[0,3],[0,2],[0,1]],
  [[1,1],[2,1],[2,2],[1,2]]
];

function renderSolidPix(kind,u,ocfg,pixN){
  const hueRGB=hsvToRgb(u.shue,1,u.cbr/100);
  const wRaw=whiteTempRgb(u.wtmp/20000);
  const wRGB=[Math.round(wRaw[0]*(u.wbr/100)),Math.round(wRaw[1]*(u.wbr/100)),Math.round(wRaw[2]*(u.wbr/100))];

  if(kind==='mx4'){
    let pixPhys=new Array(16);
    for(let i=0;i<16;i++) pixPhys[i]=hueRGB;

    const whiteCount=Math.round(16*(u.rat/100));
    let placed=0;
    for(const ring of ringCoords4){
      for(const [x,y] of ring){
        if(placed>=whiteCount) break;
        setPixMx(pixPhys,x,y,ocfg.rot||0,wRGB);
        placed++;
      }
      if(placed>=whiteCount) break;
    }
    return pixPhys;
  }

  let base=new Array(pixN);
  for(let i=0;i<pixN;i++) base[i]=hueRGB;

  const whiteCount=Math.round(pixN*(u.rat/100));
  let l=0,r=pixN-1,placed=0;
  while(placed<whiteCount && l<=r){
    base[l]=wRGB; placed++;
    if(placed>=whiteCount) break;
    if(r!==l){ base[r]=wRGB; placed++; }
    l++; r--;
  }

  let pixPhys=new Array(pixN);
  const flip=ocfg.flip?1:0;
  for(let j=0;j<pixN;j++){
    const src = flip ? (pixN-1-j) : j;
    pixPhys[j]=base[src];
  }
  return pixPhys;
}

function drawMonitor(){
  if(!cache) return requestAnimationFrame(drawMonitor);
  const u=cache;

  const active=[];
  for(let i=0;i<5;i++){
    if(u.outs[i].type!==0) active.push(i);
  }

  const W=mon.width,H=mon.height;
  mctx.clearRect(0,0,W,H);

  if(active.length===0){
    mctx.fillStyle="rgba(0,255,42,.45)";
    mctx.font="800 16px system-ui";
    mctx.fillText("No outputs mapped", 18, 32);
    return requestAnimationFrame(drawMonitor);
  }

  const pad=14,gap=12;
  const N=active.length;
  const cols = (N>=3)?3 : (N===2)?2 : 1;
  const rows = Math.ceil(N/cols);
  const tileW = (W - pad*2 - gap*(cols-1)) / cols;
  const tileH = (H - pad*2 - gap*(rows-1)) / rows;

  for(let k=0;k<N;k++){
    const i = active[k];
    const cx = k % cols;
    const cy = (k / cols) | 0;
    const x0 = pad + cx*(tileW+gap);
    const y0 = pad + cy*(tileH+gap);

    mctx.strokeStyle="rgba(0,255,42,.30)";
    mctx.lineWidth=1;
    mctx.strokeRect(x0+2,y0+2,tileW-4,tileH-4);

    const type=u.outs[i].type;
    const ocfg=u.outs[i];

    if(type===4){
      const pix=renderSolidPix('mx4',u,ocfg,16);
      const dotR=Math.max(4,Math.min(tileW,tileH)/10);
      for(let y=0;y<4;y++) for(let x=0;x<4;x++){
        const [r,g,b]=pix[xy4(x,y)];
        const px=x0+(x+.5)*(tileW/4), py=y0+(y+.5)*(tileH/4);
        mctx.beginPath(); mctx.fillStyle=`rgb(${r},${g},${b})`; mctx.arc(px,py,dotR,0,Math.PI*2); mctx.fill();
      }
    }else{
      const pixN = (type===3)?24: (type===2)?12:6;
      const pix=renderSolidPix('strip',u,ocfg,pixN);

      const Np=pix.length; const c=Math.min(Np,12); const rws=Math.ceil(Np/c);
      const cellW=tileW/c, cellH=tileH/rws;
      const dotR=Math.max(2.5,Math.min(cellW,cellH)*.25);
      for(let j=0;j<Np;j++){
        const [r,g,b]=pix[j];
        const xx=j%c, yy=(j/c)|0;
        const px=x0+(xx+.5)*cellW, py=y0+(yy+.5)*cellH;
        mctx.beginPath(); mctx.fillStyle=`rgb(${r},${g},${b})`; mctx.arc(px,py,dotR,0,Math.PI*2); mctx.fill();
      }
    }
  }

  requestAnimationFrame(drawMonitor);
}
requestAnimationFrame(drawMonitor);

// ----- mapping + sensors UI -----
function rotOut(i){
  if(!cache) return;
  const o=cache.outs[i];
  if(o.type===4) o.rot = (((o.rot||0)+1)&3);
  else if(o.type!==0) o.flip = o.flip?0:1;
  applySoon();
}

function buildMapping(){
  const wrap=qs('mapRows'); wrap.innerHTML='';
  for(let i=0;i<5;i++){
    const row=document.createElement('div');
    row.className='mapRow';
    row.innerHTML=`
      <div class="mapTag tag">OUT${i+1}</div>
      <select class="sel" data-out="${i}"></select>
      <button class="rot" onclick="rotOut(${i})">ROTATE</button>
      <button class="test" onclick="testOut(${i})">TEST</button>
    `;
    wrap.appendChild(row);

    const sel=row.querySelector('select');
    OUT_TYPES.forEach(t=>{
      const o=document.createElement('option');
      o.value=t.id; o.textContent=t.name;
      sel.appendChild(o);
    });
    sel.addEventListener('change',()=>{
      if(cache){
        cache.outs[i].type=+sel.value;
        applySoon();
      }
    });
  }
}

function buildSensors(){
  const wrap=qs('sensorRows'); wrap.innerHTML='';
  for(let i=0;i<3;i++){
    const d=document.createElement('div'); d.className='card';
    d.innerHTML=`<div class=pill>S${i+1}</div>
    <div class=row><label>Mode</label>
      <select data-sm="${i}" style="flex:3;height:44px;border-radius:12px;border:1px solid #053;background:rgba(0,0,0,.35);color:var(--g);padding:0 12px">
        <option value=0>SENSE</option><option value=1>ON</option><option value=2>OFF</option>
      </select>
      <div class=val><button onclick="ledScan()">TEST</button></div>
    </div>
    <div class=row><label>Assign</label>
      <select data-sa="${i}" style="flex:3;height:44px;border-radius:12px;border:1px solid #053;background:rgba(0,0,0,.35);color:var(--g);padding:0 12px">
        <option value=0>GI</option><option value=1>FLASHER</option>
      </select>
    </div>
    <div class=row><label>Threshold</label>
      <input data-st="${i}" type=range min=0 max=4095 step=5 oninput="paintRange(this); applySoon()">
    </div>`;
    wrap.appendChild(d);
    d.querySelector('[data-sm]').addEventListener('change',e=>{ if(cache){ cache.sensors[i].mode=+e.target.value; applySoon(); }});
    d.querySelector('[data-sa]').addEventListener('change',e=>{ if(cache){ cache.sensors[i].assign=+e.target.value; applySoon(); }});
    d.querySelector('[data-st]').addEventListener('input',e=>{ if(cache){ cache.sensors[i].threshold=+e.target.value; }});
  }
}

function currentOutPayload(){ if(!cache) return []; return cache.outs.map(o=>({type:o.type,rot:o.rot||0,flip:o.flip?1:0})); }
function currentSensorPayload(){ if(!cache) return []; return cache.sensors.map(s=>({mode:s.mode,threshold:s.threshold,assign:s.assign})); }

function testOut(i){ fetch('/test?out='+i,{method:'POST'}); }
function ledScan(){ fetch('/scan',{method:'POST'}); }

async function fetchPeers(){
  try{
    const r=await fetch('/peers',{cache:'no-store'});
    if(!r.ok) throw 0;
    const j=await r.json();
    const list=j.peers||[];
    if(!list.length){ qs('peers').textContent='(none yet)'; return; }
    qs('peers').innerHTML = list.map(p=>{
      const url = p.host ? `http://${p.host}.local/` : `http://${p.ip}/`;
      const label = (p.name? (p.name+' - ') : '') + (p.host? (p.host+'.local') : p.ip);
      return `<div><a href="${url}" target="_blank">${label}</a> <span class="small">(${p.age_s}s)</span></div>`;
    }).join('');
  }catch(e){}
}

function applyStateToUI(u){
  qs('uName').textContent=u.name||'...';
  qs('apSsid').textContent = u.ap_ssid || 'Litpin';
  qs('apPass').textContent = u.ap_pass || '12345678';

  const nb=qs('nameBox');
  if(!nameDirty && document.activeElement!==nb) nb.value=u.name||'';

  qs('wm').textContent = u.wm || '...';
  qs('apIP').textContent=u.ap_ip||'...';
  qs('staOK').textContent=(u.sta? 'connected':'not connected');
  qs('staIP').textContent=u.sta_ip||'...';

  qs('host').textContent = u.host || '...';
  const hu = qs('hostUrl');
  const url = u.host ? ('http://' + u.host + '.local/') : '#';
  hu.href = url;
  hu.textContent = u.host ? (u.host + '.local') : '...';

  const ss=qs('ssid');
  if(!ssidDirty && document.activeElement!==ss) ss.value=u.sta_ssid||'';

  qs('hue').value=u.shue||0;
  qs('cbr').value=u.cbr||0;
  qs('wbr').value=u.wbr||0;
  qs('wtmp').value=u.wtmp||6500;
  qs('rat').value=u.rat||0;
  syncVals();

  document.querySelectorAll('#mapRows select[data-out]').forEach(sel=>{
    const i=+sel.getAttribute('data-out');
    sel.value=u.outs[i].type;
  });

  document.querySelectorAll('#sensorRows select[data-sm]').forEach(sel=>{
    const i=+sel.getAttribute('data-sm');
    sel.value=u.sensors[i].mode;
  });
  document.querySelectorAll('#sensorRows select[data-sa]').forEach(sel=>{
    const i=+sel.getAttribute('data-sa');
    sel.value=u.sensors[i].assign;
  });
  document.querySelectorAll('#sensorRows input[data-st]').forEach(sl=>{
    const i=+sl.getAttribute('data-st');
    sl.value=u.sensors[i].threshold;
    paintRange(sl);
  });

  paintAllRanges();
}

buildMapping(); buildSensors();
qs('ssid').addEventListener('input',()=>{ssidDirty=true;});
qs('nameBox').addEventListener('input',()=>{nameDirty=true;});
fetchState();
setInterval(fetchState,600);
setInterval(fetchPeers,1500);
setInterval(fetchDiag,1200);
</script>
</body></html>
)HTML";

// =====================================================
// WEB HANDLERS
// =====================================================
static void handleRoot(){
  server.sendHeader("Cache-Control","no-store");
  server.send(200,"text/html",INDEX_HTML);
}

static void handleState(){
  StaticJsonDocument<4096> doc;
  doc["name"] = st.unitName;
  doc["ap_ssid"] = AP_SSID;
  doc["ap_pass"] = AP_PASS;

  doc["wm"] = (wm==WM_STA_OK) ? "STA" : (wm==WM_CONNECTING) ? "CONNECTING" : "AP";
  doc["ap_ip"] = WiFi.softAPIP().toString();
  doc["sta"] = (WiFi.status()==WL_CONNECTED) ? 1 : 0;
  doc["sta_ip"] = WiFi.localIP().toString();
  doc["sta_ssid"] = wifiCfg.hasSTA ? wifiCfg.ssid : "";
  doc["host"] = mdnsHost;

  doc["ma"]   = st.ma;
  doc["shue"] = st.shue;
  doc["cbr"]  = st.cbr;
  doc["wbr"]  = st.wbr;
  doc["wtmp"] = st.wtmp;
  doc["rat"]  = st.rat;

  JsonArray outs = doc.createNestedArray("outs");
  for(int i=0;i<OUTS_TOTAL;i++){
    JsonObject o = outs.createNestedObject();
    o["type"]=(int)st.out[i].type;
    o["rot"] =(int)(st.out[i].rot & 3);
    o["flip"]=(int)(st.out[i].flip ? 1 : 0);
  }

  JsonArray sens = doc.createNestedArray("sensors");
  for(int i=0;i<SENS_TOTAL;i++){
    JsonObject s = sens.createNestedObject();
    s["mode"]=(int)st.s[i].mode;
    s["threshold"]=(int)st.s[i].threshold;
    s["assign"]=(int)st.s[i].assign;
    s["value"]=(int)st.s[i].value;
    s["active"]=(int)(st.s[i].active?1:0);
  }

  String out; serializeJson(doc,out);
  server.sendHeader("Cache-Control","no-store");
  server.send(200,"application/json",out);
}

static void handleDiag(){
  StaticJsonDocument<512> doc;
  doc["wl"] = (int)WiFi.status();
  doc["rssi"] = (WiFi.status()==WL_CONNECTED) ? WiFi.RSSI() : -999;
  doc["fail"] = (int)failCount;
  doc["ev"] = (int)lastWifiEvent;
  doc["reason"] = (int)lastDiscReason;
  String out; serializeJson(doc,out);
  server.sendHeader("Cache-Control","no-store");
  server.send(200,"application/json",out);
}

static void handlePeers(){
  StaticJsonDocument<2048> doc;
  JsonArray arr = doc.createNestedArray("peers");

  uint32_t now = millis();
  for(auto &p: peers){
    if(!p.host[0] && !p.ip[0]) continue;
    uint32_t age = (p.lastSeenMs==0) ? 9999 : (now - p.lastSeenMs)/1000;
    if(age > 30) continue;
    JsonObject o = arr.createNestedObject();
    o["host"]=p.host;
    o["ip"]=p.ip;
    o["name"]=p.name;
    o["age_s"]=(int)age;
  }

  String out; serializeJson(doc,out);
  server.sendHeader("Cache-Control","no-store");
  server.send(200,"application/json",out);
}

static void handleSet(){
  StaticJsonDocument<2048> doc;
  if(!readJsonBody(doc)){ server.send(400,"application/json","{\"ok\":0}"); return; }

  if(doc.containsKey("ma"))   st.ma   = clamp16u(doc["ma"].as<int>(),0,MAX_MA);
  if(doc.containsKey("hue"))  st.shue = clamp16u(doc["hue"].as<int>(),0,360);
  if(doc.containsKey("shue")) st.shue = clamp16u(doc["shue"].as<int>(),0,360);
  if(doc.containsKey("cbr"))  st.cbr  = (uint8_t)clamp16(doc["cbr"].as<int>(),0,100);
  if(doc.containsKey("wbr"))  st.wbr  = (uint8_t)clamp16(doc["wbr"].as<int>(),0,100);
  if(doc.containsKey("wtmp")) st.wtmp = clamp16u(doc["wtmp"].as<int>(),1000,20000);
  if(doc.containsKey("rat"))  st.rat  = (uint8_t)clamp16(doc["rat"].as<int>(),0,100);

  if(doc.containsKey("outs") && doc["outs"].is<JsonArray>()){
    JsonArray a=doc["outs"].as<JsonArray>();
    for(int i=0;i<OUTS_TOTAL && i<(int)a.size();i++){
      JsonObject o=a[i];
      if(o.containsKey("type")) st.out[i].type = (uint8_t)clamp16(o["type"].as<int>(),0,4);
      if(o.containsKey("rot"))  st.out[i].rot  = (uint8_t)(o["rot"].as<int>() & 3);
      if(o.containsKey("flip")) st.out[i].flip = (uint8_t)(o["flip"].as<int>() ? 1:0);
    }
  }

  if(doc.containsKey("sensors") && doc["sensors"].is<JsonArray>()){
    JsonArray a=doc["sensors"].as<JsonArray>();
    for(int i=0;i<SENS_TOTAL && i<(int)a.size();i++){
      JsonObject s=a[i];
      if(s.containsKey("mode")) st.s[i].mode = (uint8_t)clamp16(s["mode"].as<int>(),0,2);
      if(s.containsKey("threshold")) st.s[i].threshold = (uint16_t)clamp16(s["threshold"].as<int>(),0,4095);
      if(s.containsKey("assign")) st.s[i].assign = (uint8_t)clamp16(s["assign"].as<int>(),0,1);
    }
  }

  if(!OUT5_PRESENT) st.out[4].type=OUT_OFF;
  server.send(200,"application/json","{\"ok\":1}");
}

static void handleSave(){ saveState(); server.send(200,"application/json","{\"ok\":1}"); }

static void handleWifi(){
  StaticJsonDocument<512> doc;
  if(!readJsonBody(doc)){ server.send(400,"application/json","{\"ok\":0}"); return; }
  const char* ssid = doc["ssid"] | "";
  const char* pass = doc["pass"] | "";
  if(strlen(ssid)==0){ server.send(400,"application/json","{\"ok\":0}"); return; }

  wifiCfg.hasSTA=true;
  safeStrcpy(wifiCfg.ssid,sizeof(wifiCfg.ssid),ssid);
  safeStrcpy(wifiCfg.pass,sizeof(wifiCfg.pass),pass);
  saveWifi();

  failCount = 0;
  nextRetryMs = 0;

  startSTAConnect(); // user-initiated connect
  server.send(200,"application/json","{\"ok\":1}");
}

static void handleName(){
  StaticJsonDocument<256> doc;
  if(!readJsonBody(doc)){ server.send(400,"application/json","{\"ok\":0}"); return; }
  const char* nm = doc["name"] | "";
  if(strlen(nm)>0) safeStrcpy(st.unitName,sizeof(st.unitName),nm);
  saveState();
  server.send(200,"application/json","{\"ok\":1}");
}

static void handleTest(){
  int out = server.hasArg("out") ? server.arg("out").toInt() : 0;
  if(out<0||out>=OUTS_TOTAL) out=0;
  if(!OUT5_PRESENT && out==4){ server.send(200,"application/json","{\"ok\":1}"); return; }

  int n = pixelsForType(st.out[out].type);
  if(n<=0){ server.send(200,"application/json","{\"ok\":1}"); return; }

  clearAll();
  for(int i=0;i<n && i<MAX_PIX;i++) ledBufs[out][i]=CRGB::White;
  FastLED.show();
  delay(120);
  clearAll();
  FastLED.show();
  server.send(200,"application/json","{\"ok\":1}");
}

static void handleScan(){
  startScan();
  server.send(200,"application/json","{\"ok\":1}");
}

// ---------------- WiFi event logger ----------------
static void wifiEvent(WiFiEvent_t event, WiFiEventInfo_t info){
  lastWifiEvent = (uint32_t)event;
  if(event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED){
    lastDiscReason = info.wifi_sta_disconnected.reason;
    DBG("[WIFI] DISCONNECTED reason=%u", (unsigned)lastDiscReason);
  } else if(event == ARDUINO_EVENT_WIFI_STA_GOT_IP){
    DBG("[WIFI] GOT_IP %s", WiFi.localIP().toString().c_str());
  } else if(event == ARDUINO_EVENT_WIFI_STA_CONNECTED){
    DBG("[WIFI] STA_CONNECTED");
  }
}

// ---------------- setup/loop ----------------
void setup(){
  Serial.begin(115200);
  delay(80);

  WiFi.onEvent(wifiEvent);

  loadWifi();
  loadState();

  WiFi.persistent(false);
  WiFi.setSleep(false);

  makeMdnsHost();
  peersClear();

  // Start AP-only always; user presses CONNECT to join
  startAPOnly();

  // schedule background retries if creds exist
  if(wifiCfg.hasSTA && wifiCfg.ssid[0]!=0){
    failCount = 0;
    scheduleRetry();
  }

  FastLED.clear(true);
  FastLED.addLeds<LED_TYPE, PIN_A, COLOR_ORDER>(ledsA, MAX_PIX);
  FastLED.addLeds<LED_TYPE, PIN_B, COLOR_ORDER>(ledsB, MAX_PIX);
  FastLED.addLeds<LED_TYPE, PIN_C, COLOR_ORDER>(ledsC, MAX_PIX);
  FastLED.addLeds<LED_TYPE, PIN_D, COLOR_ORDER>(ledsD, MAX_PIX);
  if(OUT5_PRESENT) FastLED.addLeds<LED_TYPE, PIN_E, COLOR_ORDER>(ledsE, MAX_PIX);

  server.on("/", handleRoot);
  server.on("/state", handleState);
  server.on("/diag", HTTP_GET, handleDiag);
  server.on("/peers", HTTP_GET, handlePeers);
  server.on("/set", HTTP_POST, handleSet);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/wifi", HTTP_POST, handleWifi);
  server.on("/wifiscan", HTTP_GET, handleWifiScan);
  server.on("/name", HTTP_POST, handleName);
  server.on("/test", HTTP_POST, handleTest);
  server.on("/scan", HTTP_POST, handleScan);
  server.begin();

  DBG("[BOOT] AP ready: %s / %s  ip=%s", AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());
  if(wifiCfg.hasSTA) DBG("[BOOT] saved ssid=%s", wifiCfg.ssid);
}

void loop(){
  server.handleClient();

  static uint32_t nextWifi = 0;
  uint32_t now = millis();
  if((int32_t)(now - nextWifi) >= 0){
    wifiTick();
    nextWifi = now + 200;
  }

  updateSensors();
  showLeds();
  delay(3);
}

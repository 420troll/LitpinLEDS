#include <WiFi.h>
#include <WebServer.h>
#include <NeoPixelBus.h>

/*
====================================================================
SENSOR REFERENCE / DESIGN NOTES  (SAVE THIS IN GIT)
====================================================================

Sensor 1: BPW34 Photodiode
- Pin: GPIO34 (ADC1, input-only)
- Suggested: reverse-biased + resistor to 3.3V for speed
- Fast transient detection (strobes)

Sensor 2: LDR Divider
- Pin: GPIO35 (ADC1, input-only)
- Typical divider: 3.3V -> 10k -> ADC -> LDR -> GND
- With this wiring: bright = LOW ADC, dark = HIGH ADC
- Invert in code if desired: 4095 - analogRead(35)

ADC Notes:
- GPIO34/35 are ADC1 => safe with WiFi on ESP32
- Range ~0..4095
- 34/35 input-only, no internal pulls

Design intent:
- BPW34 = fast flash detection
- LDR   = ambient / slower response
====================================================================
*/

// ===================== LED BANKS =====================
const uint8_t  PIN_S12A = 2;   // 12 LEDs
const uint8_t  PIN_S12B = 4;   // 12 LEDs
const uint8_t  PIN_S24A = 18;  // 24 LEDs
const uint8_t  PIN_S24B = 19;  // 24 LEDs

const uint16_t NUM_S12A = 12;
const uint16_t NUM_S12B = 12;
const uint16_t NUM_S24A = 24;
const uint16_t NUM_S24B = 24;

// Hard cap brightness: 60% of 255 ≈ 153
const uint8_t MAX_BRIGHT = 153;

NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt0Ws2812xMethod> stripS12A(NUM_S12A, PIN_S12A);
NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt1Ws2812xMethod> stripS12B(NUM_S12B, PIN_S12B);
NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt2Ws2812xMethod> stripS24A(NUM_S24A, PIN_S24A);
NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt3Ws2812xMethod> stripS24B(NUM_S24B, PIN_S24B);

// ===================== UI STATE =====================
uint8_t  colorBrightness = 128; // 0..255 (then hard-capped)
uint8_t  whiteBrightness = 128; // 0..255 (then hard-capped)
uint16_t hueDeg          = 0;   // 0..360
uint8_t  whiteTemp       = 50;  // 0 warm .. 100 cool

volatile bool dirty = true;

// ===================== AP WIFI =====================
const char* AP_SSID = "PinLights";
const char* AP_PASS = "12345678";
WebServer server(80);

// ===================== HTML =====================
const char MAIN_page[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>PinLights</title>
  <style>
    body{font-family:sans-serif;text-align:center;background:#111;color:#eee;margin:0;padding:10px;}
    h1{margin:10px 0 6px 0;font-size:22px;}
    .box{margin:14px auto;width:92%;max-width:420px;text-align:left;background:#1b1b1b;border-radius:10px;padding:12px;}
    .row{display:flex;justify-content:space-between;font-size:14px;color:#ccc;margin-bottom:6px;}
    input[type=range]{width:100%;}
  </style>
</head>
<body>
  <h1>PinLights</h1>

  <div class="box">
    <div class="row"><span>Color Brightness</span><span id="cb">128</span></div>
    <input type="range" min="0" max="255" value="128" oninput="setV('cBright', this.value, 'cb')">

    <div style="height:10px;"></div>

    <div class="row"><span>White Brightness</span><span id="wb">128</span></div>
    <input type="range" min="0" max="255" value="128" oninput="setV('wBright', this.value, 'wb')">
  </div>

  <div class="box">
    <div class="row"><span>Hue</span><span id="h">0°</span></div>
    <input type="range" min="0" max="360" value="0" oninput="setV('h', this.value, 'h', '°')">

    <div style="height:10px;"></div>

    <div class="row"><span>White Temp</span><span id="wt">50%</span></div>
    <input type="range" min="0" max="100" value="50" oninput="setV('wt', this.value, 'wt', '%')">
  </div>

<script>
let t=null;
function setV(key,val,id,suffix=""){
  document.getElementById(id).innerText = val + suffix;
  if(t) clearTimeout(t);
  t = setTimeout(()=>{ fetch('/set?'+key+'='+encodeURIComponent(val)).catch(()=>{}); }, 40);
}
</script>
</body>
</html>
)HTML";

// ===================== HELPERS =====================
uint8_t clamp255(int v){ if(v<0) return 0; if(v>255) return 255; return (uint8_t)v; }

RgbColor limitBright(const RgbColor& c){
  return RgbColor(
    (uint16_t)c.R * MAX_BRIGHT / 255,
    (uint16_t)c.G * MAX_BRIGHT / 255,
    (uint16_t)c.B * MAX_BRIGHT / 255
  );
}

RgbColor getColorCol() {
  float h = (float)hueDeg / 360.0f;
  RgbColor base(HsbColor(h, 1.0f, 1.0f));
  RgbColor scaled(
    (uint16_t)base.R * colorBrightness / 255,
    (uint16_t)base.G * colorBrightness / 255,
    (uint16_t)base.B * colorBrightness / 255
  );
  return limitBright(scaled);
}

RgbColor getWhiteTempCol() {
  float t = whiteTemp / 100.0f;
  uint8_t warmR = 255, warmG = 120, warmB = 0;
  uint8_t coolR = 210, coolG = 230, coolB = 255;

  uint8_t R = warmR + (int)((coolR - warmR) * t);
  uint8_t G = warmG + (int)((coolG - warmG) * t);
  uint8_t B = warmB + (int)((coolB - warmB) * t);

  RgbColor base(
    (uint16_t)R * whiteBrightness / 255,
    (uint16_t)G * whiteBrightness / 255,
    (uint16_t)B * whiteBrightness / 255
  );
  return limitBright(base);
}

template <typename TStrip>
void applySimpleMix(TStrip& strip, uint16_t count, const RgbColor& w, const RgbColor& c) {
  // Stable baseline mix: even = white, odd = color
  for (uint16_t i = 0; i < count; i++) {
    strip.SetPixelColor(i, (i % 2 == 0) ? w : c);
  }
  strip.Show();
}

void updateAllBanks() {
  RgbColor w = getWhiteTempCol();
  RgbColor c = getColorCol();

  applySimpleMix(stripS12A, NUM_S12A, w, c);
  applySimpleMix(stripS12B, NUM_S12B, w, c);
  applySimpleMix(stripS24A, NUM_S24A, w, c);
  applySimpleMix(stripS24B, NUM_S24B, w, c);
}

// ===================== HTTP =====================
void handleRoot(){ server.send_P(200, "text/html", MAIN_page); }

void handleSet() {
  bool changed = false;

  if (server.hasArg("cBright")) { uint8_t nv = clamp255(server.arg("cBright").toInt()); if(nv!=colorBrightness){ colorBrightness=nv; changed=true; } }
  if (server.hasArg("wBright")) { uint8_t nv = clamp255(server.arg("wBright").toInt()); if(nv!=whiteBrightness){ whiteBrightness=nv; changed=true; } }

  if (server.hasArg("h")) {
    int v = server.arg("h").toInt();
    if (v < 0) v = 0; if (v > 360) v = 360;
    uint16_t nv = (uint16_t)v;
    if (nv != hueDeg) { hueDeg = nv; changed = true; }
  }

  if (server.hasArg("wt")) {
    int v = server.arg("wt").toInt();
    if (v < 0) v = 0; if (v > 100) v = 100;
    uint8_t nv = (uint8_t)v;
    if (nv != whiteTemp) { whiteTemp = nv; changed = true; }
  }

  if (changed) {
    dirty = true;
    Serial.print("SET: cB="); Serial.print(colorBrightness);
    Serial.print(" wB=");     Serial.print(whiteBrightness);
    Serial.print(" h=");      Serial.print(hueDeg);
    Serial.print(" wt=");     Serial.println(whiteTemp);
  }

  server.send(200, "text/plain", "OK");
}

void handleNotFound(){ server.send(404, "text/plain", "Not found"); }

// ===================== SETUP / LOOP =====================
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("\nBOOT: PinLights stripped UI");
  Serial.print("Pins: S12A=2 S12B=4 S24A=18 S24B=19  (MAX_BRIGHT="); Serial.print(MAX_BRIGHT); Serial.println(")");

  stripS12A.Begin(); stripS12A.Show();
  stripS12B.Begin(); stripS12B.Show();
  stripS24A.Begin(); stripS24A.Show();
  stripS24B.Begin(); stripS24B.Show();

  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP start: "); Serial.println(ok ? "OK" : "FAIL");
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");

  updateAllBanks();
  dirty = false;
}

void loop() {
  server.handleClient();

  if (dirty) {
    updateAllBanks();
    dirty = false;
  }

  delay(1);
}

// file: src/main.cpp
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <FastLED.h>
#include <Preferences.h>

// ===================== MATRIX =====================
#define LED_PIN     14
#define NUM_LEDS    64
#define W           8
#define H           8
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
#define MAX_BOARD_BRIGHTNESS 153

CRGB leds[NUM_LEDS];

uint16_t XY(uint8_t x, uint8_t y) {
  uint8_t yy = (H - 1) - y;   // bottom row first
  uint8_t xx = (W - 1) - x;   // right -> left
  return (yy * W) + xx;
}
inline void drawPixel(int x, int y, const CRGB &c) {
  if (x < 0 || x >= W || y < 0 || y >= H) return;
  leds[XY((uint8_t)x, (uint8_t)y)] = c;
}
inline void fillAll(const CRGB &c) { fill_solid(leds, NUM_LEDS, c); }

// ===================== QMI8658 (I2C) =====================
#define I2C_SDA 11
#define I2C_SCL 12

static const uint8_t REG_WHOAMI = 0x00;
static const uint8_t REG_CTRL1  = 0x02;
static const uint8_t REG_CTRL2  = 0x03;
static const uint8_t REG_CTRL3  = 0x04;
static const uint8_t REG_CTRL5  = 0x06;
static const uint8_t REG_CTRL7  = 0x08;
static const uint8_t REG_AX_L   = 0x35; // AX_L..AZ_H (6 bytes)

uint8_t imuAddr = 0;
bool imuOK = false;

bool writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}
bool readBytes(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  size_t got = Wire.requestFrom((int)addr, (int)n);
  if (got != n) return false;
  for (size_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}
bool findQMI8658() {
  for (uint8_t a : { (uint8_t)0x6A, (uint8_t)0x6B }) {
    uint8_t who = 0xFF;
    if (readBytes(a, REG_WHOAMI, &who, 1) && who != 0xFF && who != 0x00) {
      imuAddr = a; return true;
    }
  }
  return false;
}
bool initQMI8658() {
  if (!findQMI8658()) return false;
  if (!writeReg(imuAddr, REG_CTRL1, (1 << 6))) return false; // auto-inc
  if (!writeReg(imuAddr, REG_CTRL2, 0x16)) return false;     // accel (best-effort)
  if (!writeReg(imuAddr, REG_CTRL3, 0x26)) return false;     // gyro  (best-effort)
  writeReg(imuAddr, REG_CTRL5, 0x11);                        // filters (best-effort)
  if (!writeReg(imuAddr, REG_CTRL7, 0x03)) return false;     // enable accel+gyro
  return true;
}
bool readAccelRaw(int16_t &ax, int16_t &ay, int16_t &az) {
  uint8_t b[6];
  if (!readBytes(imuAddr, REG_AX_L, b, 6)) return false;
  ax = (int16_t)((b[1] << 8) | b[0]);
  ay = (int16_t)((b[3] << 8) | b[2]);
  az = (int16_t)((b[5] << 8) | b[4]);
  return true;
}

// ===================== PARAMETERS =====================
struct Params {
  // Color/mix
  uint8_t brightness = 80;
  uint8_t speed      = 120;
  uint8_t mix        = 0;
  uint8_t whiteLvl   = 120;
  int8_t  whiteTemp  = -20;
  uint8_t hue        = 96;
  uint8_t sat        = 255;

  // Shake
  uint32_t shakeThr       = 2500; // sensitivity
  uint16_t shakeDebounceMs= 25;   // lower = more responsive
  uint8_t  shakeEmaDiv    = 2;    // 1..16 (1 = raw)
  uint16_t holdMinMs      = 200;
  uint16_t holdMaxMs      = 1100;
  uint16_t strobePeriodMin= 35;
  uint16_t strobePeriodMax= 220;
  uint8_t  strobeDutyPct  = 20;   // 5..95
  uint32_t bangE          = 9000; // energy for "!"

  // Bounce pattern tunables
  uint8_t  bounceWrapPct      = 20; // 0..100 chance to wrap instead of bounce
  uint8_t  bounceKeepColorPct = 50; // 0..100 chance to keep color on wall
  uint8_t  bounceTurnPct      = 30; // 0..100 chance to randomize direction on wall
  uint8_t  bounceTrailFade    = 40; // 0..255 fade for trail
} P;

Preferences prefs;

// helpers
uint16_t speedInterval(uint8_t s) { return (uint16_t)map(s, 1, 255, 220, 10); }
CRGB whiteWithTemp(uint8_t level, int8_t temp) {
  int16_t r = level, g = level, b = level;
  if (temp < 0) b += temp; else r -= temp;
  r = constrain(r, 0, 255); g = constrain(g, 0, 255); b = constrain(b, 0, 255);
  return CRGB((uint8_t)r, (uint8_t)g, (uint8_t)b);
}
CRGB applyMix(const CRGB &color) {
  CRGB w = whiteWithTemp(P.whiteLvl, P.whiteTemp);
  CRGB out;
  out.r = lerp8by8(color.r, w.r, P.mix);
  out.g = lerp8by8(color.g, w.g, P.mix);
  out.b = lerp8by8(color.b, w.b, P.mix);
  return out;
}
CRGB baseColor() { return applyMix(CHSV(P.hue, P.sat, 255)); }
void applyBrightnessCap() {
  FastLED.setBrightness(min(P.brightness, (uint8_t)MAX_BOARD_BRIGHTNESS));
}

// ===================== PATTERNS =====================
enum PatternID {
  PAT_SOLID_COLOR = 0,
  PAT_RAINBOW,
  PAT_RAINBOW_SWIRL,
  PAT_CHASE_COLOR,
  PAT_CHASE_WHITE,
  PAT_THEATER_CHASE,
  PAT_FLIPFLOP_ALL,
  PAT_FLIPFLOP_ROWS,
  PAT_FLIPFLOP_COLS,
  PAT_SPARKLE,
  PAT_CONFETTI,
  PAT_BOUNCE_DOT,
  PAT_PLASMA,
  PAT_SCROLL_TEXT,
  PAT_COUNT
};
volatile uint8_t currentPat = PAT_RAINBOW;

uint32_t lastStepMs = 0;
bool stepReady(uint32_t now) {
  uint16_t iv = speedInterval(P.speed);
  if (now - lastStepMs >= iv) { lastStepMs = now; return true; }
  return false;
}

void patSolidColor(uint32_t) { fillAll(baseColor()); FastLED.show(); }
void patRainbow(uint32_t now) {
  static uint8_t h = 0; if (!stepReady(now)) return;
  for (int i=0;i<NUM_LEDS;i++) leds[i] = applyMix(CHSV(h + i*6, 255, 255));
  h++; FastLED.show();
}
void patRainbowSwirl(uint32_t now) {
  static uint8_t h = 0; if (!stepReady(now)) return;
  for (int y=0;y<H;y++) for (int x=0;x<W;x++) {
    uint8_t hh = h + x*18 + y*10; drawPixel(x,y, applyMix(CHSV(hh,255,255)));
  }
  h++; FastLED.show();
}
void patChaseColor(uint32_t now) {
  static int pos=0; if (!stepReady(now)) return;
  fadeToBlackBy(leds, NUM_LEDS, 60);
  leds[pos] = baseColor();
  pos = (pos+1) % NUM_LEDS; FastLED.show();
}
void patChaseWhite(uint32_t now) {
  static int pos=0; if (!stepReady(now)) return;
  fadeToBlackBy(leds, NUM_LEDS, 70);
  leds[pos] = whiteWithTemp(P.whiteLvl, P.whiteTemp);
  pos = (pos+1) % NUM_LEDS; FastLED.show();
}
void patTheaterChase(uint32_t now) {
  static uint8_t phase=0; if (!stepReady(now)) return;
  CRGB c = baseColor();
  for (int i=0;i<NUM_LEDS;i++) leds[i] = ((i + phase) % 3 == 0) ? c : CRGB::Black;
  phase = (phase + 1) % 3; FastLED.show();
}
void patFlipFlopAll(uint32_t now) {
  static bool flip=false; if (!stepReady(now)) return;
  fillAll(flip ? baseColor() : applyMix(CHSV(P.hue+128, P.sat, 255)));
  flip = !flip; FastLED.show();
}
void patFlipFlopRows(uint32_t now) {
  static bool flip=false; if (!stepReady(now)) return;
  for (int y=0;y<H;y++) {
    CRGB c = ((y ^ (flip?1:0)) & 1) ? baseColor() : applyMix(CHSV(P.hue+128, P.sat, 255));
    for (int x=0;x<W;x++) drawPixel(x,y,c);
  }
  flip = !flip; FastLED.show();
}
void patFlipFlopCols(uint32_t now) {
  static bool flip=false; if (!stepReady(now)) return;
  for (int x=0;x<W;x++) {
    CRGB c = ((x ^ (flip?1:0)) & 1) ? baseColor() : applyMix(CHSV(P.hue+128, P.sat, 255));
    for (int y=0;y<H;y++) drawPixel(x,y,c);
  }
  flip = !flip; FastLED.show();
}
void patSparkle(uint32_t now) {
  if (!stepReady(now)) return;
  fadeToBlackBy(leds, NUM_LEDS, 40);
  for (int k=0;k<3;k++) leds[random8(NUM_LEDS)] = applyMix(CHSV(P.hue + random8(120), 255, 255));
  FastLED.show();
}
void patConfetti(uint32_t now) {
  if (!stepReady(now)) return;
  fadeToBlackBy(leds, NUM_LEDS, 25);
  leds[random8(NUM_LEDS)] += baseColor();
  FastLED.show();
}
void patBounceDot(uint32_t now) {
  static bool init=false;
  static int x=0,y=0, dx=1, dy=1;
  static uint8_t hueB = 96;

  if (!init) {
    x = random8(W); y = random8(H);
    do { dx = (int)random8(3) - 1; dy = (int)random8(3) - 1; } while (dx==0 && dy==0);
    hueB = random8();
    init = true;
  }
  if (!stepReady(now)) return;

  fadeToBlackBy(leds, NUM_LEDS, P.bounceTrailFade);

  int nx = x + dx;
  int ny = y + dy;
  bool hitX = (nx < 0 || nx >= W);
  bool hitY = (ny < 0 || ny >= H);

  // edge handling X
  if (hitX) {
    if (random8(100) < P.bounceWrapPct) { // wrap
      nx = (nx + W) % W;
    } else { // bounce
      dx = -dx; nx = x + dx;
    }
    if (random8(100) >= P.bounceKeepColorPct) hueB += random8(96) + 32; // change color
    if (random8(100) < P.bounceTurnPct) { // randomize direction (why: add unpredictability)
      do { dx = (int)random8(3) - 1; } while (dx==0); // ensure movement
    }
  }
  // edge handling Y
  if (hitY) {
    if (random8(100) < P.bounceWrapPct) { ny = (ny + H) % H; }
    else { dy = -dy; ny = y + dy; }
    if (random8(100) >= P.bounceKeepColorPct) hueB += random8(96) + 32;
    if (random8(100) < P.bounceTurnPct) {
      do { dy = (int)random8(3) - 1; } while (dy==0);
    }
  }

  x = constrain(nx, 0, W-1);
  y = constrain(ny, 0, H-1);

  drawPixel(x, y, applyMix(CHSV(hueB, P.sat, 255)));
  FastLED.show();
}
void patPlasma(uint32_t now) {
  static uint8_t t=0; if (!stepReady(now)) return;
  for (int y=0;y<H;y++) for (int x=0;x<W;x++) {
    uint8_t v = sin8(x*22 + t) + cos8(y*28 + t);
    drawPixel(x,y, applyMix(CHSV(P.hue + v, 255, 255)));
  }
  t++; FastLED.show();
}

// ===================== FONT + SCROLL =====================
struct Glyph { char ch; uint8_t col[5]; };
const Glyph font[] PROGMEM = {
  {' ', {0x00,0x00,0x00,0x00,0x00}},
  {'!', {0x00,0x00,0x5F,0x00,0x00}},
  {'.', {0x00,0x40,0x60,0x00,0x00}},
  {':', {0x00,0x36,0x36,0x00,0x00}},
  {'-', {0x08,0x08,0x08,0x08,0x08}},
  {'_', {0x40,0x40,0x40,0x40,0x40}},
  {'/', {0x20,0x10,0x08,0x04,0x02}},
  {'0', {0x3E,0x51,0x49,0x45,0x3E}},
  {'1', {0x00,0x42,0x7F,0x40,0x00}},
  {'2', {0x62,0x51,0x49,0x49,0x46}},
  {'3', {0x22,0x49,0x49,0x49,0x36}},
  {'4', {0x18,0x14,0x12,0x7F,0x10}},
  {'5', {0x2F,0x49,0x49,0x49,0x31}},
  {'6', {0x3E,0x49,0x49,0x49,0x32}},
  {'7', {0x01,0x71,0x09,0x05,0x03}},
  {'8', {0x36,0x49,0x49,0x49,0x36}},
  {'9', {0x26,0x49,0x49,0x49,0x3E}},
  {'A', {0x7E,0x11,0x11,0x11,0x7E}},
  {'B', {0x7F,0x49,0x49,0x49,0x36}},
  {'C', {0x3E,0x41,0x41,0x41,0x22}},
  {'D', {0x7F,0x41,0x41,0x22,0x1C}},
  {'E', {0x7F,0x49,0x49,0x49,0x41}},
  {'F', {0x7F,0x09,0x09,0x09,0x01}},
  {'G', {0x3E,0x41,0x49,0x49,0x7A}},
  {'H', {0x7F,0x08,0x08,0x08,0x7F}},
  {'I', {0x00,0x41,0x7F,0x41,0x00}},
  {'J', {0x20,0x40,0x41,0x3F,0x01}},
  {'K', {0x7F,0x08,0x14,0x22,0x41}},
  {'L', {0x7F,0x40,0x40,0x40,0x40}},
  {'M', {0x7F,0x02,0x04,0x02,0x7F}},
  {'N', {0x7F,0x04,0x08,0x10,0x7F}},
  {'O', {0x3E,0x41,0x41,0x41,0x3E}},
  {'P', {0x7F,0x09,0x09,0x09,0x06}},
  {'Q', {0x3E,0x41,0x51,0x21,0x5E}},
  {'R', {0x7F,0x09,0x19,0x29,0x46}},
  {'S', {0x46,0x49,0x49,0x49,0x31}},
  {'T', {0x01,0x01,0x7F,0x01,0x01}},
  {'U', {0x3F,0x40,0x40,0x40,0x3F}},
  {'V', {0x1F,0x20,0x40,0x20,0x1F}},
  {'W', {0x3F,0x40,0x38,0x40,0x3F}},
  {'X', {0x63,0x14,0x08,0x14,0x63}},
  {'Y', {0x07,0x08,0x70,0x08,0x07}},
  {'Z', {0x61,0x51,0x49,0x45,0x43}},
  {'a',{0x20,0x54,0x54,0x54,0x78}},
  {'b',{0x7F,0x48,0x44,0x44,0x38}},
  {'c',{0x38,0x44,0x44,0x44,0x20}},
  {'d',{0x38,0x44,0x44,0x48,0x7F}},
  {'e',{0x38,0x54,0x54,0x54,0x18}},
  {'f',{0x08,0x7E,0x09,0x01,0x02}},
  {'g',{0x18,0xA4,0xA4,0xA4,0x7C}},
  {'h',{0x7F,0x08,0x04,0x04,0x78}},
  {'i',{0x00,0x44,0x7D,0x40,0x00}},
  {'j',{0x40,0x80,0x84,0x7D,0x00}},
  {'k',{0x7F,0x10,0x28,0x44,0x00}},
  {'l',{0x00,0x41,0x7F,0x40,0x00}},
  {'m',{0x7C,0x04,0x18,0x04,0x78}},
  {'n',{0x7C,0x08,0x04,0x04,0x78}},
  {'o',{0x38,0x44,0x44,0x44,0x38}},
  {'p',{0xFC,0x24,0x24,0x24,0x18}},
  {'q',{0x18,0x24,0x24,0x18,0xFC}},
  {'r',{0x7C,0x08,0x04,0x04,0x08}},
  {'s',{0x48,0x54,0x54,0x54,0x20}},
  {'t',{0x04,0x3F,0x44,0x40,0x20}},
  {'u',{0x3C,0x40,0x40,0x20,0x7C}},
  {'v',{0x1C,0x20,0x40,0x20,0x1C}},
  {'w',{0x3C,0x40,0x30,0x40,0x3C}},
  {'x',{0x44,0x28,0x10,0x28,0x44}},
  {'y',{0x1C,0xA0,0xA0,0xA0,0x7C}},
  {'z',{0x44,0x64,0x54,0x4C,0x44}},
};
bool getGlyph(char ch, uint8_t outCols[5]) {
  for (size_t i=0;i<sizeof(font)/sizeof(font[0]);i++) {
    Glyph g; memcpy_P(&g, &font[i], sizeof(Glyph));
    if (g.ch == ch) { for (int c=0;c<5;c++) outCols[c]=g.col[c]; return true; }
  }
  for (int c=0;c<5;c++) outCols[c]=0; return false;
}
String scrollText = "PINHALLA MATRIX   ";
int scrollX = 8;
int textPixelWidth(const String &s) { return (int)s.length() * 6; }
void patScrollText(uint32_t now) {
  if (!stepReady(now)) return;
  fillAll(CRGB::Black);
  int x = scrollX;
  for (int i=0;i<(int)scrollText.length();i++) {
    uint8_t cols[5]; getGlyph(scrollText[i], cols);
    for (int cx=0;cx<5;cx++) {
      uint8_t bits = cols[cx];
      for (int cy=0;cy<7;cy++) if (bits & (1 << cy)) drawPixel(x + cx, cy, baseColor());
    }
    x += 6;
  }
  scrollX--; if (scrollX < -textPixelWidth(scrollText)) scrollX = 8;
  FastLED.show();
}

typedef void (*PatternFn)(uint32_t);
PatternFn patternFns[PAT_COUNT] = {
  patSolidColor, patRainbow, patRainbowSwirl, patChaseColor, patChaseWhite,
  patTheaterChase, patFlipFlopAll, patFlipFlopRows, patFlipFlopCols,
  patSparkle, patConfetti, patBounceDot, patPlasma, patScrollText
};

// ===================== SHAKE =====================
int16_t prevAx = 0, prevAy = 0, prevAz = 0;
uint32_t shakeUntil = 0;
uint32_t lastShakeDebounce = 0;
uint32_t shakeEnergy = 0;

CRGB velColor(uint32_t e) {
  e = constrain(e, (uint32_t)0, (uint32_t)12000);
  uint8_t hue = (uint8_t)map(e, 0, 12000, 96, 0);
  return applyMix(CHSV(hue, 255, 255));
}
void drawBang(const CRGB &c) {
  fillAll(CRGB::Black); for (int y=0; y<6; y++) drawPixel(3, y, c); drawPixel(3, 7, c);
}
void updateShake(uint32_t now) {
  if (!imuOK) return;
  int16_t ax, ay, az; if (!readAccelRaw(ax, ay, az)) return;
  uint32_t d = (uint32_t)abs(ax - prevAx) + (uint32_t)abs(ay - prevAy) + (uint32_t)abs(az - prevAz);
  prevAx = ax; prevAy = ay; prevAz = az;

  // EMA: divisor=1 -> raw; lower divisor = more responsive
  if (P.shakeEmaDiv <= 1) shakeEnergy = d;
  else shakeEnergy = ( shakeEnergy * (P.shakeEmaDiv - 1) + d ) / P.shakeEmaDiv;

  if (d > P.shakeThr && (now - lastShakeDebounce) > P.shakeDebounceMs) {
    lastShakeDebounce = now;
    uint32_t dd = constrain(d, P.shakeThr, (uint32_t)12000);
    uint32_t hold = map(dd, P.shakeThr, 12000, P.holdMinMs, P.holdMaxMs);
    shakeUntil = now + hold;
  }
}
void strobeVelocityBang(uint32_t now) {
  static uint32_t last = 0; static bool on = false;
  uint32_t e = constrain(shakeEnergy, (uint32_t)0, (uint32_t)12000);
  uint16_t period = map(e, 0, 12000, P.strobePeriodMax, P.strobePeriodMin);
  uint16_t onMs   = max<uint16_t>(2, (uint16_t)((period * (uint16_t)P.strobeDutyPct) / 100));
  uint16_t offMs  = period - onMs;

  uint32_t interval = on ? onMs : offMs;
  if (now - last < interval) return;
  last = now; on = !on;

  if (e >= P.bangE) { if (on) drawBang(CRGB::White); else fillAll(CRGB::Black); FastLED.show(); return; }
  if (on) fillAll(velColor(e)); else fillAll(CRGB::Black);
  FastLED.show();
}

// ===================== WEB UI =====================
WebServer server(80);
Preferences prefsBlob;

void addCORS() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}
const char PAGE_HTML[] PROGMEM = R"HTML(
<!doctype html><html><head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>PINHALLA MATRIX</title>
<style>
body{font-family:system-ui;background:#0b0f14;color:#e8eef6;margin:0;padding:16px}
.card{max-width:820px;margin:0 auto;background:#121a24;border-radius:16px;padding:16px}
h1{font-size:18px;margin:0 0 12px 0}
.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px}
.grid3{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:10px}
.btn{padding:12px;border:0;border-radius:12px;background:#1f2b3a;color:#e8eef6;font-size:15px}
.btn:active{transform:scale(.99)}
label{font-size:12px;opacity:.85}
input[type=range]{width:100%}
input[type=number]{width:100%;padding:8px;border-radius:10px;border:1px solid #2a3a4f;background:#0b0f14;color:#e8eef6}
input[type=text]{width:100%;padding:10px;border-radius:10px;border:1px solid #2a3a4f;background:#0b0f14;color:#e8eef6}
.small{opacity:.75;font-size:12px;margin-top:8px}
hr{border:0;border-top:1px solid #243245;margin:12px 0}
.section{margin-top:10px;padding:10px;border:1px solid #243245;border-radius:12px}
.section h2{font-size:14px;margin:0 0 8px 0;opacity:.85}
.row{display:grid;grid-template-columns:160px 1fr 64px;align-items:center;gap:8px;margin:6px 0}
.val{font-size:12px;opacity:.8;text-align:right}
</style></head><body>
<div class="card">
<h1>PINHALLA MATRIX</h1>

<div class="grid" id="pat"></div>
<hr>

<div class="section">
<h2>Color & Speed</h2>
<div class="row"><label>Brightness (capped 60%)</label><input id="br" type="range" min="1" max="255" value="80" oninput="setv()"><div class="val" id="brv"></div></div>
<div class="row"><label>Speed</label><input id="sp" type="range" min="1" max="255" value="120" oninput="setv()"><div class="val" id="spv"></div></div>
<div class="row"><label>White ↔ Color</label><input id="mx" type="range" min="0" max="255" value="0" oninput="setv()"><div class="val" id="mxv"></div></div>
<div class="row"><label>White Brightness</label><input id="wl" type="range" min="0" max="255" value="120" oninput="setv()"><div class="val" id="wlv"></div></div>
<div class="row"><label>White Temp (warm→cool)</label><input id="wt" type="range" min="-100" max="100" value="-20" oninput="setv()"><div class="val" id="wtv"></div></div>
<div class="row"><label>Base Hue</label><input id="hu" type="range" min="0" max="255" value="96" oninput="setv()"><div class="val" id="huv"></div></div>
</div>

<div class="section">
<h2>Bounce Dot</h2>
<div class="row"><label>Wrap Probability %</label><input id="wr" type="range" min="0" max="100" value="20" oninput="setv()"><div class="val" id="wrv"></div></div>
<div class="row"><label>Keep Color % (on wall)</label><input id="kc" type="range" min="0" max="100" value="50" oninput="setv()"><div class="val" id="kcv"></div></div>
<div class="row"><label>Random Turn % (on wall)</label><input id="rt" type="range" min="0" max="100" value="30" oninput="setv()"><div class="val" id="rtv"></div></div>
<div class="row"><label>Trail Fade (0=solid,255=fast)</label><input id="tf" type="range" min="0" max="255" value="40" oninput="setv()"><div class="val" id="tfv"></div></div>
<button class="btn" onclick="fetch('/pat?id=11')">Run Bounce Dot</button>
</div>

<div class="section">
<h2>Shake / Bang</h2>
<div class="row"><label>Sensitivity Threshold</label><input id="sh" type="range" min="200" max="20000" value="2500" oninput="setv()"><div class="val" id="shv"></div></div>
<div class="row"><label>Debounce (ms)</label><input id="sd" type="range" min="0" max="250" value="25" oninput="setv()"><div class="val" id="sdv"></div></div>
<div class="row"><label>Smoothing (EMA divisor)</label><input id="se" type="range" min="1" max="16" value="2" oninput="setv()"><div class="val" id="sev"></div></div>
<div class="row"><label>Hold Min (ms)</label><input id="hm" type="range" min="50" max="2000" value="200" oninput="setv()"><div class="val" id="hmv"></div></div>
<div class="row"><label>Hold Max (ms)</label><input id="hM" type="range" min="200" max="3000" value="1100" oninput="setv()"><div class="val" id="hMv"></div></div>
<div class="row"><label>Strobe Period Min (ms)</label><input id="pm" type="range" min="10" max="300" value="35" oninput="setv()"><div class="val" id="pmv"></div></div>
<div class="row"><label>Strobe Period Max (ms)</label><input id="pM" type="range" min="30" max="500" value="220" oninput="setv()"><div class="val" id="pMv"></div></div>
<div class="row"><label>Strobe Duty %</label><input id="du" type="range" min="5" max="95" value="20" oninput="setv()"><div class="val" id="duv"></div></div>
<div class="row"><label>Bang Threshold (energy)</label><input id="be" type="range" min="0" max="12000" value="9000" oninput="setv()"><div class="val" id="bev"></div></div>
</div>

<div class="section">
<h2>Scroll Text</h2>
<input id="tx" type="text" value="PINHALLA MATRIX" />
<button class="btn" style="width:100%;margin-top:10px" onclick="setText()">Update Text</button>
</div>

<div class="small">Wi-Fi: PINHALLA-MATRIX / pinhalla — open: http://192.168.4.1</div>
</div>

<script>
const names = [
  "Solid Color","Rainbow","Rainbow Swirl","Chase Color","Chase White","Theater Chase",
  "FlipFlop","Rows Flip","Cols Flip","Sparkle","Confetti","Bounce Dot","Plasma","Scroll Text"
];
const pat = document.getElementById('pat');
names.forEach((n,i)=>{ const b=document.createElement('button'); b.className='btn'; b.textContent=(i+1)+". "+n; b.onclick=()=>fetch('/pat?id='+i); pat.appendChild(b); });

const ids = ["br","sp","mx","wl","wt","hu","wr","kc","rt","tf","sh","sd","se","hm","hM","pm","pM","du","be"];
function setv(){
  const q = ids.map(id=> id+"="+encodeURIComponent(document.getElementById(id).value)).join("&");
  fetch('/set?'+q).then(()=>syncVals());
}
async function setText(){
  const body = tx.value;
  const r = await fetch('/text', { method:'POST', headers:{'Content-Type':'text/plain'}, body });
  await r.text(); await fetch('/pat?id=13');
}

function syncVals(){
  // show live numbers next to sliders
  ids.forEach(id=>{
    const v=document.getElementById(id).value;
    const el=document.getElementById(id+"v");
    if(el) el.textContent=v;
  });
}

(async ()=>{
  try{
    const s = await fetch('/state').then(r=>r.json());
    // base
    br.value=s.brightness; sp.value=s.speed; mx.value=s.mix; wl.value=s.whiteLvl; wt.value=s.whiteTemp; hu.value=s.hue;
    // bounce
    wr.value=s.bounceWrapPct; kc.value=s.bounceKeepColorPct; rt.value=s.bounceTurnPct; tf.value=s.bounceTrailFade;
    // shake
    sh.value=s.shakeThr; sd.value=s.shakeDebounceMs; se.value=s.shakeEmaDiv;
    hm.value=s.holdMinMs; hM.value=s.holdMaxMs;
    pm.value=s.strobePeriodMin; pM.value=s.strobePeriodMax; du.value=s.strobeDutyPct; be.value=s.bangE;
    tx.value=(s.scrollText||"PINHALLA MATRIX").trim();
    syncVals();
  }catch(e){}
})();
</script>
</body></html>
)HTML";

void saveParams() { prefsBlob.begin("pmatrix", false); prefsBlob.putBytes("params",&P,sizeof(P)); prefsBlob.end(); }
void loadParams() {
  prefsBlob.begin("pmatrix", true);
  if (prefsBlob.isKey("params")) { Params tmp; if (prefsBlob.getBytes("params",&tmp,sizeof(tmp))==sizeof(tmp)) P=tmp; }
  prefsBlob.end();
}
void addCORS(); // fwd

void handleRoot() { addCORS(); server.send(200, "text/html", FPSTR(PAGE_HTML)); }
void handlePat() {
  addCORS();
  if (server.hasArg("id")) {
    int id = server.arg("id").toInt();
    id = constrain(id, 0, (int)PAT_COUNT-1);
    currentPat = (uint8_t)id; scrollX = 8;
  }
  server.send(200, "text/plain", "ok");
}
void handleSet() {
  addCORS();
  bool changed=false;
  auto u8=[&](const char* k, uint8_t& dst, int mn,int mx){ if(server.hasArg(k)){ int v=constrain(server.arg(k).toInt(),mn,mx); if(v!=dst){dst=(uint8_t)v; changed=true;} } };
  auto u16=[&](const char* k, uint16_t& dst, int mn,int mx){ if(server.hasArg(k)){ int v=constrain(server.arg(k).toInt(),mn,mx); if(v!=dst){dst=(uint16_t)v; changed=true;} } };
  auto u32=[&](const char* k, uint32_t& dst, int mn,int mx){ if(server.hasArg(k)){ int v=constrain(server.arg(k).toInt(),mn,mx); if((uint32_t)v!=dst){dst=(uint32_t)v; changed=true;} } };
  auto s8=[&](const char* k, int8_t& dst, int mn,int mx){ if(server.hasArg(k)){ int v=constrain(server.arg(k).toInt(),mn,mx); if(v!=dst){dst=(int8_t)v; changed=true;} } };

  // base
  u8("b",P.brightness,1,255); u8("s",P.speed,1,255); u8("mx",P.mix,0,255);
  u8("wl",P.whiteLvl,0,255);  s8("wt",P.whiteTemp,-100,100); u8("hu",P.hue,0,255);

  // bounce
  u8("wr",P.bounceWrapPct,0,100); u8("kc",P.bounceKeepColorPct,0,100);
  u8("rt",P.bounceTurnPct,0,100); u8("tf",P.bounceTrailFade,0,255);

  // shake
  u32("sh",P.shakeThr,200,20000); u16("sd",P.shakeDebounceMs,0,250);
  u8("se",P.shakeEmaDiv,1,16);
  u16("hm",P.holdMinMs,50,3000); u16("hM",P.holdMaxMs,50,4000);
  u16("pm",P.strobePeriodMin,10,500); u16("pM",P.strobePeriodMax,10,800);
  u8("du",P.strobeDutyPct,5,95);
  u32("be",P.bangE,0,12000);

  applyBrightnessCap();
  if (changed) saveParams();
  server.send(200, "text/plain", "ok");
}
void handleTextGet() {
  addCORS();
  if (server.hasArg("t")) {
    scrollText = server.arg("t");
    if (!scrollText.endsWith("   ")) scrollText += "   ";
    scrollX = 8; saveParams();
  }
  server.send(200, "text/plain", scrollText);
}
void handleTextPost() {
  addCORS();
  String t = server.arg("plain");
  if (t.length()) {
    scrollText = t;
    if (!scrollText.endsWith("   ")) scrollText += "   ";
    scrollX = 8; saveParams();
  }
  server.send(200, "text/plain", scrollText);
}
void handleState() {
  addCORS();
  String json = "{";
  json += "\"brightness\":" + String(P.brightness) + ",";
  json += "\"speed\":" + String(P.speed) + ",";
  json += "\"mix\":" + String(P.mix) + ",";
  json += "\"whiteLvl\":" + String(P.whiteLvl) + ",";
  json += "\"whiteTemp\":" + String(P.whiteTemp) + ",";
  json += "\"hue\":" + String(P.hue) + ",";
  json += "\"sat\":" + String(P.sat) + ",";
  json += "\"shakeThr\":" + String(P.shakeThr) + ",";
  json += "\"shakeDebounceMs\":" + String(P.shakeDebounceMs) + ",";
  json += "\"shakeEmaDiv\":" + String(P.shakeEmaDiv) + ",";
  json += "\"holdMinMs\":" + String(P.holdMinMs) + ",";
  json += "\"holdMaxMs\":" + String(P.holdMaxMs) + ",";
  json += "\"strobePeriodMin\":" + String(P.strobePeriodMin) + ",";
  json += "\"strobePeriodMax\":" + String(P.strobePeriodMax) + ",";
  json += "\"strobeDutyPct\":" + String(P.strobeDutyPct) + ",";
  json += "\"bangE\":" + String(P.bangE) + ",";
  json += "\"bounceWrapPct\":" + String(P.bounceWrapPct) + ",";
  json += "\"bounceKeepColorPct\":" + String(P.bounceKeepColorPct) + ",";
  json += "\"bounceTurnPct\":" + String(P.bounceTurnPct) + ",";
  json += "\"bounceTrailFade\":" + String(P.bounceTrailFade) + ",";
  json += "\"imu\":"; json += (imuOK ? "true" : "false"); json += ",";
  json += "\"pattern\":" + String((int)currentPat) + ",";
  json += "\"scrollText\":\"" + String(scrollText) + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

// ===================== SETUP/LOOP =====================
void setup() {
  randomSeed(esp_random());

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  loadParams(); applyBrightnessCap(); fillAll(CRGB::Black); FastLED.show();

  Wire.begin(I2C_SDA, I2C_SCL); Wire.setClock(400000);
  imuOK = initQMI8658();
  if (imuOK) {
    int16_t ax, ay, az;
    if (readAccelRaw(ax, ay, az)) { prevAx=ax; prevAy=ay; prevAz=az; }
    fillAll(CRGB::Green); FastLED.show(); delay(120);
  } else { fillAll(CRGB::Red); FastLED.show(); delay(120); }
  fillAll(CRGB::Black); FastLED.show();

  WiFi.mode(WIFI_AP); WiFi.softAP("PINHALLA-MATRIX", "pinhalla");
  IPAddress ip = WiFi.softAPIP();
  scrollText = "PINHALLA MATRIX IP " + ip.toString() + "   ";

  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/pat", handlePat);
  server.on("/text", HTTP_GET,  handleTextGet);
  server.on("/text", HTTP_POST, handleTextPost);
  server.on("/state", HTTP_GET, handleState);
  server.onNotFound([](){
    if (server.method() == HTTP_OPTIONS) { addCORS(); server.send(200); return; }
    server.send(404, "text/plain", "404");
  });
  server.begin();
}
void loop() {
  server.handleClient();
  uint32_t now = millis();
  updateShake(now);
  if (imuOK && now < shakeUntil) { strobeVelocityBang(now); return; }
  patternFns[currentPat](now);
}

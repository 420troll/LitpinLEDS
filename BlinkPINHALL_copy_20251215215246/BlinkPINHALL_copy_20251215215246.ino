#include <Adafruit_NeoPixel.h>
#include <pgmspace.h>

#define PIN_NEOPIXEL 14
#define WIDTH  8
#define HEIGHT 8
#define NUM_LEDS (WIDTH * HEIGHT)

#define BRIGHTNESS 40   // 0-255
#define FRAME_DELAY 55  // ms (scroll speed)

// NeoPixel strip (8x8 treated as 64 pixels)
Adafruit_NeoPixel strip(NUM_LEDS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// ---------- 180° rotated XY mapping (row-major, non-serpentine) ----------
uint16_t XY(uint8_t x, uint8_t y) {
  x = (WIDTH  - 1) - x;   // flip X
  y = (HEIGHT - 1) - y;   // flip Y
  return (y * WIDTH) + x; // row-major
}

// ---------- Color wheel (0-255) ----------
uint32_t wheel(uint8_t pos) {
  pos = 255 - pos;
  if (pos < 85)  return strip.Color(255 - pos * 3, 0, pos * 3);
  if (pos < 170) { pos -= 85; return strip.Color(0, pos * 3, 255 - pos * 3); }
  pos -= 170;    return strip.Color(pos * 3, 255 - pos * 3, 0);
}

// ---------- Minimal 5x7 font for needed letters ----------
// Each char is 5 columns, LSB at top (bit0 = top pixel)
struct Glyph { char c; uint8_t col[5]; };

const Glyph font[] PROGMEM = {
  {' ', {0x00,0x00,0x00,0x00,0x00}},
  {'A', {0x7E,0x11,0x11,0x11,0x7E}},
  {'E', {0x7F,0x49,0x49,0x49,0x41}},
  {'H', {0x7F,0x08,0x08,0x08,0x7F}},
  {'I', {0x00,0x41,0x7F,0x41,0x00}},
  {'L', {0x7F,0x40,0x40,0x40,0x40}},
  {'N', {0x7F,0x06,0x08,0x30,0x7F}},
  {'P', {0x7F,0x09,0x09,0x09,0x06}},
  {'R', {0x7F,0x09,0x19,0x29,0x46}},
  {'S', {0x46,0x49,0x49,0x49,0x31}},
  {'U', {0x3F,0x40,0x40,0x40,0x3F}},
};

bool getGlyph(char ch, uint8_t outCols[5]) {
  for (uint8_t i = 0; i < sizeof(font)/sizeof(font[0]); i++) {
    Glyph g;
    memcpy_P(&g, &font[i], sizeof(Glyph));
    if (g.c == ch) {
      for (int k = 0; k < 5; k++) outCols[k] = g.col[k];
      return true;
    }
  }
  // unknown -> blank
  for (int k = 0; k < 5; k++) outCols[k] = 0x00;
  return false;
}

// Draw a 5x7 character at top-left (x,y) in the 8x8 buffer
void drawChar5x7(int x, int y, char ch, uint32_t color) {
  uint8_t cols[5];
  getGlyph(ch, cols);

  for (int cx = 0; cx < 5; cx++) {
    uint8_t bits = cols[cx];
    for (int cy = 0; cy < 7; cy++) {
      if (bits & (1 << cy)) {
        int px = x + cx;
        int py = y + cy;
        if (px >= 0 && px < WIDTH && py >= 0 && py < HEIGHT) {
          strip.setPixelColor(XY(px, py), color);
        }
      }
    }
  }
}

const char* MSG = "PINHALLA RULES";

void setup() {
  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  strip.clear();
  strip.show();
}

void loop() {
  static uint8_t hue = 0;

  const int charW = 5;
  const int spacing = 1;
  const int stepW = charW + spacing;

  int msgLen = 0;
  while (MSG[msgLen] != '\0') msgLen++;

  // Total scroll width: message width + full screen width for entry/exit
  int scrollWidth = (msgLen * stepW) + WIDTH;

  // Scroll from right to left
  for (int scroll = 0; scroll < scrollWidth; scroll++) {
    strip.clear();

    // Vertical placement (7px tall font on 8px display)
    int y0 = 0;

    // Draw each character
    for (int i = 0; i < msgLen; i++) {
      int x0 = (WIDTH - scroll) + (i * stepW);c:\Users\Gamer\Desktop\ESP32-S3-Matrix-Demo\Game\Game.ino c:\Users\Gamer\Desktop\ESP32-S3-Matrix-Demo\Game\WS_Matrix.cpp c:\Users\Gamer\Desktop\ESP32-S3-Matrix-Demo\Game\WS_Matrix.h c:\Users\Gamer\Desktop\ESP32-S3-Matrix-Demo\Game\WS_QMI8658.cpp c:\Users\Gamer\Desktop\ESP32-S3-Matrix-Demo\Game\WS_QMI8658.h
      drawChar5x7(x0, y0, MSG[i], wheel(hue));
    }

    strip.show();
    delay(FRAME_DELAY);
  }

  // Change color after each full cycle
  hue += 25;
}


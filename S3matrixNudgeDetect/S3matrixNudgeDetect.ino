#include <NeoPixelBus.h>

// ==== PIN + LED CONFIG ====
const uint8_t PIN_S12A = 4;
const uint8_t PIN_S12B = 5;
const uint8_t PIN_S24A = 18;
const uint8_t PIN_S24B = 19;

const uint16_t NUM_S12A = 12;
const uint16_t NUM_S12B = 12;
const uint16_t NUM_S24A = 24;
const uint16_t NUM_S24B = 24;

// Hard cap brightness: 60% of 255 ≈ 153
const uint8_t MAX_BRIGHT = 153;

// 4 strips, 4 RMT channels
NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt0Ws2812xMethod> stripS12A(NUM_S12A, PIN_S12A);
NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt1Ws2812xMethod> stripS12B(NUM_S12B, PIN_S12B);
NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt2Ws2812xMethod> stripS24A(NUM_S24A, PIN_S24A);
NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt3Ws2812xMethod> stripS24B(NUM_S24B, PIN_S24B);

// ---- brightness limiter (hard clamp) ----
RgbColor limitBright(const RgbColor& c) {
  // scale each channel by MAX_BRIGHT/255
  return RgbColor(
    (uint16_t)c.R * MAX_BRIGHT / 255,
    (uint16_t)c.G * MAX_BRIGHT / 255,
    (uint16_t)c.B * MAX_BRIGHT / 255
  );
}

void clearAll() {
  for (int i=0; i<NUM_S12A; i++) stripS12A.SetPixelColor(i, RgbColor(0));
  for (int i=0; i<NUM_S12B; i++) stripS12B.SetPixelColor(i, RgbColor(0));
  for (int i=0; i<NUM_S24A; i++) stripS24A.SetPixelColor(i, RgbColor(0));
  for (int i=0; i<NUM_S24B; i++) stripS24B.SetPixelColor(i, RgbColor(0));
  stripS12A.Show(); stripS12B.Show(); stripS24A.Show(); stripS24B.Show();
}

void showAll() {
  stripS12A.Show(); stripS12B.Show(); stripS24A.Show(); stripS24B.Show();
}

// ---- 1) GLOBAL RAINBOW ----
void effectRainbow() {
  for (int step = 0; step < 256; step++) {
    for (int i=0; i<NUM_S12A; i++) {
      uint8_t hue = (i*16 + step) & 0xFF;
      stripS12A.SetPixelColor(i, limitBright(RgbColor(HsbColor(hue/255.0f, 1, 1))));
    }
    for (int i=0; i<NUM_S12B; i++) {
      uint8_t hue = (i*16 + step + 64) & 0xFF; // offset so it’s not identical
      stripS12B.SetPixelColor(i, limitBright(RgbColor(HsbColor(hue/255.0f, 1, 1))));
    }
    for (int i=0; i<NUM_S24A; i++) {
      uint8_t hue = (i*8 + step) & 0xFF;
      stripS24A.SetPixelColor(i, limitBright(RgbColor(HsbColor(hue/255.0f, 1, 1))));
    }
    for (int i=0; i<NUM_S24B; i++) {
      uint8_t hue = (i*8 + step + 96) & 0xFF;
      stripS24B.SetPixelColor(i, limitBright(RgbColor(HsbColor(hue/255.0f, 1, 1))));
    }
    showAll();
    delay(20);
  }
}

// ---- 2) SEQUENTIAL CHASE THROUGH ALL BANKS ----
void effectSequentialChase() {
  const int total = NUM_S12A + NUM_S12B + NUM_S24A + NUM_S24B;
  RgbColor dot = limitBright(RgbColor(255,255,255));

  for (int step=0; step<total; step++) {
    clearAll();
    int i = step;

    if (i < NUM_S12A) {
      stripS12A.SetPixelColor(i, dot);
    } else if ((i -= NUM_S12A) < NUM_S12B) {
      stripS12B.SetPixelColor(i, dot);
    } else if ((i -= NUM_S12B) < NUM_S24A) {
      stripS24A.SetPixelColor(i, dot);
    } else {
      i -= NUM_S24A;
      stripS24B.SetPixelColor(i, dot);
    }

    showAll();
    delay(35);
  }
}

// ---- 3) GLOBAL SPARKLE ----
void effectSparkle() {
  for (int k=0; k<220; k++) {
    int bank = random(4);
    RgbColor sp = limitBright(RgbColor(255,255,255));

    switch(bank) {
      case 0: stripS12A.SetPixelColor(random(NUM_S12A), sp); break;
      case 1: stripS12B.SetPixelColor(random(NUM_S12B), sp); break;
      case 2: stripS24A.SetPixelColor(random(NUM_S24A), sp); break;
      case 3: stripS24B.SetPixelColor(random(NUM_S24B), sp); break;
    }

    showAll();
    delay(25);
    clearAll();
  }
}

// ---- 4) FLIP FLOP ----
void effectFlipFlop() {
  RgbColor A = limitBright(RgbColor(255, 0, 0));
  RgbColor B = limitBright(RgbColor(0, 0, 255));

  for (int t=0; t<26; t++) {
    for (int i=0; i<NUM_S12A; i++) stripS12A.SetPixelColor(i, (i%2)?A:B);
    for (int i=0; i<NUM_S12B; i++) stripS12B.SetPixelColor(i, (i%2)?A:B);
    for (int i=0; i<NUM_S24A; i++) stripS24A.SetPixelColor(i, (i%2)?A:B);
    for (int i=0; i<NUM_S24B; i++) stripS24B.SetPixelColor(i, (i%2)?A:B);
    showAll();
    delay(180);

    for (int i=0; i<NUM_S12A; i++) stripS12A.SetPixelColor(i, (i%2)?B:A);
    for (int i=0; i<NUM_S12B; i++) stripS12B.SetPixelColor(i, (i%2)?B:A);
    for (int i=0; i<NUM_S24A; i++) stripS24A.SetPixelColor(i, (i%2)?B:A);
    for (int i=0; i<NUM_S24B; i++) stripS24B.SetPixelColor(i, (i%2)?B:A);
    showAll();
    delay(180);
  }
}

// ---- 5) WHITE FADE (still obeys 60% cap) ----
void effectWhiteFade() {
  for (int b=0; b<=255; b+=5) {
    RgbColor c = limitBright(RgbColor(b,b,b));
    for (int i=0; i<NUM_S12A; i++) stripS12A.SetPixelColor(i, c);
    for (int i=0; i<NUM_S12B; i++) stripS12B.SetPixelColor(i, c);
    for (int i=0; i<NUM_S24A; i++) stripS24A.SetPixelColor(i, c);
    for (int i=0; i<NUM_S24B; i++) stripS24B.SetPixelColor(i, c);
    showAll();
    delay(12);
  }

  for (int b=255; b>=0; b-=5) {
    RgbColor c = limitBright(RgbColor(b,b,b));
    for (int i=0; i<NUM_S12A; i++) stripS12A.SetPixelColor(i, c);
    for (int i=0; i<NUM_S12B; i++) stripS12B.SetPixelColor(i, c);
    for (int i=0; i<NUM_S24A; i++) stripS24A.SetPixelColor(i, c);
    for (int i=0; i<NUM_S24B; i++) stripS24B.SetPixelColor(i, c);
    showAll();
    delay(12);
  }
}

void setup() {
  randomSeed(micros());
  stripS12A.Begin(); stripS12A.Show();
  stripS12B.Begin(); stripS12B.Show();
  stripS24A.Begin(); stripS24A.Show();
  stripS24B.Begin(); stripS24B.Show();
  clearAll();
}

void loop() {
  effectRainbow();
  effectSequentialChase();
  effectSparkle();
  effectFlipFlop();
  effectWhiteFade();
}


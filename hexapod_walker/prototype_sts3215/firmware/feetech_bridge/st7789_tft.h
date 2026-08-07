/*
  Bitbang ST7789 status panel for Uno Q — landscape 320×240.

  Center: schematic hexapod; motors brighten with current.
  Edges: activity (top-left), live (top-right), amps (bottom-left),
  volts/load (bottom-right). Glyph-diff only (no full wipe).

  Pins: SCL D13, SDA D11, RST D8, DC D7, CS D10, VCC/BL 3.3V
*/

#ifndef ST7789_TFT_H
#define ST7789_TFT_H

#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef PROGMEM
#define PROGMEM
#endif
#ifndef pgm_read_byte
#define pgm_read_byte(addr) (*(const uint8_t *)(addr))
#endif

namespace tft {

static const int PIN_CS = 10;
static const int PIN_DC = 7;
static const int PIN_RST = 8;
static const int PIN_SCK = 13;
static const int PIN_MOSI = 11;

static const uint16_t W = 320;   // landscape
static const uint16_t H = 240;
static const uint8_t SCALE = 2;
static const uint8_t CHAR_W = 6 * SCALE;
static const uint8_t CHAR_H = 8 * SCALE;
static const uint8_t EDGE_COLS = 11;   // chars per edge slot
static const uint8_t EDGE_SLOTS = 6;
static const int LOGO_CX = 160;
static const int LOGO_CY = 120;
static const int LOGO_SCALE = 85;  // % of original radial sizes

static const uint16_t C_BLACK = 0x0000;
static const uint16_t C_WHITE = 0xFFFF;
static const uint16_t C_DIM   = 0x4208;
static const uint16_t C_GREEN = 0x07E0;
static const uint16_t C_RED   = 0xF800;
static const uint16_t C_YELL  = 0xC600;  // muted gold for demo text
static const uint16_t C_CYAN  = 0x04B6;
static const uint16_t C_ORANGE = 0xD300;
// Hub: keep quieter than lit motors
static const uint16_t C_BLUE  = 0x11AA;  // muted steel blue
static const uint16_t C_RING  = 0x6280;  // dull bronze, not neon gold

// Dim / bright bases for motor glow (RGB565).
static const uint16_t RED_DIM = 0x4000;
static const uint16_t RED_BRT = 0xF800;
static const uint16_t BLU_DIM = 0x0010;
static const uint16_t BLU_BRT = 0x061F;
static const uint16_t FOOT_DIM = 0x6000;
static const uint16_t FOOT_BRT = 0xF800;

static bool ready = false;
static bool chromeDrawn = false;
static char cacheTxt[EDGE_SLOTS][EDGE_COLS];
static uint16_t cacheFg[EDGE_SLOTS];
static uint8_t lightLvl[18];  // 0..7, 0xFF = unset

// Job mode (``DJ``): full-screen text panel for calibration/demo jobs —
// 26-char rows instead of the 11-char edge slots, plus a progress bar.
static const uint8_t JOB_COLS = 26;   // 26 × 12 px = 312 of 320
static const uint8_t JOB_ROWS = 6;    // title, 4 body lines, footer
static bool jobMode = false;
static char jobTxt[JOB_ROWS][JOB_COLS];
static uint16_t jobFg[JOB_ROWS];
static int jobPct = -2;               // -2 = bar never drawn

// Edge slot → pixel origin (top/bottom bands, left & right).
static void edgeOrigin(uint8_t slot, uint16_t *x, uint16_t *y) {
  switch (slot) {
    case 0: *x = 4;              *y = 4; break;                 // TL mode
    case 1: *x = 4;              *y = 4 + CHAR_H; break;        // TL activity
    case 2: *x = 4;              *y = 4 + 2 * CHAR_H; break;    // TL detail
    case 3: *x = W - EDGE_COLS * CHAR_W - 4; *y = 4; break;     // TR live
    case 4: *x = 4;              *y = H - CHAR_H - 4; break;    // BL I=
    case 5: *x = W - EDGE_COLS * CHAR_W - 4;
            *y = H - CHAR_H - 4; break;                         // BR V=
    default: *x = 0; *y = 0; break;
  }
}

static int rad(int v) { return (v * LOGO_SCALE) / 100; }

// 5×7 font ASCII 32..127
static const uint8_t FONT5x7[] PROGMEM = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x5F,0x00,0x00,0x00,0x07,0x00,0x07,0x00,
  0x14,0x7F,0x14,0x7F,0x14,0x24,0x2A,0x7F,0x2A,0x12,0x23,0x13,0x08,0x64,0x62,
  0x36,0x49,0x55,0x22,0x50,0x00,0x05,0x03,0x00,0x00,0x00,0x1C,0x22,0x41,0x00,
  0x00,0x41,0x22,0x1C,0x00,0x14,0x08,0x3E,0x08,0x14,0x08,0x08,0x3E,0x08,0x08,
  0x00,0x50,0x30,0x00,0x00,0x08,0x08,0x08,0x08,0x08,0x00,0x60,0x60,0x00,0x00,
  0x20,0x10,0x08,0x04,0x02,0x3E,0x51,0x49,0x45,0x3E,0x00,0x42,0x7F,0x40,0x00,
  0x42,0x61,0x51,0x49,0x46,0x21,0x41,0x45,0x4B,0x31,0x18,0x14,0x12,0x7F,0x10,
  0x27,0x45,0x45,0x45,0x39,0x3C,0x4A,0x49,0x49,0x30,0x01,0x71,0x09,0x05,0x03,
  0x36,0x49,0x49,0x49,0x36,0x06,0x49,0x49,0x29,0x1E,0x00,0x36,0x36,0x00,0x00,
  0x00,0x56,0x36,0x00,0x00,0x08,0x14,0x22,0x41,0x00,0x14,0x14,0x14,0x14,0x14,
  0x00,0x41,0x22,0x14,0x08,0x02,0x01,0x51,0x09,0x06,0x32,0x49,0x79,0x41,0x3E,
  0x7E,0x11,0x11,0x11,0x7E,0x7F,0x49,0x49,0x49,0x36,0x3E,0x41,0x41,0x41,0x22,
  0x7F,0x41,0x41,0x22,0x1C,0x7F,0x49,0x49,0x49,0x41,0x7F,0x09,0x09,0x09,0x01,
  0x3E,0x41,0x49,0x49,0x7A,0x7F,0x08,0x08,0x08,0x7F,0x00,0x41,0x7F,0x41,0x00,
  0x20,0x40,0x41,0x3F,0x01,0x7F,0x08,0x14,0x22,0x41,0x7F,0x40,0x40,0x40,0x40,
  0x7F,0x02,0x0C,0x02,0x7F,0x7F,0x04,0x08,0x10,0x7F,0x3E,0x41,0x41,0x41,0x3E,
  0x7F,0x09,0x09,0x09,0x06,0x3E,0x41,0x51,0x21,0x5E,0x7F,0x09,0x19,0x29,0x46,
  0x46,0x49,0x49,0x49,0x31,0x01,0x01,0x7F,0x01,0x01,0x3F,0x40,0x40,0x40,0x3F,
  0x1F,0x20,0x40,0x20,0x1F,0x3F,0x40,0x38,0x40,0x3F,0x63,0x14,0x08,0x14,0x63,
  0x07,0x08,0x70,0x08,0x07,0x61,0x51,0x49,0x45,0x43,0x00,0x7F,0x41,0x41,0x00,
  0x02,0x04,0x08,0x10,0x20,0x00,0x41,0x41,0x7F,0x00,0x04,0x02,0x01,0x02,0x04,
  0x40,0x40,0x40,0x40,0x40,0x00,0x01,0x02,0x04,0x00,0x20,0x54,0x54,0x54,0x78,
  0x7F,0x48,0x44,0x44,0x38,0x38,0x44,0x44,0x44,0x20,0x38,0x44,0x44,0x48,0x7F,
  0x38,0x54,0x54,0x54,0x18,0x08,0x7E,0x09,0x01,0x02,0x0C,0x52,0x52,0x52,0x3E,
  0x7F,0x08,0x04,0x04,0x78,0x00,0x44,0x7D,0x40,0x00,0x20,0x40,0x44,0x3D,0x00,
  0x7F,0x10,0x28,0x44,0x00,0x00,0x41,0x7F,0x40,0x00,0x7C,0x04,0x18,0x04,0x78,
  0x7C,0x08,0x04,0x04,0x78,0x38,0x44,0x44,0x44,0x38,0x7C,0x14,0x14,0x14,0x08,
  0x08,0x14,0x14,0x18,0x7C,0x7C,0x08,0x04,0x04,0x08,0x48,0x54,0x54,0x54,0x20,
  0x04,0x3F,0x44,0x40,0x20,0x3C,0x40,0x40,0x20,0x7C,0x1C,0x20,0x40,0x20,0x1C,
  0x3C,0x40,0x30,0x40,0x3C,0x44,0x28,0x10,0x28,0x44,0x0C,0x50,0x50,0x50,0x3C,
  0x44,0x64,0x54,0x4C,0x44,0x00,0x08,0x36,0x41,0x00,0x00,0x00,0x7F,0x00,0x00,
  0x00,0x41,0x36,0x08,0x00,0x10,0x08,0x08,0x10,0x08,0x00,0x00,0x00,0x00,0x00,
};

static inline void sck(bool v) { digitalWrite(PIN_SCK, v ? HIGH : LOW); }
static inline void mosi(bool v) { digitalWrite(PIN_MOSI, v ? HIGH : LOW); }

static void spiWrite8(uint8_t b) {
  // Unrolled bitbang — full-frame clears dominate boot time.
  sck(false); mosi(b & 0x80); sck(true);
  sck(false); mosi(b & 0x40); sck(true);
  sck(false); mosi(b & 0x20); sck(true);
  sck(false); mosi(b & 0x10); sck(true);
  sck(false); mosi(b & 0x08); sck(true);
  sck(false); mosi(b & 0x04); sck(true);
  sck(false); mosi(b & 0x02); sck(true);
  sck(false); mosi(b & 0x01); sck(true);
  sck(false);
}

static void writeCmd(uint8_t c) {
  digitalWrite(PIN_DC, LOW);
  digitalWrite(PIN_CS, LOW);
  spiWrite8(c);
  digitalWrite(PIN_CS, HIGH);
}

static void writeData(uint8_t d) {
  digitalWrite(PIN_DC, HIGH);
  digitalWrite(PIN_CS, LOW);
  spiWrite8(d);
  digitalWrite(PIN_CS, HIGH);
}

static void writeData16(uint16_t d) {
  digitalWrite(PIN_DC, HIGH);
  digitalWrite(PIN_CS, LOW);
  spiWrite8(d >> 8);
  spiWrite8(d & 0xFF);
  digitalWrite(PIN_CS, HIGH);
}

static void setWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  writeCmd(0x2A);
  writeData16(x0);
  writeData16(x1);
  writeCmd(0x2B);
  writeData16(y0);
  writeData16(y1);
  writeCmd(0x2C);
}

static void pushPixel(uint16_t color) {
  spiWrite8(color >> 8);
  spiWrite8(color & 0xFF);
}

static void fillRect(int x, int y, int w, int h, uint16_t color) {
  if (w <= 0 || h <= 0) return;
  if (x >= (int)W || y >= (int)H) return;
  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }
  if (x + w > (int)W) w = (int)W - x;
  if (y + h > (int)H) h = (int)H - y;
  if (w <= 0 || h <= 0) return;
  setWindow((uint16_t)x, (uint16_t)y, (uint16_t)(x + w - 1),
            (uint16_t)(y + h - 1));
  digitalWrite(PIN_DC, HIGH);
  digitalWrite(PIN_CS, LOW);
  uint8_t hi = color >> 8, lo = color & 0xFF;
  for (uint32_t i = 0; i < (uint32_t)w * h; i++) {
    spiWrite8(hi);
    spiWrite8(lo);
  }
  digitalWrite(PIN_CS, HIGH);
}

static void drawPixel(int x, int y, uint16_t color) {
  if ((unsigned)x >= W || (unsigned)y >= H) return;
  setWindow((uint16_t)x, (uint16_t)y, (uint16_t)x, (uint16_t)y);
  digitalWrite(PIN_DC, HIGH);
  digitalWrite(PIN_CS, LOW);
  pushPixel(color);
  digitalWrite(PIN_CS, HIGH);
}

static void drawLine(int x0, int y0, int x1, int y1, uint16_t color) {
  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  for (;;) {
    drawPixel(x0, y0, color);
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

static void fillTriangle(int x0, int y0, int x1, int y1, int x2, int y2,
                         uint16_t color) {
  if (y0 > y1) { int t=x0; x0=x1; x1=t; t=y0; y0=y1; y1=t; }
  if (y1 > y2) { int t=x1; x1=x2; x2=t; t=y1; y1=y2; y2=t; }
  if (y0 > y1) { int t=x0; x0=x1; x1=t; t=y0; y0=y1; y1=t; }
  if (y0 == y2) return;
  auto edgeX = [](int xa, int ya, int xb, int yb, int y) -> int {
    if (ya == yb) return xa;
    return xa + (xb - xa) * (y - ya) / (yb - ya);
  };
  for (int y = y0; y <= y2; y++) {
    int xa, xb;
    if (y < y1) {
      xa = edgeX(x0, y0, x2, y2, y);
      xb = edgeX(x0, y0, x1, y1, y);
    } else {
      xa = edgeX(x0, y0, x2, y2, y);
      xb = edgeX(x1, y1, x2, y2, y);
    }
    if (xa > xb) { int t = xa; xa = xb; xb = t; }
    if (xb >= xa) fillRect(xa, y, xb - xa + 1, 1, color);
  }
}

static void fillQuad(int x0, int y0, int x1, int y1, int x2, int y2,
                     int x3, int y3, uint16_t color) {
  fillTriangle(x0, y0, x1, y1, x2, y2, color);
  fillTriangle(x0, y0, x2, y2, x3, y3, color);
}

// Leg angles match tripod_gait: (i+0.5)*60° → 30,90,150,210,270,330.
// cos/sin ×1000.
static void legDir(uint8_t leg, int *c, int *s) {
  static const int16_t C[6] = {866, 0, -866, -866, 0, 866};
  static const int16_t S[6] = {-500, -1000, -500, 500, 1000, 500};
  *c = C[leg % 6];
  *s = S[leg % 6];
}

static void octVertex(int cx, int cy, int r, uint8_t i, int *x, int *y) {
  // Regular octagon, flat-top-ish; angles 22.5 + i*45
  static const int16_t C[8] = {924, 383, -383, -924, -924, -383, 383, 924};
  static const int16_t S[8] = {-383, -924, -924, -383, 383, 924, 924, 383};
  i &= 7;
  *x = cx + (r * C[i]) / 1000;
  *y = cy + (r * S[i]) / 1000;
}

static uint16_t lerp565(uint16_t a, uint16_t b, uint8_t lvl /*0..7*/) {
  int ar = (a >> 11) & 0x1F, ag = (a >> 5) & 0x3F, ab = a & 0x1F;
  int br = (b >> 11) & 0x1F, bg = (b >> 5) & 0x3F, bb = b & 0x1F;
  int r = ar + ((br - ar) * (int)lvl) / 7;
  int g = ag + ((bg - ag) * (int)lvl) / 7;
  int bl = ab + ((bb - ab) * (int)lvl) / 7;
  return (uint16_t)((r << 11) | (g << 5) | bl);
}

static uint8_t levelFromMa(int ma) {
  if (ma < 0) ma = -ma;
  if (ma < 25) return 0;
  if (ma < 70) return 1;
  if (ma < 130) return 2;
  if (ma < 200) return 3;
  if (ma < 300) return 4;
  if (ma < 450) return 5;
  if (ma < 650) return 6;
  return 7;
}

// Oriented servo block: length along leg, width across.
static void drawBlock(int cx, int cy, int c, int s, int dist, int len, int wid,
                      uint16_t fill) {
  // Center of block
  int bx = cx + (c * dist) / 1000;
  int by = cy + (s * dist) / 1000;
  // Unit along (c,s), perp (-s,c)
  int hx = (c * len) / 2000;   // half-length
  int hy = (s * len) / 2000;
  int px = (-s * wid) / 2000;  // half-width
  int py = (c * wid) / 2000;
  int x0 = bx - hx - px, y0 = by - hy - py;
  int x1 = bx + hx - px, y1 = by + hy - py;
  int x2 = bx + hx + px, y2 = by + hy + py;
  int x3 = bx - hx + px, y3 = by - hy + py;
  fillQuad(x0, y0, x1, y1, x2, y2, x3, y3, fill);
  // Black outline
  drawLine(x0, y0, x1, y1, C_BLACK);
  drawLine(x1, y1, x2, y2, C_BLACK);
  drawLine(x2, y2, x3, y3, C_BLACK);
  drawLine(x3, y3, x0, y0, C_BLACK);
  // Bolt dots (4)
  drawPixel(bx - hx / 2 - px / 2, by - hy / 2 - py / 2, C_BLACK);
  drawPixel(bx + hx / 2 - px / 2, by + hy / 2 - py / 2, C_BLACK);
  drawPixel(bx - hx / 2 + px / 2, by - hy / 2 + py / 2, C_BLACK);
  drawPixel(bx + hx / 2 + px / 2, by + hy / 2 + py / 2, C_BLACK);
}

static void drawFoot(int cx, int cy, int c, int s, int dist, uint16_t fill) {
  int fx = cx + (c * dist) / 1000;
  int fy = cy + (s * dist) / 1000;
  fillRect(fx - 3, fy - 3, 7, 7, fill);
  drawLine(fx - 3, fy - 3, fx + 3, fy - 3, C_BLACK);
  drawLine(fx - 3, fy + 3, fx + 3, fy + 3, C_BLACK);
  drawLine(fx - 3, fy - 3, fx - 3, fy + 3, C_BLACK);
  drawLine(fx + 3, fy - 3, fx + 3, fy + 3, C_BLACK);
}

// Per-leg base: true → inner red / outer blue (matches reference art).
static bool legInnerRed(uint8_t leg) {
  // 30° red/blue, 90° red/blue, 150° blue/red, 210° red/blue,
  // 270° blue/red, 330° blue/red
  static const uint8_t innerRed[6] = {1, 1, 0, 1, 0, 0};
  return innerRed[leg % 6] != 0;
}

static void paintMotor(uint8_t joint, uint8_t lvl) {
  uint8_t leg = joint / 3;
  uint8_t axis = joint % 3;  // 0 yaw, 1 hip, 2 knee
  int c, s;
  legDir(leg, &c, &s);
  bool iRed = legInnerRed(leg);

  if (axis == 0) {
    uint16_t dim = iRed ? RED_DIM : BLU_DIM;
    uint16_t brt = iRed ? RED_BRT : BLU_BRT;
    drawBlock(LOGO_CX, LOGO_CY, c, s, rad(34), rad(18), rad(14),
              lerp565(dim, brt, lvl));
  } else if (axis == 1) {
    uint16_t dim = iRed ? BLU_DIM : RED_DIM;
    uint16_t brt = iRed ? BLU_BRT : RED_BRT;
    drawBlock(LOGO_CX, LOGO_CY, c, s, rad(52), rad(16), rad(12),
              lerp565(dim, brt, lvl));
  } else {
    drawFoot(LOGO_CX, LOGO_CY, c, s, rad(72),
             lerp565(FOOT_DIM, FOOT_BRT, lvl));
  }
}

static void drawLogoStatic() {
  // Hub + legs in the center of the landscape frame.
  int rx[8], ry[8];
  for (uint8_t i = 0; i < 8; i++)
    octVertex(LOGO_CX, LOGO_CY, rad(26), i, &rx[i], &ry[i]);
  for (uint8_t i = 0; i < 8; i++) {
    uint8_t j = (uint8_t)((i + 1) & 7);
    fillTriangle(LOGO_CX, LOGO_CY, rx[i], ry[i], rx[j], ry[j], C_RING);
  }
  for (uint8_t i = 0; i < 8; i++)
    octVertex(LOGO_CX, LOGO_CY, rad(20), i, &rx[i], &ry[i]);
  for (uint8_t i = 0; i < 8; i++) {
    uint8_t j = (uint8_t)((i + 1) & 7);
    fillTriangle(LOGO_CX, LOGO_CY, rx[i], ry[i], rx[j], ry[j], C_BLUE);
  }
  for (uint8_t i = 0; i < 8; i++) {
    int bx, by;
    octVertex(LOGO_CX, LOGO_CY, rad(11), i, &bx, &by);
    fillRect(bx - 1, by - 1, 2, 2, C_DIM);
  }

  for (uint8_t leg = 0; leg < 6; leg++) {
    int c, s;
    legDir(leg, &c, &s);
    int x0 = LOGO_CX + (c * rad(56)) / 1000;
    int y0 = LOGO_CY + (s * rad(56)) / 1000;
    int x1 = LOGO_CX + (c * rad(68)) / 1000;
    int y1 = LOGO_CY + (s * rad(68)) / 1000;
    drawLine(x0, y0, x1, y1, C_DIM);
    drawLine(x0 + 1, y0, x1 + 1, y1, C_DIM);
  }
  for (uint8_t j = 0; j < 18; j++) {
    lightLvl[j] = 0;
    paintMotor(j, 0);
  }
}

static void updateLights(const int *ma18) {
  for (uint8_t j = 0; j < 18; j++) {
    uint8_t lvl = levelFromMa(ma18 ? ma18[j] : 0);
    if (lvl == lightLvl[j]) continue;
    lightLvl[j] = lvl;
    paintMotor(j, lvl);
  }
}

static void drawCharAt(uint16_t x, uint16_t y, char ch, uint16_t fg, uint16_t bg) {
  if (ch < 32 || ch > 127) ch = '?';
  const uint8_t *glyph = FONT5x7 + (uint16_t)(ch - 32) * 5;
  uint8_t cols[5];
  for (uint8_t c = 0; c < 5; c++) cols[c] = pgm_read_byte(glyph + c);
  setWindow(x, y, x + CHAR_W - 1, y + CHAR_H - 1);
  digitalWrite(PIN_DC, HIGH);
  digitalWrite(PIN_CS, LOW);
  for (uint8_t row = 0; row < 8; row++) {
    for (uint8_t ys = 0; ys < SCALE; ys++) {
      for (uint8_t col = 0; col < 6; col++) {
        uint16_t color = bg;
        if (col < 5 && row < 7 && (cols[col] & (1 << row))) color = fg;
        for (uint8_t xs = 0; xs < SCALE; xs++) pushPixel(color);
      }
    }
  }
  digitalWrite(PIN_CS, HIGH);
}

static uint16_t colorForSlot(uint8_t slot, const char *line) {
  if (slot == 0) {
    if (strstr(line, "ARMED")) return C_GREEN;
    if (strstr(line, "limp") || strstr(line, "STOP") ||
        strstr(line, "ERR")) return C_RED;
    return C_WHITE;
  }
  if (slot <= 2) {
    if (strstr(line, "ERR") || strstr(line, "error") ||
        strstr(line, "NO WIFI")) return C_RED;
    if (strstr(line, "demo") || strstr(line, "calibrat") ||
        strstr(line, "zeroing")) return C_YELL;
    if (strstr(line, "STOP")) return C_RED;
    return C_WHITE;
  }
  if (slot == 3) return C_CYAN;
  if (slot == 4) return C_ORANGE;
  return C_CYAN;
}

// ---- Center "mood" badge --------------------------------------------------
// The hub of the hexapod schematic doubles as a state icon readable from
// across the room: dim hub + "Zz" while booting (Linux not up yet), the
// stock blue hub when everything is fine, red hub + white X on error.

enum : uint8_t { MOOD_BOOT = 0, MOOD_OK = 1, MOOD_ERR = 2 };
static uint8_t mood = MOOD_BOOT;

static void drawMoodBadge() {
  if (!ready || !chromeDrawn || jobMode) return;
  uint16_t hub = (mood == MOOD_ERR) ? RED_BRT
                 : (mood == MOOD_BOOT) ? C_DIM : C_BLUE;
  int rx[8], ry[8];
  for (uint8_t i = 0; i < 8; i++)
    octVertex(LOGO_CX, LOGO_CY, rad(20), i, &rx[i], &ry[i]);
  for (uint8_t i = 0; i < 8; i++) {
    uint8_t j = (uint8_t)((i + 1) & 7);
    fillTriangle(LOGO_CX, LOGO_CY, rx[i], ry[i], rx[j], ry[j], hub);
  }
  if (mood == MOOD_OK) {
    for (uint8_t i = 0; i < 8; i++) {  // restore the stock center dots
      int bx, by;
      octVertex(LOGO_CX, LOGO_CY, rad(11), i, &bx, &by);
      fillRect(bx - 1, by - 1, 2, 2, C_DIM);
    }
  } else if (mood == MOOD_BOOT) {
    drawCharAt(LOGO_CX - CHAR_W, LOGO_CY - CHAR_H / 2, 'Z', C_WHITE, hub);
    drawCharAt(LOGO_CX, LOGO_CY - CHAR_H / 2, 'z', C_WHITE, hub);
  } else {
    int r = rad(12);
    for (int8_t o = -1; o <= 1; o++) {  // 3 px thick X
      drawLine(LOGO_CX - r + o, LOGO_CY - r, LOGO_CX + r + o, LOGO_CY + r,
               C_WHITE);
      drawLine(LOGO_CX - r + o, LOGO_CY + r, LOGO_CX + r + o, LOGO_CY - r,
               C_WHITE);
    }
  }
}

static void setMood(uint8_t m) {
  if (m == mood) return;
  mood = m;
  drawMoodBadge();
}

static void drawEdgeDiff(uint8_t slot, const char *text, uint16_t fg) {
  if (!ready || slot >= EDGE_SLOTS) return;
  uint16_t x0, y0;
  edgeOrigin(slot, &x0, &y0);
  char buf[EDGE_COLS];
  for (uint8_t i = 0; i < EDGE_COLS; i++) {
    char c = text[i];
    if (c == '\0') {
      while (i < EDGE_COLS) buf[i++] = ' ';
      break;
    }
    buf[i] = (c >= 32 && c <= 126) ? c : ' ';
  }
  bool fgChanged = (cacheFg[slot] != fg);
  for (uint8_t i = 0; i < EDGE_COLS; i++) {
    if (fgChanged || cacheTxt[slot][i] != buf[i]) {
      drawCharAt(x0 + (uint16_t)i * CHAR_W, y0, buf[i], fg, C_BLACK);
      cacheTxt[slot][i] = buf[i];
    }
  }
  cacheFg[slot] = fg;
}

static void ensureChrome() {
  if (chromeDrawn) return;
  fillRect(0, 0, W, H, C_BLACK);
  drawLogoStatic();
  for (uint8_t r = 0; r < EDGE_SLOTS; r++) {
    for (uint8_t c = 0; c < EDGE_COLS; c++) cacheTxt[r][c] = ' ';
    cacheFg[r] = 0xFFFF;
  }
  chromeDrawn = true;
  drawMoodBadge();
}

static void init(bool force = false) {
  // Skip re-reset when already up (avoids a ~0.5 s blank on every Linux DI
  // at web start). Pass force=true after an unplug/replug — the panel loses
  // state but ``ready`` would otherwise stay true forever.
  if (ready && !force) {
    ensureChrome();
    return;
  }
  ready = false;
  chromeDrawn = false;
  jobMode = false;

  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_DC, OUTPUT);
  pinMode(PIN_RST, OUTPUT);
  pinMode(PIN_SCK, OUTPUT);
  pinMode(PIN_MOSI, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  digitalWrite(PIN_DC, HIGH);
  digitalWrite(PIN_SCK, LOW);
  digitalWrite(PIN_MOSI, LOW);

  // Aggressive but ST7789-safe timings (was 40/120/120/120/20 ≈ 420 ms).
  digitalWrite(PIN_RST, LOW);
  delay(10);
  digitalWrite(PIN_RST, HIGH);
  delay(50);

  writeCmd(0x01);  // SWRESET
  delay(80);
  writeCmd(0x11);  // SLPOUT
  delay(80);
  writeCmd(0x3A);
  writeData(0x55);
  writeCmd(0x36);
  // Landscape: MX | MV (Adafruit rotation 1). Swap W/H to 320×240.
  writeData(0x60);
  writeCmd(0x21);
  writeCmd(0x13);
  writeCmd(0x29);  // DISPON
  delay(10);

  ready = true;
  chromeDrawn = false;
  for (uint8_t i = 0; i < 18; i++) lightLvl[i] = 0xFF;
}

// Full hardware re-init (RST + SWRESET). Use after the ribbon was reseated.
static void reinit() {
  init(true);
  ensureChrome();
  drawEdgeDiff(0, "tft", C_WHITE);
  drawEdgeDiff(1, "reinit", C_DIM);
}

// Bright full-frame colors — use to tell "dead/BL off" from "SPI ok".
static void selfTest() {
  init(true);
  fillRect(0, 0, W, H, C_WHITE);
  delay(400);
  fillRect(0, 0, W, H, C_RED);
  delay(400);
  fillRect(0, 0, W, H, C_GREEN);
  delay(400);
  fillRect(0, 0, W, H, C_BLUE);
  delay(400);
  chromeDrawn = false;
  ensureChrome();
  drawEdgeDiff(0, "tft", C_WHITE);
  drawEdgeDiff(1, "selftest", C_GREEN);
}

// Earliest useful picture: black + hexapod logo (no Linux required).
static void bootSplash() {
  mood = MOOD_BOOT;  // dim hub + "Zz" until Linux takes over
  init();
  ensureChrome();
  drawEdgeDiff(0, "boot", C_WHITE);
  drawEdgeDiff(1, "hexapod", C_DIM);
}

// While Linux has never spoken, tick a boot counter so the panel shows
// life (and how long the SoC is taking) from ~2 s after power. Called
// from loop() only when the host RX queue is idle, so the few-ms glyph
// paint cannot collide with an in-flight command.
static void bootTick(unsigned long ms) {
  if (!ready) return;
  static unsigned long lastMs = 0;
  if (ms - lastMs < 1000) return;
  lastMs = ms;
  char buf[EDGE_COLS + 1];
  snprintf(buf, sizeof(buf), "linux %lus", ms / 1000UL);
  drawEdgeDiff(2, buf, C_DIM);
}


// ---- Job mode ------------------------------------------------------------

static const int JOB_BAR_X = 8;
static const int JOB_BAR_Y = 172;
static const int JOB_BAR_W = 304;
static const int JOB_BAR_H = 20;

static void jobRowOrigin(uint8_t row, uint16_t *x, uint16_t *y) {
  *x = 4;
  switch (row) {
    case 0: *y = 8; break;             // title
    case 1: *y = 48; break;
    case 2: *y = 76; break;
    case 3: *y = 104; break;
    case 4: *y = 132; break;
    default: *y = H - CHAR_H - 4; break;  // footer
  }
}

static void drawJobRow(uint8_t row, const char *text, uint16_t fg) {
  if (!ready || row >= JOB_ROWS) return;
  uint16_t x0, y0;
  jobRowOrigin(row, &x0, &y0);
  char buf[JOB_COLS];
  for (uint8_t i = 0; i < JOB_COLS; i++) {
    char c = text ? text[i] : '\0';
    if (c == '\0') {
      while (i < JOB_COLS) buf[i++] = ' ';
      break;
    }
    buf[i] = (c >= 32 && c <= 126) ? c : ' ';
  }
  bool fgChanged = (jobFg[row] != fg);
  for (uint8_t i = 0; i < JOB_COLS; i++) {
    if (fgChanged || jobTxt[row][i] != buf[i]) {
      drawCharAt(x0 + (uint16_t)i * CHAR_W, y0, buf[i], fg, C_BLACK);
      jobTxt[row][i] = buf[i];
    }
  }
  jobFg[row] = fg;
}

// Host-link watchdog paint: Linux refreshes the panel every ~2 s, so a
// long silence means the web service (or the whole SoC) died. In job
// mode the warning takes the footer row so an error/job text above it
// stays readable; otherwise it takes edge slot 2. The next DX/DJ from a
// recovered host repaints over it.
static void hostLostTick(unsigned long silentS) {
  if (!ready) return;
  setMood(MOOD_ERR);
  if (silentS > 99) silentS = 99;
  if (jobMode) {
    char buf[JOB_COLS + 1];
    snprintf(buf, sizeof(buf), "web silent %lus", silentS);
    drawJobRow(JOB_ROWS - 1, buf, C_RED);
  } else {
    ensureChrome();
    char buf[EDGE_COLS + 1];
    snprintf(buf, sizeof(buf), "web lost %lu", silentS);
    drawEdgeDiff(2, buf, C_RED);
  }
}

// Auto-limp notice: the sketch cut all servo torque after prolonged host
// loss (see HOST_LIMP_MS in the .ino).
static void autoLimpPaint() {
  setMood(MOOD_ERR);
  if (!ready) return;
  if (jobMode) {
    drawJobRow(JOB_ROWS - 1, "AUTO LIMP - web lost", C_RED);
  } else {
    ensureChrome();
    drawEdgeDiff(0, "AUTO LIMP", C_RED);
  }
}

static void drawJobBar(int pct) {
  if (pct > 100) pct = 100;
  if (pct == jobPct) return;
  if (pct < 0) {  // this job has no progress info — blank the strip
    fillRect(JOB_BAR_X - 2, JOB_BAR_Y - 2, JOB_BAR_W + 4, JOB_BAR_H + 4,
             C_BLACK);
    jobPct = pct;
    return;
  }
  if (jobPct < 0) {  // first draw: frame + empty interior
    fillRect(JOB_BAR_X - 2, JOB_BAR_Y - 2, JOB_BAR_W + 4, JOB_BAR_H + 4,
             C_DIM);
    fillRect(JOB_BAR_X, JOB_BAR_Y, JOB_BAR_W, JOB_BAR_H, C_BLACK);
    jobPct = 0;
  }
  int wOld = (JOB_BAR_W * jobPct) / 100;
  int wNew = (JOB_BAR_W * pct) / 100;
  if (wNew > wOld)
    fillRect(JOB_BAR_X + wOld, JOB_BAR_Y, wNew - wOld, JOB_BAR_H, C_CYAN);
  else if (wNew < wOld)
    fillRect(JOB_BAR_X + wNew, JOB_BAR_Y, wOld - wNew, JOB_BAR_H, C_BLACK);
  jobPct = pct;
}

static void enterJobMode() {
  fillRect(0, 0, W, H, C_BLACK);
  for (uint8_t r = 0; r < JOB_ROWS; r++) {
    for (uint8_t c = 0; c < JOB_COLS; c++) jobTxt[r][c] = ' ';
    jobFg[r] = 0xFFFF;
  }
  jobPct = -2;
  jobMode = true;
  chromeDrawn = false;  // normal panel repaints its chrome afterwards
}

// Payload: "<pct>|title|line1|line2|line3|footer". Fields positional,
// empty allowed (clears the row); pct −1 hides the bar. Only changed
// glyphs repaint, so periodic refreshes are cheap.
static void pushJob(const char *payload) {
  if (!ready) init();
  if (!jobMode) enterJobMode();

  const char *p = payload;
  char num[8];
  uint8_t ni = 0;
  while (*p && *p != '|' && ni + 1 < sizeof(num)) num[ni++] = *p++;
  num[ni] = '\0';
  if (*p == '|') p++;
  int pct = atoi(num);

  char line[JOB_COLS + 1];
  uint8_t row = 0, li = 0;
  for (;; p++) {
    char c = *p;
    bool end = (c == '\0');
    if (!end && c != '|') {
      if (li < JOB_COLS) line[li++] = c;
      continue;
    }
    line[li] = '\0';
    if (row < JOB_ROWS) {
      uint16_t fg = (row == 0) ? C_YELL
                    : (row == JOB_ROWS - 1) ? C_CYAN : C_WHITE;
      if (row == JOB_ROWS - 1) {  // footer keeps ARMED/limp colors
        if (strstr(line, "ARMED")) fg = C_GREEN;
        else if (strstr(line, "limp") || strstr(line, "STOP")) fg = C_RED;
      }
      drawJobRow(row, line, fg);
      row++;
    }
    li = 0;
    if (end) break;
  }
  while (row < JOB_ROWS) {
    drawJobRow(row, "", C_WHITE);
    row++;
  }
  drawJobBar(pct);
}

// Activity lines in payload; power stats on opposite edges.
static void pushPanel(const char *payload, const int *ma18,
                      int live, long iMa, int v10, int load10) {
  if (!ready) init();
  if (jobMode) {  // leaving job mode — normal chrome must repaint
    jobMode = false;
    chromeDrawn = false;
  }
  ensureChrome();
  updateLights(ma18);

  char act[3][EDGE_COLS + 1];
  for (uint8_t i = 0; i < 3; i++) act[i][0] = '\0';
  uint8_t ai = 0;
  char line[EDGE_COLS + 1];
  uint8_t li = 0;
  for (const char *p = payload;; p++) {
    char c = *p;
    bool end = (c == '\0');
    bool sep = (c == '|');
    if (!end && !sep) {
      if (li < EDGE_COLS) line[li++] = c;
      continue;
    }
    line[li] = '\0';
    if (li > 0 && ai < 3) {
      memcpy(act[ai], line, li + 1);
      ai++;
    }
    li = 0;
    if (end) break;
  }
  for (uint8_t i = 0; i < 3; i++) {
    drawEdgeDiff(i, act[i], colorForSlot(i, act[i]));
  }

  // Hub badge: red X while any status line reports an error, else normal.
  uint8_t m = MOOD_OK;
  for (uint8_t i = 0; i < 3; i++) {
    if (act[i][0] && (strstr(act[i], "ERR") || strstr(act[i], "error") ||
                      strstr(act[i], "NO WIFI"))) m = MOOD_ERR;
  }
  setMood(m);

  char buf[EDGE_COLS + 1];
  snprintf(buf, sizeof(buf), "live %d", live);
  drawEdgeDiff(3, buf, colorForSlot(3, buf));

  long a = iMa / 1000;
  long frac = (iMa % 1000) / 10;
  snprintf(buf, sizeof(buf), "I=%ld.%02ldA", a, frac);
  drawEdgeDiff(4, buf, colorForSlot(4, buf));

  if (v10 > 0) {
    snprintf(buf, sizeof(buf), "V=%d.%d L%d%%",
             v10 / 10, v10 % 10, load10 / 10);
  } else {
    snprintf(buf, sizeof(buf), "V=--");
  }
  drawEdgeDiff(5, buf, colorForSlot(5, buf));
}

}  // namespace tft

#endif

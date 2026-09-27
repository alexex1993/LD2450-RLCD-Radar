// Radar screen on the 400x300 reflective panel.
//
//   +--------------------------------------------------------------+
//   | LD2450 RADAR [OK]   2 TGT   10 Hz   25 FPS         00:12:37  |  status bar
//   | [T1 5.14m ...]     [T2 NO TARGET]      [T3 6.43m ...]        |  target cards
//   |                      .-""""-.                                |
//   |                 .-'  8m     '-.                              |  radar sector
//   |              ...   rings 1..8 m   ...                        |  +-60 deg, 1..8 m
//   | MAX SPEED      '.           .'                  RX 8495      |
//   | 2.85 m/s          '.     .'                 ERR 0            |
//   | T3 00:05:12 KEY=rst  '-_-'          FW V2.04.23101915        |
//   +--------------------------------------------------------------+
//
// The static grid is rendered once into a copy of the frame buffer and
// memcpy'd back at the start of every frame; only targets and text are drawn
// per frame. Panel is 1 bpp, so "semi-transparent" empty cards are made by
// clearing every other pixel (a 50 % checkerboard) over the drawn card.

#include <SPI.h>
#include <U8g2lib.h>

#include "board_pins.h"
#include "radar.h"

// U8G2_R1 = 400x300 landscape. Use U8G2_R3 if it is upside down in your case.
static U8G2_ST7305_300X400_F_4W_HW_SPI lcd(U8G2_R1, PIN_LCD_CS, PIN_LCD_DC, PIN_LCD_RST);

static const int W = 400;
static const int BAR_H = 15;
static const int CARD_Y = 18, CARD_H = 50, CARD_W = 130;
static const int CX = 200, CY = 294;  // sensor position on screen
static const int R_PX = 216;          // radius of the 8 m ring
static const float PPM = R_PX / RADAR_RANGE_M;
static const float DEG = 0.01745329f;

static uint8_t *bgCache = nullptr;
static size_t bufSize = 0;

static const uint8_t *FONT_S = u8g2_font_profont12_tf;  // 6 px wide
static const uint8_t *FONT_M = u8g2_font_profont17_tf;
static const uint8_t *FONT_L = u8g2_font_profont22_tf;  // 12 px wide

// ---------------------------------------------------------------- helpers --

static void toScreen(float xm, float ym, int &sx, int &sy) {
  sx = CX + (int)lroundf(xm * PPM);
  sy = CY - (int)lroundf(ym * PPM);
}

static void polar(float r, float deg, int &sx, int &sy) {
  sx = CX + (int)lroundf(r * sinf(deg * DEG));
  sy = CY - (int)lroundf(r * cosf(deg * DEG));
}

static bool onScreen(int x, int y) { return x >= 0 && x < W && y >= 0 && y < 300; }

static void dottedArc(float r, int stepPx) {
  float step = (float)stepPx / r / DEG;
  for (float a = -RADAR_FOV_DEG; a <= RADAR_FOV_DEG; a += step) {
    int x, y;
    polar(r, a, x, y);
    lcd.drawPixel(x, y);
  }
}

static void solidArc(float r) {
  int px, py;
  polar(r, -RADAR_FOV_DEG, px, py);
  float step = 2.0f / r / DEG;
  for (float a = -RADAR_FOV_DEG + step; a <= RADAR_FOV_DEG + step * 0.5f; a += step) {
    int x, y;
    polar(r, a > RADAR_FOV_DEG ? RADAR_FOV_DEG : a, x, y);
    lcd.drawLine(px, py, x, y);
    px = x; py = y;
  }
}

static void dottedRay(float deg, float r0, float r1, int stepPx) {
  for (float r = r0; r <= r1; r += stepPx) {
    int x, y;
    polar(r, deg, x, y);
    lcd.drawPixel(x, y);
  }
}

// Text with a cleared background so it stays legible over grid lines.
static void labelClear(int x, int y, const char *s) {
  int w = lcd.getUTF8Width(s);
  int a = lcd.getAscent();
  lcd.setDrawColor(0);
  lcd.drawBox(x - 1, y - a - 1, w + 2, a + 3);
  lcd.setDrawColor(1);
  lcd.drawUTF8(x, y, s);
}

// Black box, white text.
static int labelInverse(int x, int y, const char *s) {
  int w = lcd.getUTF8Width(s);
  int a = lcd.getAscent();
  lcd.drawBox(x - 2, y - a - 2, w + 4, a + 4);
  lcd.setDrawColor(0);
  lcd.drawUTF8(x, y, s);
  lcd.setDrawColor(1);
  return w + 4;
}

static void checkerClear(int x0, int y0, int w, int h) {
  lcd.setDrawColor(0);
  for (int y = y0; y < y0 + h; y++)
    for (int x = x0 + ((y ^ 1) & 1); x < x0 + w; x += 2) lcd.drawPixel(x, y);
  lcd.setDrawColor(1);
}

static void thickLine(int x0, int y0, int x1, int y1) {
  lcd.drawLine(x0, y0, x1, y1);
  if (abs(x1 - x0) > abs(y1 - y0)) lcd.drawLine(x0, y0 + 1, x1, y1 + 1);
  else lcd.drawLine(x0 + 1, y0, x1 + 1, y1);
}

// ------------------------------------------------------------ static grid --

static void drawGrid() {
  lcd.clearBuffer();

  // Range rings 1..7 m dotted, 8 m solid and thick.
  for (int m = 1; m < (int)RADAR_RANGE_M; m++) dottedArc(m * PPM, (m & 1) ? 3 : 5);
  solidArc(R_PX);
  solidArc(R_PX - 1);

  // Sector edges, thick.
  int ex, ey;
  polar(R_PX, -RADAR_FOV_DEG, ex, ey);
  thickLine(CX, CY, ex, ey);
  polar(R_PX, RADAR_FOV_DEG, ex, ey);
  thickLine(CX, CY, ex, ey);

  // Azimuth rays.
  for (int a = -45; a <= 45; a += 15) {
    if (a == 0) dottedRay(0, 10, R_PX, 3);
    else if (a % 30 == 0) dottedRay(a, 10, R_PX, 4);
    else dottedRay(a, PPM * 2, R_PX, 7);
  }

  // Range labels along the centre line.
  lcd.setFont(FONT_S);
  char buf[16];
  for (int m = 1; m <= (int)RADAR_RANGE_M; m++) {
    int x, y;
    polar(m * PPM, 0, x, y);
    snprintf(buf, sizeof(buf), "%dm", m);
    labelClear(x + 3, y + 4, buf);
  }

  // Azimuth labels just inside the outer ring.
  static const int angs[] = {-60, -30, 30, 60};
  for (int a : angs) {
    int x, y;
    polar(R_PX - 14, a, x, y);
    snprintf(buf, sizeof(buf), "%d", a);
    int w = lcd.getStrWidth(buf);
    labelClear(x - w / 2 + (a < 0 ? 6 : -6), y + 4, buf);
  }

  // Sensor body at the apex.
  lcd.drawBox(CX - 8, CY - 3, 17, 6);

  memcpy(bgCache, lcd.getBufferPtr(), bufSize);
}

// ------------------------------------------------------------ status bar ---

static void drawStatusBar(const RadarState &s, uint32_t nowMs) {
  lcd.drawBox(0, 0, W, BAR_H);
  lcd.setFont(FONT_S);
  lcd.setDrawColor(0);
  lcd.drawStr(4, 11, "LD2450 RADAR");

  const char *st = s.link == LinkStatus::OK ? "OK" : s.link == LinkStatus::WAIT ? "WAIT" : "NO DATA";
  int sw = lcd.getStrWidth(st);
  lcd.drawBox(82, 2, sw + 6, 11);  // white chip inside the black bar
  lcd.setDrawColor(1);
  lcd.drawStr(85, 11, st);
  lcd.setDrawColor(0);

  char buf[24];
  int n = 0;
  for (const Track &t : s.tracks) n += t.active;
  snprintf(buf, sizeof(buf), "%d TGT", n);
  lcd.drawStr(150, 11, buf);
  snprintf(buf, sizeof(buf), "%2.0f Hz", s.sensorHz);
  lcd.drawStr(200, 11, buf);
  snprintf(buf, sizeof(buf), "%2.0f FPS", s.displayFps);
  lcd.drawStr(250, 11, buf);

  uint32_t sec = nowMs / 1000;
  snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", (unsigned long)(sec / 3600),
           (unsigned long)(sec / 60 % 60), (unsigned long)(sec % 60));
  lcd.drawStr(W - 4 - lcd.getStrWidth(buf), 11, buf);
  lcd.setDrawColor(1);
}

// ------------------------------------------------------------ target cards --

static void drawCard(int i, const Track &t) {
  const int x0 = 2 + i * (CARD_W + 4), y0 = CARD_Y;
  char buf[24];

  lcd.setDrawColor(0);
  lcd.drawBox(x0, y0, CARD_W, CARD_H);  // cards may overlap the grid
  lcd.setDrawColor(1);
  lcd.drawFrame(x0, y0, CARD_W, CARD_H);
  lcd.drawFrame(x0 + 1, y0 + 1, CARD_W - 2, CARD_H - 2);

  lcd.setFont(FONT_S);
  snprintf(buf, sizeof(buf), "T%d", i + 1);
  lcd.drawBox(x0, y0, 20, 13);
  lcd.setDrawColor(0);
  lcd.drawStr(x0 + 4, y0 + 10, buf);
  lcd.setDrawColor(1);

  if (!t.active) {
    lcd.setFont(FONT_M);
    const char *s = "NO TARGET";
    lcd.drawStr(x0 + (CARD_W - lcd.getStrWidth(s)) / 2, y0 + 34, s);
    checkerClear(x0, y0, CARD_W, CARD_H);
    return;
  }

  snprintf(buf, sizeof(buf), "%+.1f\xC2\xB0", t.angleDeg());  // UTF-8 degree sign
  lcd.drawUTF8(x0 + CARD_W - 5 - lcd.getUTF8Width(buf), y0 + 11, buf);

  lcd.setFont(FONT_L);
  snprintf(buf, sizeof(buf), "%.2fm", t.dist());
  lcd.drawStr(x0 + 5, y0 + 31, buf);

  lcd.setFont(FONT_S);
  int cms = (int)lroundf(fabsf(t.speed) * 100.0f);
  snprintf(buf, sizeof(buf), "%d cm/s", cms);
  lcd.drawStr(x0 + 5, y0 + 44, buf);
  if (cms >= 5) {
    const char *dir = t.speed > 0 ? "OUT" : "IN";
    labelInverse(x0 + CARD_W - 6 - lcd.getStrWidth(dir), y0 + 44, dir);
  }
}

// ---------------------------------------------------------- radar targets --

static void drawArrow(int x0, int y0, float ux, float uy, int len) {
  // ux/uy: unit vector in screen space
  int x1 = x0 + (int)lroundf(ux * len), y1 = y0 + (int)lroundf(uy * len);
  thickLine(x0, y0, x1, y1);
  const float c = -0.866f, s = 0.5f;  // rotate by +-150 deg for the head
  for (int k = -1; k <= 1; k += 2) {
    float hx = ux * c - uy * s * k, hy = ux * s * k + uy * c;
    lcd.drawLine(x1, y1, x1 + (int)lroundf(hx * 7), y1 + (int)lroundf(hy * 7));
  }
}

static void drawTarget(int i, const Track &t) {
  // Trail: oldest first, fading from 1 px to 2x2 px dots.
  for (int k = 0; k < t.trailCount; k++) {
    int idx = (t.trailHead + TRAIL_LEN - t.trailCount + k) % TRAIL_LEN;
    int x, y;
    toScreen(t.trailX[idx], t.trailY[idx], x, y);
    if (!onScreen(x, y) || y < CARD_Y + CARD_H) continue;
    if (k < t.trailCount / 2) lcd.drawPixel(x, y);
    else lcd.drawBox(x, y, 2, 2);
  }

  int sx, sy;
  toScreen(t.dx, t.dy, sx, sy);
  if (!onScreen(sx, sy)) return;

  // Direction: the position-derived velocity when it is meaningful, otherwise
  // the sensor's radial speed along the line of sight.
  float vx = t.vx, vy = t.vy;
  float v = sqrtf(vx * vx + vy * vy);
  if (v < 0.10f && fabsf(t.speed) >= 0.05f) {
    float d = t.dist();
    if (d > 0.01f) {
      vx = t.x / d * t.speed;
      vy = t.y / d * t.speed;
      v = fabsf(t.speed);
    }
  }
  if (v >= 0.05f) {
    int len = 12 + (int)fminf(v * 10.0f, 24.0f);
    drawArrow(sx, sy, vx / v, -vy / v, len);  // screen y grows downward
  }

  // Bullseye: clear halo, ring, filled centre.
  lcd.setDrawColor(0);
  lcd.drawDisc(sx, sy, 10);
  lcd.setDrawColor(1);
  lcd.drawCircle(sx, sy, 8);
  lcd.drawCircle(sx, sy, 7);
  lcd.drawDisc(sx, sy, 4);

  // Label to the side away from the arrow.
  char buf[20];
  snprintf(buf, sizeof(buf), "T%d %.2fm", i + 1, t.dist());
  lcd.setFont(FONT_S);
  int w = lcd.getStrWidth(buf) + 4;
  bool left = (vx > 0) || (sx + 14 + w > W);
  if (sx - 14 - w < 0) left = false;
  int lx = left ? sx - 12 - w : sx + 14;
  labelInverse(lx, sy - 6, buf);
}

// ------------------------------------------------------------ bottom info --

static void drawFooter(const RadarState &s) {
  char buf[32];
  lcd.setFont(FONT_S);
  lcd.drawStr(4, 262, "MAX SPEED");

  lcd.setFont(FONT_L);
  snprintf(buf, sizeof(buf), "%.2f m/s", s.stats.maxSpeed);
  lcd.drawStr(4, 283, buf);

  lcd.setFont(FONT_S);
  if (s.stats.maxSlot >= 0) {
    uint32_t sec = s.stats.maxAtMs / 1000;
    snprintf(buf, sizeof(buf), "T%d %02lu:%02lu:%02lu %.1fkm/h", s.stats.maxSlot + 1,
             (unsigned long)(sec / 3600), (unsigned long)(sec / 60 % 60),
             (unsigned long)(sec % 60), s.stats.maxSpeed * 3.6f);
  } else {
    snprintf(buf, sizeof(buf), "KEY = reset");
  }
  lcd.drawStr(4, 297, buf);

  snprintf(buf, sizeof(buf), "RX %lu", (unsigned long)s.rxFrames);
  lcd.drawStr(W - 4 - lcd.getStrWidth(buf), 269, buf);
  snprintf(buf, sizeof(buf), "ERR %lu", (unsigned long)s.badFrames);
  lcd.drawStr(W - 4 - lcd.getStrWidth(buf), 283, buf);
  snprintf(buf, sizeof(buf), "FW %s", s.firmware[0] ? s.firmware : "?");
  lcd.drawStr(W - 4 - lcd.getStrWidth(buf), 297, buf);
}

// ------------------------------------------------------------------ public --

void uiBegin() {
  // U8g2 inherits the global SPI's pins; claim them first (skill rule 5).
  SPI.begin(PIN_LCD_SCLK, -1, PIN_LCD_MOSI, -1);
  lcd.begin();
  lcd.setBusClock(24000000);
  lcd.sendF("c", 0x38);  // HPM: ~32 Hz self-refresh, needed for live motion
  delay(100);

  bufSize = 8u * lcd.getBufferTileHeight() * lcd.getBufferTileWidth();
  bgCache = (uint8_t *)malloc(bufSize);
  drawGrid();
}

void uiRender(const RadarState &s, uint32_t nowMs) {
  memcpy(lcd.getBufferPtr(), bgCache, bufSize);

  for (int i = 0; i < 3; i++)
    if (s.tracks[i].active) drawTarget(i, s.tracks[i]);

  drawFooter(s);
  for (int i = 0; i < 3; i++) drawCard(i, s.tracks[i]);
  drawStatusBar(s, nowMs);

  lcd.sendBuffer();
}

#pragma once

#include <Arduino.h>

#include "ld2450.h"

// --- Wiring -----------------------------------------------------------------
// LD2450 TX -> header RXD (GPIO44), LD2450 RX -> header TXD (GPIO43),
// LD2450 5V -> header VBUS, GND -> GND. See README.md.
#define PIN_LD2450_RX 44  // ESP receives here (sensor TX)
#define PIN_LD2450_TX 43  // ESP transmits here (sensor RX)
#define LD2450_BAUD 256000

// --- Radar geometry ---------------------------------------------------------
#define RADAR_RANGE_M 8.0f   // outer ring
#define RADAR_FOV_DEG 60.0f  // half-angle, the LD2450 covers +-60 deg
// Flip left/right if targets move the wrong way on screen for how the sensor
// is mounted (e.g. facing you vs facing away from you).
#define RADAR_MIRROR_X 0

#define TRAIL_LEN 24

// One tracked slot. The LD2450 keeps its own target IDs stable across frames,
// so slot i on the sensor is track i here.
struct Track {
  bool active = false;
  float x = 0, y = 0;    // filtered position, metres (x right, y forward)
  float dx = 0, dy = 0;  // eased display position, metres
  float vx = 0, vy = 0;  // velocity from position history, m/s
  float speed = 0;       // radial speed from the sensor, m/s (+ = away)
  uint32_t lastSeenMs = 0;
  uint32_t lastUpdMs = 0;
  float trailX[TRAIL_LEN], trailY[TRAIL_LEN];
  uint8_t trailHead = 0, trailCount = 0;

  float dist() const { return sqrtf(x * x + y * y); }
  float angleDeg() const { return atan2f(x, y) * 57.29578f; }
};

struct SessionStats {
  float maxSpeed = 0;  // |radial speed|, m/s
  int maxSlot = -1;
  uint32_t maxAtMs = 0;
};

enum class LinkStatus { WAIT, OK, LOST };

struct RadarState {
  Track tracks[3];
  SessionStats stats;
  LinkStatus link = LinkStatus::WAIT;
  uint32_t rxFrames = 0, badFrames = 0;
  float displayFps = 0, sensorHz = 0;
  const char *firmware = "";
};

// tracker.cpp
void trackerUpdate(RadarState &s, const Ld2450Frame &f, uint32_t nowMs);
void trackerExpire(RadarState &s, uint32_t nowMs);
void trackerAnimate(RadarState &s);  // per rendered frame: ease display positions

// radar_ui.cpp
void uiBegin();
void uiRender(const RadarState &s, uint32_t nowMs);

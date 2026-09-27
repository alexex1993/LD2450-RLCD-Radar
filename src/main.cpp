// HLK-LD2450 radar display for the Waveshare ESP32-S3-RLCD-4.2.
//
// The sensor streams ~10 reports/s over UART; the screen redraws at ~25 FPS
// and eases target dots between reports so they glide rather than jump.
// KEY (GPIO18) resets the session maximum speed.

#include <Arduino.h>

#include "board_pins.h"
#include "ld2450.h"
#include "radar.h"

static const uint32_t FRAME_MS = 40;         // ~25 FPS; the panel refreshes at ~32 Hz
static const uint32_t LINK_TIMEOUT_MS = 1000;

static Ld2450 radar;
static RadarState state;

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) delay(10);

  Serial.printf("psram       : %s\n", psramFound() ? "ok" : "NOT FOUND (check qio_opi)");

  pinMode(PIN_BTN_KEY, INPUT_PULLUP);
  radar.begin(Serial1, PIN_LD2450_RX, PIN_LD2450_TX, LD2450_BAUD);
  uiBegin();
  radar.requestFirmware();
}

void loop() {
  const uint32_t now = millis();

  static Ld2450Frame frame;
  static uint32_t sensorFrames = 0;
  if (radar.poll(&frame)) {
    trackerUpdate(state, frame, now);
    sensorFrames++;
  } else {
    trackerExpire(state, now);
  }

  // Link status
  // millis() again, not `now`: poll() may have stamped lastFrameMs a tick
  // after `now`, and `now - lastFrameMs` would wrap to ~4e9 and read as LOST.
  if (radar.frames() == 0) state.link = LinkStatus::WAIT;
  else state.link = (millis() - radar.lastFrameMs() > LINK_TIMEOUT_MS) ? LinkStatus::LOST : LinkStatus::OK;
  if (state.link == LinkStatus::LOST)
    for (Track &t : state.tracks) t.active = false;

  // Firmware query: retry every 2 s while data is flowing (a sensor that is
  // not connected yet would not answer anyway), but only a few times. Each
  // query drops the sensor into config mode, which resets its tracking, so
  // retrying forever (e.g. while the phone app holds the sensor over BT)
  // wipes the targets every 2 s.
  static uint32_t lastFwReq = 0;
  static uint8_t fwTries = 0;
  if (!radar.hasFirmware() && fwTries < 3 && state.link == LinkStatus::OK && now - lastFwReq > 2000) {
    lastFwReq = now;
    fwTries++;
    radar.requestFirmware();
  }
  state.firmware = radar.firmware();
  state.rxFrames = radar.frames();
  state.badFrames = radar.badFrames();

  // KEY: reset session max speed (press edge, crude debounce).
  static bool keyWas = false;
  bool key = digitalRead(PIN_BTN_KEY) == LOW;
  if (key && !keyWas) {
    delay(20);
    if (digitalRead(PIN_BTN_KEY) == LOW) {
      state.stats = SessionStats();
      Serial.println("max speed reset");
    }
  }
  keyWas = key;

  // Rates, over one-second windows.
  static uint32_t rateT0 = 0, renderFrames = 0;
  if (now - rateT0 >= 1000) {
    float dt = (now - rateT0) / 1000.0f;
    state.displayFps = renderFrames / dt;
    state.sensorHz = sensorFrames / dt;
    renderFrames = sensorFrames = 0;
    rateT0 = now;

    Serial.printf("[%s] %.0f Hz  max %.2f m/s", state.link == LinkStatus::OK ? "OK" : "--",
                  state.sensorHz, state.stats.maxSpeed);
    for (int i = 0; i < 3; i++) {
      const Track &t = state.tracks[i];
      if (t.active)
        Serial.printf("  T%d %.2fm %+.0fdeg %+.0fcm/s", i + 1, t.dist(), t.angleDeg(), t.speed * 100);
    }
    Serial.println();
  }

  static uint32_t lastRender = 0;
  if (now - lastRender >= FRAME_MS) {
    lastRender = now;
    trackerAnimate(state);
    uiRender(state, now);
    renderFrames++;
  }
}

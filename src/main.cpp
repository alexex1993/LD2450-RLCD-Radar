// HLK-LD2450 radar display for the Waveshare ESP32-S3-RLCD-4.2.
//
// The sensor streams ~10 reports/s over UART; the screen redraws at ~25 FPS
// and eases target dots between reports so they glide rather than jump.
// KEY (GPIO18): a short press resets the session maximum speed, holding it
// for 2 s powers off (deep sleep with no wake source, so only a reset wakes it).

#include <Arduino.h>
#include <esp_sleep.h>

#include "board_pins.h"
#include "ld2450.h"
#include "radar.h"

static const uint32_t FRAME_MS = 40;         // ~25 FPS; the panel refreshes at ~32 Hz
static const uint32_t LINK_TIMEOUT_MS = 1000;
static const uint32_t KEY_OFF_MS = 2000;     // hold KEY this long to power off

static Ld2450 radar;
static RadarState state;

// Leaves a "power off" screen on the panel and sleeps until the chip is reset
// (PWR button or a USB replug). No wake source is armed, so KEY cannot wake it.
static void powerOff() {
  Serial.println("power off: deep sleep until reset");
  Serial.flush();
  Serial1.end();
  uiPowerOff();
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_deep_sleep_start();
}

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

  // KEY: short press resets the session max speed, a long hold powers off.
  // The board debounces KEY with a 100 nF cap; 30 ms filters the rest.
  static bool keyWas = false;
  static uint32_t keyDownMs = 0;
  bool key = digitalRead(PIN_BTN_KEY) == LOW;
  if (key && !keyWas) keyDownMs = now;
  if (key && now - keyDownMs >= KEY_OFF_MS) powerOff();
  if (!key && keyWas && now - keyDownMs >= 30) {
    state.stats = SessionStats();
    Serial.println("max speed reset");
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

    Serial.printf("[%s] %.0f Hz  max %.1f km/h", state.link == LinkStatus::OK ? "OK" : "--",
                  state.sensorHz, state.stats.maxSpeed * 3.6f);
    for (int i = 0; i < 3; i++) {
      const Track &t = state.tracks[i];
      if (t.active)
        Serial.printf("  T%d %.2fm %+.0fdeg %+.1fkm/h", i + 1, t.dist(), t.angleDeg(),
                      t.speed * 3.6f);
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

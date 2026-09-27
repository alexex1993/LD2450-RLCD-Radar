// HLK-LD2450 24 GHz multi-target radar — UART protocol.
//
// The sensor streams one 30-byte report frame per measurement (~10 Hz) at
// 256000 8N1, without being asked:
//
//   AA FF 03 00 | target1 (8 B) | target2 (8 B) | target3 (8 B) | 55 CC
//
// Each target is four little-endian uint16: X (mm), Y (mm), speed (cm/s),
// distance resolution (mm). X, Y and speed use sign-magnitude with the sign
// INVERTED from the usual convention: bit 15 set = positive, clear = negative.
// An all-zero target slot means "no target in this slot".
//
// Commands and their ACKs are framed FD FC FB FA | len | ... | 04 03 02 01.
// Only the firmware-version query is used here.

#pragma once

#include <Arduino.h>

struct Ld2450Target {
  bool present;
  int16_t x_mm;       // lateral, + = sensor's right (see RADAR_MIRROR_X)
  int16_t y_mm;       // forward from the sensor face, always >= 0
  int16_t speed_cms;  // radial; + = moving away, - = approaching
  uint16_t res_mm;    // distance gate resolution
};

struct Ld2450Frame {
  Ld2450Target t[3];
};

class Ld2450 {
 public:
  void begin(HardwareSerial &port, int rxPin, int txPin, uint32_t baud = 256000);

  // Feed all pending UART bytes to the parser. Returns true when a complete,
  // valid report frame was decoded into *out.
  bool poll(Ld2450Frame *out);

  // Enter config mode, query the firmware version, leave config mode. The
  // sensor pauses its report stream for the ~100 ms this takes.
  void requestFirmware();

  bool hasFirmware() const { return fw_[0] != '\0'; }
  const char *firmware() const { return fw_; }

  uint32_t frames() const { return frames_; }
  uint32_t badFrames() const { return bad_; }
  uint32_t lastFrameMs() const { return lastFrameMs_; }

 private:
  void sendCommand(uint16_t cmd, const uint8_t *val, uint8_t valLen);
  bool feed(uint8_t b, Ld2450Frame *out);
  void decodeReport(Ld2450Frame *out);
  void decodeAck();

  HardwareSerial *port_ = nullptr;
  uint8_t buf_[64];
  uint8_t len_ = 0;
  char fw_[24] = {0};
  uint32_t frames_ = 0;
  uint32_t bad_ = 0;
  uint32_t lastFrameMs_ = 0;
};

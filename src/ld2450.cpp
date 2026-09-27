#include "ld2450.h"

static const uint8_t REPORT_HEAD[4] = {0xAA, 0xFF, 0x03, 0x00};
static const uint8_t REPORT_TAIL[2] = {0x55, 0xCC};
static const uint8_t CMD_HEAD[4] = {0xFD, 0xFC, 0xFB, 0xFA};
static const uint8_t CMD_TAIL[4] = {0x04, 0x03, 0x02, 0x01};

static const size_t REPORT_LEN = 30;

static const uint16_t CMD_ENABLE_CONFIG = 0x00FF;
static const uint16_t CMD_END_CONFIG = 0x00FE;
static const uint16_t CMD_READ_FIRMWARE = 0x00A0;

// Sign-magnitude, bit 15 set means positive.
static int16_t decodeSigned(uint16_t raw) {
  int16_t mag = (int16_t)(raw & 0x7FFF);
  return (raw & 0x8000) ? mag : (int16_t)-mag;
}

static uint16_t le16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }

void Ld2450::begin(HardwareSerial &port, int rxPin, int txPin, uint32_t baud) {
  port_ = &port;
  // One report is 30 bytes; a generous RX buffer rides out a slow frame
  // render without dropping bytes.
  port_->setRxBufferSize(1024);
  port_->begin(baud, SERIAL_8N1, rxPin, txPin);
}

void Ld2450::sendCommand(uint16_t cmd, const uint8_t *val, uint8_t valLen) {
  uint8_t pkt[4 + 2 + 2 + 8 + 4];
  size_t n = 0;
  memcpy(pkt + n, CMD_HEAD, 4); n += 4;
  uint16_t len = 2 + valLen;
  pkt[n++] = len & 0xFF;
  pkt[n++] = len >> 8;
  pkt[n++] = cmd & 0xFF;
  pkt[n++] = cmd >> 8;
  if (valLen) { memcpy(pkt + n, val, valLen); n += valLen; }
  memcpy(pkt + n, CMD_TAIL, 4); n += 4;
  port_->write(pkt, n);
  port_->flush();
}

void Ld2450::requestFirmware() {
  if (!port_) return;
  static const uint8_t enableVal[2] = {0x01, 0x00};
  sendCommand(CMD_ENABLE_CONFIG, enableVal, 2);
  delay(50);
  sendCommand(CMD_READ_FIRMWARE, nullptr, 0);
  delay(50);
  sendCommand(CMD_END_CONFIG, nullptr, 0);
  // The ACKs are picked up by the normal poll() path.
}

bool Ld2450::poll(Ld2450Frame *out) {
  if (!port_) return false;
  bool got = false;
  while (port_->available()) {
    if (feed((uint8_t)port_->read(), out)) got = true;
  }
  return got;
}

// Byte-wise framer for both frame kinds. A byte that breaks the header prefix
// restarts the search at that byte, so the parser re-syncs within one frame
// after any corruption.
bool Ld2450::feed(uint8_t b, Ld2450Frame *out) {
  if (len_ < 4) {
    bool okReport = (len_ == 0 || buf_[0] == REPORT_HEAD[0]) && b == REPORT_HEAD[len_];
    bool okCmd = (len_ == 0 || buf_[0] == CMD_HEAD[0]) && b == CMD_HEAD[len_];
    if (okReport || okCmd) {
      buf_[len_++] = b;
    } else {
      len_ = 0;
      if (b == REPORT_HEAD[0] || b == CMD_HEAD[0]) buf_[len_++] = b;
    }
    return false;
  }

  buf_[len_++] = b;

  if (buf_[0] == REPORT_HEAD[0]) {
    if (len_ < REPORT_LEN) return false;
    len_ = 0;
    if (buf_[28] != REPORT_TAIL[0] || buf_[29] != REPORT_TAIL[1]) {
      bad_++;
      return false;
    }
    decodeReport(out);
    return true;
  }

  // Command ACK: header(4) + len(2) + payload(len) + tail(4)
  if (len_ < 6) return false;
  size_t total = 4 + 2 + le16(buf_ + 4) + 4;
  if (total > sizeof(buf_)) {  // garbage length — drop and re-sync
    len_ = 0;
    bad_++;
    return false;
  }
  if (len_ < total) return false;
  len_ = 0;
  if (memcmp(buf_ + total - 4, CMD_TAIL, 4) != 0) {
    bad_++;
    return false;
  }
  decodeAck();
  return false;
}

void Ld2450::decodeReport(Ld2450Frame *out) {
  for (int i = 0; i < 3; i++) {
    const uint8_t *p = buf_ + 4 + i * 8;
    Ld2450Target &t = out->t[i];
    bool empty = true;
    for (int k = 0; k < 8; k++) if (p[k]) { empty = false; break; }
    t.present = !empty;
    t.x_mm = decodeSigned(le16(p + 0));
    t.y_mm = decodeSigned(le16(p + 2));
    t.speed_cms = decodeSigned(le16(p + 4));
    t.res_mm = le16(p + 6);
  }
  frames_++;
  lastFrameMs_ = millis();
}

void Ld2450::decodeAck() {
  // buf_[6..7] = command word | 0x0100, buf_[8..9] = status (0 = success)
  uint16_t cmd = le16(buf_ + 6) & 0x00FF;
  uint16_t status = le16(buf_ + 8);
  if (cmd != CMD_READ_FIRMWARE || status != 0) return;
  // buf_[10..11] firmware type, buf_[12..13] major (minor.major bytes),
  // buf_[14..17] build (little endian) -> "V2.04.23101915"
  snprintf(fw_, sizeof(fw_), "V%u.%02X.%02X%02X%02X%02X", buf_[13], buf_[12], buf_[17],
           buf_[16], buf_[15], buf_[14]);
}

# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Arduino/PlatformIO firmware that shows an HLK-LD2450 24 GHz radar's targets on a Waveshare ESP32-S3-RLCD-4.2 (400×300 landscape, 1 bpp reflective ST7305 panel, no backlight). For board questions (pins, PSRAM, U8g2/ST7305, straps, USB CDC), use the `esp32s3-rlcd42` skill. `include/board_pins.h` cites "SKILL.md" and "reference/board-hardware.md", which are in that skill and not in this repo.

## Commands

```sh
pio run                          # build
pio run -t upload -t monitor     # flash over Type-C (native USB) and open serial at 115200
pio device monitor               # serial only
```

There are no tests and no linter. The board has no UART bridge, so `Serial` is USB CDC (`ARDUINO_USB_CDC_ON_BOOT=1`). Once a second, the serial log prints link state, sensor Hz, max speed and the active tracks. At boot it prints a PSRAM check.

## platformio.ini traps (read its comments before editing)

- You need `board_build.arduino.memory_type = qio_opi`. Without it, the N16R8's 8 MB PSRAM is never mapped, and nothing reports an error.
- `olikraus/U8g2@^2.36.18` is intentional. Waveshare's docs ask for "2.36.19", but that number follows Arduino Library Manager numbering and does not resolve on PlatformIO. Don't bump it.

## Architecture

Data flows one way through `loop()` in `src/main.cpp`:

1. **`Ld2450` (`src/ld2450.cpp`, `include/ld2450.h`)**: a byte-wise UART framer on `Serial1` (GPIO44 RX / GPIO43 TX, 256000 baud). It handles two frame kinds in one state machine: 30-byte report frames (`AA FF 03 00 … 55 CC`) and command ACKs (`FD FC FB FA … 04 03 02 01`). The only command used is the firmware-version query. `requestFirmware()` blocks for about 100 ms, and its ACK arrives later through the normal `poll()` path. X, Y and speed use sign-magnitude with an **inverted** sign bit (bit 15 set means positive). An all-zero slot means no target.
2. **Tracker (`src/tracker.cpp`)**: maps sensor slot *i* straight to `Track` *i*, because the LD2450 keeps IDs stable. It runs two filter stages:
   - Per sensor frame (~10 Hz): position and velocity low-pass, a trail sampled every 8 cm, and the session max speed.
   - Per screen frame (~25 Hz, `trackerAnimate`): eases the display position (`dx`/`dy`) toward the filtered position, so dots glide.
   
   A track is held for 600 ms through dropouts. `RADAR_MIRROR_X` is applied here.
3. **UI (`src/radar_ui.cpp`)**: U8g2 full-buffer (`_F_`) ST7305 in `U8G2_R1`. The static grid is drawn once, then copied into `bgCache` (malloc'd). Each frame `memcpy`s that cache back, draws the targets, footer, cards and status bar on top, and then calls `sendBuffer()`. A new static element therefore belongs in `drawGrid()`, and anything dynamic belongs in `uiRender()`. Because the panel is 1 bpp, a "dimmed" empty card is a 50 % checkerboard clear. `uiBegin()` must call `SPI.begin()` with the LCD pins before `lcd.begin()`, and it sends `0x38` to put the panel in HPM (~32 Hz self-refresh), which live motion needs.

`include/radar.h` holds the shared state (`Track`, `RadarState`, `LinkStatus`), the sensor wiring pins, the radar geometry (8 m range, ±60°) and the `tracker*`/`ui*` prototypes. The link goes `WAIT` → `OK` → `LOST` (no frame for 1 s), and all tracks are cleared on `LOST`. The KEY button (GPIO18) resets the session max speed.

## Hardware notes

- The sensor sits on UART0's header pins (TXD/RXD = GPIO43/44). The ROM boot log goes out on GPIO43 into the sensor's RX; the sensor ignores it. The fallback is GPIO1/GPIO2 (`PIN_LD2450_*` in `include/radar.h`).
- `VBUS` powers the sensor only when USB is connected. On battery, the LD2450 needs an external 5 V boost.
- Orientation fixes: `RADAR_MIRROR_X` in `radar.h` for left/right, `U8G2_R3` in `radar_ui.cpp` for upside down.
- `docs-preview.png` in the README is a host-rendered preview of `radar_ui.cpp`. The renderer that made it is not in this repo.

// Turns raw LD2450 slots into smoothed tracks with a velocity vector and a
// trail. The sensor reports ~10 times a second and its positions jitter by a
// few centimetres; the screen redraws ~25 times a second. So there are two
// filters: one per sensor frame (position + velocity), one per screen frame
// (display easing) that makes the dot glide instead of jump.

#include "radar.h"

static const float POS_ALPHA = 0.6f;       // position low-pass per sensor frame
static const float VEL_ALPHA = 0.3f;       // velocity low-pass per sensor frame
static const float EASE = 0.35f;           // display easing per screen frame
static const float TRAIL_STEP_M = 0.08f;   // add a trail dot every 8 cm of motion
static const uint32_t HOLD_MS = 600;       // keep a track through short dropouts

static void trailPush(Track &t) {
  if (t.trailCount) {
    uint8_t last = (t.trailHead + TRAIL_LEN - 1) % TRAIL_LEN;
    float ddx = t.x - t.trailX[last], ddy = t.y - t.trailY[last];
    if (ddx * ddx + ddy * ddy < TRAIL_STEP_M * TRAIL_STEP_M) return;
  }
  t.trailX[t.trailHead] = t.x;
  t.trailY[t.trailHead] = t.y;
  t.trailHead = (t.trailHead + 1) % TRAIL_LEN;
  if (t.trailCount < TRAIL_LEN) t.trailCount++;
}

void trackerUpdate(RadarState &s, const Ld2450Frame &f, uint32_t nowMs) {
  for (int i = 0; i < 3; i++) {
    const Ld2450Target &src = f.t[i];
    Track &t = s.tracks[i];
    if (!src.present) continue;  // expiry is handled by trackerExpire()

    float rx = src.x_mm / 1000.0f;
    float ry = src.y_mm / 1000.0f;
#if RADAR_MIRROR_X
    rx = -rx;
#endif
    float spd = src.speed_cms / 100.0f;

    if (!t.active) {
      t = Track();
      t.active = true;
      t.x = t.dx = rx;
      t.y = t.dy = ry;
    } else {
      float px = t.x, py = t.y;
      t.x += (rx - t.x) * POS_ALPHA;
      t.y += (ry - t.y) * POS_ALPHA;
      float dt = (nowMs - t.lastUpdMs) / 1000.0f;
      if (dt > 0.02f && dt < 1.0f) {
        t.vx += ((t.x - px) / dt - t.vx) * VEL_ALPHA;
        t.vy += ((t.y - py) / dt - t.vy) * VEL_ALPHA;
      }
    }
    t.speed = spd;
    t.lastSeenMs = t.lastUpdMs = nowMs;
    trailPush(t);

    float a = fabsf(spd);
    if (a > s.stats.maxSpeed) {
      s.stats.maxSpeed = a;
      s.stats.maxSlot = i;
      s.stats.maxAtMs = nowMs;
    }
  }
  trackerExpire(s, nowMs);
}

void trackerExpire(RadarState &s, uint32_t nowMs) {
  for (Track &t : s.tracks) {
    if (t.active && nowMs - t.lastSeenMs > HOLD_MS) t.active = false;
  }
}

void trackerAnimate(RadarState &s) {
  for (Track &t : s.tracks) {
    if (!t.active) continue;
    t.dx += (t.x - t.dx) * EASE;
    t.dy += (t.y - t.dy) * EASE;
  }
}

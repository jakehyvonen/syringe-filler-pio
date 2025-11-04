#include "hw/Encoder.hpp"
#include <ESP32Encoder.h>

namespace EncoderHW {

static ESP32Encoder s_encoder;
static volatile bool s_poll = false;
static volatile long s_offset = 0;     // software offset (home/index)
static volatile long s_lastPrinted = LONG_MIN;

// ----------------------------------------------------
// If you ever want to use a real index ISR later,
// you can re-enable this and call attachInterrupt()
// from somewhere else.
// ----------------------------------------------------
void IRAM_ATTR index_isr() {
  // Zero at index: capture current count as new offset reference
  long c = (long)s_encoder.getCount();
  s_offset = c;
}

void begin() {
  // some versions of the lib use an enum for pulls
  ESP32Encoder::useInternalWeakPullResistors = puType::NONE;

  // A/B channels
  s_encoder.attachFullQuad(PIN_A, PIN_B);
  s_encoder.clearCount();
  s_offset = 0;

  // We DON'T auto-zero on index anymore.
  // We still configure the pin so we can poll it if we want.
  pinMode(PIN_IDX, INPUT);
  // attachInterrupt((int)PIN_IDX, index_isr, RISING);  // <-- leave commented for now
}

void reset() {
  // full reset to 0
  s_encoder.clearCount();
  s_offset = 0;
}

// zero at current position (used by homing or index search)
void zeroHere() {
  s_offset = (long)s_encoder.getCount();
}

// raw hardware count, no offset
long raw() {
  return (long)s_encoder.getCount();
}

// logical count (what the rest of the app should use)
long count() {
  return (long)s_encoder.getCount() - s_offset;
}

float mm() {
  return (float)count() / COUNTS_PER_MM;
}

void setPolling(bool on) { s_poll = on; }
bool polling() { return s_poll; }

void service() {
  if (!s_poll) return;

  static uint32_t lastMs = 0;
  uint32_t now = millis();
  if (now - lastMs < 20) return; // ~50 Hz
  lastMs = now;

  long c = count();
  if (c != s_lastPrinted) {
    s_lastPrinted = c;
    Serial.printf("[ENC] count=%ld  pos=%.3f mm\n", c, mm());
  }
}

// optional helper if you later re-enable the ISR path
void onIndexPulse() {
  s_offset = (long)s_encoder.getCount();
}

} // namespace EncoderHW

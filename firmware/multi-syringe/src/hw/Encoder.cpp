/**
 * @file Encoder.cpp
 * @brief Linear encoder implementation using ESP32Encoder.
 */
#include "hw/Encoder.hpp"
#include <ESP32Encoder.h>

namespace EncoderHW {

static ESP32Encoder s_encoder;
static volatile bool s_poll = false;
static volatile long s_offset = 0;     // software offset (home/index)
static volatile long s_lastPrinted = LONG_MIN;

// Optional index ISR hook.
// ISR hook to zero the encoder on index pulse.
void IRAM_ATTR index_isr() {
  // Zero at index: capture current count as new offset reference
  long c = (long)s_encoder.getCount();
  s_offset = c;
}

// Initialize the hardware encoder and pins.
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

// Reset the hardware count and software offset to zero.
void reset() {
  // full reset to 0
  s_encoder.clearCount();
  s_offset = 0;
}

// zero at current position (used by homing or index search)
// Set the current hardware count as the logical zero.
void zeroHere() {
  s_offset = (long)s_encoder.getCount();
}

// raw hardware count, no offset
// Return the raw hardware count.
long raw() {
  return (long)s_encoder.getCount();
}

// logical count (what the rest of the app should use)
// Return the logical count with offset and sign applied.
long count() {
  // raw - offset, then apply sign
  long c = (long)s_encoder.getCount() - s_offset;
  return c * ENCODER_SIGN;
}

// Convert the logical count to millimeters.
float mm() {
  return (float)count() / COUNTS_PER_MM;
}

// Enable or disable periodic serial reporting.
void setPolling(bool on) { s_poll = on; }
// Return true when polling is enabled.
bool polling() { return s_poll; }

// Print encoder status periodically when polling is enabled.
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

// Optional helper for an index ISR path.
// Update offset to match the current hardware count.
void onIndexPulse() {
  s_offset = (long)s_encoder.getCount();
}

} // namespace EncoderHW

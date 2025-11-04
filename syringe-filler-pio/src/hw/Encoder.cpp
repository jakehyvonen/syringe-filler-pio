#include "hw/Encoder.hpp"
#include <ESP32Encoder.h>

namespace EncoderHW {

static ESP32Encoder s_encoder;
static volatile bool s_poll = false;
static volatile long s_offset = 0; // for zeroing at index
static volatile long s_lastPrinted = LONG_MIN;

void IRAM_ATTR index_isr() {
  // Zero at index: capture current count as new offset reference
  long c = (long)s_encoder.getCount();
  s_offset = c;
}

void begin() {
  // PCNT units need this on some cores
  ESP32Encoder::useInternalWeakPullResistors = false;

  // Attach quadrature
  s_encoder.attachFullQuad(PIN_A, PIN_B);
  s_encoder.clearCount();
  s_offset = 0;

  // Index pulse interrupt (rising edge)
  pinMode(PIN_IDX, INPUT); // driven by level shifter -> no pullups here
  attachInterrupt((int)PIN_IDX, index_isr, RISING);
}

void reset() {
  s_encoder.clearCount();
  s_offset = 0;
}

long count() {
  // Apply offset so index becomes zero
  return (long)s_encoder.getCount() - s_offset;
}

float mm() {
  return (float)count() / COUNTS_PER_MM;
}

void setPolling(bool on) { s_poll = on; }
bool polling() { return s_poll; }

void service() {
  if (!s_poll) return;

  // Throttle prints a bit (adjust as you like)
  static uint32_t lastMs = 0;
  uint32_t now = millis();
  if (now - lastMs < 20) return; // ~50 Hz print
  lastMs = now;

  long c = count();
  if (c != s_lastPrinted) {
    s_lastPrinted = c;
    Serial.printf("[ENC] count=%ld  pos=%.3f mm\n", c, mm());
  }
}

void onIndexPulse() {
  // Optional: call this if you ever want to force zeroing from elsewhere
  s_offset = (long)s_encoder.getCount();
}

} // namespace EncoderHW

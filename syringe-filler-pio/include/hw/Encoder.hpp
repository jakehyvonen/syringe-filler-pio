#pragma once
#include <Arduino.h>
#include "hw/Pins.hpp"   // for ENC_A / ENC_B / ENC_Z if you want, or you can repeat pins here

namespace EncoderHW {

// Logical counts/mm we derived earlier for your Phidgets strip
constexpr float COUNTS_PER_MM = 200.0f;

// Pin assignments (you can keep them here or in Pins.hpp; you're already putting them in Pins.hpp,
// so if you prefer, remove these 3 lines and just include Pins.hpp in Encoder.cpp)
constexpr uint8_t PIN_A   = Pins::ENC_A;
constexpr uint8_t PIN_B   = Pins::ENC_B;
constexpr uint8_t PIN_IDX = Pins::ENC_Z;

constexpr int ENCODER_SIGN = -1;   // was +1; flip because encoder is mounted reversed

// init hardware encoder (A/B, prepare index pin)
void begin();

// full reset: clear hardware count and software offset
void reset();

// set current hardware count as zero (software offset only)
// this is what Homing should call after it's done
void zeroHere();

// get raw hardware count straight from ESP32Encoder (no offset)
long raw();

// get logical count (raw - offset)
long count();

// convert to mm using COUNTS_PER_MM
float mm();

// enable/disable periodic serial printing
void setPolling(bool on);
bool polling();

// to be called from App::loop() so "enc on" can print
void service();

// optional, in case you re-enable the real index ISR path
void onIndexPulse();

} // namespace EncoderHW

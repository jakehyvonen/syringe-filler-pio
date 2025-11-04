#pragma once
#include <Arduino.h>

namespace EncoderHW {

// Logical counts per mm for your math (ENC4111_0 is 5 µm/pulse per channel).
// With quadrature x4, effective counts/mm = (1 mm / 0.005 mm) * 4 = 200 * 4 = 800
constexpr float COUNTS_PER_MM = 800.0f;

// Pin assignment – chosen to avoid conflicts with your Pins.hpp.
constexpr gpio_num_t PIN_A   = GPIO_NUM_23;
constexpr gpio_num_t PIN_B   = GPIO_NUM_13;
constexpr gpio_num_t PIN_IDX = GPIO_NUM_39; // input-only; fine for index

void begin();
void reset();
long count();           // raw quadrature count (can be negative if reversed)
float mm();             // position in mm
void setPolling(bool on);
bool polling();

void onIndexPulse();    // ISR-safe hook to zero-at-index (optional)

void service();         // call periodically if polling==true to print

} // namespace EncoderHW

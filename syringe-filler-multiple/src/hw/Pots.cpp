/**
 * @file Pots.cpp
 * @brief ADS1115-backed potentiometer sampling and filtering.
 */
#include "hw/Pots.hpp"
#include "hw/Drivers.hpp"
#include "hw/Pins.hpp"
#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include <Wire.h>

namespace Pots {

// ---- Config -----------------------------------------------------------------
static constexpr uint8_t ADS0_ADDR = 0x48; // ADDR->GND
static constexpr uint8_t ADS1_ADDR = 0x49; // ADDR->VDD
static constexpr adsGain_t PGA = GAIN_ONE; // ±4.096 V

static constexpr uint8_t VREF_ADS = 0; // ADS0
static constexpr uint8_t VREF_CH  = 0; // A0
static constexpr uint8_t TOOL_POT_IDX = 2; // ADS0, A3

struct Chan { uint8_t ads; uint8_t ch; };
static constexpr Chan POT_MAP[] = {
  {0, 1}, {0, 2}, {0, 3}, // ADS0 A1..A3
  {1, 0}, {1, 1}, {1, 2}, {1, 3}  // ADS1 A0..A3
};
static_assert(sizeof(POT_MAP)/sizeof(POT_MAP[0]) == NUM_POTS, "POT_MAP size");

static constexpr uint8_t  POT_EMA_SHIFT = 3; // 1/8 new
static constexpr uint16_t POT_REPORT_HYST_COUNTS = 64;

// ---- State ------------------------------------------------------------------
// NOTE: default-construct; pass address in begin(addr)
static Adafruit_ADS1115 s_ads[2];
static bool s_ads_present[2] = {false, false};
static const uint8_t s_addr[2] = { ADS0_ADDR, ADS1_ADDR };
static uint8_t s_last_ch[2] = {0xFF, 0xFF};

static bool     inited = false;
static uint16_t pot_raw[NUM_POTS];
static uint16_t pot_filt[NUM_POTS];
static uint16_t pot_last_reported[NUM_POTS];

// ---- Helpers ----------------------------------------------------------------
static inline int16_t read_counts(uint8_t ads_idx, uint8_t ch) {
  if (!s_ads_present[ads_idx]) return 0;

  // ADS1115 is muxed; after switching channels we discard one conversion so
  // command reads always reflect the selected input, not the prior channel.
  if (s_last_ch[ads_idx] != ch) {
    (void)s_ads[ads_idx].readADC_SingleEnded(ch);
    s_last_ch[ads_idx] = ch;
  }

  int16_t v = s_ads[ads_idx].readADC_SingleEnded(ch);
  if (v < 0) v = 0;
  return v; // 0..32767 at GAIN_ONE
}

static inline uint16_t read_vref_counts() {
  return read_counts(VREF_ADS, VREF_CH);
}

static inline float ratio_percent(uint16_t pot_counts, uint16_t vref_counts) {
  if (vref_counts < 16) return 0.0f;
  if (pot_counts > vref_counts) pot_counts = vref_counts;
  return (100.0f * float(pot_counts)) / float(vref_counts);
}

// ---- API --------------------------------------------------------------------
// Initialize ADS1115 devices and seed filter state.
void init() {
  // If Drivers/Wire aren’t already up elsewhere, ensure I2C is started there.

  for (int i = 0; i < 2; ++i) {
    s_ads_present[i] = s_ads[i].begin(s_addr[i]);
    if (s_ads_present[i]) {
      s_ads[i].setGain(PGA);
      // Optional: s_ads[i].setDataRate(RATE_ADS1115_250SPS);
      Serial.print("ADS"); Serial.print(i); Serial.println(" present.");
    } else {
      Serial.print("ADS"); Serial.print(i); Serial.println(" not found!");
    }
  }

  // Seed + settle
  for (uint8_t i = 0; i < NUM_POTS; ++i) {
    uint16_t r = read_counts(POT_MAP[i].ads, POT_MAP[i].ch);
    pot_raw[i]  = r;
    pot_filt[i] = r;
  }
  for (uint8_t k = 0; k < 8; ++k) {
    for (uint8_t i = 0; i < NUM_POTS; ++i) {
      uint16_t r = read_counts(POT_MAP[i].ads, POT_MAP[i].ch);
      pot_filt[i] = (pot_filt[i] + r) >> 1;
    }
    delay(2);
  }
  for (uint8_t i = 0; i < NUM_POTS; ++i) {
    pot_last_reported[i] = pot_filt[i];
    Serial.print("pot["); Serial.print(i); Serial.print("](init counts)=");
    Serial.println(pot_filt[i]);
  }
  inited = true;
}

// Poll all pots, update filters, and emit changes.
void poll() {
  if (!inited) init();

  const uint16_t vref = read_vref_counts();
  for (uint8_t i = 0; i < NUM_POTS; ++i) {
    uint16_t r = read_counts(POT_MAP[i].ads, POT_MAP[i].ch);
    pot_raw[i] = r;
    pot_filt[i] += (int16_t(r) - int16_t(pot_filt[i])) >> POT_EMA_SHIFT;

    uint16_t last = pot_last_reported[i];
    uint16_t diff = (pot_filt[i] > last) ? (pot_filt[i] - last) : (last - pot_filt[i]);
    if (diff >= POT_REPORT_HYST_COUNTS) {
      pot_last_reported[i] = pot_filt[i];
      Serial.print("pot["); Serial.print(i); Serial.print("] counts=");
      Serial.print(pot_filt[i]);
      Serial.print("  percent=");
      Serial.println(ratio_percent(pot_filt[i], vref), 2);
    }
  }
}

// Return the last raw counts for a pot index.
uint16_t raw(uint8_t i)  { return (i < NUM_POTS) ? pot_raw[i]  : 0; }
// Return the filtered counts for a pot index.
uint16_t filt(uint8_t i) { return (i < NUM_POTS) ? pot_filt[i] : 0; }

// Return the pot position as a percent of Vref.
float percent(uint8_t i) {
  if (i >= NUM_POTS) return 0.0f;
  uint16_t vref = read_vref_counts();
  return ratio_percent(pot_filt[i], vref);
}

// Convert ADS counts to a percent ratio.
float ratioFromCounts(uint16_t potCounts) {
  uint16_t vref = read_vref_counts();
  return ratio_percent(potCounts, vref);
}

// Convert a percent ratio back to ADS counts.
uint16_t countsFromRatio(float ratio) {
  if (ratio <= 0.0f) return 0;
  uint16_t vref = read_vref_counts();
  if (vref < 16) return 0;
  if (ratio >= 100.0f) return vref;
  return static_cast<uint16_t>(lroundf((ratio * 0.01f) * vref));
}

// Read a single pot channel in native ADS counts.
uint16_t readCounts(uint8_t i) {
  if (i >= NUM_POTS) return 0;
  return read_counts(POT_MAP[i].ads, POT_MAP[i].ch);
}

// Return the mapped pot index for the toolhead syringe channel.
uint8_t toolPotIndex() {
  return TOOL_POT_IDX;
}

// ---- Legacy shim ------------------------------------------------------------
// Keep AxisPair happy for now, but make it ratiometric and well-behaved.
// Read a pot channel on the legacy 0..1023 scale.
uint16_t readScaled(uint8_t i) {
  if (i >= NUM_POTS) return 0;
  float p = percent(i);                 // 0..100
  if (p <= 0) return 0;
  if (p >= 100.0f) return 1023;
  // Map 0..100% to 0..1023 (legacy scale) without using raw supply voltage.
  return (uint16_t) lroundf(p * 10.23f);
}

} // namespace Pots

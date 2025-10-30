#include "hw/Drivers.hpp"
#include "hw/Pins.hpp"
#include <Wire.h>
#include <Arduino.h>
#include "hw/RFID.hpp"

// Keep these here; headers already declare them 'extern'
Adafruit_PWMServoDriver Drivers::PCA;   // default 0x40
Adafruit_ADS1115        Drivers::ADS;   // 0x48 or 0x49

namespace {
  bool     g_i2cInit  = false;
  uint32_t g_i2cFreq  = 0;

  bool g_hasPCA = false;
  bool g_hasADS = false;

  uint8_t g_adsAddr = 0;  // 0 if none; otherwise 0x48 or 0x49

  inline bool i2cPresentQuick(uint8_t addr) {
    Wire.beginTransmission(addr);
    return (Wire.endTransmission(true) == 0);
  }
}

// -----------------------------
// Public API
// -----------------------------

// Default overload -> Pins constants
bool Drivers::initI2C() {
  return Drivers::initI2C(Pins::I2C_SDA, Pins::I2C_SCL, Pins::I2C_FREQ); // e.g. 100000
}

// Explicit overload (idempotent)
bool Drivers::initI2C(int sda, int scl, uint32_t freq) {
  if (!g_i2cInit) {
    Wire.begin(sda, scl);
    Wire.setClock(freq);
    Wire.setTimeOut(3000);
    g_i2cInit = true;
    g_i2cFreq = freq;
    Serial.printf("[I2C] started SDA=%d SCL=%d @%lu Hz\n",
                  sda, scl, (unsigned long)freq);
  } else if (g_i2cFreq != freq) {
    Wire.setClock(freq);
    g_i2cFreq = freq;
    Serial.printf("[I2C] clock updated to %lu Hz\n", (unsigned long)freq);
  }

  // ---- Probe ADS1115(s) ---------------------------------
  g_hasADS = false;
  g_adsAddr = 0;

  if ( i2cPresentQuick(0x48) && Drivers::ADS.begin(0x48) ) {
    g_hasADS  = true; g_adsAddr = 0x48;
  } else if ( i2cPresentQuick(0x49) && Drivers::ADS.begin(0x49) ) {
    g_hasADS  = true; g_adsAddr = 0x49;
  }

  if (g_hasADS) {
    Drivers::ADS.setGain(GAIN_ONE); // ±4.096 V (125 µV/LSB)
    Serial.printf("ADS1115 detected @0x%02X (GAIN_ONE)\n", g_adsAddr);
  } else {
    Serial.println("WARN: ADS1115 not found; pot readings disabled.");
  }

  // ---- Probe PCA9685 ------------------------------------
  if (i2cPresentQuick(0x40)) {
    Drivers::PCA.begin();
    Drivers::PCA.setPWMFreq(60); // Servos ~50–60 Hz
    g_hasPCA = true;
    Serial.println("PCA9685 detected @0x40");
  } else {
    g_hasPCA = false;
    Serial.println("WARN: PCA9685 not found; servo control disabled.");
  }

  // ---- PN532 presence + init (no I2C ownership here) ----
  if (i2cPresentQuick(0x24)) {
    Serial.println("INFO: PN532 ACK at 0x24");
    RFID::init();

    
  } else {
    Serial.println("INFO: PN532 not ACKing at 0x24 (mode/address mismatch?)");
  }

  return true;
}

bool Drivers::i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission(true) == 0);
}

bool Drivers::hasPCA() { return g_hasPCA; }
bool Drivers::hasADS() { return g_hasADS; }

#include "hw/Drivers.hpp"
#include "hw/Pins.hpp"
#include <Wire.h>
#include <Arduino.h>

namespace {
  bool g_hasPCA = false;
  bool g_hasADS = false;
}

Adafruit_PWMServoDriver Drivers::PCA;   // default address 0x40
Adafruit_ADS1115        Drivers::ADS;   // default address 0x48

// Default overload uses Pins.hpp constants
bool Drivers::initI2C() {
  return Drivers::initI2C(Pins::I2C_SDA, Pins::I2C_SCL, Pins::I2C_FREQ);
}

// Explicit overload
bool Drivers::initI2C(int sda, int scl, uint32_t freq) {
  Wire.begin(sda, scl);
  Wire.setClock(freq);

  // ADS1115 probe
  g_hasADS = (i2cPresent(0x48) && ADS.begin(0x48));
  if (g_hasADS) {
    ADS.setGain(GAIN_ONE);
    Serial.println("ADS1115 detected @0x48");
  } else {
    Serial.println("WARN: ADS1115 not found; pot readings disabled.");
  }

  // PCA9685 probe
  if (i2cPresent(0x40)) {
    PCA.begin();
    PCA.setPWMFreq(60);
    g_hasPCA = true;
    Serial.println("PCA9685 detected @0x40");
  } else {
    g_hasPCA = false;
    Serial.println("WARN: PCA9685 not found; servo control disabled.");
  }
  return true;
}

bool Drivers::i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

bool Drivers::hasPCA() { return g_hasPCA; }
bool Drivers::hasADS() { return g_hasADS; }

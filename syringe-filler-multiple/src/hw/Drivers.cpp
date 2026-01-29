/**
 * @file Drivers.cpp
 * @brief I2C initialization and peripheral detection.
 */
#include "hw/Drivers.hpp"
#include "hw/Pins.hpp"
#include "hw/RFID.hpp"
#include "hw/BaseRFID.hpp"

#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_PWMServoDriver.h>

// ----------------------------------------------------------
// Global hardware objects
// ----------------------------------------------------------
Adafruit_PWMServoDriver Drivers::PCA;   // default 0x40
Adafruit_ADS1115        Drivers::ADS;   // 0x48 or 0x49

// ----------------------------------------------------------
// Internal state
// ----------------------------------------------------------
namespace {
  bool     g_i2cInit  = false;
  uint32_t g_i2cFreq  = 0;

  bool     g_i2c2Init = false;
  uint32_t g_i2c2Freq = 0;

  bool g_hasPCA = false;
  bool g_hasADS = false;

  uint8_t g_adsAddr = 0;  // 0 if none; otherwise 0x48 or 0x49

  inline bool i2cPresentQuick(uint8_t addr) {
    Wire.beginTransmission(addr);
    return (Wire.endTransmission(true) == 0);
  }

  inline bool i2c2PresentQuick(uint8_t addr) {
    Wire1.beginTransmission(addr);
    return (Wire1.endTransmission(true) == 0);
  }

  // Simple helper for I2C bus scanning
  void scanBus(TwoWire &bus, const char *name) {
    Serial.printf("[%s] scanning…\n", name);
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 127; ++addr) {
      bus.beginTransmission(addr);
      uint8_t err = bus.endTransmission(true);
      if (err == 0) {
        Serial.printf("[%s] 0x%02X\n", name, addr);
        ++found;
      } else if (err == 4) {
        Serial.printf("[%s] 0x%02X (unknown error)\n", name, addr);
      }
    }
    if (!found) Serial.printf("[%s] no devices.\n", name);
  }
} // namespace

// ----------------------------------------------------------
// Public API
// ----------------------------------------------------------

// Default overload
// Initialize the primary I2C bus with default pins.
bool Drivers::initI2C() {
  return Drivers::initI2C(Pins::I2C_SDA, Pins::I2C_SCL, Pins::I2C_FREQ);
}

// Primary bus init
// Initialize the primary I2C bus with explicit pins/frequency.
bool Drivers::initI2C(int sda, int scl, uint32_t freq) {
  if (!g_i2cInit) {
    Wire.begin(sda, scl);
    Wire.setClock(freq);
    Wire.setTimeOut(3000);
    g_i2cInit = true;
    g_i2cFreq = freq;
    Serial.printf("[I2C] started SDA=%d SCL=%d @%lu Hz\n", sda, scl, (unsigned long)freq);
  } else if (g_i2cFreq != freq) {
    Wire.setClock(freq);
    g_i2cFreq = freq;
    Serial.printf("[I2C] clock updated to %lu Hz\n", (unsigned long)freq);
  }

  // ---- ADS1115 detection ----
  g_hasADS = false;
  g_adsAddr = 0;
  if (i2cPresentQuick(0x48) && Drivers::ADS.begin(0x48)) {
    g_hasADS = true; g_adsAddr = 0x48;
  } else if (i2cPresentQuick(0x49) && Drivers::ADS.begin(0x49)) {
    g_hasADS = true; g_adsAddr = 0x49;
  }

  if (g_hasADS) {
    Drivers::ADS.setGain(GAIN_ONE);
    Serial.printf("ADS1115 detected @0x%02X (GAIN_ONE)\n", g_adsAddr);
  } else {
    Serial.println("WARN: ADS1115 not found; pot readings disabled.");
  }

  // ---- PCA9685 detection ----
  if (i2cPresentQuick(0x40)) {
    Drivers::PCA.begin();
    Drivers::PCA.setPWMFreq(60);
    g_hasPCA = true;
    Serial.println("PCA9685 detected @0x40");
  } else {
    g_hasPCA = false;
    Serial.println("WARN: PCA9685 not found; servo control disabled.");
  }

  // ---- PN532 detection (I2C0) ----
  if (i2cPresentQuick(0x24)) {
    Serial.println("INFO: PN532 ACK at 0x24 (Wire0)");
    RFID::init();
  } else {
    Serial.println("INFO: PN532 not ACKing at 0x24 (mode/address mismatch?)");
  }

  return true;
}

// Return true if a device acknowledges on the primary I2C bus.
bool Drivers::i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission(true) == 0);
}

// Secondary I2C (Wire1)
// Initialize the secondary I2C bus with default pins.
bool Drivers::initI2C2() {
  if (!g_i2c2Init) {
    Wire1.begin(Pins::I2C2_SDA, Pins::I2C2_SCL);
    Wire1.setClock(Pins::I2C2_FREQ);
    Wire1.setTimeOut(3000);
    g_i2c2Init = true;
    g_i2c2Freq = Pins::I2C2_FREQ;
    Serial.printf("[I2C2] started SDA=%d SCL=%d @%lu Hz\n",
                  Pins::I2C2_SDA, Pins::I2C2_SCL, (unsigned long)Pins::I2C2_FREQ);
  } else if (g_i2c2Freq != Pins::I2C2_FREQ) {
    Wire1.setClock(Pins::I2C2_FREQ);
    g_i2c2Freq = Pins::I2C2_FREQ;
    Serial.printf("[I2C2] clock updated to %lu Hz\n", (unsigned long)Pins::I2C2_FREQ);
  }

  // ---- PN532 detection (I2C1) ----
  if (i2c2PresentQuick(0x24)) {
    Serial.println("INFO: PN532 ACK at 0x24 (Wire1)");
    BaseRFID::init();
  } else {
    Serial.println("INFO: PN532 not ACKing at 0x24 on Wire1 (mode/address mismatch?)");
  }

  return true;
}

// Return true if a device acknowledges on the secondary I2C bus.
bool Drivers::i2c2Present(uint8_t addr) {
  Wire1.beginTransmission(addr);
  return (Wire1.endTransmission(true) == 0);
}

// ---- I2C scanning (both buses) ----
// Scan both I2C buses and print detected addresses.
void Drivers::i2cScanBoth() {
  if (!g_i2cInit) {
    Wire.begin(Pins::I2C_SDA, Pins::I2C_SCL);
    Wire.setClock(Pins::I2C_FREQ);
    Wire.setTimeOut(3000);
    g_i2cInit = true;
  }
  if (!g_i2c2Init) {
    Wire1.begin(Pins::I2C2_SDA, Pins::I2C2_SCL);
    Wire1.setClock(Pins::I2C2_FREQ);
    Wire1.setTimeOut(3000);
    g_i2c2Init = true;
  }
  scanBus(Wire,  "I2C0");
  scanBus(Wire1, "I2C1");
}

// ---- Status getters ----
// Return true if the PCA9685 was detected.
bool Drivers::hasPCA() { return g_hasPCA; }
// Return true if the ADS1115 was detected.
bool Drivers::hasADS() { return g_hasADS; }

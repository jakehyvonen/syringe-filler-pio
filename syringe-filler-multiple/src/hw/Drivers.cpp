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
#include <Adafruit_MCP23X17.h>

Adafruit_MCP23X17 Drivers::MCP;
Adafruit_ADS1115 Drivers::ADS;
Adafruit_MCP23X17 Drivers::BASE_EN_EXPANDER;

namespace {
  bool g_i2cInit = false;
  uint32_t g_i2cFreq = 0;

  bool g_hasMCP = false;
  uint8_t g_mcpAddr = 0;
  bool g_hasADS = false;
  bool g_hasBaseEnableExpander = false;
  uint8_t g_adsAddr = 0;

  inline bool i2cPresentQuick(uint8_t addr) {
    Wire.beginTransmission(addr);
    return (Wire.endTransmission(true) == 0);
  }

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

bool Drivers::initI2C() {
  return Drivers::initI2C(Pins::I2C_SDA, Pins::I2C_SCL, Pins::I2C_FREQ);
}

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

  g_hasMCP = false;
  g_mcpAddr = 0;
  for (uint8_t addr = 0x20; addr <= 0x27; ++addr) {
    if (!i2cPresentQuick(addr)) continue;

    if (Drivers::MCP.begin_I2C(addr, &Wire)) {
      g_hasMCP = true;
      g_mcpAddr = addr;
      break;
    }

    Serial.printf("WARN: MCP23017 ACK at 0x%02X but init failed.\n", addr);
  }

  if (g_hasMCP) {
    Serial.printf("MCP23017 detected @0x%02X on I2C0\n", g_mcpAddr);
  } else {
    Serial.println("WARN: MCP23017 not found on I2C0 (checked 0x20-0x27). Base expander offline.");
  }

  g_hasBaseEnableExpander = Drivers::BASE_EN_EXPANDER.begin_I2C(Pins::BASE_EN_MCP_ADDR, &Wire);
  if (g_hasBaseEnableExpander) {
    Serial.printf("MCP23X17 base enable expander detected @0x%02X on I2C0\n", Pins::BASE_EN_MCP_ADDR);
  } else {
    Serial.printf("WARN: MCP23X17 base enable expander not found @0x%02X on I2C0.\n", Pins::BASE_EN_MCP_ADDR);
  }

  g_hasADS = false;
  g_adsAddr = 0;
  if (i2cPresentQuick(0x48) && Drivers::ADS.begin(0x48, &Wire)) {
    g_hasADS = true;
    g_adsAddr = 0x48;
  } else if (i2cPresentQuick(0x49) && Drivers::ADS.begin(0x49, &Wire)) {
    g_hasADS = true;
    g_adsAddr = 0x49;
  }

  if (g_hasADS) {
    Drivers::ADS.setGain(GAIN_ONE);
    Serial.printf("ADS1115 detected @0x%02X on I2C (GAIN_ONE)\n", g_adsAddr);
  } else {
    Serial.println("WARN: ADS1115 not found on I2C; pot readings disabled.");
  }

  if (i2cPresentQuick(0x24)) {
    Serial.println("INFO: Base PN532 ACK at 0x24 (Wire0)");
    BaseRFID::init();
  } else {
    Serial.println("INFO: Base PN532 not ACKing at 0x24 (mode/address mismatch?)");
  }

  RFID::init();
  return true;
}

bool Drivers::i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission(true) == 0);
}

void Drivers::i2cScanBoth() {
  if (!g_i2cInit) {
    Wire.begin(Pins::I2C_SDA, Pins::I2C_SCL);
    Wire.setClock(Pins::I2C_FREQ);
    Wire.setTimeOut(3000);
    g_i2cInit = true;
  }
  scanBus(Wire, "I2C0");
}

bool Drivers::hasMCP() { return g_hasMCP; }
bool Drivers::hasADS() { return g_hasADS; }
bool Drivers::hasBaseEnableExpander() { return g_hasBaseEnableExpander; }
